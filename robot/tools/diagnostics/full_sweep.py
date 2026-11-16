"""Whole-XLeRobot joint check: every arm/head joint moves a small amount and back, one at a time.

Safety: goals clamped to joint_limits.json (two-arm intersection), max_relative_target per tick,
goal=present before torque-on, abort on tracking error / held-joint drift, base wheels held at 0,
the user confirms before torque on and before torque off.
While joints move, the three cameras are watched:
  * arm joints  -> which wrist camera sees motion (verifies left/right camera assignment)
  * head motors -> image shift of the head camera (tells which motor pans/tilts and its sign)
Usage (via full_sweep.sh): full_sweep.py [left,head,right]
"""
import csv, json, logging, os, sys, threading, time
import cv2
import numpy as np
from lerobot_robot_xlerobot import XLerobot, XLerobotConfig

PARTS = sys.argv[1].split(",") if len(sys.argv) > 1 else ["left", "head", "right"]
HZ, SPEED = 30, 20.0                 # loop rate, deg/s (gripper: %/s)
MAX_TRACK_ERR, MAX_DRIFT = 8.0, 10.0
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
AMP = {"wrist_roll": 15.0, "gripper": 20.0}
HERE = os.path.dirname(os.path.abspath(__file__))
logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)

limits = json.load(open(os.path.join(HERE, "joint_limits.json")))["lerobot_deg_limits"]
limits.update({"left_arm_gripper": [0.0, 100.0], "right_arm_gripper": [0.0, 100.0]})

# ---------------------------------------------------------------- cameras
class Cam:
    def __init__(self, path):
        self.cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(3, 320); self.cap.set(4, 240)
        self.frame, self.energy, self.lock = None, [], threading.Lock()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        prev = None
        while True:
            ok, fr = self.cap.read()
            if not ok:
                time.sleep(0.01); continue
            g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY).astype(np.float32)
            t = time.perf_counter()
            with self.lock:
                self.frame = g
                if prev is not None:
                    self.energy.append((t, float(np.abs(g - prev).mean())))
            prev = g

    def motion(self, t0, t1):
        with self.lock:
            v = [e for t, e in self.energy if t0 <= t <= t1]
        return sum(v) / len(v) if v else float("nan")

    def snap(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

cams = {k: Cam(os.environ[v]) for k, v in
        (("head", "XLR_CAM_HEAD"), ("left_wrist", "XLR_CAM_LEFT_WRIST"), ("right_wrist", "XLR_CAM_RIGHT_WRIST"))}

# ---------------------------------------------------------------- robot
robot = XLerobot(XLerobotConfig(id=os.environ["XLR_ID"], port1=os.environ["XLR_PORT1"],
                                port2=os.environ["XLR_PORT2"], use_degrees=True, max_relative_target=None))
# XLeRobot's own max_relative_target path raises KeyError (looks up "<motor>.pos" in a dict keyed by motor
# name, seen 2026-10-19), so it stays off and the per-tick step limit is applied here instead.
MAX_STEP = 8.0  # deg per tick relative to the present position
input("Support both arms in the middle pose, head facing forward, wheels clear. ENTER to turn torque ON...")
for attempt in range(1, 4):
    try:
        for bus in (robot.bus1, robot.bus2):
            if not bus.is_connected:
                bus.connect()
            bus.calibration = {k: v for k, v in robot.calibration.items() if k in bus.motors}
            bus.disable_torque(num_retry=2)
        pos_motors1 = robot.left_arm_motors + robot.head_motors
        robot.bus1.sync_write("Goal_Position", robot.bus1.sync_read("Present_Position", pos_motors1))
        robot.bus2.sync_write("Goal_Position", robot.bus2.sync_read("Present_Position", robot.right_arm_motors))
        robot.bus2.sync_write("Goal_Velocity", dict.fromkeys(robot.base_motors, 0))
        time.sleep(0.05)
        robot.configure()
        break
    except ConnectionError as exc:
        print(f"setup attempt {attempt} failed: {exc}", flush=True); time.sleep(0.5)
else:
    sys.exit("setup failed 3 times; torque left OFF")


def _release_on_crash(exc_type, exc, tb):
    """Uncaught error or Ctrl-C after torque-on: report, ask the user to hold the arms, release torque."""
    import traceback
    traceback.print_exception(exc_type, exc, tb)
    if robot.bus1.is_connected or robot.bus2.is_connected:
        try:
            input("ERROR - HOLD BOTH ARMS (torque will be released), then press ENTER...")
        except BaseException:
            pass
        for bus in (robot.bus1, robot.bus2):
            if bus.is_connected:
                bus.disconnect(disable_torque=True)
        print("torque off")


sys.excepthook = _release_on_crash
input("Torque ON. Let go gently, check everything holds. ENTER to start...")
hold = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}

joints = []
if "left" in PARTS: joints += [f"left_arm_{j}" for j in ARM]
if "head" in PARTS: joints += ["head_motor_1", "head_motor_2"]
if "right" in PARTS: joints += [f"right_arm_{j}" for j in ARM]

os.makedirs(os.path.expanduser("~/fmp_logs"), exist_ok=True)
log_path = os.path.expanduser(f"~/fmp_logs/full_sweep_{time.strftime('%Y%m%d_%H%M%S')}.csv")
summary, aborted = [], None
t_origin = time.perf_counter()
try:
    with open(log_path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t", "joint", "target", "present"])
        for m in joints:
            key = f"{m}.pos"; p0 = hold[key]; lo, hi = limits[m]
            amp = next((a for k, a in AMP.items() if m.endswith(k)), 10.0)
            sign = 1 if hi - p0 >= p0 - lo else -1
            if max(hi - p0, p0 - lo) < amp + 2:
                summary.append(f"{m:24s} SKIPPED: pos {p0:+.1f} too close to limits {lo:+.1f}..{hi:+.1f}"); continue
            n = int(round(amp / (SPEED / HZ)))
            out = [p0 + sign * amp * i / n for i in range(n + 1)]
            profile = [("go", x) for x in out] + [("peak", out[-1])] * (HZ // 2) + \
                      [("back", x) for x in reversed(out)] + [("settle", p0)] * (HZ // 2)
            base0 = time.perf_counter(); time.sleep(0.4)
            quiet = {c: cams[c].motion(base0, time.perf_counter()) for c in cams}
            img0 = cams["head"].snap() if m.startswith("head") else None
            t_start, errs, drift_max, shift = time.perf_counter(), [], 0.0, None
            present = robot.get_observation()
            for phase, tgt in profile:
                tick = time.perf_counter()
                if phase == "peak" and img0 is not None and shift is None and tick - t_start > n / HZ + 0.3:
                    img1 = cams["head"].snap()
                    (dx, dy), resp = cv2.phaseCorrelate(img0, img1)
                    shift = (dx, dy, resp)
                a = dict(hold); a[key] = max(lo, min(hi, tgt))
                a[key] = max(present[key] - MAX_STEP, min(present[key] + MAX_STEP, a[key]))
                robot.send_action(a)
                o = robot.get_observation(); present = o
                e = o[key] - a[key]; errs.append(e)
                drift = max(abs(o[k] - hold[k]) for k in hold if k != key and not k.endswith("gripper.pos"))
                drift_max = max(drift_max, drift)
                w.writerow([f"{tick - t_origin:.3f}", m, f"{a[key]:.2f}", f"{o[key]:.2f}"])
                lim = 15.0 if m.endswith("gripper") else MAX_TRACK_ERR
                if abs(e) > lim:
                    aborted = f"{m}: tracking error {e:+.1f}"; break
                if drift > MAX_DRIFT:
                    aborted = f"{m}: held joint drift {drift:.1f}"; break
                time.sleep(max(0.0, 1 / HZ - (time.perf_counter() - tick)))
            t_end = time.perf_counter()
            mot = {c: cams[c].motion(t_start, t_end) - quiet[c] for c in cams}
            moving = [abs(x) for x, (ph, _) in zip(errs, profile) if ph in ("go", "back")]
            line = (f"{m:24s} {'+' if sign > 0 else '-'}{amp:g}  mean|err| {sum(moving) / max(1, len(moving)):.2f}  "
                    f"settle {errs[-1]:+.2f}  drift {drift_max:.2f}  cam motion " +
                    " ".join(f"{c}:{v:+.1f}" for c, v in mot.items()))
            if shift:
                line += f"  head image shift dx {shift[0]:+.1f}px dy {shift[1]:+.1f}px (resp {shift[2]:.2f})"
            summary.append(line)
            print(line, flush=True)
            if aborted:
                break
            time.sleep(0.3)

finally:
    # always reached, also after an exception: never leave the robot torqued without telling the user
    print("\n== summary" + (f"  ABORTED: {aborted}" if aborted else "  (all done)"))
    for s in summary:
        print(s)
    print("log:", log_path)
    print("Notes: 'cam motion' = extra image change while the joint moved (bigger = that camera saw it).\n"
          "Head image shift: scene moves opposite to the camera; dx<0 => camera turned right, dy<0 => camera tilted down.")
    input("HOLD BOTH ARMS (torque will be released), then press ENTER...")
    robot.disconnect()
    print("torque off, disconnected")
