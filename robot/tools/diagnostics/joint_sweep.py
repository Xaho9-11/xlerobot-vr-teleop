"""Powered test of all six SO101 joints, one at a time.
Each joint moves AMP toward the side with more calibrated margin and back, slowly, while every
other joint is held. Safety: per-step limit, goal=present before torque-on, abort on tracking error
or held-joint drift, user confirms before torque on/off. Logs every sample to one CSV."""
import csv, os, sys, time
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

PORT, RID, HZ = "/dev/ttyACM0", "left_arm_test", 20
# joint: (amplitude, step per tick). Body joints in degrees, gripper in 0..100 %.
PLAN = {"shoulder_pan": (10, 0.5), "shoulder_lift": (8, 0.4), "elbow_flex": (10, 0.5),
        "wrist_flex": (10, 0.5), "wrist_roll": (15, 0.75), "gripper": (20, 1.0)}
MAX_TRACK_ERR = {"gripper": 15.0}; DEFAULT_TRACK_ERR = 8.0
MAX_DRIFT = 10.0

# usage: joint_sweep.py [joints comma list|all] [max_relative_target deg] [P coefficient]
SEL = sys.argv[1] if len(sys.argv) > 1 else "all"
MAX_REL = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
P_GAIN = int(sys.argv[3]) if len(sys.argv) > 3 else 16
if SEL != "all":
    PLAN = {j: PLAN[j] for j in SEL.split(",")}
print(f"joints={list(PLAN)} max_relative_target={MAX_REL} P={P_GAIN}", flush=True)
robot = SOFollower(SOFollowerRobotConfig(port=PORT, id=RID, use_degrees=True, max_relative_target=MAX_REL,
                                         position_p_coefficient=P_GAIN))

def limits(j):
    if j == "gripper":
        return 0.0, 100.0
    c = robot.calibration[j]
    h = (c.range_max - c.range_min) / 2 * 360 / 4096
    return -h, h

input("Hold the arm in the calibration middle pose (upper arm vertical, forearm forward), then press ENTER to turn torque ON...")
robot.bus.connect()
for attempt in range(1, 4):  # transient "Incorrect status packet" seen once on a write; retry is safe (idempotent)
    try:
        robot.bus.disable_torque(num_retry=2)
        robot.bus.sync_write("Goal_Position", robot.bus.sync_read("Present_Position"))
        time.sleep(0.05)
        robot.configure()  # torque back on, holding the current pose
        break
    except ConnectionError as exc:
        print(f"setup attempt {attempt} failed: {exc}", flush=True)
        time.sleep(0.3)
else:
    print("setup failed 3 times; torque left OFF. Check cables/power.")
    sys.exit(1)
input("Torque is ON and holding. Let go gently and check the arm holds still. Press ENTER to start the sweep...")
hold = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}

os.makedirs(os.path.expanduser("~/fmp_logs"), exist_ok=True)
log_path = os.path.expanduser(f"~/fmp_logs/joint_sweep_{time.strftime('%Y%m%d_%H%M%S')}_rel{MAX_REL:g}_p{P_GAIN}.csv")
summary, aborted = [], None
t0 = time.perf_counter()
with open(log_path, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t", "joint", "target", "present", "max_held_drift"])
    for j, (amp, step) in PLAN.items():
        key = f"{j}.pos"; p0 = hold[key]; lo, hi = limits(j)
        sign = 1 if (hi - p0) >= (p0 - lo) else -1
        if max(hi - p0, p0 - lo) < amp + 3:
            summary.append(f"{j:13s} SKIPPED (margin too small: pos {p0:+.1f}, limits {lo:+.1f}..{hi:+.1f})"); continue
        n = int(round(amp / step))
        profile = [p0 + sign * step * i for i in range(n + 1)] + [p0 + sign * step * i for i in range(n, -1, -1)] + [p0] * HZ
        errs, worst_drift = [], 0.0
        print(f"-> {j}: {p0:+.1f} -> {p0 + sign * amp:+.1f} -> back", flush=True)
        for tgt in profile:
            tick = time.perf_counter()
            action = dict(hold); action[key] = tgt
            robot.send_action(action)
            o = robot.get_observation()
            e = o[key] - tgt; errs.append(e)
            drift = max(abs(o[k] - hold[k]) for k in hold if k not in (key, "gripper.pos"))
            worst_drift = max(worst_drift, drift)
            w.writerow([f"{tick - t0:.3f}", j, f"{tgt:.2f}", f"{o[key]:.2f}", f"{drift:.2f}"])
            if abs(e) > MAX_TRACK_ERR.get(j, DEFAULT_TRACK_ERR):
                aborted = f"{j}: tracking error {e:+.1f}"; break
            if drift > MAX_DRIFT:
                aborted = f"{j}: held joint drift {drift:.1f} deg"; break
            time.sleep(max(0.0, 1 / HZ - (time.perf_counter() - tick)))
        moving = errs[: 2 * n + 1]
        unit = "%" if j == "gripper" else "deg"
        summary.append(f"{j:13s} {'+' if sign > 0 else '-'}{amp}{unit}  mean|err| {sum(abs(x) for x in moving)/len(moving):.2f}  "
                       f"max|err| {max(abs(x) for x in moving):.2f}  settle {errs[-1]:+.2f}  held-drift max {worst_drift:.2f} deg")
        if aborted:
            break
        time.sleep(0.5)

print("\n== summary" + (f"  ABORTED: {aborted}" if aborted else "  (all joints done)"))
for s in summary:
    print(s)
print(f"log: {log_path}")
input("HOLD THE ARM (it will drop when torque is released), then press ENTER...")
robot.disconnect()
print("torque off, disconnected")
