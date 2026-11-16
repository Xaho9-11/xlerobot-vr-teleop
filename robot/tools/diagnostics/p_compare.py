"""Compare servo P gains on one SO101 arm in a single powered session.
For each P in P_LIST and each joint in JOINTS: move +/-AMP at SPEED deg/s and back, then hold 1 s.
P is changed with torque ON (Lock off -> write -> Lock on), so the arm never drops between runs.
Metrics: tracking error, lag (best shift), overshoot, settle error, hold jitter. CSV logged."""
import csv, os, statistics as st, sys, time
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

PORT, RID, HZ = "/dev/ttyACM0", "left_arm_test", 30
P_LIST = [16, 24, 32]
JOINTS = ["shoulder_pan", "elbow_flex", "wrist_flex"]
AMP, SPEED = 15.0, 30.0          # deg, deg/s
MAX_TRACK_ERR, MAX_DRIFT = 12.0, 10.0
step = SPEED / HZ

robot = SOFollower(SOFollowerRobotConfig(port=PORT, id=RID, use_degrees=True, max_relative_target=8.0,
                                         position_p_coefficient=P_LIST[0]))

def set_p(p):
    for m in robot.bus.motors:
        if m == "gripper":
            continue
        robot.bus.write("Lock", m, 0, num_retry=2)
        robot.bus.write("P_Coefficient", m, p, num_retry=2)
        robot.bus.write("Lock", m, 1, num_retry=2)
    back = {m: robot.bus.read("P_Coefficient", m, num_retry=2) for m in robot.bus.motors if m != "gripper"}
    if any(v != p for v in back.values()):
        raise RuntimeError(f"P readback mismatch: {back}")

def limits(j):
    c = robot.calibration[j]; h = (c.range_max - c.range_min) / 2 * 360 / 4096
    return -h, h

input("Hold the arm in the middle pose, then press ENTER to turn torque ON...")
robot.bus.connect()
for attempt in range(1, 4):
    try:
        robot.bus.disable_torque(num_retry=2)
        robot.bus.sync_write("Goal_Position", robot.bus.sync_read("Present_Position"))
        time.sleep(0.05)
        robot.configure()
        break
    except ConnectionError as exc:
        print(f"setup attempt {attempt} failed: {exc}", flush=True); time.sleep(0.3)
else:
    print("setup failed; torque left OFF"); sys.exit(1)
input("Torque ON. Let go gently, check it holds still, press ENTER to start (~25 s)...")
hold = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}

os.makedirs(os.path.expanduser("~/fmp_logs"), exist_ok=True)
log_path = os.path.expanduser(f"~/fmp_logs/p_compare_{time.strftime('%Y%m%d_%H%M%S')}.csv")
results, aborted = [], None
t0 = time.perf_counter()
with open(log_path, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t", "P", "joint", "phase", "target", "present"])
    for p in P_LIST:
        set_p(p); time.sleep(0.3)
        for j in JOINTS:
            key = f"{j}.pos"; p0 = hold[key]; lo, hi = limits(j)
            sign = 1 if (hi - p0) >= (p0 - lo) else -1
            if max(hi - p0, p0 - lo) < AMP + 3:
                results.append((p, j, None)); continue
            n = int(round(AMP / step))
            move = [p0 + sign * step * i for i in range(n + 1)] + [p0 + sign * step * i for i in range(n - 1, -1, -1)]
            profile = [("move", x) for x in move] + [("hold", p0)] * HZ
            tg, pr, ph = [], [], []
            for phase, tgt in profile:
                tick = time.perf_counter()
                a = dict(hold); a[key] = tgt
                robot.send_action(a)
                o = robot.get_observation()
                tg.append(tgt); pr.append(o[key]); ph.append(phase)
                w.writerow([f"{tick - t0:.3f}", p, j, phase, f"{tgt:.2f}", f"{o[key]:.2f}"])
                drift = max(abs(o[k] - hold[k]) for k in hold if k not in (key, "gripper.pos"))
                if abs(o[key] - tgt) > MAX_TRACK_ERR:
                    aborted = f"P={p} {j}: tracking error {o[key] - tgt:+.1f}"; break
                if drift > MAX_DRIFT:
                    aborted = f"P={p} {j}: held drift {drift:.1f}"; break
                time.sleep(max(0.0, 1 / HZ - (time.perf_counter() - tick)))
            if aborted:
                break
            nm = len(move)
            err = [b - a for a, b in zip(tg[:nm], pr[:nm])]
            lag = min(range(0, 10), key=lambda k: sum((pr[i + k] - tg[i]) ** 2 for i in range(nm)))
            peak = max(sign * (x - p0) for x in pr)
            holdv = pr[nm + HZ // 2:]
            results.append((p, j, dict(mean=sum(map(abs, err)) / nm, mx=max(map(abs, err)), lag_ms=lag * 1000 / HZ,
                                       over=peak - AMP, settle=st.mean(holdv) - p0, jitter=st.pstdev(holdv))))
            time.sleep(0.4)
        if aborted:
            break

try:
    set_p(16)
except Exception as exc:
    print("could not restore P=16:", exc)
print("\n== P comparison" + (f"  ABORTED: {aborted}" if aborted else ""))
print(f"{'P':>3} {'joint':13s} {'mean|e|':>8} {'max|e|':>7} {'lag':>6} {'overshoot':>9} {'settle':>7} {'jitter':>7}")
for p, j, r in results:
    if r is None:
        print(f"{p:>3} {j:13s} skipped (margin)"); continue
    print(f"{p:>3} {j:13s} {r['mean']:8.2f} {r['mx']:7.2f} {r['lag_ms']:5.0f}ms {r['over']:+9.2f} {r['settle']:+7.2f} {r['jitter']:7.3f}")
print("(deg; lag = best-fit shift incl. read/write; jitter = std while holding)")
print(f"log: {log_path}")
input("HOLD THE ARM (it will drop when torque is released), then press ENTER...")
robot.disconnect()
print("torque off, disconnected")
