"""First powered motion test for one SO101 arm.
Holds all joints at their current pose and moves ONLY wrist_flex by +/-AMP degrees and back, slowly.
Safety: max_relative_target per step, range-margin check, abort on tracking error, user confirms
before torque on and before torque off. Logs target/present to CSV."""
import csv, os, sys, time
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

PORT, RID = "/dev/ttyACM0", "left_arm_test"
JOINT, AMP, STEP, HZ = "wrist_flex", 10.0, 0.5, 20   # deg, deg per tick, loop rate
MAX_TRACK_ERR = 8.0    # deg, abort if the moving joint lags this much
MAX_DRIFT = 10.0       # deg, abort if a held joint drifts this much

cfg = SOFollowerRobotConfig(port=PORT, id=RID, use_degrees=True, max_relative_target=3.0)
robot = SOFollower(cfg)

# range margin from calibration (degrees mode: raw mid = (min+max)/2, 4096 ticks per turn)
c = robot.calibration[JOINT]
half_span = (c.range_max - c.range_min) / 2 * 360 / 4096
input(f"Torque will turn ON and hold the current pose. Arm clamped and area clear? Press ENTER...")
robot.bus.connect()
# avoid a jump at torque-on: set every goal to the present position while torque is still off
robot.bus.disable_torque()
robot.bus.sync_write("Goal_Position", robot.bus.sync_read("Present_Position"))
robot.configure()  # writes PID/mode with torque off, then re-enables torque
obs = robot.get_observation()
start = {k: v for k, v in obs.items() if k.endswith(".pos")}
p0 = start[f"{JOINT}.pos"]
room_pos, room_neg = half_span - p0, half_span + p0
sign = 1 if room_pos >= room_neg else -1
print(f"{JOINT}: now {p0:+.1f} deg, limits +/-{half_span:.1f}; moving {'+' if sign > 0 else '-'}{AMP} deg")
if max(room_pos, room_neg) < AMP + 5:
    print("not enough margin, aborting"); robot.disconnect(); sys.exit(1)

os.makedirs(os.path.expanduser("~/fmp_logs"), exist_ok=True)
log_path = os.path.expanduser(f"~/fmp_logs/motion_test_{time.strftime('%Y%m%d_%H%M%S')}.csv")
n = int(AMP / STEP)
profile = [p0 + sign * STEP * i for i in range(n + 1)] + [p0 + sign * STEP * i for i in range(n, -1, -1)]
profile += [p0] * HZ  # settle 1 s
aborted = None
t0 = time.perf_counter()
with open(log_path, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t", "target", "present"] + [k for k in start if k != f"{JOINT}.pos"])
    for tgt in profile:
        tick = time.perf_counter()
        action = dict(start); action[f"{JOINT}.pos"] = tgt
        robot.send_action(action)
        o = robot.get_observation()
        pres = o[f"{JOINT}.pos"]
        others = [o[k] for k in start if k != f"{JOINT}.pos"]
        w.writerow([f"{tick - t0:.3f}", f"{tgt:.2f}", f"{pres:.2f}"] + [f"{x:.2f}" for x in others])
        if abs(pres - tgt) > MAX_TRACK_ERR:
            aborted = f"tracking error {pres - tgt:+.1f} deg"; break
        drift = {k: o[k] - start[k] for k in start if k != f"{JOINT}.pos" and k != "gripper.pos"}
        bad = {k: d for k, d in drift.items() if abs(d) > MAX_DRIFT}
        if bad:
            aborted = f"held joint drift {bad}"; break
        time.sleep(max(0.0, 1 / HZ - (time.perf_counter() - tick)))

final = robot.get_observation()
print("ABORTED: " + aborted if aborted else "motion done")
print(f"{JOINT}: start {p0:+.1f}  end {final[f'{JOINT}.pos']:+.1f} deg")
print("held joints drift (deg): " + ", ".join(f"{k.split('.')[0]} {final[k] - start[k]:+.1f}" for k in start if k != f"{JOINT}.pos"))
print(f"log: {log_path}")
input("HOLD THE ARM (it may drop when torque is released), then press ENTER...")
robot.disconnect()
print("torque off, disconnected")
