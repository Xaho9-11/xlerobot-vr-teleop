"""Slow, step-confirmed check of the head's usable range (cable strain / collisions) before widening
head_follow limits. Moves head_motor_1/2 (pan/tilt) one target at a time at SPEED deg/s, holds 1 s,
returns to centre. Directions use the verified signs: pan + = turn right, tilt - = look up.
Usage (via head_range_check.sh): head_range_check.py [yaw_deg] [pitch_deg]   (defaults 90 45)
"""
import os, sys, time
sys.argv, _args = sys.argv[:1], sys.argv[1:]
import head_follow as hf

YAW = float(_args[0]) if len(_args) > 0 else 90.0
PITCH = float(_args[1]) if len(_args) > 1 else 45.0
SPEED, HZ = 30.0, 50
rig = hf.HeadRig(os.environ["XLR_PORT1"], p_gain=32, max_rel=20.0)
pan, tilt = rig.joints
input("Point the head straight ahead and level by hand, then ENTER to turn head torque ON...")
if not rig.connect():
    sys.exit("setup failed; torque left OFF")
c = rig.get_observation()
centre = {pan: c[f"{pan}.pos"], tilt: c[f"{tilt}.pos"]}
print(f"centre (LeRobot deg): pan {centre[pan]:+.1f}  tilt {centre[tilt]:+.1f}; limits {rig.limits[pan]} / {rig.limits[tilt]}")


def go(target_pan, target_tilt):
    """Ramp from the current goal to the target in head-relative degrees (pan + = right, tilt + = up)."""
    cur = rig.get_observation()
    start = (cur[f"{pan}.pos"] - centre[pan], -(cur[f"{tilt}.pos"] - centre[tilt]))
    end = (target_pan, target_tilt)
    n = max(1, int(max(abs(e - s) for s, e in zip(start, end)) / SPEED * HZ))
    worst = 0.0
    for i in range(1, n + 1):
        p = start[0] + (end[0] - start[0]) * i / n
        t = start[1] + (end[1] - start[1]) * i / n
        sent = rig.send_action({f"{pan}.pos": centre[pan] + p, f"{tilt}.pos": centre[tilt] - t})
        obs = rig.get_observation()
        worst = max(worst, abs(obs[f"{pan}.pos"] - sent[f"{pan}.pos"]), abs(obs[f"{tilt}.pos"] - sent[f"{tilt}.pos"]))
        time.sleep(1 / HZ)
    time.sleep(1.0)
    obs = rig.get_observation()
    reached = (obs[f"{pan}.pos"] - centre[pan], -(obs[f"{tilt}.pos"] - centre[tilt]))
    return reached, worst


try:
    for label, tgt in [(f"turn RIGHT {YAW:g}", (YAW, 0)), ("back to centre", (0, 0)),
                       (f"turn LEFT {YAW:g}", (-YAW, 0)), ("back to centre", (0, 0)),
                       (f"look UP {PITCH:g}", (0, PITCH)), ("back to centre", (0, 0)),
                       (f"look DOWN {PITCH:g}", (0, -PITCH)), ("back to centre", (0, 0))]:
        input(f"\nNEXT: {label}. Watch the camera cable and clearances. ENTER to move...")
        (rp, rt), worst = go(*tgt)
        print(f"  reached pan {rp:+.1f} (right +), tilt {rt:+.1f} (up +); max lag during move {worst:.1f} deg")
finally:
    try:
        go(0, 0)
    except Exception:
        pass
    input("\nTorque will be released (the head may sag a little). ENTER...")
    rig.disconnect()
    print("head torque off")
