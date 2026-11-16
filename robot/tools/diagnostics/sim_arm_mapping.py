"""Offline simulation of teleop.py's arm mapping with synthetic controller poses (no robot, no headset).
Checks: neutral grip -> gripper horizontal/level; hand translation -> straight-line end-effector translation
with unchanged gripper pitch/roll; wrist roll -> gripper roll only. Leaves no neutral file behind."""
import json, math, os, sys
sys.argv = sys.argv[:1]
import teleop as t, head_follow as hf
from xlerobot_model.SO101Robot import SO101Kinematics


def q(axis, deg):
    r = math.radians(deg) / 2; s = math.sin(r)
    return tuple([s * a for a in axis] + [math.cos(r)])


def ee(T):
    x, y = t.fk_consistent(T["shoulder_lift"], T["elbow_flex"])
    pan = math.radians(T["shoulder_pan"])
    pitch = T["shoulder_lift"] + T["elbow_flex"] + T["wrist_flex"]
    return f"ee fwd {x * math.cos(pan):.3f} right {x * math.sin(pan):+.3f} up {y:.3f} m | gripper pitch {pitch:+.1f} roll {T['wrist_roll']:+.1f} | pan {T['shoulder_pan']:+.1f}"


# never touch the real controller_neutral.json (2026-10-20: an earlier version left a fake neutral there)
import tempfile
t.NEUTRAL_FILE = os.path.join(tempfile.mkdtemp(), "controller_neutral_sim.json")
try:
    cal = json.load(open(hf.XLR_CAL)); lim = json.load(open(os.path.join(t.HERE, "joint_limits.json")))["pose_deg_limits_both_arms"]
    arm = t.ArmTeleop("left", SO101Kinematics(), cal, lim, 1.0, 60.0)
    ready = {f"left_arm_{j}.pos": t.to_lerobot_deg(cal, f"left_arm_{j}", 0.0) for j in t.ARM_JOINTS}
    ready["left_arm_gripper.pos"] = 1.0
    arm.init_from(ready)
    neutral = q((1, 0, 0), 40)  # assumed natural grip: controller forward axis 40 deg below horizontal
    H = (0, 0, 0, 1)
    c = lambda pos, rot: {"tracked": True, "pos": pos, "rot": rot, "grip": 1.0, "trigger": 0.0}

    def hold(pos, rot, n=80):
        for _ in range(n):
            arm.update(c(pos, rot), H, True, 0.02)

    print("start (ready pose)        ", ee(arm.target))
    hold([0, 1.0, 0.30], neutral, 1)
    print("grip pressed, neutral grip", ee(arm.target))
    hold([0, 1.0, 0.35], neutral); print("hand +5 cm forward        ", ee(arm.target))
    hold([0.08, 1.0, 0.35], neutral); print("then +8 cm right          ", ee(arm.target))
    hold([0.08, 1.05, 0.35], neutral); print("then +5 cm up             ", ee(arm.target))
    hold([0.08, 1.05, 0.35], t.q_mul(neutral, q((0, 0, 1), 30))); print("wrist rolled 30 CCW       ", ee(arm.target))
    hold([0.08, 1.05, 0.35], t.q_mul(q((1, 0, 0), 60), q((0, 0, 1), 0))); print("hand tipped 20 more down  ", ee(arm.target))
finally:
    if os.path.exists(t.NEUTRAL_FILE):
        os.remove(t.NEUTRAL_FILE)
