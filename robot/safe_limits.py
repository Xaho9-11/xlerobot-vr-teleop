"""Conservative software joint limits for XLeRobot (user decision 2026-10-19: use the intersection of
both arms' calibrated ranges). The calibration file is NOT modified; controllers clamp goals with this.

Angles here are "pose degrees": 0 = the pose held when calibrating (raw 2047 after half-turn homing),
same sign convention as the servo. Per joint we take, on each side, the smaller of the two arms' reach;
shoulder_pan is additionally made symmetric (smallest of all four half-ranges), because body obstacles
are mirrored between left and right arms. A MARGIN is kept inside every limit.
LeRobot DEGREES mode measures from the middle of each motor's own calibrated range instead, so
`to_lerobot_deg` converts per motor.

Run:  python safe_limits.py [calibration.json]  -> writes joint_limits.json next to this file and prints it.
"""
import json, os, sys

CAL = os.path.expanduser("~/.cache/huggingface/lerobot/calibration/robots/xlerobot/xlerobot_fmp.json")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "joint_limits.json")
HOME_RAW, MAX_RES = 2047, 4095
MARGIN = 3.0  # deg kept inside each limit
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
SYMMETRIC = {"shoulder_pan"}
FULL_TURN = {"wrist_roll"}


def pose_deg(raw):
    return (raw - HOME_RAW) * 360 / MAX_RES


def build(cal):
    limits = {}
    for j in ARM_JOINTS:
        if j in FULL_TURN:
            limits[j] = [-170.0, 170.0]
            continue
        lo = [pose_deg(cal[f"{s}_arm_{j}"]["range_min"]) for s in ("left", "right")]
        hi = [pose_deg(cal[f"{s}_arm_{j}"]["range_max"]) for s in ("left", "right")]
        neg, pos = min(-v for v in lo), min(hi)  # reach on each side, smaller of the two arms
        if j in SYMMETRIC:
            neg = pos = min(neg, pos)
        limits[j] = [round(-neg + MARGIN, 1), round(pos - MARGIN, 1)]
    head = {}
    for m in ("head_motor_1", "head_motor_2"):
        head[m] = [round(pose_deg(cal[m]["range_min"]) + MARGIN, 1), round(pose_deg(cal[m]["range_max"]) - MARGIN, 1)]
    return {"pose_deg_limits_both_arms": limits, "pose_deg_limits_head": head,
            "gripper_percent": [0.0, 100.0], "margin_deg": MARGIN, "source": os.path.basename(CAL)}


def to_lerobot_deg(cal, motor, pose_angle):
    """pose degrees -> LeRobot DEGREES value for one motor."""
    c = cal[motor]
    mid = (c["range_min"] + c["range_max"]) / 2
    return (pose_angle * MAX_RES / 360 + HOME_RAW - mid) * 360 / MAX_RES


def lerobot_limits(cal, table):
    """Per-motor limits in LeRobot DEGREES units, e.g. {'left_arm_shoulder_pan': [lo, hi], ...}."""
    out = {}
    for side in ("left", "right"):
        for j, (lo, hi) in table["pose_deg_limits_both_arms"].items():
            m = f"{side}_arm_{j}"
            out[m] = [round(to_lerobot_deg(cal, m, lo), 2), round(to_lerobot_deg(cal, m, hi), 2)]
    for m, (lo, hi) in table["pose_deg_limits_head"].items():
        out[m] = [round(to_lerobot_deg(cal, m, lo), 2), round(to_lerobot_deg(cal, m, hi), 2)]
    return out


def clamp_action(action, lerobot_lims):
    """Clamp a LeRobot action dict ('<motor>.pos' keys) in place to the safe limits; returns clamped names."""
    hit = []
    for m, (lo, hi) in lerobot_lims.items():
        k = f"{m}.pos"
        if k in action and not lo <= action[k] <= hi:
            action[k] = max(lo, min(hi, action[k])); hit.append(m)
    return hit


if __name__ == "__main__":
    cal = json.load(open(sys.argv[1] if len(sys.argv) > 1 else CAL))
    table = build(cal)
    table["lerobot_deg_limits"] = lerobot_limits(cal, table)
    json.dump(table, open(OUT, "w"), indent=2)
    print(json.dumps({k: table[k] for k in ("pose_deg_limits_both_arms", "pose_deg_limits_head")}, indent=1))
    print("written", OUT)
