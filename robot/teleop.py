"""XLeRobot teleop with the (FMP-modified) reachy-vr-quest Quest app: head follow + arm(s), one process.

One process must own both buses (bus1 = left arm + head, bus2 = right arm + base). Head control is the
2026-10-20 head_follow pipeline (One Euro -> gap extrapolation -> SpringFollower; P48, Minimum_Startup_Force 40).

Arms (clutch / "grip to move"):
  * press a controller's GRIP (> 0.6, release < 0.4) to engage that arm; release: the arm holds
  * --pos-mode joint (default since 2026-10-21): each robot joint follows the corresponding joint of the
    operator's arm (see "Joint-correspondence mapping" below); calibrate once per session by clicking the
    thumbstick with that arm straight forward and horizontal; eased in over 0.8 s after each press
  * --pos-mode clutch: hand displacement since the press moves the gripper by the same amount (x SCALE),
    straight-line Cartesian through SO101Kinematics IK, target kept reachable (status MOVE@edge)
  * X / A while not gripping: neutral grip (gripper horizontal, wrist camera level); hand orientation drives
    wrist_flex / wrist_roll in both modes. (Y opens a menu in the app - not used.)
  * TRIGGER closes the gripper (0 = released -> GRIP_OPEN %, 1 = fully pressed -> 0 %)
  * prompts are shown in the headset: written to /dev/shm/fmp_hud.json, drawn by video_daemon.py
  * shoulder_lift stops ~52 deg forward (calibrated range): the arm cannot point forward-horizontal
Angles: kinematics in "pose degrees" (0 = calibration pose, raw 2047; signs checked 2026-10-20 on the left
arm: pan + = right, lift + = forward/down, elbow + = forearm down, wrist_flex + = gripper down,
wrist_roll + = CCW seen from behind) and converted per motor to LeRobot DEGREES (mid of calibrated range).
Every joint goal is clamped to joint_limits.json (two-arm intersection) and smoothed by a SpringFollower.
Base: velocity held at 0 (base work is a separate step).

Start-up / shut-down (user request 2026-10-20, no more holding the arms by hand): torque comes on where the
arm(s) and head rest (folded), then a minimum-jerk move goes to the ready pose (calibration pose: upper arm
vertical, forearm forward, wrist straight; head straight and level); on exit everything glides back to that
rest pose before torque is released.

Usage (via teleop.sh): teleop.py [--arms left|right|both|none] [--scale 1.0]
Keys: c+ENTER recentre head, q+ENTER quit.
"""
import argparse, csv, json, logging, math, os, sys, threading, time

sys_args = sys.argv[1:]
sys.argv = sys.argv[:1]
import head_follow as hf
from safe_limits import to_lerobot_deg

HERE = os.path.dirname(os.path.abspath(__file__))
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


# ---------------------------------------------------------------- small quaternion helpers (Unity: left-handed)
def q_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def q_inv(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def q_rot(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    r = q_mul(q_mul(q, qv), q_inv(q))
    return r[:3]


def heading_deg(q):
    """yaw of a Unity rotation's forward axis, + = turned right."""
    f = q_rot(q, (0, 0, 1))
    return math.degrees(math.atan2(f[0], f[2]))


def rel_pitch_roll(q0, q):
    """controller rotation since latch, in its own frame: pitch + = tip down, roll + = CCW seen from behind.
    Roll is the twist about the controller's own forward axis (swing-twist decomposition). The first version
    took atan2 of the rotated up vector, which became unstable once the controller was pitched far up or
    down: wrist_roll targets jumped between -85 and +150 deg in the 2026-10-20 test."""
    r = q_mul(q_inv(q0), q)
    f = q_rot(r, (0, 0, 1))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -f[1]))))
    x, y, z, w = r
    n = math.hypot(z, w)
    roll = 0.0 if n < 1e-6 else math.degrees(2 * math.atan2(z / n, w / n))
    roll = ((roll + 180) % 360) - 180
    return pitch, roll


# XLeRobot SO101Kinematics.forward_kinematics is inconsistent with its own inverse_kinematics (elbow comes
# back as 32.4 - elbow; checked 2026-10-20), while the IK is right (IK(0.1629, 0.1131) ~ (0, 2) = the zero
# pose used by the official VR example). This FK is derived from the IK code and round-trips exactly.
_O1 = math.atan2(0.028, 0.11257)
_O2 = math.atan2(0.0052, 0.1349) + _O1


def fk_consistent(lift_deg, elbow_deg, l1=0.1159, l2=0.1350):
    t1 = math.radians(90 - lift_deg) - _O1
    t2 = math.radians(elbow_deg + 90) - _O2
    return l1 * math.cos(t1) + l2 * math.cos(t1 - t2), l1 * math.sin(t1) + l2 * math.sin(t1 - t2)


def abs_pitch_roll(q):
    """Orientation relative to gravity (Unity: x right, y up, z forward): pitch of its forward axis
    below horizontal (+ = tip down) and roll about that axis (+ = CCW seen from behind, i.e. the right side
    rises). Well defined while the forward axis is not pointing straight up or down."""
    f = q_rot(q, (0, 0, 1)); r = q_rot(q, (1, 0, 0)); u = q_rot(q, (0, 1, 0))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -f[1]))))
    roll = math.degrees(math.atan2(r[1], u[1]))
    return pitch, roll


def yaw_q(deg):
    """rotation about Unity +y (up) by deg, + = turn right."""
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


def gripper_q(rot, heading, neutral_q):
    """Virtual gripper orientation in the operator's frame (2026-10-21). The controller's own forward axis is
    ~80 deg from horizontal in the natural grip (measured: pitch -80), where pitch/roll of the controller
    itself are singular - wrist_roll targets jumped between +-170 deg. Instead the controller rotation is taken
    relative to the neutral grip: G = yaw(-heading) * rot * neutral^-1, identity in the neutral grip; a hand
    rotation in the world (left-multiplied) rotates G the same way, so G's pitch/roll are well defined."""
    return q_mul(q_mul(yaw_q(-heading), rot), q_inv(neutral_q))


F_MIN = 0.08   # m, the end effector is kept at least this far in front of the base axis
R_MAX = 0.245  # m, shoulder-to-wrist distance kept below the straight-arm 0.251 (IK clamps there)
R_MIN = 0.07   # m
NEUTRAL_FILE = os.path.join(HERE, "controller_neutral.json")


def load_neutral():
    """side -> yaw-normalised neutral controller quaternion (older pitch/roll entries are ignored)."""
    try:
        return {k: tuple(v) for k, v in json.load(open(NEUTRAL_FILE)).items() if len(v) == 4}
    except (OSError, ValueError):
        return {}


def save_neutral(side, value):
    data = load_neutral(); data[side] = list(value)
    json.dump(data, open(NEUTRAL_FILE, "w"), indent=1)


def ctrl_tag(c):
    """Raw controller state for the status line (2026-10-21: two runs logged grip 0 throughout)."""
    if not c:
        return "(none)"
    if not c.get("tracked"):
        return "(not tracked)"
    return (f"(g{c.get('grip', 0):.1f} t{c.get('trigger', 0):.1f}{' X/A' if c.get('primary') else ''}"
            f"{' stick' if c.get('stick_click') else ''})")


# Joint-correspondence mapping (user request 2026-10-21, after a body-relative Cartesian version: "the robot
# shoulder stayed vertical, only the joints below the upper arm moved; each servo should correspond to my
# joint"). The operator's shoulder is estimated from the headset: a neck pivot NECK (behind/below the eyes,
# so turning the head barely moves it) plus a per-side shoulder offset in the torso frame. Hand - shoulder,
# solved as a two-link human arm (elbow hanging below the shoulder-hand line), gives the upper-arm azimuth
# and forward swing and the elbow flexion; they drive the robot's segments:
#   shoulder_pan    = upper-arm azimuth (+ = right), 1:1
#   shoulder_lift   = upper-arm swing forward from hanging x LIFT_GAIN (the robot stops ~52 deg forward)
#   elbow_flex      "direction" (default): the forearm points the way the operator's does - the robot's
#                   upper arm stands UP where a human's hangs DOWN, so equal elbow angles would tilt the
#                   forearm the opposite way whenever the shoulder moves; "angle": robot bend = operator bend
#   wrist_flex      = the operator's wrist bend relative to the forearm; wrist_roll = forearm twist (gripper_q)
# The operator's neutral pose (upper arm hanging, forearm forward, wrist straight) = the robot's ready pose.
# Calibrated each session by clicking that controller's thumbstick (Y opens a menu in the app, user note
# 2026-10-21) with the arm straight forward, horizontal. The facing comes from that arm's direction, not the
# head (2026-10-21: a calibration made while looking at the terminal put the left shoulder 0.28 m right of
# the neck).
NECK = (0.0, -0.10, -0.08)  # m, eyes -> neck pivot in the head frame (anthropometric estimate)
SHOULDER_W = 0.18           # m, shoulder joint beside the neck pivot (a one-arm calibration cannot see it)
UPPER_FRAC = 0.48           # upper arm share of the shoulder-to-controller length
BODY_FILE = os.path.join(HERE, "body_calib.json")
# elbow-map position: operator upper-arm swing (deg forward from hanging) -> robot shoulder_lift (pose deg).
# Curled with the upper arm back = folded back like the rest pose (-104.8); raised 45 deg = upper arm
# vertical; raised fully forward = horizontal, clamped by the joint limit (user requests 2026-10-21 v8-v10).
LIFT_KNOTS = [(-40.0, -98.0), (45.0, 0.0), (90.0, 90.0)]
SIDE_CN = {"left": ("左臂", "左摇杆"), "right": ("右臂", "右摇杆")}

# Headset prompts (user request 2026-10-21: the terminal is hard to read with the headset on): teleop writes
# a few short lines here; video_daemon.py draws them on top of the video.
HUD_FILE = "/dev/shm/fmp_hud.json"
_hud_events = []


def hud_event(msg, keep=4.0):
    _hud_events.append((time.time() + keep, msg))


def hud_write(lines):
    now = time.time()
    _hud_events[:] = [e for e in _hud_events if e[0] > now]
    try:
        with open(HUD_FILE + ".tmp", "w") as fh:
            json.dump({"t": now, "lines": lines + [m for _, m in _hud_events]}, fh, ensure_ascii=False)
        os.replace(HUD_FILE + ".tmp", HUD_FILE)
    except OSError:
        pass


def interp(x, knots):
    """piecewise-linear through (x, y) knots, held flat beyond the ends."""
    if x <= knots[0][0]:
        return knots[0][1]
    for (x0, y0), (x1, y1) in zip(knots, knots[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return knots[-1][1]


def load_body():
    try:
        return json.load(open(BODY_FILE))
    except (OSError, ValueError):
        return {}


def save_body(side, value):
    data = load_body(); data[side] = value
    json.dump(data, open(BODY_FILE, "w"), indent=1)


def neck_point(head):
    p = q_rot(tuple(head["rot"]), NECK)
    return [head["pos"][i] + p[i] for i in range(3)]


# ---------------------------------------------------------------- arm
class ArmTeleop:
    def __init__(self, side, kin, cal, limits, scale, grip_open, pos_mode="joint", elbow_map="position",
                 lift_gain=0.6, lift_blend=0.0):
        self.side, self.kin, self.cal, self.limits = side, kin, cal, limits
        self.scale, self.grip_open, self.pos_mode = scale, grip_open, pos_mode
        self.elbow_map, self.lift_gain, self.lift_blend = elbow_map, lift_gain, lift_blend
        self.engaged = False
        self.latch = None
        self.target = None          # pose-degree joint targets (dict)
        self.gripper = None         # percent
        # wrist joints slower (2026-10-21: "wrist too fast / too sensitive")
        self.followers = {j: hf.SpringFollower(w=20.0, a_max=1500.0, jerk=40000.0,
                                               v_max=120.0 if j.startswith("wrist") else 180.0)
                          for j in ARM_JOINTS + ["gripper"]}
        # controller smoothing (2026-10-20: raw hand jitter went straight to the joints): One Euro on the
        # position (metres; beta per m/s) and on the gripper pitch/roll (degrees)
        self.fpos = [hf.OneEuro(1.5, 4.0) for _ in range(3)]
        self.fang = [hf.OneEuro(1.2, 0.03) for _ in range(2)]
        self.smooth_pos = None
        self.smooth_ang = (0.0, 0.0)
        self.cmd = None             # clutch mode: commanded end-effector point (f, s, h), kept reachable
        self.at_edge = False
        # neutral = controller rotation in the operator's natural grip (handle vertical, wrist relaxed), with
        # the facing removed; that pose maps to the gripper horizontal-forward, wrist camera level
        self.neutral = load_neutral().get(side)
        self.prev_primary = False
        self.body = load_body().get(side)
        self.torso_heading = None   # joint mode: operator's facing, set by the stick-click calibration
        self.blend = None           # joint mode: (start targets, elapsed s), eased in after each grip press
        self.h_pan = 0.0
        self.human = None           # last (azimuth, upper-arm swing, elbow flexion) in degrees
        self.prev_click = False

    def calibrate_body(self, ctrl, head):
        """arm straight forward and horizontal. The neck-to-hand line misses the forward direction by the
        shoulder's sideways offset: heading = its direction + atan(W / L) for the left arm (- for the right),
        arm length L = sqrt(D^2 - W^2); the shoulder is level with the hand."""
        name, key = SIDE_CN[self.side]
        n = neck_point(head)
        v = [ctrl["pos"][i] - n[i] for i in range(3)]
        dist = math.hypot(v[0], v[2])
        arm = math.sqrt(max(0.0, dist ** 2 - SHOULDER_W ** 2))
        if not 0.4 < arm < 0.95:
            print(f"[{self.side}] body calibration rejected: arm {arm:.2f} m (straight forward?)", flush=True)
            hud_event(f"{name}标定失败：手臂水平伸直再按{key}")
            return
        sgn = 1.0 if self.side == "left" else -1.0
        self.torso_heading = math.degrees(math.atan2(v[0], v[2])) + sgn * math.degrees(math.atan2(SHOULDER_W, arm))
        self.body = dict(shoulder=[-sgn * SHOULDER_W, round(v[1], 3), 0.0], arm=round(arm, 3))
        save_body(self.side, self.body)
        print(f"[{self.side}] body calibrated: arm {arm:.2f} m, shoulder {v[1]:+.2f} m above the neck pivot, "
              f"facing {self.torso_heading:+.0f} deg (head {heading_deg(head['rot']):+.0f})", flush=True)
        hud_event(f"{name}已标定  臂长 {arm:.2f} m")

    def human_arm(self, head):
        """(azimuth, upper-arm swing forward from hanging, elbow flexion) of the operator's arm, degrees."""
        b = self.body
        n = neck_point(head)
        x, y, z = q_rot(yaw_q(-self.torso_heading), tuple(self.smooth_pos[i] - n[i] for i in range(3)))
        x, y, z = x - b["shoulder"][0], y - b["shoulder"][1], z - b["shoulder"][2]
        lu = UPPER_FRAC * b["arm"]; lf = b["arm"] - lu
        if z > 0.05 and math.hypot(x, z) > 0.06:  # hand in front: azimuth defined (else keep the last one)
            self.h_pan = math.degrees(math.atan2(x, z))
        pa = math.radians(self.h_pan)
        rho = z * math.cos(pa) + x * math.sin(pa)
        d = max(abs(lu - lf) + 0.01, min(lu + lf - 1e-6, math.hypot(rho, y)))
        alpha = math.atan2(rho, -y)  # shoulder->hand from straight down, + = forward
        beta = math.acos(max(-1.0, min(1.0, (lu * lu + d * d - lf * lf) / (2 * lu * d))))
        phi = math.pi - math.acos(max(-1.0, min(1.0, (lu * lu + lf * lf - d * d) / (2 * lu * lf))))
        return self.h_pan, math.degrees(alpha - beta), math.degrees(phi), rho, y

    def calibrate_neutral(self, ctrl, head_q):
        if self.pos_mode == "joint" and self.torso_heading is not None:
            h = self.torso_heading
        else:
            h = heading_deg(head_q) if head_q else 0.0
        self.neutral = q_mul(yaw_q(-h), tuple(ctrl["rot"]))
        save_neutral(self.side, self.neutral)
        p, r = abs_pitch_roll(ctrl["rot"])
        print(f"[{self.side}] neutral grip set (controller pitch {p:+.1f}, roll {r:+.1f} deg)", flush=True)
        hud_event(f"{SIDE_CN[self.side][0]}中性握姿已记录")

    def hud_line(self, ctrl):
        name, key = SIDE_CN[self.side]
        if not ctrl:
            return f"{name}：没有手柄数据"
        if not ctrl.get("tracked"):
            return f"{name}：手柄未被追踪"
        if self.pos_mode == "joint" and self.torso_heading is None:
            return f"{name}：请把{name}水平向前伸直，按下{key}标定"
        return f"{name}：跟随中" if self.engaged else f"{name}：保持（按住握把移动）"

    def motor(self, j):
        return f"{self.side}_arm_{j}"

    def lr_to_pose(self, j, v):
        c = self.cal[self.motor(j)]
        mid = (c["range_min"] + c["range_max"]) / 2
        return ((v * 4095 / 360 + mid) - 2047) * 360 / 4095

    def pose_to_lr(self, j, p):
        return to_lerobot_deg(self.cal, self.motor(j), p)

    def init_from(self, present_lr):
        self.target = {j: self.lr_to_pose(j, present_lr[f"{self.motor(j)}.pos"]) for j in ARM_JOINTS}
        self.gripper = present_lr[f"{self.motor('gripper')}.pos"]
        for j in ARM_JOINTS:
            self.followers[j].pos = self.target[j]
        self.followers["gripper"].pos = self.gripper

    def ee_from_joints(self, t):
        x, y = fk_consistent(t["shoulder_lift"], t["elbow_flex"])
        pitch = t["shoulder_lift"] + t["elbow_flex"] + t["wrist_flex"]  # gripper pitch, + = down
        return x, y, pitch

    def update(self, ctrl, head, new_sample, sdt):
        """ctrl / head: dicts from the app (or None); new_sample/sdt: whether ctrl is a fresh message and its
        spacing. Returns True if engaged."""
        head = head if head and head.get("pos") and head.get("rot") else None
        head_q = head["rot"] if head else None
        ok = bool(ctrl and ctrl.get("tracked") and ctrl.get("pos") and ctrl.get("rot"))
        click = bool(ok and ctrl.get("stick_click"))
        if click and not self.prev_click and not self.engaged and head:  # stick click: body calibration
            self.calibrate_body(ctrl, head)
        self.prev_click = click
        joint = self.pos_mode == "joint"
        grip = ctrl.get("grip", 0.0) if ok else 0.0
        primary = bool(ok and ctrl.get("primary"))
        if primary and not self.prev_primary and not self.engaged:  # X / A while not moving: set neutral grip
            self.calibrate_neutral(ctrl, head_q)
        self.prev_primary = primary
        ready = not joint or (self.torso_heading is not None and head is not None)  # joint mode: calibrate first
        if ok and ready and not self.engaged and grip > 0.6:
            if self.neutral is None:  # first use: take the pose at the first grip press as the neutral grip
                self.calibrate_neutral(ctrl, head_q)
            x0, y0, _ = self.ee_from_joints(self.target)
            for flt in self.fpos + self.fang:
                flt.reset()
            heading = self.torso_heading if joint else (heading_deg(head_q) if head_q else 0.0)
            self.smooth_pos = list(ctrl["pos"])
            self.smooth_ang = abs_pitch_roll(gripper_q(ctrl["rot"], heading, self.neutral))
            pan0 = math.radians(self.target["shoulder_pan"])
            self.latch = dict(heading=heading)
            self.cmd = [x0 * math.cos(pan0), x0 * math.sin(pan0), y0]
            self.blend = (dict(self.target), 0.0)
            self.engaged = True
        elif self.engaged and (not ok or grip < 0.4):
            self.engaged = False
        if ok:
            self.gripper = self.grip_open * (1.0 - max(0.0, min(1.0, ctrl.get("trigger", 0.0))))
        if not self.engaged:
            return False
        L = self.latch
        if new_sample:
            prev = self.smooth_pos
            self.smooth_pos = [self.fpos[i](ctrl["pos"][i], sdt) for i in range(3)]
            p, r = abs_pitch_roll(gripper_q(ctrl["rot"], L["heading"], self.neutral))
            self.smooth_ang = (self.fang[0](p, sdt), self.fang[1](r, sdt))
            if not joint:
                # clutch: straight-line Cartesian translation of the end effector (user request 2026-10-20) in
                # the facing at the press; incremental, the commanded point kept reachable (below)
                dx, dy, dz = (self.smooth_pos[i] - prev[i] for i in range(3))
                h = math.radians(L["heading"])
                self.cmd[0] += (dz * math.cos(h) + dx * math.sin(h)) * self.scale
                self.cmd[1] += (dx * math.cos(h) - dz * math.sin(h)) * self.scale
                self.cmd[2] += dy * self.scale
        # Orientation (both modes): gripper pitch (from horizontal, + = down) and roll (+ = CCW seen from
        # behind) = those of the controller rotation relative to the neutral grip (gripper_q). No wrist yaw joint.
        g_pitch, g_roll = self.smooth_ang
        if joint:
            if head is None:
                return True  # hold the last targets until the headset pose is back
            pan, swing, flex, rho, up = self.human_arm(head)
            self.human = (pan, swing, flex)
            lo, hi = self.limits["shoulder_lift"]
            forearm_down = 90.0 - swing - flex  # operator's forearm pitch below horizontal
            if self.elbow_map == "position":
                # 2026-10-21 v7: in the robot, both curling the arm and lowering the hand mean folding the
                # forearm DOWN, while a human does them with opposite elbow motions - so the elbow follows the
                # hand's position instead: the target is the hand relative to the shoulder x robot reach / arm
                # length; shoulder_lift = a blend of the upper-arm swing and the lift that reaches the target
                # (the robot wrist only gets below its shoulder axis with the shoulder far forward); the elbow
                # then points the forearm at the target.
                # v8 (user, same day): the upper arm follows the operator's - curled = folded BACK like the rest
                # pose, raising the upper arm turns it forward - so shoulder_lift comes from the swing through
                # LIFT_KNOTS (lift_blend 0 by default; > 0 mixes in the lift that reaches the target).
                k = R_MAX / self.body["arm"]
                tx, ty = max(0.03, rho * k), up * k
                lift_j = interp(swing, LIFT_KNOTS)
                lift_ik = self.kin.inverse_kinematics(tx, ty)[0] if self.lift_blend > 0 else lift_j
                lift = max(lo, min(hi, (1 - self.lift_blend) * lift_j + self.lift_blend * lift_ik))
                t1 = math.radians(90.0 - lift) - _O1
                ex, ey = 0.1159 * math.cos(t1), 0.1159 * math.sin(t1)  # robot elbow (fk_consistent l1)
                t2 = t1 - math.atan2(ty - ey, tx - ex)
                # wrap into the turn centred on the middle of the elbow range, so a target past the fold
                # clamps at the folded end (with the upper arm folded back the raw angle crosses +-180 deg)
                e_lo, e_hi = self.limits["elbow_flex"]
                mid = math.radians((e_lo + e_hi) / 2 + 90.0) - _O2
                t2 = (t2 - mid + math.pi) % (2 * math.pi) - math.pi + mid
                elbow = math.degrees(t2 + _O2) - 90.0
                wrist = g_pitch - lift - max(self.limits["elbow_flex"][0], min(self.limits["elbow_flex"][1], elbow))
            else:
                lift = max(lo, min(hi, self.lift_gain * swing))
                elbow = forearm_down - lift if self.elbow_map == "direction" else flex - 90.0
                wrist = g_pitch - forearm_down
            t = dict(shoulder_pan=pan, shoulder_lift=lift, elbow_flex=elbow, wrist_flex=wrist, wrist_roll=g_roll)
            start, el = self.blend
            el += sdt if new_sample else 0.0
            self.blend = (start, el)
            a = min(1.0, el / 0.8); a = a * a * (3 - 2 * a)  # ease in after the press: no jump
            for j, v in t.items():
                lo, hi = self.limits[j]
                v = max(lo, min(hi, v))
                self.target[j] = start[j] + (v - start[j]) * a
            self.at_edge = False
            return True
        f, s, hgt = self.cmd
        # f is kept >= F_MIN in front of the base so atan2 cannot flip (the first version's jumping base);
        # shoulder-to-wrist distance kept within [R_MIN, R_MAX]; past R_MAX the height is kept and only the
        # horizontal reach shortened (scaling towards the shoulder made over-reaching sink the gripper)
        f = max(F_MIN, f)
        reach = math.hypot(f, s)
        r = math.hypot(reach, hgt)
        if r > R_MAX and abs(hgt) < R_MAX - 0.01:
            k = math.sqrt(R_MAX ** 2 - hgt ** 2) / reach
            f, s = f * k, s * k
        elif r > R_MAX or 0 < r < R_MIN:
            k = (R_MAX if r > R_MAX else R_MIN) / r
            f, s, hgt = f * k, s * k, hgt * k
        pan = math.degrees(math.atan2(s, f))
        lift, elbow = self.kin.inverse_kinematics(math.hypot(f, s), hgt)
        t = dict(shoulder_pan=pan, shoulder_lift=lift, elbow_flex=elbow,
                 wrist_flex=g_pitch - lift - elbow, wrist_roll=g_roll)
        for j, v in t.items():
            lo, hi = self.limits[j]
            self.target[j] = max(lo, min(hi, v))
        # the commanded point = what the clamped joints actually reach, so it never runs off past a limit
        x, y = fk_consistent(self.target["shoulder_lift"], self.target["elbow_flex"])
        pc = math.radians(self.target["shoulder_pan"])
        new_cmd = [x * math.cos(pc), x * math.sin(pc), y]
        self.at_edge = math.dist(new_cmd, self.cmd) > 1e-4
        self.cmd = new_cmd
        return True

    def goals(self, dt):
        """smoothed LeRobot-unit goals for this arm."""
        out = {}
        for j in ARM_JOINTS:
            out[f"{self.motor(j)}.pos"] = self.pose_to_lr(j, self.followers[j].step(self.target[j], 0.0, dt))
        out[f"{self.motor('gripper')}.pos"] = max(0.0, min(100.0, self.followers["gripper"].step(self.gripper, 0.0, dt)))
        return out


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", choices=["left", "right", "both", "none"], default="left")
    ap.add_argument("--scale", type=float, default=1.0, help="clutch mode: hand-to-gripper displacement ratio")
    ap.add_argument("--pos-mode", choices=["joint", "clutch"], default="joint",
                    help="joint: each robot joint follows the operator's corresponding joint (default); "
                         "clutch: hand displacement since the grip press")
    ap.add_argument("--elbow-map", choices=["position", "direction", "angle"], default="position",
                    help="joint mode: elbow points the forearm at the scaled hand position (default), forearm "
                         "points like the operator's, or equal elbow bend")
    ap.add_argument("--lift-gain", type=float, default=0.6,
                    help="joint mode: robot shoulder_lift per degree of the operator's upper-arm swing")
    ap.add_argument("--lift-blend", type=float, default=0.0,
                    help="elbow-map position: shoulder_lift share taken from reaching the hand position (0 = "
                         "upper-arm swing only, 1 = position only)")
    ap.add_argument("--grip-open", type=float, default=60.0, help="gripper %% when the trigger is released")
    ap.add_argument("--arm-p", type=int, default=24, help="arm servo P (LeRobot/XLeRobot default 16)")
    ap.add_argument("--arm-msf", type=int, default=30, help="arm servo Minimum_Startup_Force (factory 16; stiction)")
    ap.add_argument("--hz", type=float, default=50.0)
    args = ap.parse_args(sys_args)
    logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)

    from lerobot_robot_xlerobot import XLerobot, XLerobotConfig
    from lerobot.motors.feetech import OperatingMode
    from xlerobot_model.SO101Robot import SO101Kinematics

    cal = json.load(open(hf.XLR_CAL))
    lim = json.load(open(os.path.join(HERE, "joint_limits.json")))
    arm_limits = dict(lim["pose_deg_limits_both_arms"])
    head_limits = lim["lerobot_deg_limits"]
    robot = XLerobot(XLerobotConfig(id=os.environ["XLR_ID"], port1=os.environ["XLR_PORT1"],
                                    port2=os.environ["XLR_PORT2"], use_degrees=True, max_relative_target=None))
    sides = {"left": ["left"], "right": ["right"], "both": ["left", "right"], "none": []}[args.arms]
    kin = SO101Kinematics()
    arms = {s: ArmTeleop(s, kin, cal, arm_limits, args.scale, args.grip_open, args.pos_mode, args.elbow_map,
                           args.lift_gain, args.lift_blend) for s in sides}

    pose, stop = hf.LatestPose(), threading.Event()
    hf.start_reachy(pose, stop)
    print(f"arms={sides} scale={args.scale} arm P={args.arm_p} MSF={args.arm_msf} | app: Robot IP {os.popen('hostname -I').read().split()[0]}, "
          f"ports 8000/8443, enable ConnectVideo + Connect With Pose Data")

    input("Keep clear of the arm(s). ENTER to turn torque ON where they rest now (no motion yet)...")
    for bus in (robot.bus1, robot.bus2):
        bus.connect()
        bus.calibration = {k: v for k, v in robot.calibration.items() if k in bus.motors}
    head = list(robot.head_motors)
    active = {robot.bus1: list(head), robot.bus2: []}
    for s in sides:
        active[robot.bus1 if s == "left" else robot.bus2] += [f"{s}_arm_{j}" for j in ARM_JOINTS + ["gripper"]]
    for attempt in range(1, 4):
        try:
            for bus, motors in active.items():
                if not motors:
                    continue
                bus.disable_torque(motors, num_retry=2)
                for m in motors:
                    bus.write("Operating_Mode", m, OperatingMode.POSITION.value, num_retry=2)
                    p = 48 if m in head else args.arm_p
                    bus.write("P_Coefficient", m, p, num_retry=2)
                    bus.write("I_Coefficient", m, 0, num_retry=2)
                    bus.write("D_Coefficient", m, 32, num_retry=2)
                    bus.write("Minimum_Startup_Force", m, 40 if m in head else args.arm_msf, num_retry=2)
                bus.sync_write("Goal_Position", bus.sync_read("Present_Position", motors))
                time.sleep(0.05)
                bus.enable_torque(motors, num_retry=2)
            break
        except ConnectionError as exc:
            print(f"setup attempt {attempt} failed: {exc}", flush=True); time.sleep(0.3)
    else:
        sys.exit("setup failed; torque left OFF")

    def read_all():
        out = {}
        for bus, motors in active.items():
            if motors:
                out.update({f"{k}.pos": v for k, v in bus.sync_read("Present_Position", motors).items()})
        return out

    def glide(targets, speed=30.0, t_min=1.5, t_max=5.0):
        """Minimum-jerk joint-space move from the present pose to `targets` (LeRobot units, '<motor>.pos')."""
        start = read_all()
        span = max(abs(targets[k] - start[k]) for k in targets)
        T = max(t_min, min(t_max, span / speed))
        n = max(1, int(T * args.hz))
        for i in range(1, n + 1):
            tk = time.perf_counter()
            s = i / n
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            g = {k: start[k] + (targets[k] - start[k]) * s for k in targets}
            for bus, motors in active.items():
                part = {k[:-4]: v for k, v in g.items() if k[:-4] in motors}
                if part:
                    bus.sync_write("Goal_Position", part)
            time.sleep(max(0.0, 1 / args.hz - (time.perf_counter() - tk)))
        time.sleep(0.4)
        end = read_all()
        return max(abs(end[k] - targets[k]) for k in targets if not k.endswith("gripper.pos"))

    # rest = how the arm(s) and head sit with torque off (folded); we return there before releasing torque
    rest = read_all()
    pan_m, tilt_m = head
    # ready = calibration pose (pose 0 deg: arm upper arm vertical, forearm forward; head straight and level)
    ready = {f"{m}.pos": to_lerobot_deg(cal, m, 0.0) for m in head}
    for s in sides:
        ready.update({f"{s}_arm_{j}.pos": to_lerobot_deg(cal, f"{s}_arm_{j}", 0.0) for j in ARM_JOINTS})
        ready[f"{s}_arm_gripper.pos"] = rest[f"{s}_arm_gripper.pos"]
    input("Torque ON, holding. ENTER to move slowly to the ready pose (arm(s) unfold, head centres, ~3 s)...")
    err = glide(ready)
    print(f"ready pose reached (max error {err:.1f} deg)")
    present = read_all()
    for a in arms.values():
        a.init_from(present)
    centre = {m: ready[f"{m}.pos"] for m in head}
    yaw_lim = min(90.0, min(head_limits[pan_m][1] - centre[pan_m], centre[pan_m] - head_limits[pan_m][0]))
    pitch_lim = min(45.0, min(head_limits[tilt_m][1] - centre[tilt_m], centre[tilt_m] - head_limits[tilt_m][0]))
    input("Torque ON. Put the headset on, face forward, ENTER to start (grip = move arm, trigger = close gripper)...")

    cmd_queue = []
    threading.Thread(target=lambda: [cmd_queue.append(l.strip()) for l in sys.stdin], daemon=True).start()
    status = [None]

    def printer():
        while not stop.is_set():
            time.sleep(1.0)
            if status[0]:
                print(status[0], flush=True)
    threading.Thread(target=printer, daemon=True).start()

    dt = 1 / args.hz
    filters = [hf.OneEuro(2.0, 0.05), hf.OneEuro(2.0, 0.05)]
    hfoll = [hf.SpringFollower(w=25.0, ff=0.6) for _ in (0, 1)]
    zero, filt, tvel, prev_filt, last_hts, since_new, last_count = None, [0.0, 0.0], [0.0, 0.0], None, None, 0.0, 0
    last_ctrl_recv = 0.0
    os.makedirs(os.path.expanduser("~/fmp_logs"), exist_ok=True)
    log_path = os.path.expanduser(f"~/fmp_logs/teleop_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    cols = ["t", "head_state", "cmd_yaw", "cmd_pitch"] + [f"{s}_{k}" for s in sides for k in
            ["engaged", "grip", "trigger"] + [f"tgt_{j}" for j in ARM_JOINTS] + [f"act_{j}" for j in ARM_JOINTS] + ["gripper", "h_pan", "h_swing", "h_flex"]]
    reason, t0 = "stopped", time.perf_counter()
    last_hud = 0.0
    try:
        with open(log_path, "w", newline="") as f:
            w = csv.writer(f); w.writerow(cols)
            while True:
                tick = time.perf_counter()
                while cmd_queue:
                    c = cmd_queue.pop(0)
                    if c == "q":
                        raise KeyboardInterrupt
                    if c == "c":
                        zero = None
                s = pose.get()
                state = "no_data"
                if s is not None:
                    yaw, pitch, hts, recv = s
                    age = tick - recv
                    state = "ok" if age <= 0.3 else ("stale" if age <= 2.0 else "lost")
                    if zero is None:
                        zero = (yaw, pitch); [fl.reset() for fl in filters]
                    raw = ((((yaw - zero[0] + 180) % 360) - 180), pitch - zero[1])
                new = pose.count != last_count; last_count = pose.count
                # ---- head
                if state == "ok":
                    if new:
                        hs = hts / 1000 if hts else tick
                        sdt = hs - last_hts if last_hts else dt
                        sdt = sdt if 0.002 < sdt < 0.5 else dt
                        last_hts = hs
                        filt = [filters[i](raw[i], sdt) for i in (0, 1)]
                        if prev_filt is not None:
                            a = 1 - math.exp(-2 * math.pi * 8.0 * sdt)
                            tvel = [tvel[i] + a * ((filt[i] - prev_filt[i]) / sdt - tvel[i]) for i in (0, 1)]
                        prev_filt = list(filt)
                    since_new = 0.0 if new else since_new + dt
                    ex = min(since_new, 0.06)
                    tgt = [max(-yaw_lim, min(yaw_lim, filt[0] + tvel[0] * ex)),
                           max(-pitch_lim, min(pitch_lim, filt[1] + tvel[1] * ex))]
                    tv = [0.0 if abs(tgt[0]) >= yaw_lim else tvel[0], 0.0 if abs(tgt[1]) >= pitch_lim else tvel[1]]
                else:
                    tgt = [hfoll[0].pos, hfoll[1].pos] if state == "stale" else [0.0, 0.0]
                    tv = [0.0, 0.0]; tvel = [0.0, 0.0]
                hcmd = [hfoll[i].step(tgt[i], tv[i], dt) for i in (0, 1)]
                goals = {f"{pan_m}.pos": centre[pan_m] - hcmd[0], f"{tilt_m}.pos": centre[tilt_m] - hcmd[1]}
                # ---- arms
                ctrls = pose.controllers if (pose.controllers and tick - pose.controllers_recv < 0.3) else None
                c_new = pose.controllers_recv != last_ctrl_recv
                c_dt = pose.controllers_recv - last_ctrl_recv if last_ctrl_recv else dt
                c_dt = c_dt if 0.002 < c_dt < 0.5 else dt
                last_ctrl_recv = pose.controllers_recv
                head_pose = (ctrls or {}).get("head")
                eng = {}
                for sname, arm in arms.items():
                    eng[sname] = arm.update((ctrls or {}).get(sname), head_pose, c_new, c_dt)
                    goals.update(arm.goals(dt))
                # ---- write
                for bus, motors in active.items():
                    g = {k[:-4]: v for k, v in goals.items() if k[:-4] in motors}
                    if g:
                        bus.sync_write("Goal_Position", g)
                present = read_all()
                bad = [k for k, v in goals.items() if k in present and abs(present[k] - v) > 40]
                if bad:  # far off target: blocked, or a bad goal - stop rather than push
                    reason = f"tracking error > 40 deg on {bad}, stopped"
                    hud_event("跟踪误差过大，已停止", keep=30.0)
                    break
                row = [f"{tick - t0:.3f}", state, f"{hcmd[0]:.2f}", f"{hcmd[1]:.2f}"]
                for sname, arm in arms.items():
                    c = (ctrls or {}).get(sname) or {}
                    row += [int(eng[sname]), f"{c.get('grip', 0):.2f}", f"{c.get('trigger', 0):.2f}"]
                    row += [f"{arm.target[j]:.2f}" for j in ARM_JOINTS]
                    row += [f"{arm.lr_to_pose(j, present[f'{arm.motor(j)}.pos']):.2f}" for j in ARM_JOINTS]
                    gk = f"{arm.motor('gripper')}.pos"
                    row += [f"{present[gk]:.1f}"]
                    row += [f"{v:.1f}" for v in arm.human] if arm.human else ["", "", ""]
                w.writerow(row)
                status[0] = (f"t={tick - t0:5.1f}s head {state} yaw {hcmd[0]:+5.1f} pitch {hcmd[1]:+5.1f} | " +
                             " ".join(f"{s}:{('MOVE@edge' if arms[s].at_edge else 'MOVE') if eng[s] else 'hold'}{ctrl_tag((ctrls or {}).get(s))}"
                                      for s in arms) +
                             (" | no controller data" if arms and ctrls is None else ""))
                if tick - last_hud > 0.1:
                    last_hud = tick
                    hud_write([arm.hud_line((ctrls or {}).get(s)) for s, arm in arms.items()] +
                              (["头显没有发送手柄数据"] if arms and ctrls is None else []))
                time.sleep(max(0.0, dt - (time.perf_counter() - tick)))
    except KeyboardInterrupt:
        reason = "stopped by user"
    finally:
        stop.set()
        hud_write(["已停止：手臂正在收回折叠位"])
        print(f"\n{reason}. log: {log_path}")
        try:  # fold back to where everything rested at start, then release (nothing drops)
            print("returning to the rest pose...")
            err = glide({k: rest[k] for k in rest})
            print(f"rest pose reached (max error {err:.1f} deg)")
            if err > 8:
                input(f"rest pose off by {err:.0f} deg - HOLD THE ARM(S), then ENTER to release torque...")
        except BaseException as exc:
            print("could not glide back to rest:", exc)
            try:
                input("HOLD THE ARM(S) (torque will be released), then press ENTER...")
            except BaseException:
                pass
        for bus, motors in active.items():
            if motors:
                try:
                    bus.disable_torque(motors, num_retry=2)
                except Exception as exc:
                    print("torque release error:", exc)
        print("torque off")
        try:
            os.remove(HUD_FILE)
        except OSError:
            pass


if __name__ == "__main__":
    main()
