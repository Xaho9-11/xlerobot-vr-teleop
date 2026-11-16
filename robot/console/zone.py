"""Automatic "arrived above the target" event (2026-11-11; part-a-session-protocol: the researcher sits as the
receiver and should not press keys; the research line asked for this event to be detected from the arm).

Kinematics: forward kinematics of each SO-101 arm from the same URDF and the same joint conversion
(web/twin_map.json: URDF angle = sign * pose_deg + offset_deg, URDF left/right chains swapped) as the digital twin,
so the point the core tests is the point the twin draws. Positions are in the arm's own base frame (z up), so
nothing outside the arm (cart, base, arm-base turns in twin_model.json) can shift them.
Tool point (TCP): between the jaw tips, about 2 cm back from the fixed jaw's tip (Fixed_Jaw.stl tip at y = -0.106).

Target zone: a vertical cylinder above the tray, set once per session by putting the gripper at the tray centre,
just above its surface, and pressing "记录托盘位置" (torque off and moving the arm by hand also works).

Filtering against repeated / flickering events (user 2026-11-11):
  * hysteresis - entering needs the TCP inside the zone shrunk by `margin`, leaving needs it outside the zone
    grown by `margin` (default 2 cm), so jitter at the edge cannot toggle the state;
  * dwell - both have to hold for `dwell` seconds (default 0.3 s); the event time is the FIRST frame of that run
    (the dwell confirms, it does not delay the timestamp);
  * one primary event per episode - the first entry is "到达目标上方" (target_arrive); later entries are logged as
    target_reenter and exits as target_leave, so analyses that use the primary event are not polluted.
The raw joints stay in robot.csv; recompute.py-style offline runs (`python zone.py EPISODE_DIR [--radius ...]`)
give the same events from the recording with any parameters.
"""
import json, math, os, sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.expanduser("~/code/XLeRobot/simulation/xlerobot/xlerobot.urdf")
TWIN_MAP = os.path.join(HERE, "web", "twin_map.json")
ZONE_FILE = os.path.expanduser("~/fmp_data/target_zone.json")
TCP = (-0.005, -0.085, 0.0)  # in the Fixed_Jaw frame (metres)
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DEFAULTS = dict(radius=0.10, below=0.03, above=0.25, margin=0.02, dwell=0.3)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(a, q):
    a = np.asarray(a, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


class Kinematics:
    def __init__(self, urdf=URDF, twin_map=TWIN_MAP):
        root = ET.parse(urdf).getroot()
        joints = {j.get("name"): j for j in root.findall("joint")}
        tmap = json.load(open(twin_map))
        self.arms = {}
        for side in ("left", "right"):
            conv = [tmap[f"{side}_arm_{j}"] for j in ARM_JOINTS]
            chain = []
            for c in conv:
                j = joints[c["joint"]]
                o = j.find("origin")
                xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
                rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
                T = np.eye(4); T[:3, :3] = _rpy(*rpy); T[:3, 3] = xyz
                chain.append(dict(T=T, axis=[float(v) for v in j.find("axis").get("xyz").split()],
                                  sign=c.get("sign", 1), offset=c.get("offset_deg", 0)))
            first, last = joints[conv[0]["joint"]], joints[conv[-1]["joint"]]
            self.arms[side] = dict(chain=chain, root=first.find("parent").get("link"),
                                   tip=last.find("child").get("link"))

    def tcp(self, side, pose_deg):
        """pose_deg: list of the 5 arm joints in pose degrees -> TCP (x, y, z) in the arm base frame, metres."""
        T = np.eye(4)
        for c, d in zip(self.arms[side]["chain"], pose_deg):
            q = math.radians(c["sign"] * d + c["offset"])
            R = np.eye(4); R[:3, :3] = _axis_angle(c["axis"], q)
            T = T @ c["T"] @ R
        return (T @ np.array([*TCP, 1.0]))[:3]

    def tcp_from_joints(self, side, joints):
        """joints: core/recorder joint dict name -> {pos} (pose degrees); None if a reading is missing."""
        vals = [(joints.get(f"{side}_arm_{j}") or {}).get("pos") for j in ARM_JOINTS]
        return None if any(v is None for v in vals) else self.tcp(side, vals)


def load_zone():
    try:
        return json.load(open(ZONE_FILE))
    except (OSError, ValueError):
        return None


def save_zone(z):
    os.makedirs(os.path.dirname(ZONE_FILE), exist_ok=True)
    tmp = ZONE_FILE + ".tmp"
    json.dump(z, open(tmp, "w"), ensure_ascii=False, indent=1)
    os.replace(tmp, ZONE_FILE)


class ZoneTracker:
    """Hysteresis + dwell state machine; update() returns the events confirmed at this sample."""

    def __init__(self, zone):
        self.z = {**DEFAULTS, **zone}
        self.inside, self.cand, self.arrived = False, None, False

    def where(self, p):
        c = self.z["center"]
        return math.hypot(p[0] - c[0], p[1] - c[1]), p[2] - c[2]  # horizontal distance, height above the tray

    def _in(self, p, grow):
        r, h = self.where(p)
        return r <= self.z["radius"] + grow and -self.z["below"] - grow <= h <= self.z["above"] + grow

    def update(self, t, p):
        if p is None:
            return []
        m = self.z["margin"]
        flip = (not self.inside and self._in(p, -m)) or (self.inside and not self._in(p, +m))
        if not flip:
            self.cand = None
            return []
        if self.cand is None:
            self.cand = (t, p)
        if t - self.cand[0] < self.z["dwell"]:
            return []
        t0, p0 = self.cand
        self.cand, self.inside = None, not self.inside
        r, h = self.where(p0)
        extra = dict(t=t0, side=self.z["arm"], dist_cm=round(r * 100, 1), height_cm=round(h * 100, 1))
        if not self.inside:
            return [dict(kind="target_leave", label="离开目标上方", **extra)]
        first, self.arrived = not self.arrived, True
        return [dict(kind="target_arrive" if first else "target_reenter",
                     label="到达目标上方" if first else "再次进入目标上方", **extra)]


def recompute(ep_dir, **override):
    """Offline: the zone events of a recorded episode from robot.csv (zone from meta.json, overridable)."""
    import csv
    meta = json.load(open(os.path.join(ep_dir, "meta.json")))
    zone = {**(meta.get("target_zone") or load_zone() or {}), **override}
    if not zone.get("center"):
        raise SystemExit("no target zone in meta.json or ~/fmp_data/target_zone.json")
    kin, tr, out = Kinematics(), ZoneTracker(zone), []
    for row in csv.DictReader(open(os.path.join(ep_dir, "robot.csv"))):
        js = {f"{zone['arm']}_arm_{j}": {"pos": float(row[f"{zone['arm']}_arm_{j}.pos"])}
              for j in ARM_JOINTS if row.get(f"{zone['arm']}_arm_{j}.pos")}
        out += tr.update(float(row["t"]), kin.tcp_from_joints(zone["arm"], js))
    return zone, out


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="recompute the target-zone events of a recorded episode")
    ap.add_argument("episode_dir")
    for k in DEFAULTS:
        ap.add_argument("--" + k, type=float)
    a = ap.parse_args()
    zone, evs = recompute(a.episode_dir, **{k: v for k, v in vars(a).items() if k in DEFAULTS and v is not None})
    t0 = json.load(open(os.path.join(a.episode_dir, "meta.json")))["t_start"]
    print("zone:", {k: zone[k] for k in ("arm", "radius", "below", "above", "margin", "dwell")})
    for e in evs:
        print(f"{e['t'] - t0:7.2f} s  {e['label']}  ({e['dist_cm']} cm from centre, {e['height_cm']} cm above)")
