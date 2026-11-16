"""Where the IKEA RASKOG cart goes in the MuJoCo (three-omniwheel) model, 2026-11-10. The MJCF has the omni base,
arms and head but only a translucent box for the cart; the older URDF has the cart mesh. A part present in both
with the same mesh (arm Rotation_Pitch, head pan link) ties the two frames together:
  T_cart_in_mjcf_chassis = T_mjcf(chassis -> part) . inv(T_urdf(chassis -> part)) . T_urdf(chassis -> cart visual)
Computed for both arms and the head; their agreement shows how well the two models match.
Prints the cart transform (position, quaternion xyzw, scale) for web/twin_model.json."""
import os, xml.etree.ElementTree as ET
import numpy as np

SIM = os.path.expanduser("~/code/XLeRobot/simulation")
U = ET.parse(f"{SIM}/xlerobot/xlerobot.urdf").getroot()
M = ET.parse(f"{SIM}/mujoco/xlerobot.xml").getroot()
f = lambda s, d="0 0 0": np.array([float(x) for x in (s or d).split()])


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z); w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def euler_xyz(a, b, c):  # MuJoCo default eulerseq xyz (intrinsic) = Rx Ry Rz
    rx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    ry = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    rz = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return rx @ ry @ rz


def T(R=np.eye(3), p=(0, 0, 0), s=(1, 1, 1)):
    m = np.eye(4); m[:3, :3] = R @ np.diag(s); m[:3, 3] = p; return m


def origin(el):
    o = el.find("origin")
    return T(rpy(*f(o.get("rpy") if o is not None else None)), f(o.get("xyz") if o is not None else None))


def urdf_to(link):
    """chassis -> link frame at joint zero (fixed/revolute origins)."""
    joints = {j.find("child").get("link"): j for j in U.findall("joint")}
    chain = []
    while link != "chassis":
        j = joints[link]; chain.append(origin(j)); link = j.find("parent").get("link")
    m = np.eye(4)
    for t in reversed(chain):
        m = m @ t
    return m


def urdf_visual(link, mesh_part):
    for l in U.findall("link"):
        if l.get("name") == link:
            for v in l.findall("visual"):
                me = v.find("geometry/mesh")
                if me is not None and mesh_part in me.get("filename"):
                    return origin(v), f(me.get("scale"), "1 1 1")
    raise KeyError(link)


def mjcf_body(name):
    """chassis -> MJCF body (by name)."""
    def walk(el, m):
        for b in el.findall("body"):
            R = quat(*f(b.get("quat"))) if b.get("quat") else euler_xyz(*f(b.get("euler"))) if b.get("euler") else np.eye(3)
            mb = m @ T(R, f(b.get("pos")))
            if b.get("name") == name:
                return mb
            r = walk(b, mb)
            if r is not None:
                return r
    chassis = M.find("worldbody/body")
    return walk(chassis, np.eye(4))


cart_link = next(l.get("name") for l in U.findall("link")
                 if any("raskogbody" in (m.get("filename") or "") for m in l.iter("mesh")))
cart_vis, cart_scale = urdf_visual(cart_link, "raskogbody")
T_cart_urdf = urdf_to(cart_link) @ cart_vis  # mesh scale is applied separately (it is not a rigid transform)
results = []
for part_urdf, part_mjcf, mesh in [("Rotation_Pitch", "Rotation_Pitch", "Rotation_Pitch"),
                                   ("Rotation_Pitch_2", "Rotation_Pitch_2", "Rotation_Pitch"),
                                   ("head_pan_link", "head_pan_link", "")]:
    try:
        tu = urdf_to(part_urdf)
        tm = mjcf_body(part_mjcf)
        if tm is None:
            print(part_mjcf, "not in MJCF"); continue
        X = tm @ np.linalg.inv(tu) @ T_cart_urdf
        results.append((part_urdf, X))
        print(f"{part_urdf:18s} cart pos {X[:3, 3].round(4)}  rot\n{X[:3, :3].round(3)}")
    except KeyError as e:
        print("missing", e)
print("cart mesh scale", cart_scale, "cart link", cart_link)
if len(results) > 1:
    ps = np.array([r[1][:3, 3] for r in results]); print("position spread (m):", (ps.max(0) - ps.min(0)).round(4))


# ---- second method (2026-11-10): the link frames differ between the two exports, so match JOINT AXIS POSITIONS
# (same physical points in both models) instead: left/right shoulder pan axes and the head pan axis.
def urdf_joint_pos(jname):
    j = next(x for x in U.findall("joint") if x.get("name") == jname)
    return (urdf_to(j.find("parent").get("link")) @ origin(j))[:3, 3]


def mjcf_joint_pos(jname):
    def walk(el, m):
        for b in el.findall("body"):
            R = quat(*f(b.get("quat"))) if b.get("quat") else euler_xyz(*f(b.get("euler"))) if b.get("euler") else np.eye(3)
            mb = m @ T(R, f(b.get("pos")))
            j = b.find("joint")
            if j is not None and j.get("name") == jname:
                return (mb @ T(p=f(j.get("pos"))))[:3, 3]
            r = walk(b, mb)
            if r is not None:
                return r
    return walk(M.find("worldbody/body"), np.eye(4))


names = ["Rotation_L", "Rotation_R", "head_pan_joint"]
pu = np.array([urdf_joint_pos(n) for n in names]); pm = np.array([mjcf_joint_pos(n) for n in names])
print("\nURDF joint positions (chassis):\n", pu.round(4), "\nMJCF joint positions (chassis):\n", pm.round(4))
best = None
for yaw in (0, 90, 180, 270):
    Rz = rpy(0, 0, np.radians(yaw))
    t = (pm - pu @ Rz.T).mean(0)
    res = np.abs(pm - (pu @ Rz.T + t)).max()
    print(f"yaw {yaw:3d}: offset {t.round(4)}  max residual {res:.4f} m")
    if best is None or res < best[0]:
        best = (res, yaw, Rz, t)
res, yaw, Rz, t = best
X = T(Rz, t) @ T_cart_urdf
from scipy.spatial.transform import Rotation as Rot  # noqa: E402 (only for printing a quaternion)
print(f"\nBEST yaw {yaw}, residual {res:.4f} m\ncart in MJCF chassis: pos {X[:3, 3].round(5).tolist()} "
      f"quat_xyzw {Rot.from_matrix(X[:3, :3]).as_quat().round(6).tolist()} scale {cart_scale.tolist()}")
