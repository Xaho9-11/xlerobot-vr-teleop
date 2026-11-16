"""Which way is + for each arm joint? (before using XLeRobot's SO101 IK model for teleop)

XLeRobot's SO101Kinematics assumes, in degrees from the calibration pose (upper arm vertical, forearm
horizontal forward): shoulder_lift + = upper arm tips forward/down, elbow_flex + = forearm bends down.
This script holds the whole arm (goal = present, torque on), then moves ONE joint +5 deg (raw +57 ticks,
i.e. + in our pose-degree convention) slowly and back, one confirmed step at a time. Note what you see.
Usage (via arm_direction_check.sh): arm_direction_check.py [left|right]
"""
import os, sys, time
import scservo_sdk as scs

SIDE = sys.argv[1] if len(sys.argv) > 1 else "left"
PORT = os.environ["XLR_PORT1" if SIDE == "left" else "XLR_PORT2"]
JOINTS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5}
ASK = {"shoulder_pan": "turns LEFT or RIGHT (robot's own view)?",
       "shoulder_lift": "upper arm tips FORWARD/DOWN or BACK/UP?",
       "elbow_flex": "forearm bends DOWN or UP?",
       "wrist_flex": "gripper tips DOWN or UP?",
       "wrist_roll": "gripper rolls CLOCKWISE or COUNTER-clockwise (looking from behind the arm, toward the gripper)?"}
TICKS = 57  # 5 deg

ph = scs.PortHandler(PORT); pk = scs.PacketHandler(0)
ph.openPort(); ph.setBaudRate(1_000_000)


def rd(i):
    v, r, _ = pk.read2ByteTxRx(ph, i, 56)
    return v if r == scs.COMM_SUCCESS else None


def wr_goal(i, v):
    pk.write2ByteTxRx(ph, i, 42, int(v))


def torque(i, on):
    for _ in range(3):
        if pk.write1ByteTxRx(ph, i, 40, 1 if on else 0)[0] == scs.COMM_SUCCESS:
            return


ids = list(range(1, 7))
input(f"{SIDE.upper()} arm: support it in a comfortable pose with room to move ~5 deg every way. "
      f"ENTER to hold it (torque ON)...")
for i in ids:
    p = rd(i)
    if p is not None:
        wr_goal(i, p)
    torque(i, True)
try:
    for name, i in JOINTS.items():
        input(f"\nNEXT: {name} (ID {i}) +5 deg. Watch it: {ASK[name]}  ENTER to move...")
        p0 = rd(i)
        for k in range(1, 26):          # 0.5 s ramp out
            wr_goal(i, p0 + TICKS * k / 25); time.sleep(0.02)
        time.sleep(0.8)
        p1 = rd(i)
        for k in range(24, -1, -1):     # and back
            wr_goal(i, p0 + TICKS * k / 25); time.sleep(0.02)
        print(f"  moved raw {p0} -> {p1} ({(p1 - p0) * 360 / 4096:+.1f} deg) and back")
finally:
    input("\nHOLD THE ARM (torque will be released), then ENTER...")
    for i in ids:
        torque(i, False)
    ph.closePort()
    print("arm torque off")
