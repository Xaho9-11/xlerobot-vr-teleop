"""XLeRobot omni-wheel base check, one confirmed step at a time. Only the three wheels get torque.

Part A (wheels lifted off the ground): each wheel alone at a slow raw speed, to see which wheel is which
and its spin direction; Present_Velocity is read back.
Part B (on the floor): slow body motions using XLeRobot's own kinematics (_body_to_wheel_raw):
forward/back/left/right at 0.05 m/s and rotation at 20 deg/s, 1.5 s each.
Every motion waits for ENTER; wheels are zeroed after each step and on any error / Ctrl-C.
Usage (via base_test.sh): base_test.py [A|B|AB|ROT]   (ROT = only the two rotations)
"""
import os, sys, time
import scservo_sdk as scs
from lerobot_robot_xlerobot import XLerobot, XLerobotConfig

PART = sys.argv[1].upper() if len(sys.argv) > 1 else "AB"
WHEELS = {"base_left_wheel": 7, "base_back_wheel": 8, "base_right_wheel": 9}
ADDR_MODE, ADDR_TORQUE, ADDR_GOAL_VEL, ADDR_PRES_VEL = 33, 40, 46, 58
DUR, RAW_SINGLE = 1.5, 300  # seconds per motion; raw speed for single-wheel test (~26 deg/s wheel)

ph = scs.PortHandler(os.environ["XLR_PORT2"]); pk = scs.PacketHandler(0)
ph.openPort(); ph.setBaudRate(1_000_000)
kin = XLerobot(XLerobotConfig(id=os.environ["XLR_ID"], port1=os.environ["XLR_PORT1"], port2=os.environ["XLR_PORT2"]))


def enc(v):   # STS sign-magnitude, sign bit 15
    return (abs(int(v)) & 0x7FFF) | (0x8000 if v < 0 else 0)


def dec(v):
    return -(v & 0x7FFF) if v & 0x8000 else v


def w1(i, a, v):
    for _ in range(3):
        if pk.write1ByteTxRx(ph, i, a, v)[0] == scs.COMM_SUCCESS:
            return True
        time.sleep(0.01)
    return False


def set_vel(raw_by_name):
    for name, i in WHEELS.items():
        for _ in range(3):
            if pk.write2ByteTxRx(ph, i, ADDR_GOAL_VEL, enc(raw_by_name.get(name, 0)))[0] == scs.COMM_SUCCESS:
                break


def read_vel():
    out = {}
    for name, i in WHEELS.items():
        v, r, _ = pk.read2ByteTxRx(ph, i, ADDR_PRES_VEL)
        out[name] = dec(v) if r == scs.COMM_SUCCESS else None
    return out


def stop_all(torque_off=False):
    set_vel({})
    if torque_off:
        for i in WHEELS.values():
            w1(i, ADDR_TORQUE, 0)


def run(raw_by_name, label):
    input(f"\nNEXT: {label}. ENTER to run {DUR}s (Ctrl-C to abort)...")
    set_vel(raw_by_name)
    t0, peak = time.perf_counter(), {}
    while time.perf_counter() - t0 < DUR:
        for k, v in read_vel().items():
            if v is not None and abs(v) > abs(peak.get(k, 0)):
                peak[k] = v
        time.sleep(0.05)
    stop_all()
    print(f"  commanded raw {dict((k, raw_by_name.get(k, 0)) for k in WHEELS)}")
    print(f"  measured peak raw {peak}")


try:
    for i in WHEELS.values():          # zero goal first, then velocity mode, then torque
        pk.write2ByteTxRx(ph, i, ADDR_GOAL_VEL, 0)
        w1(i, ADDR_TORQUE, 0); w1(i, ADDR_MODE, 1); w1(i, ADDR_TORQUE, 1)
    print("wheel torque ON, speed 0. Arms stay limp.")
    if "A" in PART:
        input("PART A: lift the base so all three wheels are OFF the ground. ENTER when lifted...")
        for name in WHEELS:
            run({name: RAW_SINGLE}, f"only {name} (ID {WHEELS[name]}) forward-spin raw +{RAW_SINGLE}")
    if "B" in PART or PART in ("R", "ROT"):
        input("\nPART B: put the robot on the floor with ~0.5 m free space around. ENTER when ready...")
        moves = [((0.05, 0, 0), "x +0.05 m/s (expected: forward)"),
                 ((-0.05, 0, 0), "x -0.05 m/s (expected: backward)"),
                 ((0, 0.05, 0), "y +0.05 m/s (expected: left)"),
                 ((0, -0.05, 0), "y -0.05 m/s (expected: right)"),
                 ((0, 0, 20), "theta +20 deg/s (expected: turn left / CCW from above)"),
                 ((0, 0, -20), "theta -20 deg/s (expected: turn right / CW from above)")]
        for (x, y, th), label in (moves[4:] if PART in ("R", "ROT") else moves):
            run(kin._body_to_wheel_raw(x, y, th), label)
finally:
    stop_all(torque_off=True)
    ph.closePort()
    print("\nwheels stopped, torque off")
