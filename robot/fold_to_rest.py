"""Glide whatever servos on a bus are torqued back to the saved folded rest pose, then release torque.
For recovery when a program died with torque still on (e.g. the laptop's battery ran out mid-teleop,
2026-10-20): nothing drops, nobody has to hold the arm. Uses rest_pose.json (pose degrees per servo ID).
Servos not in the file (e.g. the gripper) keep their present position. Usage: fold_to_rest.py [bus1|bus2]"""
import json, os, sys, time
import scservo_sdk as scs

BUS = sys.argv[1] if len(sys.argv) > 1 else "bus1"
PORT = os.environ["XLR_PORT1" if BUS == "bus1" else "XLR_PORT2"]
rest = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "rest_pose.json")))[BUS]
ph = scs.PortHandler(PORT); pk = scs.PacketHandler(0); ph.openPort(); ph.setBaudRate(1_000_000)
ids = [i for i in range(1, 10) if pk.ping(ph, i)[1] == scs.COMM_SUCCESS]
torqued = [i for i in ids if pk.read1ByteTxRx(ph, i, 40)[0] == 1]
start = {i: pk.read2ByteTxRx(ph, i, 56)[0] for i in torqued}
goal = {i: int(round(2047 + rest[str(i)] * 4095 / 360)) if str(i) in rest else start[i] for i in torqued}
span = max((abs(goal[i] - start[i]) * 360 / 4095 for i in torqued), default=0)
T = max(2.0, min(5.0, span / 30.0))
print(f"{BUS}: torqued ids {torqued}; largest move {span:.0f} deg over {T:.1f} s")
n = int(T * 50)
for k in range(1, n + 1):
    s = k / n; s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    for i in torqued:
        pk.write2ByteTxRx(ph, i, 42, int(start[i] + (goal[i] - start[i]) * s))
    time.sleep(0.02)
time.sleep(0.6)
err = {i: round((pk.read2ByteTxRx(ph, i, 56)[0] - goal[i]) * 360 / 4095, 1) for i in torqued}
print("error at rest (deg):", err)
for i in torqued:
    for _ in range(3):
        if pk.write1ByteTxRx(ph, i, 40, 0)[0] == scs.COMM_SUCCESS:
            break
print("torque now:", {i: pk.read1ByteTxRx(ph, i, 40)[0] for i in ids})
ph.closePort()
