"""Emergency helper: release torque on every servo of both XLeRobot buses (with retries), then verify.
Arms will drop; hold them first. Usage: torque_off.py (ports from xlerobot_ports.sh env)."""
import os, time
import scservo_sdk as scs
for port, ids in ((os.environ["XLR_PORT1"], range(1, 9)), (os.environ["XLR_PORT2"], range(1, 10))):
    ph = scs.PortHandler(port); pk = scs.PacketHandler(0); ph.openPort(); ph.setBaudRate(1_000_000)
    state = {}
    for i in ids:
        for _ in range(3):
            if pk.write1ByteTxRx(ph, i, 40, 0)[0] == scs.COMM_SUCCESS:
                break
            time.sleep(0.01)
        v, r, _ = pk.read1ByteTxRx(ph, i, 40)
        state[i] = v if r == scs.COMM_SUCCESS else "?"
    ph.closePort()
    print(os.path.basename(port)[-16:], "torque:", state)
