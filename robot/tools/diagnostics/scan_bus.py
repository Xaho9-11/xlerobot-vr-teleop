"""Read-only scan of a Feetech STS bus: ping IDs, read model/position/voltage/temp/torque state.
Never writes to any register."""
import sys
import scservo_sdk as scs

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 1_000_000
ids = range(0, 21)

ph = scs.PortHandler(port)
pk = scs.PacketHandler(0)  # protocol 0 = STS/SMS series
if not ph.openPort() or not ph.setBaudRate(baud):
    print(f"cannot open {port} @ {baud}")
    sys.exit(1)

print(f"port={port} baud={baud}")
found = 0
for i in ids:
    model, res, err = pk.ping(ph, i)
    if res != scs.COMM_SUCCESS:
        continue
    found += 1
    pos, r1, _ = pk.read2ByteTxRx(ph, i, 56)   # Present_Position
    volt, r2, _ = pk.read1ByteTxRx(ph, i, 62)  # Present_Voltage (0.1 V)
    temp, r3, _ = pk.read1ByteTxRx(ph, i, 63)  # Present_Temperature (C)
    torq, r4, _ = pk.read1ByteTxRx(ph, i, 40)  # Torque_Enable
    print(f"id={i:2d} model={model} pos={pos if r1 == 0 else '?'} "
          f"volt={volt/10 if r2 == 0 else '?'}V temp={temp if r3 == 0 else '?'}C torque={torq if r4 == 0 else '?'}")
print(f"found {found} servo(s)")
ph.closePort()
