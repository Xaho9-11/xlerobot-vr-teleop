"""Low-level ping of one servo ID with the raw SDK result text (diagnose 'no servos found')."""
import sys, time
import scservo_sdk as scs
port, sid = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 1
for baud in (1_000_000, 500_000, 115_200):
    ph = scs.PortHandler(port); pk = scs.PacketHandler(0)
    ok = ph.openPort() and ph.setBaudRate(baud)
    results = []
    for _ in range(3):
        model, res, err = pk.ping(ph, sid)
        results.append(pk.getTxRxResult(res) if res != scs.COMM_SUCCESS else f"OK model {model}")
        time.sleep(0.02)
    ph.closePort()
    print(f"{port} baud {baud}: open={ok} ping id {sid}: {results}")
