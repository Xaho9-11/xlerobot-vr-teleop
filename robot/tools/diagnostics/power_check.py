"""Power-sag check: enable torque on one bus motor by motor (holding the current pose, no motion),
reading bus voltage after each; then hold for HOLD_S seconds sampling min voltage; then torque off.
Any motor that stops answering is reported. Usage: power_check.py PORT [ids comma] [hold_s]"""
import sys, time
import scservo_sdk as scs

port = sys.argv[1]
ids = [int(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2 else list(range(1, 9))
hold_s = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
ph = scs.PortHandler(port); pk = scs.PacketHandler(0)
ph.openPort(); ph.setBaudRate(1_000_000)

def volt(i):
    v, r, _ = pk.read1ByteTxRx(ph, i, 62)
    return v / 10 if r == scs.COMM_SUCCESS else None

def write1(i, addr, val, tries=3):
    for _ in range(tries):
        r, _ = pk.write1ByteTxRx(ph, i, addr, val)
        if r == scs.COMM_SUCCESS:
            return True
        time.sleep(0.01)
    return False

input(f"Hold the arm in a supported middle pose. ENTER to enable torque on {ids} one by one (no motion)...")
base = {i: volt(i) for i in ids}
print("idle voltages:", base)
enabled, lost = [], []
try:
    for i in ids:
        pos, r, _ = pk.read2ByteTxRx(ph, i, 56)
        if r != scs.COMM_SUCCESS:
            lost.append(i); print(f"id {i}: no answer before enable"); continue
        pk.write2ByteTxRx(ph, i, 42, pos)          # goal = present, so torque-on causes no jump
        ok = write1(i, 40, 1)                       # Torque_Enable
        enabled.append(i) if ok else lost.append(i)
        time.sleep(0.15)
        vs = [volt(j) for j in ids]
        print(f"torque on id {i}: {'ok' if ok else 'WRITE FAILED'}  bus volts min {min(v for v in vs if v) :.1f}  "
              f"no-answer ids {[j for j, v in zip(ids, vs) if v is None]}")
    t0, vmin, misses = time.perf_counter(), 99.0, 0
    while time.perf_counter() - t0 < hold_s:
        for i in ids:
            v = volt(i)
            if v is None: misses += 1
            else: vmin = min(vmin, v)
    print(f"hold {hold_s:.0f}s with {len(enabled)} motors torqued: min voltage {vmin:.1f} V, missed reads {misses}")
finally:
    input("HOLD THE ARM (torque will be released), then press ENTER...")
    for i in ids:
        write1(i, 40, 0)
    ph.closePort()
    print("torque off. lost/failed ids:", lost or "none")
