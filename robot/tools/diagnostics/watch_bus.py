"""Read-only joint identification helper: prints one line whenever a servo moves.
Ctrl-C to stop. Never writes to the bus."""
import sys, time
import scservo_sdk as scs

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
ids = [int(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2 else [1, 2, 3, 4, 5, 6]
STEP = 30  # raw ticks (~2.6 deg) before a new line is printed
ph = scs.PortHandler(port); pk = scs.PacketHandler(0)
ph.openPort(); ph.setBaudRate(1_000_000)

def read(i):
    pos, r, _ = pk.read2ByteTxRx(ph, i, 56)
    return pos if r == 0 else None

start = {i: read(i) for i in ids}
last = dict(start)
print("start: " + "  ".join(f"ID{i}={p}" for i, p in start.items()))
print("Move one joint at a time by hand. Ctrl-C to quit.", flush=True)
try:
    while True:
        for i in ids:
            p = read(i)
            if p is None or last[i] is None:
                continue
            if abs(p - last[i]) >= STEP:
                deg = (p - start[i]) * 360 / 4096
                print(f"{time.strftime('%H:%M:%S')}  ID{i} moved  pos={p:4d}  total={deg:+6.1f} deg", flush=True)
                last[i] = p
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nstopped. final: " + "  ".join(f"ID{i}={read(i)}" for i in ids))
finally:
    ph.closePort()
