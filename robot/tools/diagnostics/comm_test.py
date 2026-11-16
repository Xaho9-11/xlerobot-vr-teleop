"""Read-only bus reliability test: repeated per-servo Present_Position reads, counts failures."""
import sys, time, collections
import scservo_sdk as scs
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
rounds = int(sys.argv[2]) if len(sys.argv) > 2 else 1000
ph = scs.PortHandler(port); pk = scs.PacketHandler(0)
ph.openPort(); ph.setBaudRate(1_000_000)
fails = collections.Counter(); kinds = collections.Counter()
t0 = time.perf_counter()
for _ in range(rounds):
    for i in range(1, 7):
        _, r, err = pk.read2ByteTxRx(ph, i, 56)
        if r != scs.COMM_SUCCESS:
            fails[i] += 1; kinds[pk.getTxRxResult(r)] += 1
        elif err:
            fails[i] += 1; kinds[f"servo error flag {err}"] += 1
dt = time.perf_counter() - t0
n = rounds * 6
print(f"{n} reads in {dt:.2f}s ({dt / n * 1000:.2f} ms/read), failures {sum(fails.values())} ({sum(fails.values()) / n * 100:.3f}%)")
print("per id:", dict(sorted(fails.items())) or "none")
print("kinds:", dict(kinds) or "none")
ph.closePort()
