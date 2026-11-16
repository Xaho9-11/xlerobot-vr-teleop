"""Print frame-by-frame traces (20 ms) around large head stops in a head_follow log, to see how the
command and the servo come to rest. Usage: stop_trace.py [log.csv] [axis] [how_many]"""
import csv, glob, sys
path = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(__import__("os").path.expanduser("~/fmp_logs/head_follow_reachy_*.csv")))[-1]
ax = sys.argv[2] if len(sys.argv) > 2 else "yaw"
k = int(sys.argv[3]) if len(sys.argv) > 3 else 3
rows = [r for r in csv.DictReader(open(path)) if r["state"] == "ok"]
DT = 0.02
filt = [float(r[f"filt_{ax}"]) for r in rows]; cmd = [float(r[f"cmd_{ax}"]) for r in rows]
act = [float(r[f"present_{ax}"]) for r in rows]; new = [r["new_sample"] for r in rows]
sm = [sum(filt[max(0, i - 9):i + 1]) / len(filt[max(0, i - 9):i + 1]) for i in range(len(filt))]
sv = [(b - a) / DT for a, b in zip(sm, sm[1:])]
ev, moving, peak = [], False, 0.0
for i in range(20, len(sv) - 50):
    if abs(sv[i]) > 40:
        moving, peak = True, max(peak, abs(sv[i]))
    elif moving and abs(sv[i]) < 5:
        moving = False
        if peak > 80:
            ev.append(i)
        peak = 0.0
print(path, ax, f"{len(ev)} large stops")
for i in ev[2:2 + k]:
    t = float(rows[i]["t"])
    print(f"\n--- stop at t={t:.2f}s   ms  new  filt_target  command  servo  servo_deg/s")
    for j in range(i - 15, i + 30):
        v = (act[j] - act[j - 1]) / DT
        print(f"{(j - i) * 20:5d}   {new[j]}   {filt[j]:8.2f} {cmd[j]:8.2f} {act[j]:7.2f} {v:8.1f}")
