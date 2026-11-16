"""Head servo stiction test (no headset): slow small motions under several servo settings.

2026-10-20 trace analysis: after a head stop the pan servo stayed put for ~180 ms while the command moved
~0.8 deg away, then jumped - the user's "it stops twice". Dead zone is only 1 tick, Minimum_Startup_Force
is 16 and P=32, so small errors give too little torque to overcome gear friction (inferred).
For each setting the head (pan, then tilt) runs: slow ramp +6 deg at 6 deg/s, hold, ramp back, then
0.5 / 1 / 2 deg steps. Measured: tracking RMS, worst "stuck" time (servo not moving while |error| > 0.3 deg),
and hold jitter (servo std while the goal is constant; hunting would show here). Listen for buzzing.
Usage (via head_stiction_test.sh): head_stiction_test.py
"""
import os, statistics as st, sys, time
sys.argv = sys.argv[:1]
import head_follow as hf

HZ = 50
SETTINGS = [("baseline P32 MSF16 I0", 32, 16, 0),
            ("MSF40", 32, 40, 0),
            ("MSF80", 32, 80, 0),
            ("P48 MSF40", 48, 40, 0),
            ("P32 MSF40 I2", 32, 40, 2)]

rig = hf.HeadRig(os.environ["XLR_PORT1"], p_gain=32, max_rel=20.0)


def profile():
    seg = []
    for i in range(HZ):              # ramp +6 deg in 1 s
        seg.append(6.0 * i / HZ)
    seg += [6.0] * (HZ // 2)         # hold 0.5 s
    for i in range(HZ):              # ramp back
        seg.append(6.0 - 6.0 * i / HZ)
    seg += [0.0] * (HZ // 2)
    for s in (0.5, 1.0, 2.0):        # small steps, 0.6 s each, and back
        seg += [s] * int(0.6 * HZ) + [0.0] * int(0.6 * HZ)
    return seg


def apply(p, msf, i_gain):
    for m in rig.joints:
        for reg, val in (("P_Coefficient", p), ("Minimum_Startup_Force", msf), ("I_Coefficient", i_gain)):
            rig.bus.write("Lock", m, 0, num_retry=2)
            rig.bus.write(reg, m, val, num_retry=2)
            rig.bus.write("Lock", m, 1, num_retry=2)


def run_axis(joint, centre, sign):
    other = [m for m in rig.joints if m != joint][0]
    hold_other = rig.get_observation()[f"{other}.pos"]
    goals, pres = [], []
    for g in profile():
        t = time.perf_counter()
        goal = centre + sign * g
        rig.send_action({f"{joint}.pos": goal, f"{other}.pos": hold_other})
        o = rig.get_observation()
        goals.append(goal); pres.append(o[f"{joint}.pos"])
        time.sleep(max(0.0, 1 / HZ - (time.perf_counter() - t)))
    err = [p - g for p, g in zip(pres, goals)]
    stuck, worst = 0, 0
    for k in range(1, len(pres)):
        if abs(pres[k] - pres[k - 1]) < 0.05 and abs(err[k]) > 0.3:
            stuck += 1; worst = max(worst, stuck)
        else:
            stuck = 0
    holds = [pres[k] for k in range(2, len(goals)) if goals[k] == goals[k - 1] == goals[k - 2]]
    return (sum(e * e for e in err) / len(err)) ** 0.5, worst * 1000 / HZ, (st.pstdev(holds) if holds else 0.0)


input("Point the head straight ahead by hand, then ENTER to turn head torque ON...")
if not rig.connect():
    sys.exit("setup failed; torque left OFF")
c = rig.get_observation()
centre = {m: c[f"{m}.pos"] for m in rig.joints}
results = []
try:
    input("Head torque ON. ENTER to start (about 1.5 min; listen for buzzing at each setting)...")
    for name, p, msf, ig in SETTINGS:
        apply(p, msf, ig)
        time.sleep(0.3)
        print(f"\n== {name}", flush=True)
        for joint in rig.joints:
            rms, worst, jit = run_axis(joint, centre[joint], +1)
            results.append((name, joint, rms, worst, jit))
            print(f"   {joint}: tracking rms {rms:.2f} deg | worst stuck {worst:.0f} ms | hold jitter {jit:.3f} deg", flush=True)
finally:
    apply(32, 16, 0)  # restore the defaults found on 2026-10-20
    print("\nservo settings restored to P32 MSF16 I0")
    print("\n== summary (lower is better; hold jitter > ~0.1 deg suggests hunting)")
    for name, joint, rms, worst, jit in results:
        print(f"   {name:22s} {joint}: rms {rms:.2f}  stuck {worst:4.0f} ms  jitter {jit:.3f}")
    input("\nTorque will be released (the head may sag a little). ENTER...")
    rig.disconnect()
    print("head torque off")
