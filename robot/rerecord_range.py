"""Re-record the calibrated range of ONE servo, without redoing the whole calibration (2026-10-21: the left
shoulder_lift range had been recorded conservatively, +54.9 deg forward vs +94.8 on the right arm, which
capped the arm well above horizontal).

Torque is released on that servo only; move the joint by hand through its whole range, then ENTER. The
range only grows (new min/max = the wider of old and recorded). It is written to both places LeRobot checks:
the calibration json (a .bak copy is kept) and the servo's Min/Max_Position_Limit (EEPROM). Homing offset is
unchanged, so pose degrees (raw - 2047) keep their meaning; LeRobot DEGREES (mid of range) shift - re-run
safe_limits.py afterwards. The servo must not be in use by another program.

Usage (via rerecord_range.sh): rerecord_range.py --bus 1 --id 2 --motor left_arm_shoulder_lift
"""
import argparse, json, os, shutil, sys, threading, time
import scservo_sdk as scs

CAL = os.path.expanduser("~/.cache/huggingface/lerobot/calibration/robots/xlerobot/xlerobot_fmp.json")
TORQUE, MIN_LIM, MAX_LIM, LOCK, PRESENT = 40, 9, 11, 55, 56  # STS3215 register addresses


def pose_deg(raw):
    return (raw - 2047) * 360 / 4095


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, choices=[1, 2], required=True)
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--motor", required=True, help="name in the calibration json, e.g. left_arm_shoulder_lift")
    a = ap.parse_args()
    cal = json.load(open(CAL))
    c = cal[a.motor]
    if c["id"] != a.id:
        sys.exit(f"{a.motor} has id {c['id']} in the calibration, not {a.id}")
    ph = scs.PortHandler(os.environ[f"XLR_PORT{a.bus}"]); pk = scs.PacketHandler(0)
    ph.openPort(); ph.setBaudRate(1_000_000)

    def rd2(addr):
        v, r, _ = pk.read2ByteTxRx(ph, a.id, addr)
        return v if r == scs.COMM_SUCCESS else None

    lo_lim, hi_lim = rd2(MIN_LIM), rd2(MAX_LIM)
    print(f"{a.motor} (id {a.id}): calibration {c['range_min']}..{c['range_max']} "
          f"({pose_deg(c['range_min']):+.1f}..{pose_deg(c['range_max']):+.1f} pose deg), "
          f"servo limits {lo_lim}..{hi_lim}")
    input("HOLD THAT JOINT (its torque goes off), then ENTER...")
    pk.write1ByteTxRx(ph, a.id, TORQUE, 0)
    print("Move it slowly through its WHOLE range (both ends), then ENTER.")
    done = threading.Event()
    threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
    lo = hi = None
    while not done.is_set():
        p = rd2(PRESENT)
        if p is not None and p < 4096:
            lo = p if lo is None else min(lo, p); hi = p if hi is None else max(hi, p)
            print(f"\r now {p:4d} ({pose_deg(p):+6.1f})  recorded {lo}..{hi} "
                  f"({pose_deg(lo):+6.1f}..{pose_deg(hi):+6.1f})   ", end="", flush=True)
        time.sleep(0.03)
    print()
    if lo is None:
        sys.exit("no readings")
    new_min, new_max = min(c["range_min"], lo), max(c["range_max"], hi)
    print(f"new range {new_min}..{new_max} ({pose_deg(new_min):+.1f}..{pose_deg(new_max):+.1f} pose deg)")
    if (new_min, new_max) == (c["range_min"], c["range_max"]):
        print("unchanged, nothing written"); return
    if input("write it to the calibration file and the servo? [y/N] ").strip().lower() != "y":
        print("not written"); return
    shutil.copy(CAL, CAL + ".bak")
    c["range_min"], c["range_max"] = new_min, new_max
    json.dump(cal, open(CAL, "w"), indent=4)
    pk.write1ByteTxRx(ph, a.id, LOCK, 0)
    pk.write2ByteTxRx(ph, a.id, MIN_LIM, new_min); pk.write2ByteTxRx(ph, a.id, MAX_LIM, new_max)
    pk.write1ByteTxRx(ph, a.id, LOCK, 1)
    print(f"written. servo limits now {rd2(MIN_LIM)}..{rd2(MAX_LIM)}; calibration backup {CAL}.bak")
    ph.closePort()


if __name__ == "__main__":
    main()
