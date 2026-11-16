"""Robot core (console-architecture.md, 2026-11-10): the ONLY process that owns both servo buses.

A 50 Hz control loop runs a mode machine
    idle (torque off, read only) -> hold (torque on in place) -> ready (glide to the calibration pose)
    -> teleop (head follow + arm teleop, the teleop.py pipelines) -> fold (glide to the rest pose, torque off) -> idle
with estop (all torque off at once) from any mode. Glides run inside the loop, so estop works mid-move.
All 14 arm/head joints are read every tick whatever the mode (state for the console and the digital twin);
temperature and voltage are polled one servo per tick. The base wheels are never touched.
Safety kept from teleop.py: two-arm joint limits, tracking error > 40 deg -> fold, repeated bus errors -> estop,
headset prompts (HUD file for video_daemon).

Interfaces: Quest pose server :8000 (head_follow, always up); console channel ws://127.0.0.1:8770 (state out at
~30 Hz, {"type":"cmd","cmd":...,"arg":...} in). Usage (via core.sh): core.py [--hz 50]
"""
import argparse, asyncio, csv, json, math, os, signal, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # ~/fmp_scripts: teleop.py, head_follow.py, safe_limits.py
_args = sys.argv[1:]
sys.argv = sys.argv[:1]
import teleop as tp  # noqa: E402  (also imports head_follow as tp.hf)
from safe_limits import to_lerobot_deg  # noqa: E402
from recorder import Recorder, DATA  # noqa: E402
import zone as zn  # noqa: E402  (target-zone events, 2026-11-11)

hf = tp.hf
ARM_JOINTS = tp.ARM_JOINTS
SIDES = ("left", "right")
HEAD = ["head_motor_1", "head_motor_2"]
JOINTS = HEAD + [f"{s}_arm_{j}" for s in SIDES for j in ARM_JOINTS + ["gripper"]]
LOG_DIR = os.path.expanduser("~/fmp_logs")
HOLD_B = 0.8  # seconds the right controller's B must be held to start / stop a recording
# fixed fold (rest) pose, pose degrees per arm/head joint (2026-11-13: the rest pose used to be read at torque-on,
# so after the arm had been moved by hand "stop" folded back to that hand-set pose). Set with the fold_set command.
FOLD_FILE = os.path.expanduser("~/fmp_data/fold_pose.json")


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


class Core:
    def __init__(self, args):
        self.args = args
        self.dt = 1.0 / args.hz
        self.mode, self.motion = "idle", None   # motion: dict(kind, start, target, t0, T, then)
        self.arms_sel = "left"                  # chosen in idle; applies from the next torque-on
        self.events = []
        self.cmds = []
        self.lock = threading.Lock()
        self.state = {}
        self.links = dict(bus1=False, bus2=False)
        self.bus_err = 0
        self.telemetry = {}                     # motor -> {temp, volt}
        self.poll_i = 0
        self.pose = hf.LatestPose()
        self.stop = threading.Event()
        self.cal = json.load(open(hf.XLR_CAL))
        lim = json.load(open(os.path.join(os.path.dirname(HERE), "joint_limits.json")))
        self.arm_limits = dict(lim["pose_deg_limits_both_arms"])
        self.head_limits = lim["lerobot_deg_limits"]
        self.robot = None
        self.torqued = []                       # motors with torque on
        self.rest = None                        # LeRobot goals of the rest pose (fold pose file, else torque-on pose)
        try:
            self.fold = {k: v for k, v in json.load(open(FOLD_FILE)).items() if not k.startswith("_")}
        except (OSError, ValueError):
            self.fold = None
        self.goals = {}                         # current LeRobot goals of torqued motors
        self.teleop = None                      # per-session teleop state
        self.rec = Recorder(JOINTS)             # episode recording (recorder.py; cameras via video_daemon :8771)
        self.prev_b, self.last_vr = False, 0.0
        self.b_down, self.b_fired = 0.0, True   # B long press (poll_headset)
        self.replay = None                      # robot replay of a recorded episode (commanded joints)
        try:                                    # tool point of each arm (zone.py; same FK as the digital twin)
            self.kin = zn.Kinematics()
        except Exception as exc:  # noqa: BLE001 - no URDF: everything else still works
            self.kin = None; log("kinematics disabled:", exc)
        self.zone = zn.load_zone()              # target zone above the tray (set from the console)
        self.zone_live = zn.ZoneTracker(self.zone) if self.zone else None  # display only, never writes events

    # ------------------------------------------------------------------ helpers
    def event(self, msg):
        log("event:", msg)
        with self.lock:
            self.events.append([round(time.time(), 1), msg]); self.events = self.events[-50:]
        try:
            with open(os.path.join(LOG_DIR, "core_events.log"), "a") as f:
                f.write(time.strftime("%Y-%m-%d %H:%M:%S ") + msg + "\n")
        except OSError:
            pass

    def bus_of(self, m):
        return self.robot.bus1 if (m.startswith("left") or m.startswith("head")) else self.robot.bus2

    def lr_to_pose(self, m, v):
        if m.endswith("gripper"):
            return v  # gripper stays in percent
        c = self.cal[m]
        mid = (c["range_min"] + c["range_max"]) / 2
        return ((v * 4095 / 360 + mid) - 2047) * 360 / 4095

    def write_goals(self, goals):
        for bus in (self.robot.bus1, self.robot.bus2):
            part = {k: v for k, v in goals.items() if k in bus.motors and k in self.torqued}
            if part:
                bus.sync_write("Goal_Position", part)
        self.goals.update(goals)

    def read_positions(self):
        out = {}
        for bus, motors in ((self.robot.bus1, [m for m in JOINTS if m in self.robot.bus1.motors]),
                            (self.robot.bus2, [m for m in JOINTS if m in self.robot.bus2.motors])):
            out.update(bus.sync_read("Present_Position", motors))
        return out

    def poll_telemetry(self):
        m = JOINTS[self.poll_i % len(JOINTS)]; self.poll_i += 1
        bus = self.bus_of(m)
        try:
            t = bus.read("Present_Temperature", m, normalize=False)
            v = bus.read("Present_Voltage", m, normalize=False)
            self.telemetry[m] = dict(temp=int(t), volt=round(int(v) / 10, 1))
        except Exception:
            self.telemetry.setdefault(m, {})["err"] = self.telemetry.get(m, {}).get("err", 0) + 1

    # ------------------------------------------------------------------ setup
    def connect(self):
        from lerobot_robot_xlerobot import XLerobot, XLerobotConfig
        self.robot = XLerobot(XLerobotConfig(id=os.environ["XLR_ID"], port1=os.environ["XLR_PORT1"],
                                             port2=os.environ["XLR_PORT2"], use_degrees=True, max_relative_target=None))
        for name, bus in (("bus1", self.robot.bus1), ("bus2", self.robot.bus2)):
            bus.connect()
            bus.calibration = {k: v for k, v in self.robot.calibration.items() if k in bus.motors}
            self.links[name] = True
        # everything starts released: the core never moves anything until asked
        for bus in (self.robot.bus1, self.robot.bus2):
            bus.disable_torque([m for m in JOINTS if m in bus.motors], num_retry=2)
        self.event("机器人核心已连接两条总线，全部扭矩关闭")

    def active_motors(self):
        sides = {"left": ["left"], "right": ["right"], "both": ["left", "right"]}[self.arms_sel]
        return list(HEAD) + [f"{s}_arm_{j}" for s in sides for j in ARM_JOINTS + ["gripper"]]

    def torque_on(self):
        from lerobot.motors.feetech import OperatingMode
        motors = self.active_motors()
        for attempt in range(1, 4):
            try:
                for bus in (self.robot.bus1, self.robot.bus2):
                    ms = [m for m in motors if m in bus.motors]
                    if not ms:
                        continue
                    bus.disable_torque(ms, num_retry=2)
                    for m in ms:
                        bus.write("Operating_Mode", m, OperatingMode.POSITION.value, num_retry=2)
                        bus.write("P_Coefficient", m, 48 if m in HEAD else self.args.arm_p, num_retry=2)
                        bus.write("I_Coefficient", m, 0, num_retry=2)
                        bus.write("D_Coefficient", m, 32, num_retry=2)
                        bus.write("Minimum_Startup_Force", m, 40 if m in HEAD else self.args.arm_msf, num_retry=2)
                    present = bus.sync_read("Present_Position", ms)
                    bus.sync_write("Goal_Position", present)
                    time.sleep(0.05)
                    bus.enable_torque(ms, num_retry=2)
                    self.goals.update(present)
                break
            except ConnectionError as exc:
                log("torque-on attempt", attempt, "failed:", exc); time.sleep(0.3)
        else:
            self.release_all("上扭矩失败，已保持扭矩关闭")
            return False
        self.torqued = motors
        # fold target: the saved fold pose (grippers keep their torque-on opening); joints without one keep the
        # pose they had at torque-on
        fold = self.fold or {}
        self.rest = {m: to_lerobot_deg(self.cal, m, fold[m]) if m in fold and not m.endswith("gripper") else self.goals[m]
                     for m in motors}
        return True

    def release_all(self, why):
        for bus in (self.robot.bus1, self.robot.bus2):
            try:
                bus.disable_torque([m for m in JOINTS if m in bus.motors], num_retry=2)
            except Exception as exc:
                log("torque release error:", exc)
        self.torqued, self.goals, self.motion, self.teleop = [], {}, None, None
        self.mode = "idle"
        self.event(why)

    # ------------------------------------------------------------------ motions
    def start_glide(self, kind, target, then, speed=30.0, t_min=1.5, t_max=5.0):
        start = {m: self.goals[m] for m in target}
        span = max(abs(target[m] - start[m]) for m in target)
        self.motion = dict(kind=kind, start=start, target=target, t0=time.perf_counter(),
                           T=max(t_min, min(t_max, span / speed)), then=then)

    def step_motion(self, now):
        mo = self.motion
        s = min(1.0, (now - mo["t0"]) / mo["T"])
        s5 = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5  # minimum jerk
        self.write_goals({m: mo["start"][m] + (mo["target"][m] - mo["start"][m]) * s5 for m in mo["target"]})
        if s >= 1.0 and now - mo["t0"] > mo["T"] + 0.4:  # settle
            then, self.motion = mo["then"], None
            then()

    def ready_pose(self):
        tgt = {m: to_lerobot_deg(self.cal, m, 0.0) for m in self.torqued if not m.endswith("gripper")}
        tgt.update({m: self.goals[m] for m in self.torqued if m.endswith("gripper")})
        return tgt

    # ------------------------------------------------------------------ commands
    def command(self, cmd, arg=None):
        m = self.mode
        if cmd == "estop":
            self.release_all("紧急释放：全部扭矩已关闭"); return
        if cmd.startswith("rec_"):  # recording works in any mode, also during a glide
            self.rec_command(cmd, arg); return
        if cmd.startswith("zone_"):  # target zone: any mode (with torque off the arm can be moved by hand)
            self.zone_command(cmd, arg); return
        if cmd == "fold_set":  # the arms were put in the fold pose by hand (torque off) or held there
            if m not in ("idle", "hold"):
                self.event("只能在空闲或保持状态下记录折叠位"); return
            js = (self.state.get("joints") or {})
            pose = {j: round(v["pos"], 2) for j, v in js.items() if not j.endswith("gripper") and v.get("pos") is not None}
            if len(pose) < len(JOINTS) - 2:
                self.event("记录折叠位失败：有关节读不到"); return
            tmp = FOLD_FILE + ".tmp"
            json.dump(dict(_note="fold (rest) pose, pose degrees; set from the console", _set_at=time.strftime("%Y-%m-%d %H:%M:%S"),
                           **pose), open(tmp, "w"), ensure_ascii=False, indent=1)
            os.replace(tmp, FOLD_FILE)
            self.fold = pose
            if self.torqued:  # holding now: this pose is also the target of the next stop
                self.rest.update({j: to_lerobot_deg(self.cal, j, pose[j]) for j in self.torqued if j in pose})
            self.event("折叠位已记录（以后“停止并收回”都回到这个姿态）"); return
        if cmd == "arms":
            if m != "idle":
                self.event("只能在空闲时切换手臂"); return
            if arg in ("left", "right", "both"):
                self.arms_sel = arg; self.event(f"手臂：{ {'left': '左臂', 'right': '右臂', 'both': '双臂'}[arg]}")
            return
        if self.motion and cmd not in ("stop", "replay_stop"):
            self.event("正在移动，请稍候或急停"); return
        if cmd == "torque_on" and m == "idle":
            if self.torque_on():
                self.mode = "hold"; self.event("已上扭矩，保持当前位置")
        elif cmd == "unfold" and m == "hold":
            self.event("展开到就绪姿态")
            self.start_glide("unfold", self.ready_pose(), self.enter_ready)
        elif cmd == "start" and m == "ready":
            self.enter_teleop()
        elif cmd == "replay" and m == "ready":
            self.start_replay(str(arg or ""))
        elif cmd == "replay_stop" and m == "replay":
            self.replay, self.motion = None, None; self.mode = "ready"; self.event("回放已停止，原地保持")
        elif cmd == "stop" and m in ("hold", "ready", "teleop", "replay"):
            self.replay = None
            self.end_teleop()
            self.mode = "fold"; self.event("停止，收回折叠位")
            self.start_glide("fold", dict(self.rest), self.after_fold)
        elif cmd == "recenter" and m == "teleop" and self.teleop:
            self.teleop["zero"] = None; self.event("头部已重新居中")
        else:
            self.event(f"当前模式下不能执行：{cmd}")

    def enter_ready(self):
        self.mode = "ready"; self.event("已到就绪姿态")

    def after_fold(self):
        present = self.read_positions()
        err = max((abs(present[m] - self.rest[m]) for m in self.rest if not m.endswith("gripper")), default=0)
        self.release_all(f"已收回折叠位（误差 {err:.1f}°），扭矩已释放")

    # ------------------------------------------------------------------ teleop (teleop.py pipelines)
    def enter_teleop(self):
        a = self.args
        kin = __import__("xlerobot_model.SO101Robot", fromlist=["SO101Kinematics"]).SO101Kinematics()
        sides = [s for s in SIDES if any(m.startswith(s) for m in self.torqued)]
        arms = {s: tp.ArmTeleop(s, kin, self.cal, self.arm_limits, 1.0, a.grip_open, "joint", "position",
                                0.6, 0.0) for s in sides}
        present = {f"{k}.pos": v for k, v in self.read_positions().items()}  # ArmTeleop uses '<motor>.pos' keys
        for arm in arms.values():
            arm.init_from(present)
        pan_m, tilt_m = HEAD
        centre = {m: to_lerobot_deg(self.cal, m, 0.0) for m in HEAD}
        hl = self.head_limits
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"teleop_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(["t", "head_state", "cmd_yaw", "cmd_pitch"] + [f"{s}_{k}" for s in sides for k in
                   ["engaged", "grip", "trigger"] + [f"tgt_{j}" for j in ARM_JOINTS] + [f"act_{j}" for j in ARM_JOINTS]
                   + ["gripper", "h_pan", "h_swing", "h_flex"]])
        self.teleop = dict(
            arms=arms, centre=centre, file=f, writer=w, path=path, t0=time.perf_counter(),
            yaw_lim=min(90.0, hl[pan_m][1] - centre[pan_m], centre[pan_m] - hl[pan_m][0]),
            pitch_lim=min(45.0, hl[tilt_m][1] - centre[tilt_m], centre[tilt_m] - hl[tilt_m][0]),
            filters=[hf.OneEuro(2.0, 0.05), hf.OneEuro(2.0, 0.05)],
            hfoll=[hf.SpringFollower(w=25.0, ff=0.6) for _ in (0, 1)],
            zero=None, filt=[0.0, 0.0], tvel=[0.0, 0.0], prev_filt=None, last_hts=None, since_new=0.0,
            last_count=self.pose.count, last_ctrl_recv=0.0, eng={})
        self.mode = "teleop"
        self.event("开始遥操作（握把移动手臂，扳机合夹爪）")

    def end_teleop(self):
        t = self.teleop
        if t:
            t["file"].close(); self.event(f"遥操作日志：{os.path.basename(t['path'])}")
        self.teleop = None

    def step_teleop(self, tick, present):
        T, dt = self.teleop, self.dt
        pan_m, tilt_m = HEAD
        s = self.pose.get()
        state = "no_data"
        if s is not None:
            yaw, pitch, hts, recv = s
            age = tick - recv
            state = "ok" if age <= 0.3 else ("stale" if age <= 2.0 else "lost")
            if T["zero"] is None:
                T["zero"] = (yaw, pitch); [fl.reset() for fl in T["filters"]]
            raw = ((((yaw - T["zero"][0] + 180) % 360) - 180), pitch - T["zero"][1])
        new = self.pose.count != T["last_count"]; T["last_count"] = self.pose.count
        if state == "ok":
            if new:
                hs = hts / 1000 if hts else tick
                sdt = hs - T["last_hts"] if T["last_hts"] else dt
                sdt = sdt if 0.002 < sdt < 0.5 else dt
                T["last_hts"] = hs
                T["filt"] = [T["filters"][i](raw[i], sdt) for i in (0, 1)]
                if T["prev_filt"] is not None:
                    a = 1 - math.exp(-2 * math.pi * 8.0 * sdt)
                    T["tvel"] = [T["tvel"][i] + a * ((T["filt"][i] - T["prev_filt"][i]) / sdt - T["tvel"][i]) for i in (0, 1)]
                T["prev_filt"] = list(T["filt"])
            T["since_new"] = 0.0 if new else T["since_new"] + dt
            ex = min(T["since_new"], 0.06)
            yl, pl = T["yaw_lim"], T["pitch_lim"]
            tgt = [max(-yl, min(yl, T["filt"][0] + T["tvel"][0] * ex)), max(-pl, min(pl, T["filt"][1] + T["tvel"][1] * ex))]
            tv = [0.0 if abs(tgt[0]) >= yl else T["tvel"][0], 0.0 if abs(tgt[1]) >= pl else T["tvel"][1]]
        else:
            tgt = [T["hfoll"][0].pos, T["hfoll"][1].pos] if state == "stale" else [0.0, 0.0]
            tv = [0.0, 0.0]; T["tvel"] = [0.0, 0.0]
        hcmd = [T["hfoll"][i].step(tgt[i], tv[i], dt) for i in (0, 1)]
        goals = {pan_m: T["centre"][pan_m] - hcmd[0], tilt_m: T["centre"][tilt_m] - hcmd[1]}
        p = self.pose
        ctrls = p.controllers if (p.controllers and tick - p.controllers_recv < 0.3) else None
        c_new = p.controllers_recv != T["last_ctrl_recv"]
        c_dt = p.controllers_recv - T["last_ctrl_recv"] if T["last_ctrl_recv"] else dt
        c_dt = c_dt if 0.002 < c_dt < 0.5 else dt
        T["last_ctrl_recv"] = p.controllers_recv
        head_pose = (ctrls or {}).get("head")
        for sname, arm in T["arms"].items():
            T["eng"][sname] = arm.update((ctrls or {}).get(sname), head_pose, c_new, c_dt)
            goals.update({k[:-4]: v for k, v in arm.goals(dt).items()})
        self.write_goals(goals)
        bad = [k for k, v in goals.items() if k in present and abs(present[k] - v) > 40]
        row = [f"{tick - T['t0']:.3f}", state, f"{hcmd[0]:.2f}", f"{hcmd[1]:.2f}"]
        for sname, arm in T["arms"].items():
            c = (ctrls or {}).get(sname) or {}
            row += [int(T["eng"][sname]), f"{c.get('grip', 0):.2f}", f"{c.get('trigger', 0):.2f}"]
            row += [f"{arm.target[j]:.2f}" for j in ARM_JOINTS]
            row += [f"{arm.lr_to_pose(j, present[arm.motor(j)]):.2f}" for j in ARM_JOINTS]
            row += [f"{present[arm.motor('gripper')]:.1f}"]
            row += [f"{v:.1f}" for v in arm.human] if arm.human else ["", "", ""]
        T["writer"].writerow(row)
        T["hud"] = [arm.hud_line((ctrls or {}).get(s)) for s, arm in T["arms"].items()] + \
            (["头显没有发送手柄数据"] if T["arms"] and ctrls is None else [])
        T["head_state"], T["ctrls"] = state, ctrls is not None
        if bad:  # far off target: blocked, or a bad goal - stop rather than push
            tp.hud_event("跟踪误差过大，已停止", keep=30.0)
            self.event(f"跟踪误差超过 40°（{', '.join(bad)}），自动收回")
            self.command("stop")

    # ------------------------------------------------------------------ replay (2026-11-10)
    def start_replay(self, rel):
        """Replay the COMMANDED joints of a recorded episode (robot.csv goals) at the recorded timing - the
        study's 'original demonstration replay'. Only joints torqued now are driven; every goal is clamped to the
        joint limits; first a glide to the episode's start pose."""
        d = os.path.realpath(os.path.join(DATA, rel))
        if not d.startswith(os.path.realpath(DATA) + os.sep) or not os.path.isfile(os.path.join(d, "robot.csv")):
            self.event("找不到这条录制"); return
        ts, goals, missing = [], [], set()
        with open(os.path.join(d, "robot.csv")) as f:
            for r in csv.DictReader(f):
                g = {}
                for m in self.torqued:
                    v = r.get(f"{m}.goal")
                    if v in (None, ""):
                        missing.add(m); continue
                    g[m] = self.pose_to_lr_clamped(m, float(v))
                ts.append(float(r["t"])); goals.append(g)
        if len(ts) < 2 or not goals[0]:
            self.event("这条录制没有可回放的关节数据"); return
        skipped = sorted({k.rsplit(".", 1)[0] for k in open(os.path.join(d, "robot.csv")).readline().strip().split(",")
                          if k.endswith(".goal")} - set(self.torqued))
        moved = [m for m in self.torqued if m not in missing]
        if any(m.startswith(("left_arm", "right_arm")) for m in skipped if not m.endswith("gripper")):
            self.event("提示：录制中的部分手臂当前未上扭矩，这部分不回放")
        self.replay = dict(ts=ts, goals=goals, name=rel, i=0, joints=moved)
        self.mode = "replay"
        self.event(f"开始回放：{rel}（{ts[-1] - ts[0]:.0f} 秒），先移到起始姿态")
        self.start_glide("replay_in", dict(goals[0]), self.begin_replay)

    def pose_to_lr_clamped(self, m, v):
        if m.endswith("gripper"):
            return max(0.0, min(100.0, v))
        if m in HEAD:
            lo, hi = self.head_limits[m]
            return max(lo, min(hi, to_lerobot_deg(self.cal, m, v)))
        lo, hi = self.arm_limits[m.split("_arm_")[1]]
        return to_lerobot_deg(self.cal, m, max(lo, min(hi, v)))

    def begin_replay(self):
        if self.replay:
            self.replay["t0"] = time.perf_counter()
            self.event("回放中")

    def step_replay(self, tick, present):
        R = self.replay
        target = R["ts"][0] + (tick - R["t0"])
        while R["i"] + 1 < len(R["ts"]) and R["ts"][R["i"] + 1] <= target:
            R["i"] += 1
        g = R["goals"][R["i"]]
        self.write_goals(g)
        bad = [k for k, v in g.items() if k in present and abs(present[k] - v) > 40]
        if bad:
            self.event(f"回放中跟踪误差超过 40°（{', '.join(bad)}），自动收回"); self.command("stop"); return
        if R["i"] >= len(R["ts"]) - 1:
            self.replay = None; self.mode = "ready"; self.event("回放结束，原地保持")

    # ------------------------------------------------------------------ recording
    def rec_command(self, cmd, arg):
        if cmd == "rec_start":
            if self.mode == "nobus" or self.rec.ep:
                return
            d = self.rec.start(dict(arms=self.arms_sel, robot_mode=self.mode, hz=self.args.hz,
                                    mapping=dict(pos_mode="joint", elbow_map="position", lift_knots=tp.LIFT_KNOTS,
                                                 grip_open=self.args.grip_open),
                                    target_zone=dict(self.zone, tcp=zn.TCP) if self.zone and self.kin else None))
            st = self.rec.status()
            self.event(f"开始录制：第 {st['episode']} 条（{self.rec.session['scenario']} v{st['version']}）")
            tp.hud_event(f"开始录制 · 第 {st['episode']} 条", keep=2.0)
            self.video_cmd(dict(cmd="rec_start", dir=d), lambda r: None if r.get("ok") else
                           self.event(f"视频录制未启动：{r.get('error')}"))
        elif cmd == "rec_stop":
            if not self.rec.ep:
                return
            st = self.rec.status()
            self.rec.stop()
            tp.hud_event("录制已保存", keep=2.0)
            last = self.rec.last
            self.event(f"结束录制：第 {st['episode']} 条，{st['elapsed']:.0f} 秒")

            def saved(r):
                self.rec.write_meta(last, video=r.get("stats") if r.get("ok") else dict(error=r.get("error")))
            self.video_cmd(dict(cmd="rec_stop"), saved)
        elif cmd == "rec_event" and self.rec.ep:
            self.rec.event("manual", str(arg or ""), source="researcher")
            self.event(f"事件：{arg}")
        elif cmd == "rec_session" and isinstance(arg, dict):
            self.rec.set_session(**arg)
            s = self.rec.session
            self.event(f"场次：{s['participant']} · {s['scenario']}" + (f" · 风格 {s['style']}" if s['style'] else ""))
        elif cmd == "rec_annotate" and isinstance(arg, dict):
            if self.rec.annotate(**arg):
                self.event("已更新这条录制的标注")

    def zone_command(self, cmd, arg):
        if not self.kin:
            self.event("无法设置目标区域：没有找到机器人模型（URDF）"); return
        if cmd == "zone_set":  # the gripper is at the tray centre, just above its surface
            side = arg if arg in SIDES else (self.arms_sel if self.arms_sel in SIDES else "left")
            p = self.kin.tcp_from_joints(side, self.state.get("joints") or {})
            if p is None:
                self.event("无法设置目标区域：读不到该手臂的关节"); return
            keep = {k: self.zone[k] for k in zn.DEFAULTS if self.zone and k in self.zone}
            self.zone = dict(zn.DEFAULTS, **keep, arm=side, center=[round(float(v), 4) for v in p],
                             set_at=time.strftime("%Y-%m-%d %H:%M:%S"))
            self.event(f"目标区域已记录（{ {'left': '左臂', 'right': '右臂'}[side]}夹爪当前位置为托盘中心）")
        elif cmd == "zone_params" and self.zone and isinstance(arg, dict):  # cm from the console
            for k, lo, hi in (("radius", 2, 40), ("below", 0, 20), ("above", 2, 50)):
                if arg.get(k) is not None:
                    self.zone[k] = round(min(hi, max(lo, float(arg[k]))) / 100, 3)
            self.event(f"目标区域：半径 {self.zone['radius'] * 100:.0f} cm，托盘上方 {self.zone['above'] * 100:.0f} cm 内")
        elif cmd == "zone_clear":
            self.zone = None
            try:
                os.remove(zn.ZONE_FILE)
            except OSError:
                pass
            self.event("目标区域已清除")
        else:
            return
        if self.zone:
            zn.save_zone(self.zone)
        self.zone_live = zn.ZoneTracker(self.zone) if self.zone else None
        if self.rec.ep:
            self.event("注意：本条录制仍按开始录制时的目标区域判定")

    def zone_state(self, joints, now):
        """for the console: the zone, where the tool point is relative to it, and whether it counts as inside."""
        if not self.kin:
            return None
        z = self.zone
        side = z["arm"] if z else (self.arms_sel if self.arms_sel in SIDES else "left")
        p = self.kin.tcp_from_joints(side, joints)
        out = dict(set=bool(z), arm=side, root=self.kin.arms[side]["root"], tcp=None if p is None else [round(float(v), 4) for v in p])
        if z:
            self.zone_live.update(now, p)
            out.update({k: z[k] for k in ("center", "radius", "below", "above", "margin", "dwell", "set_at") if k in z},
                       inside=self.zone_live.inside)
            if p is not None:
                r, h = self.zone_live.where(p)
                out.update(dist_cm=round(r * 100, 1), height_cm=round(h * 100, 1))
        return out, p

    def video_cmd(self, msg, then=None):
        """talk to video_daemon in a thread: closing the encoders can take a moment, the loop must not wait."""
        def run():
            try:
                from websockets.sync.client import connect
                with connect("ws://127.0.0.1:8771", open_timeout=2) as ws:
                    ws.send(json.dumps(msg)); r = json.loads(ws.recv(timeout=30))
            except Exception as exc:
                r = dict(ok=False, error=str(exc))
            if then:
                then(r)
        threading.Thread(target=run, daemon=True).start()

    def poll_headset(self):
        """raw headset messages into the recording; right controller B toggles recording (user choice 2026-11-10:
        Y opens the app menu, X / stick clicks are taken)."""
        p = self.pose
        if p.controllers_recv == self.last_vr:
            return
        self.last_vr = p.controllers_recv
        wall = time.time() - (time.perf_counter() - p.controllers_recv)
        h = p.get()
        self.rec.vr(dict(ctrl=p.controllers, head_yaw_pitch=h[:2] if h else None, headset_ms=h[2] if h else None), wall)
        # long press (user 2026-11-13: a tap on B started/stopped recordings by accident): fires once after
        # HOLD_B seconds held; releasing before that does nothing; the HUD shows "keep holding" meanwhile
        b = bool(((p.controllers or {}).get("right") or {}).get("secondary"))
        now = time.perf_counter()
        if b and not self.prev_b:
            self.b_down, self.b_fired = now, False
        if b and not self.b_fired and now - self.b_down >= HOLD_B:
            self.b_fired = True
            self.rec_command("rec_stop" if self.rec.ep else "rec_start", None)
        self.prev_b = b

    # ------------------------------------------------------------------ loop
    def loop(self):
        last_hud = 0.0
        while not self.stop.is_set():
            tick = time.perf_counter()
            with self.lock:
                cmds, self.cmds = self.cmds, []
            for c, arg in cmds:
                try:
                    self.command(c, arg)
                except Exception as exc:
                    self.event(f"命令 {c} 出错：{exc}")
            try:
                present = self.read_positions()
                self.bus_err = 0
                if self.motion:
                    self.step_motion(tick)
                elif self.mode == "teleop" and self.teleop:
                    self.step_teleop(tick, present)
                elif self.mode == "replay" and self.replay and "t0" in self.replay:
                    self.step_replay(tick, present)
                self.poll_telemetry()
                self.poll_headset()
            except ConnectionError as exc:
                self.bus_err += 1
                if self.bus_err == 1:
                    log("bus error:", exc)
                if self.bus_err >= 10 and self.torqued:
                    self.release_all("总线连续通信失败，已紧急释放扭矩")
                present = None
            self.publish(present)
            if tick - last_hud > 0.1:
                last_hud = tick
                self.hud_write()
            time.sleep(max(0.0, self.dt - (time.perf_counter() - tick)))

    def hud_write(self):
        """Headset HUD v2 (2026-11-13, user: prompts all at the top were easy to miss and looked bare). Structured
        state for video_daemon.HudOverlay instead of text lines:
          center  a card only when the operator has to act (calibrate, no controller data, stopped on an error);
          rec     recording badge (start time, episode) / hold: progress of a B long press;
          status  one quiet line (arm states or what the robot is doing); toast: one-off confirmations that fade.
        Research-only signals (target zone) are deliberately not shown to the participant."""
        now = time.time()
        tp._hud_events[:] = [e for e in tp._hud_events if e[0] > now]
        center, status = None, ""
        warn = [(until, m) for until, m in tp._hud_events if "停止" in m or "失败" in m]
        T = self.teleop or {}
        if self.mode == "teleop" and T:
            arms = T.get("arms", {})
            states = []
            for s, arm in arms.items():
                name, key = tp.SIDE_CN[s]
                if arm.pos_mode == "joint" and arm.torso_heading is None and center is None:
                    center = dict(title=f"把{name}水平向前伸直", sub=f"然后按下{key}完成标定", level="info")
                states.append((name, "跟随中" if arm.engaged else "握住握把开始移动", bool(arm.engaged)))
            if T.get("arms") and not T.get("ctrls"):
                center = dict(title="收不到手柄数据", sub="请确认两只手柄已开启并在视野内", level="warn")
            status = [dict(text=f"{n} · {t}", on=on) for n, t, on in states]
        else:
            status = [dict(text={"idle": "等待开始", "hold": "准备中", "ready": "准备就绪", "fold": "正在收回",
                                 "replay": "正在回放", "nobus": "机器人未连接"}.get(self.mode, ""), on=False)]
        if warn:
            until, m = warn[-1]
            center = dict(title=m, sub="请等待研究者处理", level="warn")
        toast = next(((m, until) for until, m in reversed(tp._hud_events) if (until, m) not in warn), None)
        rec = None
        if self.rec.ep:
            st = self.rec.status()
            rec = dict(start=self.rec.ep["meta"]["t_start"], episode=st["episode"])
        hold = None
        if self.prev_b and not self.b_fired:
            hold = dict(progress=min(1.0, (time.perf_counter() - self.b_down) / HOLD_B),
                        action="结束录制" if self.rec.ep else "开始录制")
        state = dict(v=2, t=now, center=center, rec=rec, hold=hold, status=status,
                     toast=dict(text=toast[0], until=toast[1]) if toast else None)
        try:
            with open(tp.HUD_FILE + ".tmp", "w") as fh:
                json.dump(state, fh, ensure_ascii=False)
            os.replace(tp.HUD_FILE + ".tmp", tp.HUD_FILE)
        except OSError:
            pass

    def hud_lines(self):
        rec = []
        if self.rec.ep:
            st = self.rec.status()
            rec = [f"● 录制中 {int(st['elapsed']) // 60:02d}:{int(st['elapsed']) % 60:02d}  第 {st['episode']} 条（长按 B 结束）"]
        if self.prev_b and not self.b_fired:  # B is being held: show that the hold counts
            left = max(0.0, HOLD_B - (time.perf_counter() - self.b_down))
            rec = rec + [f"继续按住 B {left:.1f} 秒：{'结束' if self.rec.ep else '开始'}录制"]
        if self.mode == "teleop" and self.teleop:
            return rec + self.teleop.get("hud", [])
        return rec + {"idle": ["机器人空闲：请在控制台上扭矩"], "hold": ["已上扭矩：可在控制台展开"], "replay": ["正在回放录制"],
                      "ready": ["已就绪：在控制台开始遥操作"], "fold": ["正在收回折叠位"]}.get(self.mode, [])

    def publish(self, present):
        joints = {}
        for m in JOINTS:
            p = present.get(m) if present else None
            g = self.goals.get(m, p)
            tel = self.telemetry.get(m, {})
            joints[m] = dict(pos=None if p is None else round(self.lr_to_pose(m, p), 2),
                             goal=None if g is None else round(self.lr_to_pose(m, g), 2),
                             torque=m in self.torqued, temp=tel.get("temp"), volt=tel.get("volt"), err=tel.get("err", 0))
        T = self.teleop or {}
        zs = self.zone_state(joints, time.time())
        zone, tcp = zs if zs else (None, None)
        if self.rec.ep:  # recording: the same joint view the console shows, every tick
            ready = {m: 0.0 for m in self.torqued if "_arm_" in m and not m.endswith("gripper")}
            self.rec.tick(self.mode, joints, T.get("eng", {}), T.get("head_state"), ready, tcp=tcp)
        motion = None
        if self.motion:
            motion = dict(kind=self.motion["kind"],
                          progress=round(min(1.0, (time.perf_counter() - self.motion["t0"]) / self.motion["T"]), 2))
        with self.lock:
            self.state = dict(type="state", mock=False, t=time.time(), mode=self.mode, motion=motion,
                              arms=self.arms_sel, joints=joints,
                              teleop=dict(head=T.get("head_state"), engaged=T.get("eng", {}), hud=T.get("hud", [])) if T else None,
                              links=dict(self.links, bus_ok=self.bus_err == 0,
                                         quest=self.pose.get() is not None and time.perf_counter() - self.pose.get()[3] < 1.0),
                              rec=self.rec.status(), events=list(self.events[-8:]), zone=zone)

    # ------------------------------------------------------------------ console channel
    async def serve(self):
        import websockets

        async def handler(ws):
            log("console connected")
            await ws.send(json.dumps(dict(type="hello", joints=JOINTS, limits=self.limits_pose())))

            async def pump():
                while True:
                    with self.lock:
                        msg = json.dumps(self.state)
                    await ws.send(msg)
                    await asyncio.sleep(1 / 30)
            task = asyncio.create_task(pump())
            try:
                async for raw in ws:
                    m = json.loads(raw)
                    if m.get("type") == "cmd":
                        with self.lock:
                            self.cmds.append((m.get("cmd"), m.get("arg")))
            except Exception:
                pass
            finally:
                task.cancel(); log("console disconnected")

        async with websockets.serve(handler, "127.0.0.1", self.args.port):
            await asyncio.Future()

    def limits_pose(self):
        out = {m: v for m, v in json.load(open(os.path.join(os.path.dirname(HERE), "joint_limits.json")))["pose_deg_limits_head"].items()}
        for s in SIDES:
            for j, v in self.arm_limits.items():
                out[f"{s}_arm_{j}"] = v
            out[f"{s}_arm_gripper"] = [0.0, 100.0]
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--arm-p", type=int, default=24)
    ap.add_argument("--arm-msf", type=int, default=30)
    ap.add_argument("--grip-open", type=float, default=60.0)
    args = ap.parse_args(_args)
    core = Core(args)
    os.makedirs(LOG_DIR, exist_ok=True)
    hf.start_reachy(core.pose, core.stop)  # Quest pose server :8000
    threading.Thread(target=lambda: asyncio.run(core.serve()), daemon=True).start()  # console channel first
    core.mode = "nobus"
    warned = False
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))  # TERM = fold, release, exit
    signal.signal(signal.SIGINT, signal.default_int_handler)  # started with nohup ... & SIGINT is ignored otherwise
    try:
        while True:
            try:
                core.connect(); core.mode = "idle"; break
            except KeyboardInterrupt:
                raise
            except Exception as exc:  # no servo power / USB needs replugging after sleep: keep retrying, tell the console
                log("bus connect failed:", exc)
                if not warned:
                    core.event("连不上舵机总线：请检查驱动板12V电源，并在驱动板一端拔插USB"); warned = True
                core.links = dict(bus1=False, bus2=False)
                for _ in range(3):  # keep the console's view fresh (it calls the core offline after 2 s of silence)
                    core.publish(None); time.sleep(1)
    except KeyboardInterrupt:
        core.stop.set(); return
    try:
        core.loop()
    except KeyboardInterrupt:
        pass
    finally:
        core.stop.set()
        if core.rec.ep:
            core.rec_command("rec_stop", None); time.sleep(1.0)
        core.end_teleop()
        if core.torqued:  # never leave torque on unattended: fold first if we can
            log("shutting down: folding and releasing")
            try:
                core.command("stop")
                t_end = time.perf_counter() + 8
                while core.motion and time.perf_counter() < t_end:
                    core.step_motion(time.perf_counter()); time.sleep(core.dt)
            except Exception as exc:
                log("fold on shutdown failed:", exc)
            core.release_all("核心退出，扭矩已释放")
        try:
            os.remove(tp.HUD_FILE)
        except OSError:
            pass


if __name__ == "__main__":
    main()
