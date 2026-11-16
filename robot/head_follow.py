"""Head-follow controller (robot side): headset yaw/pitch -> two servo joints.

Rigs
  head  (default) the XLeRobot head: head_motor_1 (ID7, pan, + = turn right) and head_motor_2
        (ID8, tilt, - = look up) on bus1, verified 2026-10-19; arms are not touched (torque stays off).
        Calibration from xlerobot_fmp.json, limits from joint_limits.json.
  arm   2026-10-14 stand-in: shoulder_pan / wrist_flex of one SO101 arm (left_arm_test calibration).

Sources
  mock   synthetic head motion (no headset needed)
  xlevr  Quest browser runs the XLeVR WebXR page (https://<robot-ip>:8443), which streams headset
         pose JSON over wss://<robot-ip>:8442. This script serves that page and socket itself.
  reachy the reachy-vr-quest Unity app (HRI-EU), as if this were the Reachy Mini daemon:
         ws://<robot-ip>:8000/api/move/ws/set_target; video comes from video_daemon.py (port 8443).

Pipeline per tick: recentre -> stale-data handling -> One Euro filter -> clamp -> motion shaper
(velocity + acceleration limited, so direction changes are smooth) -> servo goal. Every tick is logged to CSV with headset timestamp and receive age for later
latency analysis (headset and robot clocks are NOT synchronised; only receive-side ages are exact).

Keys while running (type + ENTER in the terminal): c = recentre, q = quit.
"""
import argparse
import asyncio
import csv
import functools
import json
import logging
import math
import os
import ssl
import sys
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler

XLEVR_DIR = os.path.expanduser("~/code/XLeRobot/XLeVR")
# copy of XLeVR/web-ui with per-frame console.log disabled (see make_xlevr_web.sh); falls back to upstream
WEB_DIR = os.path.expanduser("~/fmp_scripts/xlevr-web")
if not os.path.isdir(WEB_DIR):
    WEB_DIR = os.path.join(XLEVR_DIR, "web-ui")


# ---------------------------------------------------------------- pose sources
class LatestPose:
    """Thread-safe holder of the newest (yaw, pitch, headset_ts_ms, recv_perf) sample."""

    def __init__(self):
        self._lock = threading.Lock()
        self._v = None
        self.count = 0
        self.transit_ms = float("nan")  # reachy source: receive wall time - headset send time
        self.controllers, self.controllers_recv = None, 0.0  # reachy source: latest fmp_controllers dict

    def set(self, yaw, pitch, headset_ts):
        with self._lock:
            self._v = (yaw, pitch, headset_ts, time.perf_counter())
            self.count += 1

    def get(self):
        with self._lock:
            return self._v


class OneEuro:
    """One Euro filter (Casiez et al. 2012): smooth when slow, responsive when fast."""

    def __init__(self, min_cutoff, beta, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.reset()

    def reset(self):
        self.x = self.dx = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1 / (2 * math.pi * cutoff)
        return 1 / (1 + tau / dt)

    def __call__(self, x, dt):
        if self.x is None:
            self.x, self.dx = x, 0.0
            return x
        dx = (x - self.x) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * dx + (1 - a_d) * self.dx
        cutoff = self.min_cutoff + self.beta * abs(self.dx)
        a = self._alpha(cutoff, dt)
        self.x = a * x + (1 - a) * self.x
        return self.x


class SpringFollower:
    """Critically damped spring-damper tracker (one axis) with acceleration and jerk limits.
    a = w^2 * (target - pos) + 2*w*(target_vel - vel): fastest response without overshoot for a stopping
    target; the damping term doubles as velocity feed-forward, so steady turns do not lag. w in rad/s."""

    def __init__(self, w=25.0, a_max=2000.0, jerk=60000.0, v_max=250.0, ff=1.0):
        self.kp, self.kd, self.a_max, self.jerk, self.v_max = w * w, 2 * w, a_max, jerk, v_max
        self.ff = ff  # scale on the target-velocity term; < 1 trims stop overshoot from a lagging estimate
        self.pos = self.vel = self.acc = 0.0

    def step(self, target, target_vel, dt):
        a_des = self.kp * (target - self.pos) + self.kd * (self.ff * target_vel - self.vel)
        a_des = max(-self.a_max, min(self.a_max, a_des))
        dj = self.jerk * dt
        self.acc += max(-dj, min(dj, a_des - self.acc))
        self.vel = max(-self.v_max, min(self.v_max, self.vel + self.acc * dt))
        self.pos += self.vel * dt
        return self.pos


class SmoothFollower:
    """Jerk-limited follower with braking-aware speed and velocity feed-forward (one axis).

    Replaces the 2026-10-14 shaper (v_des = gain * error, accel-limited), which the user found jerky at the
    start and end of head turns (2026-10-20): acceleration jumped instantly and, near the target, the speed
    was too high to stop in time, so it overshot and settled with a bump.
      * v_des = ff * target_velocity + sign(e) * min(gain*|e|, sqrt(2*a_brake*|e|)), |v_des| <= v_max
        -> arrives with ~zero speed instead of overshooting; feed-forward keeps steady turns from lagging
      * acceleration tracks (v_des - v)/tau but its change per tick is limited by jerk -> S-curves
    """

    def __init__(self, v_max=200.0, a_max=1500.0, jerk=20000.0, gain=30.0, ff=1.0, tau=0.04, brake=0.8):
        self.v_max, self.a_max, self.jerk, self.gain, self.ff, self.tau = v_max, a_max, jerk, gain, ff, tau
        self.a_brake = brake * a_max
        self.pos = self.vel = self.acc = 0.0

    def step(self, target, target_vel, dt):
        e = target - self.pos
        corr = math.copysign(min(self.gain * abs(e), math.sqrt(2 * self.a_brake * abs(e))), e)
        v_des = max(-self.v_max, min(self.v_max, self.ff * target_vel + corr))
        a_des = max(-self.a_max, min(self.a_max, (v_des - self.vel) / self.tau))
        dj = self.jerk * dt
        self.acc += max(-dj, min(dj, a_des - self.acc))
        self.vel = max(-self.v_max, min(self.v_max, self.vel + self.acc * dt))
        self.pos += self.vel * dt
        return self.pos


def quat_to_yaw_pitch(x, y, z, w):
    """three.js/WebXR frame (x right, y up, -z forward), Euler order YXZ.
    yaw > 0 = turn left, pitch > 0 = look up. Degrees."""
    m13 = 2 * (x * z + w * y)
    m33 = 1 - 2 * (x * x + y * y)
    m23 = 2 * (y * z - w * x)
    yaw = math.degrees(math.atan2(m13, m33))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -m23))))
    return yaw, pitch


def start_mock(pose, stop):
    def run():
        t0 = time.perf_counter()
        while not stop.is_set():
            t = time.perf_counter() - t0
            yaw = 25 * math.sin(2 * math.pi * 0.20 * t)
            pitch = 12 * math.sin(2 * math.pi * 0.13 * t)
            pose.set(yaw, pitch, time.time() * 1000)
            time.sleep(1 / 72)  # Quest-like frame rate
    threading.Thread(target=run, daemon=True).start()


def start_xlevr(pose, stop, https_port=8443, wss_port=8442):
    import websockets

    cert, key = os.path.join(XLEVR_DIR, "cert.pem"), os.path.join(XLEVR_DIR, "key.pem")
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert, key)

    # static HTTPS server for the WebXR page
    handler = functools.partial(QuietHandler, directory=WEB_DIR)
    httpd = HTTPServer(("0.0.0.0", https_port), handler)
    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    async def on_client(ws):
        print(f"[xlevr] headset connected from {ws.remote_address[0]}", flush=True)
        async for msg in ws:
            try:
                data = json.loads(msg)
                q = (data.get("headset") or {}).get("quaternion")
                if q:
                    yaw, pitch = quat_to_yaw_pitch(q["x"], q["y"], q["z"], q["w"])
                    pose.set(yaw, pitch, data.get("timestamp"))
            except (ValueError, KeyError, TypeError):
                continue
        print("[xlevr] headset disconnected", flush=True)

    async def serve():
        async with websockets.serve(on_client, "0.0.0.0", wss_port, ssl=ctx):
            while not stop.is_set():
                await asyncio.sleep(0.2)

    threading.Thread(target=lambda: asyncio.run(serve()), daemon=True).start()
    return httpd


def start_reachy(pose, stop, port=8000):
    """Pose input from the reachy-vr-quest Quest app, acting as the Reachy Mini daemon endpoint
    ws://<host>:8000/api/move/ws/set_target. The app only sends (30 Hz FullBodyTarget JSON), never waits
    for replies. target_head_pose.m is a row-major 4x4 world head pose in FLU (x forward, y left, z up);
    yaw/pitch come from its forward axis (first rotation column). target_body_yaw and antennas are
    ignored: XLeRobot has no body-yaw joint, the world head yaw is what the pan servo should follow."""
    import websockets

    import socket

    async def on_client(ws):
        print(f"[reachy] app connected from {ws.remote_address[0]} path {ws.request.path}", flush=True)
        sock = ws.transport.get_extra_info("socket")
        async for msg in ws:
            # ACK at once: the app's socket keeps Nagle on (websocket-sharp exposes no NoDelay), so with
            # Linux delayed ACKs each small message waited for the previous ACK (2026-10-20: ~14 Hz cap)
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
            except (OSError, AttributeError):
                pass
            try:
                data = json.loads(msg)
                m = data["target_head_pose"]["m"]
                fx, fy, fz = m[0], m[4], m[8]          # forward axis in FLU
                yaw = math.degrees(math.atan2(fy, fx))  # > 0 = turn left
                pitch = math.degrees(math.atan2(fz, math.hypot(fx, fy)))  # > 0 = look up
                now_ms = time.time() * 1000
                sent_ms = data.get("send_time_ms")  # FMP-modified app (2026-10-20) stamps headset send time
                pose.transit_ms = now_ms - sent_ms if sent_ms else float("nan")  # includes unknown clock offset
                pose.set(yaw, pitch, sent_ms if sent_ms else now_ms)
                if "fmp_controllers" in data:  # FMP-modified app: controller state for arm teleop (teleop.py)
                    pose.controllers, pose.controllers_recv = data["fmp_controllers"], time.perf_counter()
            except (ValueError, KeyError, TypeError, IndexError):
                continue
        print("[reachy] app disconnected", flush=True)

    async def serve():
        async with websockets.serve(on_client, "0.0.0.0", port):
            while not stop.is_set():
                await asyncio.sleep(0.2)

    threading.Thread(target=lambda: asyncio.run(serve()), daemon=True).start()


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


# ---------------------------------------------------------------- rigs
HERE = os.path.dirname(os.path.abspath(__file__))
XLR_CAL = os.path.expanduser("~/.cache/huggingface/lerobot/calibration/robots/xlerobot/xlerobot_fmp.json")


class HeadRig:
    """Only the two head servos on XLeRobot bus1; the arm servos on the same bus are never addressed."""
    joints = ("head_motor_1", "head_motor_2")
    release_msg = "Torque will be released (the head may sag a little). Press ENTER..."

    def __init__(self, port, p_gain, max_rel, min_startup_force=40):
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        cal = json.load(open(XLR_CAL))
        motors = {m: Motor(cal[m]["id"], "sts3215", MotorNormMode.DEGREES) for m in self.joints}
        self.bus = FeetechMotorsBus(port=port, motors=motors,
                                    calibration={m: MotorCalibration(**cal[m]) for m in self.joints})
        self.limits = json.load(open(os.path.join(HERE, "joint_limits.json")))["lerobot_deg_limits"]
        self.p_gain, self.max_rel, self.last = p_gain, max_rel, {}
        self.msf = min_startup_force

    def connect(self):
        from lerobot.motors.feetech import OperatingMode
        self.bus.connect()
        for attempt in range(1, 4):
            try:
                self.bus.disable_torque(num_retry=2)
                for m in self.joints:
                    self.bus.write("Operating_Mode", m, OperatingMode.POSITION.value, num_retry=2)
                    self.bus.write("P_Coefficient", m, self.p_gain, num_retry=2)
                    self.bus.write("I_Coefficient", m, 0, num_retry=2)
                    self.bus.write("D_Coefficient", m, 32, num_retry=2)
                    # 2026-10-20 stiction test: factory 16 let the tilt servo stick up to 660 ms on small errors
                    # ("stops twice"); 40 with P48 cut it to 40-60 ms. EEPROM value, persists after power-off.
                    self.bus.write("Minimum_Startup_Force", m, self.msf, num_retry=2)
                self.bus.sync_write("Goal_Position", self.bus.sync_read("Present_Position"))
                time.sleep(0.05)
                self.bus.enable_torque(num_retry=2)
                return True
            except ConnectionError as exc:
                print(f"setup attempt {attempt} failed: {exc}", flush=True)
                time.sleep(0.3)
        return False

    def get_observation(self):
        self.last = {f"{m}.pos": v for m, v in self.bus.sync_read("Present_Position").items()}
        return dict(self.last)

    def send_action(self, action):
        goal = {}
        for m in self.joints:
            g = action[f"{m}.pos"]
            lo, hi = self.limits[m]
            g = max(lo, min(hi, g))
            p = self.last.get(f"{m}.pos")
            if p is not None:
                g = max(p - self.max_rel, min(p + self.max_rel, g))
            goal[m] = g
        self.bus.sync_write("Goal_Position", goal)
        return {f"{m}.pos": v for m, v in goal.items()}

    def room(self, joint, centre):
        lo, hi = self.limits[joint]
        return min(hi - centre, centre - lo)

    def disconnect(self):
        self.bus.disconnect(disable_torque=True)


class ArmRig:
    """2026-10-14 stand-in: one SO101 arm as a pan/tilt."""
    joints = ("shoulder_pan", "wrist_flex")
    release_msg = "HOLD THE ARM (it will drop when torque is released), then press ENTER..."

    def __init__(self, port, p_gain, max_rel, robot_id="left_arm_test"):
        from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
        from lerobot.robots.so_follower.so_follower import SOFollower
        self.robot = SOFollower(SOFollowerRobotConfig(port=port, id=robot_id, use_degrees=True,
                                                      max_relative_target=max_rel, position_p_coefficient=p_gain))

    def connect(self):
        r = self.robot
        r.bus.connect()
        for attempt in range(1, 4):
            try:
                r.bus.disable_torque(num_retry=2)
                r.bus.sync_write("Goal_Position", r.bus.sync_read("Present_Position"))
                time.sleep(0.05)
                r.configure()
                return True
            except ConnectionError as exc:
                print(f"setup attempt {attempt} failed: {exc}", flush=True)
                time.sleep(0.3)
        return False

    def get_observation(self):
        return {k: v for k, v in self.robot.get_observation().items() if k.endswith(".pos")}

    def send_action(self, action):
        return self.robot.send_action(action)

    def room(self, joint, centre):
        cal = self.robot.calibration[joint]
        half = (cal.range_max - cal.range_min) / 2 * 360 / 4096
        return min(half - centre, half + centre) - 3

    def disconnect(self):
        self.robot.disconnect()


# ---------------------------------------------------------------- control
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["mock", "xlevr", "reachy"], default="mock")
    ap.add_argument("--duration", type=float, default=0, help="seconds, 0 = until q")
    ap.add_argument("--rig", choices=["head", "arm"], default="head")
    # headset yaw > 0 = turn left, pitch > 0 = look up. Both rigs need -1 on both axes:
    #   head: head_motor_1 + = turn right, head_motor_2 - = look up (camera image shift, 2026-10-19)
    #   arm stand-in: confirmed by the user in Quest runs on 2026-10-14
    ap.add_argument("--yaw-sign", type=float, default=-1.0)
    ap.add_argument("--pitch-sign", type=float, default=-1.0)
    ap.add_argument("--gain", type=float, default=1.0, help="head-to-joint angle ratio; real head camera needs 1.0")
    ap.add_argument("--yaw-limit", type=float, default=None, help="deg (default head 90, arm 45)")
    ap.add_argument("--pitch-limit", type=float, default=None, help="deg (default head 45, arm 25)")
    ap.add_argument("--min-cutoff", type=float, default=2.0, help="One Euro: cutoff Hz when the head is still")
    ap.add_argument("--beta", type=float, default=0.05, help="One Euro: cutoff increase per deg/s of head speed")
    ap.add_argument("--shaper", choices=["spring", "old"], default="spring")
    ap.add_argument("--spring-w", type=float, default=25.0, help="spring: natural frequency, rad/s")
    ap.add_argument("--spring-amax", type=float, default=2000.0, help="spring: deg/s^2")
    ap.add_argument("--spring-jerk", type=float, default=60000.0, help="spring: deg/s^3")
    ap.add_argument("--spring-vmax", type=float, default=250.0, help="spring: deg/s")
    ap.add_argument("--ff", type=float, default=0.6, help="spring: velocity feed-forward scale (replay 2026-10-20)")
    ap.add_argument("--extrap", type=float, default=0.06, help="s: advance the target by velocity during input gaps, cap")
    ap.add_argument("--msf", type=int, default=40, help="head servo Minimum_Startup_Force (factory 16)")
    ap.add_argument("--track-gain", type=float, default=30.0, help="old shaper: 1/s, desired speed = gain * error")
    ap.add_argument("--max-speed", type=float, default=180.0, help="shaper: deg/s")
    ap.add_argument("--max-accel", type=float, default=1200.0, help="shaper: deg/s^2")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--p", type=int, default=None, help="servo P coefficient (default head 32, arm 24)")
    ap.add_argument("--max-rel", type=float, default=None, help="max goal lead per tick, deg (default head 20, arm 10)")
    ap.add_argument("--port", default=os.environ.get("XLR_PORT1", "/dev/ttyACM0"))
    args = ap.parse_args()

    # the per-tick "goal clamped" warning floods the terminal; clamps are counted and logged instead
    logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)

    # head is light: stiffer servo and a longer goal lead keep up with fast head turns (2026-10-19)
    p_gain = args.p if args.p is not None else (48 if args.rig == "head" else 24)  # head 48: stiction test 2026-10-20
    max_rel = args.max_rel if args.max_rel is not None else (20.0 if args.rig == "head" else 10.0)
    # head range checked by hand on 2026-10-19 (head_range_check.py): +-90 pan, +-45 tilt, no cable strain or contact
    if args.yaw_limit is None:
        args.yaw_limit = 90.0 if args.rig == "head" else 45.0
    if args.pitch_limit is None:
        args.pitch_limit = 45.0 if args.rig == "head" else 25.0
    robot = (HeadRig(args.port, p_gain, max_rel, args.msf) if args.rig == "head"
             else ArmRig(args.port, p_gain, max_rel))
    print(f"rig={args.rig} P={p_gain} max_rel={max_rel} gain={args.gain} signs yaw {args.yaw_sign:+.0f} pitch {args.pitch_sign:+.0f} "
          f"shaper={args.shaper}" + (f" w={args.spring_w} amax={args.spring_amax} jerk={args.spring_jerk} ff={args.ff}" if args.shaper == "spring" else "")
          + f" extrap={args.extrap}s" + (f" msf={args.msf}" if args.rig == "head" else ""))
    yaw_joint, pitch_joint = robot.joints
    yk, pk = f"{yaw_joint}.pos", f"{pitch_joint}.pos"

    pose, stop = LatestPose(), threading.Event()
    if args.source == "mock":
        start_mock(pose, stop)
    elif args.source == "reachy":
        start_reachy(pose, stop)
        print("In the reachy-vr-quest app: Robot IP = this laptop's IP, pose port 8000, webrtc port 8443;"
              " enable 'Connect With Pose Data' (and 'ConnectVideo' when video_daemon.py runs).")
    else:
        start_xlevr(pose, stop)
        print("Open on the Quest browser:  https://<robot-ip>:8443  (accept the certificate;"
              " if the socket fails, also open https://<robot-ip>:8442 once and accept), then press Start.")

    if args.rig == "head":
        input("Point the head straight ahead and level (by hand), then press ENTER to turn head torque ON...")
    else:
        input("Hold the arm in the middle pose, then press ENTER to turn torque ON...")
    if not robot.connect():
        print("setup failed; torque left OFF")
        sys.exit(1)
    hold = robot.get_observation()
    centre = {yk: hold[yk], pk: hold[pk]}
    yaw_lim = min(args.yaw_limit, robot.room(yaw_joint, centre[yk]))
    pitch_lim = min(args.pitch_limit, robot.room(pitch_joint, centre[pk]))
    print(f"limits: yaw +/-{yaw_lim:.1f}  pitch +/-{pitch_lim:.1f} deg (joint range and args)")
    input("Torque ON. Let go gently. Press ENTER to start following (type c+ENTER recentre, q+ENTER quit)...")

    cmd_queue = []
    threading.Thread(target=lambda: [cmd_queue.append(line.strip()) for line in sys.stdin], daemon=True).start()

    os.makedirs(os.path.expanduser("~/fmp_logs"), exist_ok=True)
    log_path = os.path.expanduser(f"~/fmp_logs/head_follow_{args.source}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    dt = 1 / args.hz
    zero = None
    filters = [OneEuro(args.min_cutoff, args.beta), OneEuro(args.min_cutoff, args.beta)]
    filt = [0.0, 0.0]
    cmd, vel = [0.0, 0.0], [0.0, 0.0]
    tvel, prev_filt = [0.0, 0.0], None  # head angular velocity estimate (8 Hz low-pass), deg/s
    followers = [SpringFollower(w=args.spring_w, a_max=args.spring_amax, jerk=args.spring_jerk,
                                v_max=args.spring_vmax, ff=args.ff) for _ in (0, 1)]
    since_new = 0.0  # s since the last new pose sample (for gap extrapolation)

    def shape(targets, target_vel=(0.0, 0.0)):
        """spring (default since 2026-10-20): critically damped tracker, offline replay 20 ms lag vs 60 ms and
        lower jerk than old; old: v_des = gain*e with accel limit (2026-10-14, jerky at start/stop)."""
        for i in (0, 1):
            if args.shaper == "spring":
                cmd[i] = followers[i].step(targets[i], target_vel[i], dt)
                vel[i] = followers[i].vel
            else:
                v_des = max(-args.max_speed, min(args.max_speed, args.track_gain * (targets[i] - cmd[i])))
                dv = args.max_accel * dt
                vel[i] += max(-dv, min(dv, v_des - vel[i]))
                cmd[i] += vel[i] * dt
    last_count, ticks, stale_ticks, clamp_ticks, reason = 0, 0, 0, 0, "duration reached"
    STALL_ERR, STALL_S, HARD_ERR = 15.0, 0.4, 45.0
    status = [None]

    def status_printer():
        while not stop.is_set():
            time.sleep(1.0)
            if status[0]:
                print(status[0], flush=True)

    threading.Thread(target=status_printer, daemon=True).start()
    stall_s, prev_obs = 0.0, None
    last_hts = None
    t0 = time.perf_counter()
    with open(log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "headset_ts_ms", "sample_age_ms", "new_sample", "raw_yaw", "raw_pitch",
                    "filt_yaw", "filt_pitch", "cmd_yaw", "cmd_pitch", "present_yaw", "present_pitch",
                    "state", "clamped", "transit_ms"])
        try:
            while True:
                tick = time.perf_counter()
                if args.duration and tick - t0 > args.duration:
                    break
                while cmd_queue:
                    c = cmd_queue.pop(0)
                    if c == "q":
                        raise KeyboardInterrupt
                    if c == "c":
                        zero = None
                        print("recentre", flush=True)
                s = pose.get()
                state, raw = "ok", (float("nan"), float("nan"))
                age = float("inf")
                if s is None:
                    state = "no_data"
                else:
                    yaw, pitch, hts, recv = s
                    age = tick - recv
                    if zero is None:
                        zero = (yaw, pitch)
                        for flt in filters:
                            flt.reset()
                    raw = (args.gain * (((yaw - zero[0] + 180) % 360) - 180), args.gain * (pitch - zero[1]))
                    if age > 2.0:
                        state = "lost"      # glide back to centre
                    elif age > 0.3:
                        state = "stale"     # hold last command
                new = pose.count != last_count
                last_count = pose.count
                target = list(cmd)
                if state == "ok":
                    if new:  # filter only real samples (Quest ~45 Hz < loop 50 Hz), timed by headset clock
                        hts_s = hts / 1000 if hts else tick
                        sdt = hts_s - last_hts if last_hts is not None else dt
                        sdt = sdt if 0.002 < sdt < 0.5 else dt
                        last_hts = hts_s
                        filt = [filters[i](raw[i], sdt) for i in (0, 1)]
                        if prev_filt is not None:
                            a = 1 - math.exp(-2 * math.pi * 8.0 * sdt)
                            tvel = [tvel[i] + a * ((filt[i] - prev_filt[i]) / sdt - tvel[i]) for i in (0, 1)]
                        prev_filt = list(filt)
                    since_new = 0.0 if new else since_new + dt
                    ex = min(since_new, args.extrap)  # input gaps: keep moving at the estimated speed briefly
                    goal = [filt[i] + tvel[i] * ex for i in (0, 1)]
                    target = [max(-yaw_lim, min(yaw_lim, goal[0])), max(-pitch_lim, min(pitch_lim, goal[1]))]
                    # no velocity feed-forward while pinned at a limit, or the tracker pushes into it
                    tv_use = [0.0 if abs(target[i]) >= lim else tvel[i] for i, lim in ((0, yaw_lim), (1, pitch_lim))]
                else:
                    tv_use = [0.0, 0.0]
                    tvel = [0.0, 0.0]
                    if state in ("lost", "no_data"):
                        target = [0.0, 0.0]
                stale_ticks += state != "ok"
                shape(target, tv_use)
                action = dict(hold)
                action[yk] = centre[yk] + args.yaw_sign * cmd[0]
                action[pk] = centre[pk] + args.pitch_sign * cmd[1]
                sent = robot.send_action(action)
                clamped = int(abs(sent[yk] - action[yk]) > 0.01 or abs(sent[pk] - action[pk]) > 0.01)
                clamp_ticks += clamped
                obs = robot.get_observation()
                py, pp = obs[yk] - centre[yk], obs[pk] - centre[pk]
                w.writerow([f"{tick - t0:.4f}", s[2] if s else "", f"{age * 1000:.1f}" if s else "", int(new),
                            f"{raw[0]:.2f}", f"{raw[1]:.2f}", f"{filt[0]:.2f}", f"{filt[1]:.2f}",
                            f"{cmd[0]:.2f}", f"{cmd[1]:.2f}",
                            f"{args.yaw_sign * py:.2f}", f"{args.pitch_sign * pp:.2f}", state, clamped,
                            f"{pose.transit_ms:.1f}"])
                # Lagging during a fast head turn is normal; stop only when the servo is far off AND not
                # moving (blocked/stalled) for STALL_S, or when the error is beyond any normal lag.
                # (2026-10-19: a plain 25 deg threshold stopped a normal fast turn on the real head.)
                err = max(abs(obs[yk] - action[yk]), abs(obs[pk] - action[pk]))
                moved = max(abs(obs[yk] - prev_obs[yk]), abs(obs[pk] - prev_obs[pk])) if prev_obs else 99.0
                stall_s = stall_s + dt if (err > STALL_ERR and moved < 0.2) else 0.0
                prev_obs = obs
                if stall_s > STALL_S:
                    reason = f"servo stalled {err:.0f} deg off target for {STALL_S}s, stopped"
                    break
                if err > HARD_ERR:
                    reason = f"tracking error {err:.0f} deg > {HARD_ERR}, stopped"
                    break
                ticks += 1
                # never print from the control loop: a paused terminal (e.g. text selected) blocks print() and
                # stalled the loop for 5.4 s on 2026-10-20; the status thread prints the latest line instead
                status[0] = (f"t={tick - t0:5.1f}s yaw cmd {cmd[0]:+6.1f} / act {args.yaw_sign * py:+6.1f}   "
                             f"pitch cmd {cmd[1]:+6.1f} / act {args.pitch_sign * pp:+6.1f}   {state}")
                time.sleep(max(0.0, dt - (time.perf_counter() - tick)))
        except KeyboardInterrupt:
            reason = "stopped by user"
    stop.set()
    print(f"\n{reason}. ticks={ticks} non-ok ticks={stale_ticks} clamped ticks={clamp_ticks} samples received={pose.count}")
    print(f"log: {log_path}")
    print("Returning to centre...")
    for _ in range(int(5 * args.hz)):
        if max(abs(c) for c in cmd) < 0.05 and max(abs(v) for v in vel) < 1:
            break
        shape([0.0, 0.0])
        action = dict(hold)
        action[yk] = centre[yk] + args.yaw_sign * cmd[0]
        action[pk] = centre[pk] + args.pitch_sign * cmd[1]
        robot.send_action(action)
        time.sleep(dt)
    robot.send_action(dict(hold))
    time.sleep(0.3)
    if args.rig == "head":  # light head: release at once so no process lingers holding bus1 (2026-10-20)
        print("releasing head torque (the head may sag a little)")
    else:
        input(robot.release_msg)
    robot.disconnect()
    print("torque off, disconnected")


if __name__ == "__main__":
    main()
