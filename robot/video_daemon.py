"""Head-camera video for the reachy-vr-quest Quest app, pretending to be the Reachy Mini daemon's
GStreamer WebRTC producer.

The Quest app (Assets/Scripts/Runtime/Transport/WebRTCClient.cs) speaks the gst-plugins-rs
webrtcsink signalling protocol over plain ws://<host>:8443:
  server -> {"type":"welcome","peerId"}         client -> setPeerStatus(listener), list
  server -> {"type":"list","producers":[{id, meta:{name:"reachymini"}}]}
  client -> {"type":"startSession","peerId":<producer>}   server -> {"type":"sessionStarted",...}
  server -> {"type":"peer","sessionId","sdp":{"type":"offer",...}}   (the PRODUCER makes the offer)
  client -> {"type":"peer","sessionId","sdp":{"type":"answer",...}} and trickled {"ice":{...}}
This one process is both the signalling server and the producer (aiortc), so no GStreamer is needed.
The camera is opened once and shared (MediaRelay); H.264 is preferred so the Quest can decode in hardware.

Usage (via video_daemon.sh): video_daemon.py [--device PATH] [--size 1280x720] [--fps 30] [--port 8443]
"""
import argparse
import asyncio
import json
import os
import threading
import time
import uuid

import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.codecs import h264
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender
from aiortc.sdp import candidate_from_sdp

PRODUCER_ID = "xlerobot-head-camera"
PRODUCER_META = {"name": "reachymini"}  # the Quest app selects the producer by this name


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


class LatestFrameCamera:
    """OpenCV capture thread that keeps only the newest frame (no queue, so no added latency).
    Self-healing (2026-11-13: during teleop the left wrist camera dropped off USB and came back, and the head camera
    stopped delivering; both stayed black until the daemon was restarted): when no frame has arrived for STALL_S,
    the device is closed and reopened by its by-path link (which follows a re-enumerated device), retried every
    STALL_S until frames come back. The recording sink stays attached, so a recording continues on its own; the
    gap shows in cam_<name>.csv."""
    STALL_S = 2.0

    def __init__(self, device, width, height, fps, name=""):
        import threading
        self.device, self.width, self.height, self.fps, self.name = device, width, height, fps, name or device
        self.reopens = 0
        self.cap = self._open()
        self.frame, self.stamp, self.ok = None, 0.0, self.cap.isOpened()
        self.sink = None  # recording: called with (frame, wall time) for EVERY captured frame
        if not self.ok:  # 2026-10-20: a silent failure here just streamed black frames
            import os
            denied = os.path.exists(device) and not os.access(os.path.realpath(device), os.R_OK | os.W_OK)
            why = ("permission denied (user not in the video group and not logged in at the desktop?)"
                   if denied else "cannot open")
            log(f"CAMERA ERROR {device}: {why}")
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _open(self):
        import cv2
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        return cap

    def _loop(self):
        last_ok = last_try = time.perf_counter()
        while self._run:
            ok, fr = self.cap.read() if self.cap is not None else (False, None)
            now = time.perf_counter()
            if ok:
                if self.reopens and now - last_ok > self.STALL_S:
                    log(f"camera {self.name}: frames back after {now - last_ok:.1f} s")
                last_ok = now
                self.frame, self.stamp = fr, now
                sink = self.sink
                if sink:
                    sink(fr, time.time())
                continue
            time.sleep(0.01)
            if now - last_ok > self.STALL_S and now - last_try > self.STALL_S and self._run:
                last_try = now
                log(f"camera {self.name}: no frame for {now - last_ok:.1f} s, reopening {self.device}")
                try:
                    if self.cap is not None:
                        self.cap.release()
                except Exception:  # noqa: BLE001 - a vanished device may fail to close; reopen anyway
                    pass
                time.sleep(0.3)
                self.cap = self._open()
                self.ok = self.cap.isOpened()
                self.reopens += 1
                if not self.ok:
                    log(f"camera {self.name}: reopen failed (device gone?), retrying")

    def stop(self):
        self._run = False
        time.sleep(0.05)
        self.cap.release()


class CamWriter:
    """Records one camera to <dir>/cam_<name>.mp4 (H.264, full captured resolution, variable frame rate: each
    frame's pts is its capture time) plus cam_<name>.csv (frame index, wall-clock capture time) - the same clock as
    the robot core's robot.csv/events.jsonl (2026-11-10 recording). Encoding runs in its own thread; if it falls
    behind, frames are dropped and counted rather than delaying the live stream."""

    def __init__(self, directory, name, fps=30):
        import queue
        self.q = queue.Queue(maxsize=60)
        self.path = os.path.join(directory, f"cam_{name}.mp4")
        self.csv = open(os.path.join(directory, f"cam_{name}.csv"), "w")
        self.csv.write("frame,t\n")
        self.fps, self.n, self.dropped, self.t0 = fps, 0, 0, None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def put(self, frame, t):
        try:
            self.q.put_nowait((frame, t))
        except Exception:
            self.dropped += 1

    def _run(self):
        import av
        from fractions import Fraction
        container, stream, last_pts = None, None, -1
        while True:
            item = self.q.get()
            if item is None:
                break
            frame, t = item
            if container is None:
                h, w = frame.shape[:2]
                container = av.open(self.path, "w")
                stream = container.add_stream("libx264", rate=self.fps)
                stream.width, stream.height, stream.pix_fmt = w, h, "yuv420p"
                stream.codec_context.time_base = Fraction(1, 1000)
                stream.options = {"preset": "veryfast", "crf": "21"}
                self.t0 = t
            # pts in ms must rise strictly: two frames read within the same millisecond (2026-11-13, scene camera
            # after a reboot) made the muxer fail with EINVAL and killed this thread, which hung rec_stop. The csv
            # keeps the true capture time; the video pts is only nudged by 1 ms when needed.
            pts = max(int(round((t - self.t0) * 1000)), last_pts + 1)
            last_pts = pts
            try:
                vf = av.VideoFrame.from_ndarray(frame, format="bgr24")
                vf.pts, vf.time_base = pts, Fraction(1, 1000)
                for p in stream.encode(vf):
                    container.mux(p)
            except Exception as exc:  # noqa: BLE001 - one bad frame must not end the recording of this camera
                self.dropped += 1
                log(f"recording {os.path.basename(self.path)}: frame skipped ({exc})")
                continue
            self.csv.write(f"{self.n},{t:.4f}\n"); self.n += 1
        if container is not None:
            for p in stream.encode():
                container.mux(p)
            container.close()
        self.csv.close()

    def close(self):
        self.q.put(None)
        self.thread.join(timeout=10)
        return dict(frames=self.n, dropped=self.dropped)


class HudOverlay:
    """Short prompts from teleop.py (/dev/shm/fmp_hud.json, user request 2026-10-21: the terminal cannot be
    read with the headset on), drawn top-centre on a dark band. Ignored when the file is older than 1.5 s
    (teleop not running). Rendered with Pillow + Noto Sans CJK (OpenCV fonts have no Chinese), re-rendered
    only when the text changes."""
    PATH = "/dev/shm/fmp_hud.json"
    FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

    def __init__(self, size=30):
        from PIL import ImageFont
        self.font = ImageFont.truetype(self.FONT, size)
        self.lines, self.patch, self.checked = (), None, 0.0

    def _read(self):
        now = time.perf_counter()
        if now - self.checked < 0.1:
            return
        self.checked = now
        try:
            with open(self.PATH) as fh:
                data = json.load(fh)
            lines = tuple(data.get("lines", ())) if time.time() - data.get("t", 0) < 1.5 else ()
        except (OSError, ValueError):
            lines = ()
        if lines != self.lines:
            self.lines, self.patch = lines, self._render(lines) if lines else None

    def _render(self, lines):
        from PIL import Image, ImageDraw
        import numpy as np
        pad, gap = 12, 6
        boxes = [self.font.getbbox(t) for t in lines]
        w = max(b[2] for b in boxes) + 2 * pad
        lh = max(b[3] for b in boxes)
        h = len(lines) * (lh + gap) - gap + 2 * pad
        im = Image.new("RGB", (w, h), (25, 25, 25))
        d = ImageDraw.Draw(im)
        for k, t in enumerate(lines):
            d.text((pad, pad + k * (lh + gap)), t, font=self.font, fill=(255, 255, 255))
        return np.asarray(im)[:, :, ::-1].copy()  # RGB -> BGR

    def draw(self, img):
        self._read()
        p = self.patch
        if p is None:
            return
        h, w = p.shape[:2]
        H, W = img.shape[:2]
        h, w = min(h, H), min(w, W)
        x0, y0 = (W - w) // 2, int(H * 0.06)
        roi = img[y0:y0 + h, x0:x0 + w]
        roi[:] = (roi * 0.25 + p[:h, :w] * 0.75).astype(roi.dtype)


class CompositeTrack:
    """Head camera with the two wrist cameras as picture-in-picture insets (left wrist bottom-left,
    right wrist bottom-right, as seen from the robot), composed on the robot so the Quest app is unchanged
    (user request 2026-10-20). Inset width = inset_frac * head width; a stale inset (> 0.5 s) is shown grey."""

    def __init__(self, head_dev, wrist_devs, size, fps, inset_frac=0.28, scene_dev=None, scene_size="1280x720"):
        from aiortc import VideoStreamTrack
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        w, h = (int(v) for v in size.split("x"))
        self.w, self.h, self.fps, self.inset_frac = w, h, fps, inset_frac
        self.head = LatestFrameCamera(head_dev, w, h, fps, name="head")
        self.wrists = [LatestFrameCamera(d, 640, 480, fps, name=n) if d else None
                       for d, n in zip(wrist_devs, ("left", "right"))]
        # third-person camera (2026-11-11, DJI Pocket 3 in UVC mode): recorded only, not part of the headset image
        sw, sh = (int(v) for v in scene_size.split("x"))
        self.scene = LatestFrameCamera(scene_dev, sw, sh, fps, name="scene") if scene_dev else None
        try:
            from hud import Hud  # HUD v2 (hud.py, 2026-11-13); HudOverlay above is the old one-line version
            self.hud = Hud()
        except Exception as exc:  # no Pillow / font: video still works, prompts only in the terminal
            log("HUD disabled:", exc)
            self.hud = None
        outer = self

        class _Track(VideoStreamTrack):
            async def recv(self):
                pts, time_base = await self.next_timestamp()
                import av
                frame = av.VideoFrame.from_ndarray(outer.compose(), format="bgr24")
                frame.pts, frame.time_base = pts, time_base
                return frame

        self.track = _Track()

    def compose(self):
        cv2, np = self.cv2, self.np
        base = self.head.frame
        img = np.zeros((self.h, self.w, 3), np.uint8) if base is None else base.copy()
        if img.shape[1] != self.w or img.shape[0] != self.h:
            img = cv2.resize(img, (self.w, self.h))
        iw = int(self.w * self.inset_frac); ih = iw * 3 // 4
        m = 10
        now = time.perf_counter()
        for k, cam in enumerate(self.wrists):
            if cam is None:
                continue
            x0 = m if k == 0 else self.w - iw - m
            y0 = self.h - ih - m
            fr = cam.frame
            if fr is None or now - cam.stamp > 0.5:
                inset = np.full((ih, iw, 3), 60, np.uint8)
            else:
                inset = cv2.resize(fr, (iw, ih), interpolation=cv2.INTER_AREA)
            img[y0:y0 + ih, x0:x0 + iw] = inset
            # hairline frame and a small label in the HUD's style (2026-11-13: the 2 px white frame and Hershey
            # "L"/"R" looked crude next to the new HUD)
            cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + iw, y0 + ih), (150, 150, 150), 1)
            if self.hud:
                self.hud.inset_label(img, "左手" if k == 0 else "右手", x0, y0)
        if self.hud:
            self.hud.draw(img)
        return img

    def cameras(self):
        """name -> camera for recording (the names become cam_<name>.mp4)."""
        return {k: c for k, c in (("head", self.head), ("left", self.wrists[0]), ("right", self.wrists[1]),
                                  ("scene", self.scene)) if c is not None}

    def stop(self):
        for cam in self.cameras().values():
            cam.stop()


class Daemon:
    def __init__(self, args):
        self.args = args
        self.player = None
        self.relay = MediaRelay()
        self.sessions = {}  # session_id -> (pc, websocket)
        self.writers = {}   # recording: camera name -> CamWriter

    def ensure_composite(self):
        if self.player is None:
            self.player = CompositeTrack(self.args.device, [self.args.left_wrist, self.args.right_wrist],
                                         self.args.size, self.args.fps, self.args.inset,
                                         scene_dev=self.args.scene, scene_size=self.args.scene_size)
            log(f"composite opened: head {self.args.device} {self.args.size}, wrists L={self.args.left_wrist} "
                f"R={self.args.right_wrist} 640x480, inset {self.args.inset:.0%} of width, "
                f"scene {self.args.scene or '-'} {self.args.scene_size if self.args.scene else ''}")
        return self.player

    def video_track(self):
        if self.args.pip:  # head + wrist insets composed here (one video track to the app)
            return self.relay.subscribe(self.ensure_composite().track, buffered=False)
        if self.player is None:
            w, h = self.args.size.split("x")
            self.player = MediaPlayer(self.args.device, format="v4l2", options={
                "video_size": f"{w}x{h}", "framerate": str(self.args.fps), "input_format": "mjpeg"})
            log(f"camera opened: {self.args.device} {self.args.size}@{self.args.fps} MJPEG")
        return self.relay.subscribe(self.player.video, buffered=False)

    async def send(self, ws, msg):
        await ws.send(json.dumps(msg))

    async def start_session(self, ws):
        sid = str(uuid.uuid4())
        await self.send(ws, {"type": "sessionStarted", "peerId": PRODUCER_ID, "sessionId": sid})
        pc = RTCPeerConnection()
        self.sessions[sid] = (pc, ws)

        @pc.on("connectionstatechange")
        async def _state():
            log(f"session {sid[:8]} connection: {pc.connectionState}")
            if pc.connectionState in ("failed", "closed"):
                await self.end_session(sid)

        transceiver = pc.addTransceiver(self.video_track(), direction="sendonly")
        caps = RTCRtpSender.getCapabilities("video").codecs
        prefer = [c for c in caps if c.mimeType.lower() == "video/h264"] + \
                 [c for c in caps if c.mimeType.lower() not in ("video/h264",)]
        transceiver.setCodecPreferences(prefer)
        await pc.setLocalDescription(await pc.createOffer())  # aiortc gathers all ICE candidates here
        await self.send(ws, {"type": "peer", "sessionId": sid,
                             "sdp": {"type": "offer", "sdp": pc.localDescription.sdp}})
        log(f"session {sid[:8]}: offer sent")

    async def end_session(self, sid):
        entry = self.sessions.pop(sid, None)
        if entry:
            await entry[0].close()
            log(f"session {sid[:8]} closed")
        if not self.sessions and self.player is not None and self.args.close_camera_when_idle and not self.writers:
            (self.player.stop() if self.args.pip else self.player.video.stop())
            self.player = None
            log("camera released (no sessions)")

    async def on_peer(self, msg):
        sid = msg.get("sessionId")
        if sid not in self.sessions:
            return
        pc = self.sessions[sid][0]
        if "sdp" in msg and msg["sdp"].get("type") == "answer":
            await pc.setRemoteDescription(RTCSessionDescription(sdp=msg["sdp"]["sdp"], type="answer"))
            codec = [l for l in msg["sdp"]["sdp"].splitlines() if l.startswith("a=rtpmap:")][:1]
            log(f"session {sid[:8]}: answer received {codec}")
        elif "ice" in msg:
            ice = msg["ice"]
            cand_str = (ice.get("candidate") or "").strip()
            if not cand_str:
                return
            if cand_str.startswith("candidate:"):
                cand_str = cand_str[len("candidate:"):]
            cand = candidate_from_sdp(cand_str)
            cand.sdpMid = ice.get("sdpMid")
            cand.sdpMLineIndex = ice.get("sdpMLineIndex", 0)
            await pc.addIceCandidate(cand)

    async def handle(self, ws):
        peer_id = str(uuid.uuid4())
        log(f"client connected {ws.remote_address[0]} as {peer_id[:8]}")
        await self.send(ws, {"type": "welcome", "peerId": peer_id})
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                t = msg.get("type")
                if t == "setPeerStatus":
                    log(f"{peer_id[:8]} roles={msg.get('roles')} meta={msg.get('meta')}")
                elif t == "list":
                    await self.send(ws, {"type": "list",
                                         "producers": [{"id": PRODUCER_ID, "meta": PRODUCER_META}]})
                elif t == "startSession":
                    if msg.get("peerId") == PRODUCER_ID:
                        await self.start_session(ws)
                elif t == "peer":
                    await self.on_peer(msg)
                elif t == "endSession":
                    await self.end_session(msg.get("sessionId"))
        finally:
            for sid, (_, w) in list(self.sessions.items()):
                if w is ws:
                    await self.end_session(sid)
            log(f"client {peer_id[:8]} disconnected")

    async def control(self, ws):
        """local recording channel for the robot core (127.0.0.1:8771): {"cmd":"rec_start","dir":...} /
        {"cmd":"rec_stop"}; answers with the cameras recorded and frame counts."""
        async for raw in ws:
            try:
                m = json.loads(raw)
                if m.get("cmd") == "rec_start":
                    if self.writers:
                        await self.stop_recording()
                    if not self.args.pip:
                        raise RuntimeError("recording needs the composite (pip) mode")
                    comp = self.ensure_composite()
                    for name, cam in comp.cameras().items():
                        if cam.ok:
                            self.writers[name] = w = CamWriter(m["dir"], name, self.args.fps)
                            cam.sink = w.put
                    log(f"recording {sorted(self.writers)} -> {m['dir']}")
                    await ws.send(json.dumps(dict(ok=True, cams=sorted(self.writers))))
                elif m.get("cmd") == "rec_stop":
                    await ws.send(json.dumps(dict(ok=True, stats=await self.stop_recording())))
                elif m.get("cmd") == "status":
                    await ws.send(json.dumps(dict(ok=True, recording=sorted(self.writers))))
            except Exception as exc:
                log("control error:", exc)
                await ws.send(json.dumps(dict(ok=False, error=str(exc))))

    async def stop_recording(self):
        comp, stats = self.player, {}
        if comp is not None and self.args.pip:
            for cam in comp.cameras().values():
                cam.sink = None
        for name, w in self.writers.items():
            stats[name] = await asyncio.get_running_loop().run_in_executor(None, w.close)
        self.writers = {}
        log("recording stopped", stats)
        return stats

    async def run(self):
        await websockets.serve(self.control, "127.0.0.1", 8771)
        async with websockets.serve(self.handle, "0.0.0.0", self.args.port):
            log(f"signalling on ws://0.0.0.0:{self.args.port}  producer '{PRODUCER_META['name']}'  "
                f"camera {self.args.device}  H.264 start {h264.DEFAULT_BITRATE / 1e6:.1f} Mbps, "
                f"max {h264.MAX_BITRATE / 1e6:.1f} Mbps")
            await asyncio.Future()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", required=True)
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--port", type=int, default=8443)
    ap.add_argument("--bitrate", type=float, default=2.5, help="H.264 start bitrate, Mbps")
    ap.add_argument("--max-bitrate", type=float, default=5.0, help="H.264 max bitrate, Mbps")
    ap.add_argument("--close-camera-when-idle", action="store_true")
    ap.add_argument("--no-pip", dest="pip", action="store_false", help="head camera only (MediaPlayer path)")
    ap.add_argument("--left-wrist", default=None, help="left wrist camera device (bottom-left inset)")
    ap.add_argument("--right-wrist", default=None, help="right wrist camera device (bottom-right inset)")
    ap.add_argument("--inset", type=float, default=0.28, help="inset width as a fraction of the head image width")
    ap.add_argument("--scene", default=None, help="third-person camera device (recorded as cam_scene, not streamed)")
    ap.add_argument("--scene-size", default="1280x720")
    args = ap.parse_args()
    h264.DEFAULT_BITRATE = int(args.bitrate * 1e6)  # module constants read when an encoder is created
    h264.MAX_BITRATE = int(args.max_bitrate * 1e6)
    try:
        asyncio.run(Daemon(args).run())
    except KeyboardInterrupt:
        log("stopped")


if __name__ == "__main__":
    main()
