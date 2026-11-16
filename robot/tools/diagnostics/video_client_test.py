"""Loopback test client for video_daemon.py that follows the Quest app's signalling order exactly
(welcome -> setPeerStatus(listener) + list -> startSession -> answer the producer's offer, trickle ICE).
Receives frames for DURATION s and reports codec, resolution and fps. Usage: video_client_test.py [host] [s]"""
import asyncio, json, sys, time
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription

HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 6
SAVE = sys.argv[3] if len(sys.argv) > 3 else None  # optional: save one downscaled frame (layout check)


async def main():
    pc, frames, t_first = RTCPeerConnection(), [], None
    done = asyncio.Event()

    @pc.on("track")
    def on_track(track):
        async def pull():
            nonlocal t_first
            while not done.is_set():
                f = await track.recv()
                if SAVE and len(frames) == 60:
                    import cv2
                    cv2.imwrite(SAVE, cv2.resize(f.to_ndarray(format="bgr24"), (480, 270)))
                frames.append((time.perf_counter(), f.width, f.height))
                t_first = t_first or time.perf_counter()
        asyncio.ensure_future(pull())

    async with websockets.connect(f"ws://{HOST}:8443") as ws:
        sid = None
        t0 = time.perf_counter()
        async for raw in ws:
            m = json.loads(raw)
            if m["type"] == "welcome":
                await ws.send(json.dumps({"type": "setPeerStatus", "roles": ["listener"], "meta": {"name": "test"}}))
                await ws.send(json.dumps({"type": "list"}))
            elif m["type"] == "list":
                prod = next(p for p in m["producers"] if p["meta"]["name"] == "reachymini")
                await ws.send(json.dumps({"type": "startSession", "peerId": prod["id"]}))
            elif m["type"] == "sessionStarted":
                sid = m["sessionId"]
            elif m["type"] == "peer" and "sdp" in m:
                await pc.setRemoteDescription(RTCSessionDescription(**m["sdp"]))
                await pc.setLocalDescription(await pc.createAnswer())
                await ws.send(json.dumps({"type": "peer", "sessionId": sid,
                                          "sdp": {"type": "answer", "sdp": pc.localDescription.sdp}}))
                codec = [l for l in pc.localDescription.sdp.splitlines() if l.startswith("a=rtpmap:")][0]
                print("answer sent, first codec:", codec, flush=True)
                break
        await asyncio.sleep(DUR)
        done.set()
        await ws.send(json.dumps({"type": "endSession", "sessionId": sid}))
    await pc.close()
    if len(frames) < 2:
        print("NO VIDEO received", len(frames)); return
    span = frames[-1][0] - frames[0][0]
    print(f"first frame {(t_first - t0) * 1000:.0f} ms after connect; {len(frames)} frames, "
          f"{(len(frames) - 1) / span:.1f} fps, size {frames[-1][1]}x{frames[-1][2]}")

asyncio.run(main())
