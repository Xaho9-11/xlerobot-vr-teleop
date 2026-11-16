"""Open each XLeRobot camera the way video_daemon.py does, by its stable by-path link and by the real
/dev/videoN node, and report backend / first frame brightness. Usage: cam_open_check.py"""
import os, sys, time
sys.argv = sys.argv[:1]
import video_daemon as vd

for var in ("XLR_CAM_HEAD", "XLR_CAM_LEFT_WRIST", "XLR_CAM_RIGHT_WRIST"):
    link = os.environ[var]
    for label, p in (("by-path", link), ("realpath", os.path.realpath(link))):
        c = vd.LatestFrameCamera(p, 640, 480, 30)
        time.sleep(1.2)
        fr = c.frame
        backend = c.cap.getBackendName() if c.ok else "-"
        shot = "no frame" if fr is None else f"{fr.shape[1]}x{fr.shape[0]} mean {float(fr.mean()):.0f}"
        print(f"{var:20s} {label:8s} opened={c.ok!s:5s} backend={backend:8s} {shot}", flush=True)
        c.stop()
