"""Render HUD v2 mock-ups for review (2026-11-13). Background: a recorded head frame, heavily blurred (layout and
contrast only, no identifiable content), with the two wrist insets placed as video_daemon.CompositeTrack does.
Usage: hud_preview.py OUT_DIR"""
import glob, os, sys, time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hud import Hud  # noqa: E402

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
W, H = 1280, 720


def frame_from(pattern, size):
    for f in sorted(glob.glob(os.path.expanduser(pattern)), reverse=True):
        cap = cv2.VideoCapture(f)
        cap.set(cv2.CAP_PROP_POS_FRAMES, 40)
        ok, fr = cap.read()
        if ok:
            return cv2.GaussianBlur(cv2.resize(fr, size), (0, 0), 18)
    return np.full((size[1], size[0], 3), 60, np.uint8)


bg = frame_from("~/fmp_data/sessions/*/*/cam_head.mp4", (W, H))
wr = frame_from("~/fmp_data/sessions/*/*/cam_right.mp4", (640, 480))
iw = int(W * 0.28); ih = iw * 3 // 4; m = 10
for k in (0, 1):
    x0 = m if k == 0 else W - iw - m; y0 = H - ih - m
    bg[y0:y0 + ih, x0:x0 + iw] = cv2.resize(wr, (iw, ih))
    cv2.rectangle(bg, (x0 - 1, y0 - 1), (x0 + iw, y0 + ih), (150, 150, 150), 1)
    Hud().inset_label(bg, "左手" if k == 0 else "右手", x0, y0)

now = time.time()
states = {
    "1_calibrate": dict(v=2, center=dict(title="把左臂水平向前伸直", sub="然后按下左摇杆完成标定", level="info"),
                        status=[dict(text="左臂 · 握住握把开始移动", on=False)]),
    "2_teleop_recording": dict(v=2, rec=dict(start=now - 12, episode=3),
                               status=[dict(text="左臂 · 跟随中", on=True)]),
    "3_hold_b": dict(v=2, hold=dict(progress=0.6, action="开始录制"),
                     status=[dict(text="左臂 · 跟随中", on=True)]),
    "4_toast": dict(v=2, rec=dict(start=now - 1, episode=3), toast=dict(text="开始录制 · 第 3 条", until=now + 1.5),
                    status=[dict(text="左臂 · 跟随中", on=True)]),
    "5_warning": dict(v=2, center=dict(title="跟踪误差过大，已停止", sub="请等待研究者处理", level="warn"),
                      status=[dict(text="正在收回", on=False)]),
    "6_both_arms": dict(v=2, rec=dict(start=now - 75, episode=12),
                        status=[dict(text="左臂 · 跟随中", on=True), dict(text="右臂 · 握住握把开始移动", on=False)]),
}
hud = Hud()
tiles = []
for name, st in states.items():
    img = bg.copy()
    hud.draw(img, state=st, now=now)
    cv2.imwrite(os.path.join(out, f"hud_{name}.png"), img)
    tiles.append(cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA))
grid = np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)])
cv2.imwrite(os.path.join(out, "hud_overview.png"), grid)
print("written", sorted(os.listdir(out)))
