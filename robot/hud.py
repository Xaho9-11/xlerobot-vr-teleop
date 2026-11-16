"""Headset HUD v2, drawn into the video the headset shows (2026-11-13; user: prompts were all in one line at the
top, easy to miss and bare; wanted it minimal and refined, no clutter). Replaces video_daemon.HudOverlay.

Reads /dev/shm/fmp_hud.json written by the robot core (core.py hud_write, format v2):
  center  {title, sub, level: info|warn}  a card in the middle, only when the operator has to act
  rec     {start, episode}                 recording badge, top right: pulsing dot, REC, elapsed time
  hold    {progress 0..1, action}          B long press in progress: a hairline under the badge fills up
  status  [{text, on}]                     one quiet line at the bottom centre, between the wrist insets
  toast   {text, until}                    a one-off confirmation above the status line; fades out
The older format {"lines": [...]} (teleop.py run on its own) is shown as a card.
Only the headset image carries the HUD; recordings take the camera frames before it is drawn.
Design: one typeface (Noto Sans CJK), two greys, one accent (the console's teal), red only for recording and
errors; dark translucent rounded surfaces with a hairline edge; everything rendered 2x and downsampled.
"""
import json, math, time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

PATH = "/dev/shm/fmp_hud.json"
FONT_R = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_B = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FACE = 2   # face index in the .ttc: Simplified Chinese
SS = 2     # supersampling for smooth edges
INK, INK2, INK3 = (238, 240, 243), (172, 178, 186), (120, 126, 134)
SURFACE = (14, 16, 20)
ACCENT, RED = (134, 211, 200), (255, 93, 79)


class Patch:
    """RGBA image prepared for blending onto a BGR frame."""

    def __init__(self, im):
        w, h = im.size[0] // SS, im.size[1] // SS
        arr = np.asarray(im.resize((w, h), Image.LANCZOS)).astype(np.float32) / 255.0
        self.bgr = arr[:, :, 2::-1].copy() * 255.0
        self.a = arr[:, :, 3:4].copy()
        self.w, self.h = w, h


def blit(img, p, x, y, alpha=1.0):
    H, W = img.shape[:2]
    x, y = int(round(x)), int(round(y))
    x0, y0, x1, y1 = max(0, x), max(0, y), min(W, x + p.w), min(H, y + p.h)
    if x1 <= x0 or y1 <= y0 or alpha <= 0:
        return
    a = p.a[y0 - y:y1 - y, x0 - x:x1 - x] * alpha
    roi = img[y0:y1, x0:x1].astype(np.float32)
    img[y0:y1, x0:x1] = (roi * (1 - a) + p.bgr[y0 - y:y1 - y, x0 - x:x1 - x] * a).astype(np.uint8)


class Hud:
    def __init__(self, scale=1.0):
        self.s = scale
        self._fonts, self._cache = {}, {}
        self.data, self.checked = None, 0.0

    # ------------------------------------------------------------------ primitives
    def font(self, px, bold=False):
        key = (px, bold)
        if key not in self._fonts:
            self._fonts[key] = ImageFont.truetype(FONT_B if bold else FONT_R, int(px * self.s * SS), index=FACE)
        return self._fonts[key]

    def u(self, v):  # design units (1280x720 frame) -> supersampled pixels
        return int(round(v * self.s * SS))

    def surface(self, w, h, radius, fill_alpha=0.66, edge=True):
        im = Image.new("RGBA", (w, h), (0, 0, 0, 0))
        d = ImageDraw.Draw(im)
        d.rounded_rectangle((0, 0, w - 1, h - 1), radius=radius, fill=SURFACE + (int(255 * fill_alpha),),
                            outline=(255, 255, 255, 26) if edge else None, width=max(1, self.u(1)))
        return im, d

    def cached(self, key, make):
        if key not in self._cache:
            if len(self._cache) > 64:
                self._cache.clear()
            self._cache[key] = make()
        return self._cache[key]

    # ------------------------------------------------------------------ components
    def card(self, title, sub, level):
        def make():
            ft, fs = self.font(30), self.font(19)
            px, py, gap, dot = self.u(34), self.u(24), self.u(10), self.u(9)
            tw = int(ft.getlength(title)); sw = int(fs.getlength(sub)) if sub else 0
            th, sh = self.u(40), self.u(26) if sub else 0
            w = px * 2 + dot + self.u(16) + max(tw, sw)
            h = py * 2 + th + (gap + sh if sub else 0)
            im, d = self.surface(w, h, self.u(18), fill_alpha=0.72)
            cx = px + dot // 2; cy = py + th // 2 + self.u(1)
            col = RED if level == "warn" else ACCENT
            d.ellipse((cx - dot // 2, cy - dot // 2, cx + dot // 2, cy + dot // 2), fill=col + (255,))
            tx = px + dot + self.u(16)
            d.text((tx, py + th // 2), title, font=ft, fill=INK + (255,), anchor="lm")
            if sub:
                d.text((tx, py + th + gap + sh // 2), sub, font=fs, fill=INK2 + (255,), anchor="lm")
            return Patch(im)
        return self.cached(("card", title, sub, level), make)

    def badge(self, clock):
        """REC badge without its dot (the dot pulses, drawn separately)."""
        def make():
            fb, fr = self.font(14, bold=True), self.font(18)
            h, px = self.u(38), self.u(16)
            rec_w = sum(fb.getlength(c) for c in "REC") + self.u(2.5) * 2
            w = px + self.u(8) + self.u(10) + int(rec_w) + self.u(12) + int(fr.getlength(clock)) + px
            im, d = self.surface(w, h, h // 2)
            x = px + self.u(8) + self.u(10)
            for c in "REC":  # letter-spaced
                d.text((x, h // 2), c, font=fb, fill=INK + (255,), anchor="lm")
                x += fb.getlength(c) + self.u(2.5)
            d.text((x + self.u(12) - self.u(2.5), h // 2), clock, font=fr, fill=INK + (255,), anchor="lm")
            return Patch(im)
        return self.cached(("badge", clock), make)

    def dot(self, d, color):
        def make():
            im = Image.new("RGBA", (d, d), (0, 0, 0, 0))
            ImageDraw.Draw(im).ellipse((0, 0, d - 1, d - 1), fill=color + (255,))
            return Patch(im)
        return self.cached(("dot", d, color), make)

    def pill(self, text, size=16, color=INK, fill_alpha=0.62, items=None):
        """a quiet pill: plain text, or status items [(text, on)] each with a small dot"""
        def make():
            f = self.font(size)
            h, px, gap, dd = self.u(size + 18), self.u(16), self.u(22), self.u(6)
            parts = items or [(text, None)]
            widths = [int(f.getlength(t)) + (dd + self.u(9) if on is not None else 0) for t, on in parts]
            w = px * 2 + sum(widths) + gap * (len(parts) - 1)
            im, d = self.surface(w, h, h // 2, fill_alpha=fill_alpha)
            x = px
            for (t, on), tw in zip(parts, widths):
                if on is not None:
                    c = ACCENT if on else INK3
                    cy = h // 2
                    d.ellipse((x, cy - dd // 2, x + dd, cy + dd // 2), fill=c + (255,))
                    x += dd + self.u(9)
                d.text((x, h // 2), t, font=f, fill=(color if on is None else (INK if on else INK2)) + (255,), anchor="lm")
                x += tw - (dd + self.u(9) if on is not None else 0) + gap
            return Patch(im)
        return self.cached(("pill", text, size, color, fill_alpha, tuple(items or ())), make)

    def inset_label(self, img, text, x, y):
        """small label in the top-left corner of a wrist inset (drawn by video_daemon.CompositeTrack)"""
        k = img.shape[1] / 1280.0
        if abs(k - self.s) > 1e-3:
            self.s, self._fonts, self._cache = k, {}, {}
        p = self.pill(text, size=13, fill_alpha=0.6)
        blit(img, p, x + 8 * k, y + 8 * k)

    # ------------------------------------------------------------------ state
    def read(self):
        now = time.perf_counter()
        if now - self.checked < 0.1:
            return self.data
        self.checked = now
        try:
            with open(PATH) as fh:
                d = json.load(fh)
            self.data = d if time.time() - d.get("t", 0) < 1.5 else None
        except (OSError, ValueError):
            self.data = None
        return self.data

    def draw(self, img, state=None, now=None):
        st = state if state is not None else self.read()
        if not st:
            return
        now = time.time() if now is None else now
        H, W = img.shape[:2]
        k = W / 1280.0
        if abs(k - self.s) > 1e-3:  # frame size changed: re-render at the new scale
            self.s, self._fonts, self._cache = k, {}, {}
        m = 24 * k
        if st.get("v") != 2:  # old format: lines as one card
            lines = st.get("lines") or []
            if lines:
                p = self.card(lines[0], " · ".join(lines[1:3]), "info")
                blit(img, p, (W - p.w) / 2, H * 0.08)
            return

        # bottom: status line, toast above it (both between the wrist insets)
        items = [(s.get("text", ""), bool(s.get("on"))) for s in (st.get("status") or []) if s.get("text")]
        status_y = H - m
        if items:
            p = self.pill("", size=17, fill_alpha=0.55, items=items)
            status_y = H - m - p.h
            blit(img, p, (W - p.w) / 2, status_y, 0.92)
        t = st.get("toast")
        if t and t.get("text"):
            left = t.get("until", now) - now
            if left > 0:
                p = self.pill(t["text"], size=20, fill_alpha=0.72)
                blit(img, p, (W - p.w) / 2, status_y - p.h - 12 * k, min(1.0, left / 0.4))

        # top right: recording badge, B long-press hairline
        top = m
        rec = st.get("rec")
        if rec:
            el = max(0, int(now - rec.get("start", now)))
            p = self.badge(f"{el // 60:02d}:{el % 60:02d}")
            x = W - m - p.w
            blit(img, p, x, top)
            dd = self.dot(int(round(11 * k)), RED)
            pulse = 0.6 + 0.4 * (0.5 + 0.5 * math.cos(2 * math.pi * now))
            blit(img, dd, x + 16 * k, top + (p.h - dd.h) / 2, pulse)
            top += p.h + 8 * k
        hold = st.get("hold")
        if hold:
            p = self.pill(f"按住 B · {hold.get('action', '')}", size=17, fill_alpha=0.66)
            x, y = W - m - p.w, top
            blit(img, p, x, y)
            # hairline progress along the bottom edge of the pill
            prog = max(0.0, min(1.0, float(hold.get("progress", 0))))
            x0, x1 = int(x + p.h / 2), int(x + p.w - p.h / 2)
            y0 = int(y + p.h - 4 * k)
            xe = int(x0 + (x1 - x0) * prog)
            col = np.array(RED[::-1] if "结束" in hold.get("action", "") else ACCENT[::-1], np.float32)
            if xe > x0:
                roi = img[y0:y0 + max(2, int(3 * k)), x0:xe].astype(np.float32)
                img[y0:y0 + max(2, int(3 * k)), x0:xe] = (roi * 0.1 + col * 0.9).astype(np.uint8)

        # centre: only when the operator has to act
        c = st.get("center")
        if c and c.get("title"):
            p = self.card(c["title"], c.get("sub", ""), c.get("level", "info"))
            blit(img, p, (W - p.w) / 2, H * 0.40 - p.h / 2)
