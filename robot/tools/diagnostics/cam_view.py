"""Live preview of a camera in a window on the robot laptop's own screen. Esc / q / close window to quit.
Usage (via cam_view.sh so DISPLAY is set): cam_view.py [/dev/videoN] [width height]"""
import os, sys, time
import cv2
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/video2"
w, h = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else (1280, 720)
cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h); cap.set(cv2.CAP_PROP_FPS, 30)
pygame.init()
screen = pygame.display.set_mode((w, h), pygame.RESIZABLE)
pygame.display.set_caption(f"camera {dev} - Esc to close")
font = pygame.font.SysFont(None, 28)
n, t0, fps = 0, time.perf_counter(), 0.0
running = True
while running:
    for e in pygame.event.get():
        if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q)):
            running = False
    ok, fr = cap.read()
    if not ok:
        continue
    n += 1
    if time.perf_counter() - t0 >= 1:
        fps, n, t0 = n / (time.perf_counter() - t0), 0, time.perf_counter()
    surf = pygame.image.frombuffer(cv2.cvtColor(fr, cv2.COLOR_BGR2RGB).tobytes(), (fr.shape[1], fr.shape[0]), "RGB")
    screen.blit(pygame.transform.scale(surf, screen.get_size()), (0, 0))
    screen.blit(font.render(f"{dev} {fr.shape[1]}x{fr.shape[0]} {fps:.0f} fps", True, (255, 255, 0)), (10, 10))
    pygame.display.flip()
cap.release(); pygame.quit()
