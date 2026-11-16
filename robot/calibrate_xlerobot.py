"""Full XLeRobot calibration, same flow as `lerobot-calibrate --robot.type=xlerobot`, but the connect
step (which torques every motor once, without retry) is retried, since a single missed status packet
there aborted calibration on 2026-10-19. Ports come from xlerobot_ports.sh environment variables."""
import os, time
from lerobot_robot_xlerobot import XLerobot, XLerobotConfig

cfg = XLerobotConfig(id=os.environ["XLR_ID"], port1=os.environ["XLR_PORT1"], port2=os.environ["XLR_PORT2"])
robot = XLerobot(cfg)
for attempt in range(1, 4):
    try:
        robot.connect(calibrate=False)
        break
    except ConnectionError as exc:
        print(f"connect attempt {attempt} failed: {exc}", flush=True)
        try:
            robot.disconnect()
        except Exception:
            for bus in (robot.bus1, robot.bus2):
                if bus.is_connected:
                    bus.disconnect(disable_torque=True)
        time.sleep(0.5)
else:
    raise SystemExit("could not connect after 3 attempts; check 12 V power (power bank auto-off?)")
try:
    robot.calibrate()
finally:
    input("HOLD BOTH ARMS (torque will be released), then press ENTER...")
    robot.disconnect()
    print("calibration file:", robot.calibration_fpath)
