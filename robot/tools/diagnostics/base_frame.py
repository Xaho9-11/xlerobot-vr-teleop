"""Base velocity frame correction for this XLeRobot build.

Verified 2026-10-19 (base_test.py, user observation from the robot's own viewpoint): XLeRobot's wheel
naming is rotated 180 deg relative to where our head/arms face (base_back_wheel ID8 is our FRONT wheel,
base_left_wheel ID7 our right-rear, base_right_wheel ID9 our left-rear). So XLeRobot +x drives the robot
backward and +y to its right, while rotation sense is unchanged (+theta = turn left / CCW from above).

Controllers think in the HEAD frame (x forward = where the head camera looks, y left, theta CCW) and
convert here before calling XLerobot.send_action / _body_to_wheel_raw. XLeRobot code is not modified.
"""


def head_to_xlerobot(x_fwd: float, y_left: float, theta_ccw_deg: float) -> dict:
    """Head-frame body velocity (m/s, m/s, deg/s) -> XLeRobot action keys."""
    return {"x.vel": -x_fwd, "y.vel": -y_left, "theta.vel": theta_ccw_deg}


def xlerobot_to_head(obs: dict) -> tuple:
    """XLeRobot observation base velocities -> head frame (x forward, y left, theta CCW)."""
    return -obs.get("x.vel", 0.0), -obs.get("y.vel", 0.0), obs.get("theta.vel", 0.0)
