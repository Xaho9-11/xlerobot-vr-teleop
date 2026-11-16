# Architecture

Everything runs on the robot's onboard laptop. Four systemd user services start on boot
(`loginctl enable-linger` keeps them running without a desktop login):

| Service | Program | Ports | Role |
|---|---|---|---|
| `fmp-core` | `console/core.py` | Quest pose server `:8000`; console channel `127.0.0.1:8770` | The only process that opens the two servo buses. 50 Hz loop: reads all 14 arm/head joints every tick, runs the mode machine and teleoperation, writes the recording, publishes state (~30 Hz) and the HUD state. |
| `fmp-console` | `console/server.py` | `:8080` | Web console, phone remote (`/m`), REST API for recordings, annotations, export and training jobs; relays commands to the core. |
| `fmp-video` | `video_daemon.py` | WebRTC signalling `:8443`; recording control `127.0.0.1:8771` | Opens the cameras, composes the headset image (head camera + wrist insets + HUD), streams it as H.264, and writes one video per camera while recording. |
| `fmp-train-dash` | `act_test/train_dashboard.py` | `:8090` | Live training page for ACT runs. |

## Robot core (`console/core.py`)

Mode machine, each transition from the console, the phone or the controllers:

```
idle (torque off) → hold (torque on, in place) → ready (glide to the calibration pose) → teleop → fold (glide to the fold pose, torque off) → idle
estop from any mode: all torque off at once
```

- **Glides** run inside the control loop, so an e-stop also works mid-move.
- **Fold pose** is a fixed file (`~/fmp_data/fold_pose.json`) set from the console; it no longer depends on where
  the arms were at torque-on.
- **Safety**: per-joint limits for both arms (`safe_limits.py`, from the calibration ranges), automatic fold when the
  tracking error exceeds 40°, release after repeated bus errors.
- **Teleoperation** reuses `teleop.py` (see below) and the head-follow controller from `head_follow.py`.

## Arm mapping (`teleop.py`)

Joint correspondence rather than end-effector following, so the robot's posture resembles the operator's:

- **Body frame** from a one-gesture calibration: the operator stretches the arm forward and clicks the thumbstick;
  the arm direction gives the torso heading, the hand distance the arm length.
- **Shoulder pan** = upper-arm azimuth in the body frame (1:1).
- **Shoulder lift** from upper-arm elevation through a piecewise-linear map (arm down → folded back, 45° → upper arm
  vertical, horizontal → fully forward).
- **Elbow** so that the forearm points at the operator's hand position scaled by the arm-length ratio.
- **Wrist flex/roll** from the controller orientation relative to a neutral grip recorded with X/A (avoids the
  gimbal singularity of absolute angles).
- **Clutch**: the arm follows only while the grip is held; the trigger closes the gripper.
- **Head**: yaw/pitch follow the headset with a small dead zone; recentre from the console.

## Video (`video_daemon.py`, `hud.py`)

- Cameras are opened by stable `/dev/v4l/by-path` links. A camera that delivers no frame for 2 s is reopened, so a
  USB reset (e.g. a cable pulled during arm motion) heals by itself; recordings continue on the same file.
- The headset image is 1280×720: head camera, wrist insets at the bottom corners, HUD on top. The HUD reads a small
  state file written by the core (`/dev/shm/fmp_hud.json`): a centre card only when the operator has to act, a
  recording badge, one status line and short confirmations. Research signals (e.g. the target zone) are not shown
  to the operator.
- Recording uses the raw camera frames (never the HUD), H.264 with variable frame rate; each frame's capture time is
  written to `cam_<name>.csv` on the same wall clock as the robot data.

## Console (`console/web/`)

- Digital twin: the XLeRobot URDF (arms, head, cart) on the MJCF omniwheel base, measured and commanded poses as two
  layers; joints highlight both ways between the list and the 3D model.
- Recording panel, target-zone set-up, fold-pose set-up.
- Data view: recordings grouped by session, playback in the twin with synchronised videos, event list, post-hoc
  marks, edit labels with an edit log, trash with restore, export to a LeRobot dataset, start training jobs.

## Network

All traffic stays on the local network. The robot laptop, the Quest, the operator's PC and the phone share one
router; the robot laptop needs internet only for installing software. See [hardware-notes.md](hardware-notes.md)
for the lab set-up (portable router with phone tethering).
