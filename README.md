# XLeRobot VR Teleoperation and Demonstration Recording

A complete teleoperation and data-collection stack for the [XLeRobot](https://github.com/Vector-Wangel/XLeRobot)
(two SO-101 arms and a 2-DoF camera head on an IKEA RÅSKOG cart with a three-omniwheel base). An operator in a
Meta Quest 3 drives both arms and the head; the system records synchronised robot, headset and camera data,
detects study events, and exports demonstrations to [LeRobot](https://github.com/huggingface/lerobot) for policy
learning (ACT).

It is the research platform of my MSc Final Master Project (Industrial Design, TU Eindhoven), which investigates
how non-experts can teach a robot their preferred way of performing a task through first-person VR teleoperation,
and how these demonstrations can be encoded into reusable behaviour patterns. It is a research prototype: it runs
every day on one robot, but it has not been packaged as a product.

## What it does

| | |
|---|---|
| **VR teleoperation** | Joint-correspondence arm mapping (the operator's upper arm, elbow and wrist map onto the robot's joints, so a stretched arm gives a stretched robot arm), clutch on the grip, gripper on the trigger, head follows the headset. One-gesture body calibration. ~50 Hz control loop. |
| **Headset view** | Head camera with both wrist cameras as picture-in-picture, streamed over WebRTC (H.264) to a patched Quest app; a minimal HUD for calibration prompts, recording state and confirmations. |
| **Robot core** | One process owns both servo buses: mode machine (idle → hold → ready → teleop → fold), smooth glides, fixed fold pose, joint limits, tracking-error stop, software e-stop. |
| **Console** | Browser UI with a live digital twin (URDF + MJCF in three.js), joint telemetry, recording controls, playback of recordings in the twin with synchronised videos, post-hoc event marking, export and training jobs. |
| **Recording** | Lossless research record per episode: robot state at the control rate, every headset message, events, and four cameras (head, two wrists, third-person) with per-frame capture times on one clock. |
| **Events** | Automatic study events, e.g. *arrived above the target* from forward kinematics and a calibrated zone, with hysteresis and dwell so edge jitter cannot create duplicates; manual and post-hoc marks kept separate from the raw record. |
| **Phone remote** | Large e-stop and long-press recording control for the experimenter's seat. |
| **Learning** | Export to LeRobot datasets; ACT training on the robot laptop with a live training page; offline evaluation of predicted vs. recorded actions. |

## Architecture

```
 Meta Quest 3 ──pose/buttons (WebSocket :8000)──▶ robot core ──▶ 2 × STS3215 servo buses (arms, head)
      ▲                                              │  ▲
      └──── WebRTC video + HUD (:8443) ◀── video daemon │  └── commands ◀── console server (:8080) ◀── browser / phone
                 ▲  cameras ×4                        └── state (:8770, local) ──▶ console server
                 └── recording control (:8771, local) ◀── robot core
 training page (:8090) ◀── LeRobot ACT runs (background jobs started from the console)
```

All processes run on the robot's onboard laptop (Ubuntu 24.04, RTX 3060 Laptop) as systemd user services and
start on boot. Details: [docs/architecture.md](docs/architecture.md).

## Repository layout

```
robot/              everything that runs on the robot laptop (deploy as ~/fmp_scripts)
  teleop.py         arm teleoperation pipeline (mapping, calibration, safety)
  head_follow.py    headset pose server (Reachy-Mini-daemon compatible) and head follow
  video_daemon.py   cameras, picture-in-picture, WebRTC, recording writers
  hud.py            headset HUD
  console/          robot core, recorder, console server, export, jobs, target zone, web UI, systemd units
  act_test/         ACT training runs, offline evaluation, training page
  tools/diagnostics bring-up and measurement scripts used while building the system
quest/              patch for the Quest app (HRI-EU/reachy-vr-quest) and a PC helper to control the headset over ADB
docs/               architecture, set-up, recording format, hardware notes
```

## Getting started

See [docs/setup.md](docs/setup.md). In short: assemble and calibrate the XLeRobot following its own
documentation, install LeRobot, copy `robot/` to `~/fmp_scripts`, create `xlerobot_ports.sh` from the example,
fetch the web libraries, enable the four services, and build the Quest app with the patch in `quest/`.

## Data and privacy

The repository contains code only. No recordings, datasets, logs or participant data are included; the recorder
writes them to the robot laptop (`~/fmp_data`) and they never leave the local network unless you move them.

## Status and limitations

- The e-stop in the console and the phone page is a **software** stop over Wi-Fi; add a hardware power switch for
  anything beyond supervised lab use.
- Calibrations, joint limits, camera ports and the twin's mounting offsets are specific to my robot; the files
  ending in `.example` show the format.
- User-facing text in the console and HUD is in Chinese (the language of the operators); code and documentation
  are in English.

## Acknowledgements

- [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) hardware design and simulation assets (URDF/MJCF).
- [LeRobot](https://github.com/huggingface/lerobot) for servo drivers, datasets and ACT.
- The Quest app is [HRI-EU/reachy-vr-quest](https://github.com/HRI-EU/reachy-vr-quest) (MIT, Honda Research
  Institute Europe) with the changes in `quest/`.
- [three.js](https://threejs.org) and [urdf-loader](https://github.com/gkjohnson/urdf-loaders) for the digital twin.

See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Developed with the help of AI coding assistants.

## License

MIT, see [LICENSE](LICENSE).
