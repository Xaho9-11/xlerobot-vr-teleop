# Set-up

This describes how my robot is set up. Paths assume the onboard laptop user's home directory.

## 1. Robot and software

1. Build and calibrate the XLeRobot following its documentation, and clone it to `~/code/XLeRobot` (the console
   serves the URDF and MJCF from there).
2. Install LeRobot in a conda environment named `lerobot` (`~/miniforge3/envs/lerobot`).
   Extra Python packages used here: `aiohttp`, `websockets`, `aiortc`, `av`, `opencv-python`, `pillow`, `numpy`.
3. Copy `robot/` to `~/fmp_scripts`.
4. Create `~/fmp_scripts/xlerobot_ports.sh` from `xlerobot_ports.example.sh`: the two servo buses by
   `/dev/serial/by-id`, the cameras by `/dev/v4l/by-path` (the three robot cameras share one serial number, so the
   USB port is the only way to tell them apart; see [hardware-notes.md](hardware-notes.md)).
5. Calibrate (`calibrate_xlerobot.sh`), then generate the joint limits: `python safe_limits.py` writes
   `joint_limits.json` (format in `joint_limits.example.json`). `rerecord_range.sh` re-records one servo's range if
   the first calibration was too conservative.
6. Fetch the web libraries: `bash console/web/vendor/fetch_vendor.sh`.

## 2. Services

```bash
mkdir -p ~/.config/systemd/user
cp ~/fmp_scripts/console/systemd/*.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now fmp-core fmp-console fmp-video fmp-train-dash
sudo loginctl enable-linger $USER          # keep them running without a desktop login
```

Logs go to `~/fmp_logs/`. Restart one part with `systemctl --user restart <service>`; stopping the core folds the
arms before releasing torque.

## 3. Quest app

See [../quest/README.md](../quest/README.md): build HRI-EU/reachy-vr-quest with the patch, sideload it, and set the
robot address once with `quest.ps1 ip <address>`. The app then connects by itself on start and after drops.

## 4. Use

- Console: `http://<robot>:8080` — torque on → unfold → start teleoperation → stop (folds and releases).
- Phone remote: `http://<robot>:8080/m` — e-stop, long-press recording.
- Training page: `http://<robot>:8090`.
- In the headset: stretch the arm forward and click the thumbstick to calibrate, hold the grip to move the arm,
  trigger closes the gripper, X/A record the neutral grip, hold B for 0.8 s to start or stop a recording.
