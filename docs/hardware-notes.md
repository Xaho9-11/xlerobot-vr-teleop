# Hardware notes

Lessons from running this robot daily; most of them cost an afternoon to find.

## Compute and power

- Onboard laptop: Lenovo Legion (Ryzen, RTX 3060 Laptop 6 GB), Ubuntu 24.04. System sleep is disabled
  (`systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target`): deep sleep drops Wi-Fi.
- The servo boards need 12 V. A power bank that switches itself off when idle silently removes the buses; the core
  then reports "no bus" and reconnects by itself once power is back.
- After a power loss, re-plug USB at the **driver board** end if the bus does not come back.

## Servos

- Two Feetech STS3215 buses through CH343 USB-serial adapters: bus 1 = left arm + head, bus 2 = right arm + base.
- Positions are handled in *pose degrees* (raw − 2047, 0 = calibration pose); LeRobot's degrees are centred on the
  middle of each calibrated range, conversions are in `safe_limits.py`.

## Cameras and USB bandwidth

- All four cameras are USB 2.0 UVC devices (head, two wrists, third-person DJI Pocket 3 in its UVC mode).
- Cheap UVC cameras reserve full isochronous bandwidth when they start streaming, whatever their MJPEG bitrate, so
  **one USB 2.0 bus carries at most two of them**; a third gets no frames (`Not enough bandwidth` in the kernel log).
- The laptop has two USB 2.0 buses. Layout that works (all four at 30 fps together): head + third-person on the
  controller behind the laptop's side ports; both wrist cameras on the robot's hub.
- The three robot cameras report the same serial number: identify them by USB port (`/dev/v4l/by-path`) and do not
  swap plugs.
- `error -71` / "Cannot enable. Maybe the USB cable is bad?" on a port means a flaky cable or connector; a reboot
  cleared a port that had given up after repeated resets.

## Network

- Lab set-up: a portable router (Cudy TR3000) in router mode, internet from a phone over USB tethering. The
  teleoperation and video traffic stay on the router's LAN; only the PC's tools use the uplink.
- If the router loses power or restarts, Wi-Fi clients may join another known network; give the lab network a
  higher NetworkManager priority and unlimited retries on the robot laptop.
- Reserve fixed addresses for the robot laptop and the Quest in the router.
- On 5 GHz the robot laptop's Wi-Fi measured ~6 ms to the router; lock its connection to 5 GHz.
