# Third-party notices

This repository contains original code under the MIT licence (see `LICENSE`). It builds on the following projects,
which are **not** included here and keep their own licences:

| Project | Used for | Licence | How it is used |
|---|---|---|---|
| [HRI-EU/reachy-vr-quest](https://github.com/HRI-EU/reachy-vr-quest) | Quest 3 teleoperation app | MIT, © Honda Research Institute Europe GmbH | `quest/reachy-vr-quest-fmp.patch` contains changes to it; apply the patch to your own clone. |
| [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) | Robot design, URDF and MJCF assets | see the XLeRobot repository | Assets are served from a local clone by the console. |
| [LeRobot](https://github.com/huggingface/lerobot) | Servo drivers, calibration, datasets, ACT | Apache-2.0 | Installed as a Python dependency. |
| [three.js](https://github.com/mrdoob/three.js) r160 | Digital twin rendering | MIT | Downloaded by `robot/console/web/vendor/fetch_vendor.sh`. |
| [urdf-loader](https://github.com/gkjohnson/urdf-loaders) 0.12.3 | URDF loading in the browser | Apache-2.0 | Downloaded by the same script. |
| aiohttp, websockets, aiortc, PyAV, OpenCV, Pillow, NumPy | Servers, WebRTC, video, HUD | their respective licences | Python dependencies. |
| Noto Sans CJK | HUD font | SIL Open Font License | System font on Ubuntu (`fonts-noto-cjk`). |
