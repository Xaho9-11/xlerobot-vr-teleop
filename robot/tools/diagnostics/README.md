# Bring-up and diagnostic scripts

Small tools written while building the system, kept because they document how problems were found. They were run
from the flat `~/fmp_scripts` folder; to run one from here, use `PYTHONPATH=~/fmp_scripts python <script>`
(their `.sh` wrappers expect the script next to the runtime modules).

| Area | Scripts |
|---|---|
| Servo buses | `scan_bus`, `bus_ping`, `watch_bus`, `comm_test`, `power_check`, `joint_sweep`, `full_sweep`, `motion_test`, `p_compare` |
| Arm mapping | `arm_direction_check`, `sim_arm_mapping` |
| Head follow | `analyze_head_follow`, `head_range_check`, `head_stiction_test`, `replay_follower`, `replay_ff`, `replay_shaping`, `replay_stillness`, `stop_trace` |
| Base | `base_frame`, `base_test` |
| Cameras and video | `cam_probe`, `cam_identify`, `cam_multi`, `cam_open_check`, `cam_fps`, `cam_latency`, `cam_view`, `cam_bus_test` (USB bandwidth with several cameras), `video_client_test` |
| HUD | `hud_preview` (renders HUD mock-ups) |
