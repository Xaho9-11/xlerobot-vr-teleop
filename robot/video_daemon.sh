#!/usr/bin/env bash
# head camera + both wrist cameras as picture-in-picture (pass --no-pip for head only); third-person camera
# (XLR_CAM_SCENE, 2026-11-11) is recorded only
source ~/fmp_scripts/xlerobot_ports.sh
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python ~/fmp_scripts/video_daemon.py --device "$XLR_CAM_HEAD" --left-wrist "$XLR_CAM_LEFT_WRIST" --right-wrist "$XLR_CAM_RIGHT_WRIST" \
  ${XLR_CAM_SCENE:+--scene "$XLR_CAM_SCENE"} "$@"
