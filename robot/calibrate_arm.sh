#!/usr/bin/env bash
# Interactive SO101 follower calibration for a single arm (standalone test before full XLeRobot calibration)
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
lerobot-calibrate --robot.type=so101_follower --robot.port="${1:-/dev/ttyACM0}" --robot.id="${2:-left_arm_test}"
echo "== calibration files"
find ~/.cache/huggingface/lerobot/calibration -name '*.json' -newer ~/fmp_scripts/calibrate_arm.sh -print 2>/dev/null
