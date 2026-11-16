#!/usr/bin/env bash
# Interactive full XLeRobot calibration (left arm + head on bus1, right arm on bus2; wheels need none).
source ~/fmp_scripts/xlerobot_ports.sh
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python ~/fmp_scripts/calibrate_xlerobot.py
