#!/usr/bin/env bash
source ~/fmp_scripts/xlerobot_ports.sh
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python ~/fmp_scripts/torque_off.py
