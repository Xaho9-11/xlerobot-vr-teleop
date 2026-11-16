#!/usr/bin/env bash
source ~/fmp_scripts/xlerobot_ports.sh
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/fmp_scripts && python head_stiction_test.py
