#!/usr/bin/env bash
source ~/fmp_scripts/xlerobot_ports.sh
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python ~/fmp_scripts/power_check.py "${1:-$XLR_PORT1}" "${2:-1,2,3,4,5,6,7,8}" "${3:-5}"
