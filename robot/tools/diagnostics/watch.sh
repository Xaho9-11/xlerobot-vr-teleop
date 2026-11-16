#!/usr/bin/env bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python ~/fmp_scripts/watch_bus.py "${1:-/dev/ttyACM0}" "${2:-1,2,3,4,5,6}"
