#!/usr/bin/env bash
# note: do not pipe stderr through a filter here; Python writes input() prompts to stderr on a TTY
source ~/fmp_scripts/xlerobot_ports.sh
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
export OPENCV_LOG_LEVEL=ERROR
python ~/fmp_scripts/full_sweep.py "$@"
