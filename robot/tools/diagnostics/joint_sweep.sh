#!/usr/bin/env bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python ~/fmp_scripts/joint_sweep.py "$@"
