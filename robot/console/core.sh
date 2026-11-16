#!/usr/bin/env bash
# Robot core (owns both servo buses). Logs to ~/fmp_logs/core.log when started via restart_core.sh.
source ~/fmp_scripts/xlerobot_ports.sh
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/fmp_scripts/console && exec python core.py "$@"
