#!/usr/bin/env bash
# open the camera preview on the robot laptop's desktop (detached, survives the SSH session)
export DISPLAY="${DISPLAY_OVERRIDE:-$(ls /tmp/.X11-unix/ | sed -n "s/^X/:/p" | tail -1)}" XAUTHORITY=/run/user/1000/gdm/Xauthority
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
nohup python ~/fmp_scripts/cam_view.py "$@" > ~/fmp_logs/cam_view.out 2>&1 &
echo "preview started (pid $!) on $DISPLAY"
