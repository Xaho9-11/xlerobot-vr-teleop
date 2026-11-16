#!/usr/bin/env bash
# run cam_latency.py on the robot laptop's own screen from an SSH session (the logged-in GNOME X11 session)
# desktop display of the logged-in user (GDM greeter is not running :0 here; the session is :1)
export DISPLAY="${DISPLAY_OVERRIDE:-$(ls /tmp/.X11-unix/ | sed -n "s/^X/:/p" | tail -1)}" XAUTHORITY=/run/user/1000/gdm/Xauthority
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
python ~/fmp_scripts/cam_latency.py "$@"
