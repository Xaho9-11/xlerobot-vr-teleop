# Recording format

The recorder (`console/recorder.py`) writes a lossless research record. Nothing is smoothed, trimmed or resampled;
learning datasets are exported from it separately, so the raw record never depends on a training format.

```
~/fmp_data/sessions/<YYYYMMDD>_<participant>/<NNN>_<scenario>_v<version>/
    meta.json          episode information and labels
    robot.csv          one row per control tick (~50 Hz)
    vr.jsonl           every headset message as received
    events.jsonl       automatic and manual events, written live, never rewritten
    annotations.json   post-hoc marks made in playback (optional)
    cam_<name>.mp4     one video per camera: head, left, right (wrists), scene (third person)
    cam_<name>.csv     frame index and capture time of every video frame
```

All times are wall-clock seconds (Unix time) of the robot laptop, so every file can be aligned without offsets.

## meta.json

Session fields (`participant`, `scenario`, `style`, `operator`, `note`), `episode`, `version`, `t_start`, `t_end`,
selected arms, robot mode at start, control rate, the teleoperation mapping parameters, the target zone in use,
per-camera frame counts, and labels: `final`, `repeat_of_final`, `discarded`, `reason`. Every edit made in the
console is appended to `edit_log` with the old and new value. Episodes are never deleted, only moved to a trash
folder that can be restored.

## robot.csv

`t`, `mode`, then for each of the 14 joints (`head_motor_1/2`, `<side>_arm_<joint>`) the measured position, the
commanded position and the torque flag (`.pos`, `.goal`, `.torque`), then `left_engaged`, `right_engaged`
(clutch held) and `head_state`. Angles are in *pose degrees* (0 = calibration pose); grippers in percent.

## events.jsonl

One JSON object per line: `t`, `kind`, `label`, `source` (`auto` or `researcher`) and extra fields. Automatic
kinds: `episode_start`, `motion_start`, `gripper_open`, `back_at_ready`, `target_arrive` (first entry into the
target zone), `target_reenter`, `target_leave`, `episode_end`. The zone events carry the distance from the zone
centre and the height above the tray at the moment of entry; `console/zone.py EPISODE_DIR` recomputes them
offline with other parameters.

## annotations.json

Marks set afterwards on the videos (for example *receiver picks up the object*), one per kind, plus a log of every
set, move and delete. Kept apart from `events.jsonl` so the live record stays untouched.

## LeRobot export

`console/export_lerobot.py` builds a LeRobot dataset from selected episodes: a uniform timeline (default 30 fps)
over the span where all chosen cameras have frames, `observation.state` = measured joints, `action` = commanded
joints (chosen arms plus the head), images resized (head 640×360, wrists 640×480), task string from scenario and
style. `export_manifest.json` lists the source episodes so each training run traces back to raw data.
