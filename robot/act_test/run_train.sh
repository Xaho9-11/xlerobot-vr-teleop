#!/usr/bin/env bash
# One LeRobot ACT training run with GPU sampling, for the training dashboard (2026-11-10; local datasets added for
# the console's train button). Usage: run_train.sh NAME STEPS BATCH [extra lerobot-train args...]
# Env: REPO (dataset name / hub id), ROOT (local dataset folder, optional), LABEL (dashboard text),
#      SAVE_FREQ (checkpoint every N steps; default only at the end).
# Writes ~/fmp_train/runs/NAME/{train.log,gpu.csv,meta.json,exit} and the checkpoint under out/.
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
NAME=$1; STEPS=$2; BATCH=$3; shift 3
REPO=${REPO:-lerobot/svla_so100_pickplace}
D=~/fmp_train/runs/$NAME
mkdir -p "$D"; rm -rf "$D/out" "$D/exit"
# json.dumps: extra args may contain JSON with quotes (2026-11-10: printf broke meta.json and the dashboard)
python -c 'import json,sys,time; n,l,s,b,r,root,e=sys.argv[1:8]; json.dump(dict(name=n,label=l,steps=int(s),batch=int(b),repo=r,root=root or None,extra=e,start=int(time.time())),open(sys.argv[8],"w"),ensure_ascii=False)' \
  "$NAME" "${LABEL:-}" "$STEPS" "$BATCH" "$REPO" "${ROOT:-}" "$*" "$D/meta.json"
nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$D/gpu.csv" &
SMI=$!
ROOT_ARG=(); [ -n "$ROOT" ] && ROOT_ARG=(--dataset.root="$ROOT")
lerobot-train --dataset.repo_id="$REPO" "${ROOT_ARG[@]}" --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --wandb.enable=false --output_dir="$D/out" --job_name="$NAME" --steps="$STEPS" --batch_size="$BATCH" \
  --log_freq=25 --save_freq="${SAVE_FREQ:-10000000}" "$@" > "$D/train.log" 2>&1
RC=$?
echo $RC > "$D/exit"
kill $SMI
exit $RC  # the console chains the offline check only after a successful run
