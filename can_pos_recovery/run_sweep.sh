#!/bin/bash
# Per-trial process isolation for the batched placement sweep: a genesis solver hang on
# one trial (e.g. 245's 22h wedge, 2026-07-05) can no longer take down the whole run.
# Each trial gets its own python process and a hard timeout; hangs are recorded and
# skipped. Resumable: trials already in $OUT are skipped without paying gs.init.
PY=/home/james/workspace/genesis_sim2real/venv/bin/python
DIR=/home/james/workspace/genesis_pickaplace/can_pos_recovery
OUT=${SWEEP_OUT:-$DIR/sweep_v2.json}
LOG=${SWEEP_LOG:-$DIR/sweep_v2.log}
HANGS=${SWEEP_HANGS:-$DIR/sweep_v2_hangs.txt}
TIMEOUT=${SWEEP_TIMEOUT:-1500}
FLAGS=${SWEEP_FLAGS:-}

cd "$DIR" || exit 1
for u in "$@"; do
    if [ -f "$OUT" ] && $PY -c "import json,sys; sys.exit(0 if '$u' in json.load(open('$OUT')) else 1)" 2>/dev/null; then
        continue
    fi
    echo "=== trial $u $(date +%H:%M:%S) ===" >> "$LOG"
    timeout $TIMEOUT $PY -u batch_rescue.py --uids "$u" --out "$OUT" $FLAGS >> "$LOG" 2>&1
    rc=$?
    if [ $rc -eq 124 ]; then
        echo "$u" >> "$HANGS"
        echo "=== trial $u TIMED OUT (solver hang?) ===" >> "$LOG"
    fi
done
echo "sweep wrapper done: $(date)" >> "$LOG"
