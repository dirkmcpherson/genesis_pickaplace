#!/bin/bash
# Stage 2c: merge shard manifests + census for one world (ddpv2_build.sh merge/census part). Usage: merge.sh <old|w3>
source /cluster/tufts/shortlab/jstale02/genesis_pickaplace/cluster/a31_chain/common.sh; STAGE=merge; world_cfg "${1:?world}"; cd "$GPR" || exit 1
export CUDA_VISIBLE_DEVICES= GENESIS_PICKAPLACE_ROOT=$GPR MUJOCO_GL=egl; activate
clog "merge start $OUTDIR (+_fails): npz $(ls $OUTDIR/*.npz 2>/dev/null | wc -l) / fails $(ls ${OUTDIR}_fails/*.npz 2>/dev/null | wc -l)"
set -e
for D in $SET ${SET}_fails; do python baselines/record_demos.py --teacher dp --merge --outdir baselines/demos_v2/$D; done
set +e
python analysis/characterize_demo_sets.py ${SET}=$OUTDIR ${SET}fails=${OUTDIR}_fails --out baselines/demos_v2/census_${SET}.md || clog "census FAILED (non-fatal)"
python3 - "$OUTDIR/manifest.json" <<'PY' | tee -a "$CHAIN_LOG"
import json, sys; m = json.load(open(sys.argv[1]))
print('MERGE-RESULT', sys.argv[1], 'n_kept=', m.get('n_kept'), 'rollouts=', m.get('rollouts', m.get('n_rollouts')), 'records_complete=', m.get('records_complete'), 'teacher_rate=', m.get('teacher_rate', m.get('teacher_success_rate')))
PY
clog "merge done"
