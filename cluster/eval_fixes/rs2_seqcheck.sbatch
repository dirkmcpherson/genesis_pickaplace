#!/usr/bin/env bash
# Does a FULL end-to-end cell reproduce in its own episode order? Re-runs dHfull_all s3 rnd30 MODE exactly as the record
# did (30 episodes, one process, same flags) and diffs per-episode outcome/steps/stages + the aggregate stage rates.
# This is the test for whether PHASE_RESULTS 5.1's published 8v8 aggregates stand.
#SBATCH -J rs2_seqcheck
#SBATCH -p preempt,batch
#SBATCH --qos=preempt
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 4
#SBATCH --mem=16g
#SBATCH -t 0-04:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -uo pipefail
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/r2d_venv/bin/python
export TORCHINDUCTOR_CACHE_DIR=$W/inductor_cache/${SLURM_JOB_ID} MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export GENESIS_PICKAPLACE_ROOT=$W/gp_root R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
RUN=full_r2d_state_dHfull_all_bnormclampS8ent5_s3
echo "# $(date -Is) host=$(hostname) sequence check on $RUN rnd30 mode (30 episodes, record order)"
cd $W/r2dreamer_fix
"$PY" eval_genesis.py --checkpoint $W/runs/$RUN/latest.pt --episodes 30 --mode mode --max-steps 1200 \
  --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set rnd --seed 0 \
  --out $W/rs2_iso/seq_rnd30_mode --device cpu 2>&1 | grep -E "^\[eval\] [0-9]+ episodes|Traceback|Error" | tail -2
"$PY" - <<'PY'
import json
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"; R = "full_r2d_state_dHfull_all_bnormclampS8ent5_s3"
rec = json.load(open(f"{W}/runs/{R}/fresh_eval_rnd30_mode/metrics.json"))
new = json.load(open(f"{W}/rs2_iso/seq_rnd30_mode/metrics.json"))
a, b = rec["per_episode"], new["per_episode"]
print(f"episodes: record {len(a)} vs rerun {len(b)}")
KEYS = ("picked", "contact", "nested")
diff = [(x["ep"], x["outcome"], x["steps"], y["outcome"], y["steps"]) for x, y in zip(a, b)
        if x["outcome"] != y["outcome"] or x["steps"] != y["steps"]]
print(f"per-episode differences (outcome or steps): {len(diff)}/{len(a)}")
for d in diff[:12]: print(f"   ep{d[0]:2d}: record {d[1]}/{d[2]} -> rerun {d[3]}/{d[4]}")
for k in KEYS:
    ra = sum(bool((x.get('stages') or {}).get(k)) for x in a); rb = sum(bool((y.get('stages') or {}).get(k)) for y in b)
    print(f"AGGREGATE {k:8s}: record {ra}/{len(a)} ({ra/len(a):.3f})  rerun {rb}/{len(b)} ({rb/len(b):.3f})  delta {(rb-ra)/len(a):+.3f}")
print("rerun-only columns:", {k: new.get(k) for k in ("nested_honest", "nested_proxy", "slide_success", "slide_routes", "slide_fails", "outcomes_honest")})
PY
echo "# $(date -Is) SEQCHECK-DONE"
