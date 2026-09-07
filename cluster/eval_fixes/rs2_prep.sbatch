#!/usr/bin/env bash
# amendment (j) PREP (CPU, one job, before the re-score): (1) restore-survival of every entry of the three policy-generated
# banks, raw-grip vs physical-grip, in the env's own restore; (2) smoke of the patched evaluator: pinned place scope on a
# tiny bank (the 5 entries that failed to restore in carrycontact + 2 controls), --dump-entries in physical units, and two
# scope=full episodes reporting nested_honest / nested_proxy / placed_v2. usage: sbatch rs2_prep.sbatch <staging_suffix>
#SBATCH -J rs2_prep
#SBATCH -p batch,preempt
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 4
#SBATCH --mem=16g
#SBATCH -t 0-04:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -uo pipefail
SUF=${1:-_physgrip_staging}
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/r2d_venv/bin/python; B=$W/phase_banks
export TORCHINDUCTOR_CACHE_DIR=$W/inductor_cache/${SLURM_JOB_ID}
export GENESIS_PICKAPLACE_ROOT=$W/gp_root R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1 W
mkdir -p $W/rs2_prep
echo "# $(date -Is) prep start; evaluator md5 $(md5sum $W/r2dreamer_fix/eval_genesis.py | cut -c1-12) adapter $(md5sum $W/r2dreamer_fix/envs/genesis.py | cut -c1-12) full_env $(md5sum $W/gp_root/baselines/rl/full_env.py | cut -c1-12)"
cd $W/r2dreamer_fix
for spec in "place polE_place" "carrycontact polE_place" "place polE_place_dDP" "contact polE_contact"; do
  set -- $spec; SC=$1; BK=$2
  echo "# $(date -Is) restore-check scope=$SC bank=$BK"
  "$PY" $W/bank_restore_check.py $SC $B/${BK}_rawgrip.json $B/${BK}${SUF}.json $W/rs2_prep/restore_${SC}_${BK}.json 2>&1 | grep -E "restore-check|Traceback|Error" | tail -8
done
# --- evaluator smoke (tiny bank: 5 carrycontact restore-failures + 2 controls, from restore_diag.py) ---
"$PY" - <<PY
import json
b = json.load(open("$B/polE_place${SUF}.json")); ids = ["900002", "900007", "900055", "900058", "900126", "900000", "900001"]
json.dump({k: b[k] for k in ids}, open("$W/rs2_prep/smoke_bank_place.json", "w"), indent=1); print("smoke bank", ids)
PY
RUN=s2_r2d_place_state_dH_bnormclamp1ent5_s0
echo "# $(date -Is) smoke place pinned + dump"
"$PY" eval_genesis.py --checkpoint $W/runs/$RUN/latest.pt --episodes 1 --mode mode --max-steps 600 --ic-set all --entry-bank $W/rs2_prep/smoke_bank_place.json \
    --seed 0 --out $W/rs2_prep/smoke_place --device cpu --dump-entries $W/rs2_prep/smoke_dump.json 2>&1 | grep -E "^\[eval\]|^ep[0-9]|Traceback|Error|assert" | tail -20
echo "# rc=${PIPESTATUS[0]}"
RUNF=full_r2d_state_dHfull_all_bnormclampS8ent5_s0
echo "# $(date -Is) smoke full (hold index 0 and 1)"
for i in 0 1; do
"$PY" eval_genesis.py --checkpoint $W/runs/$RUNF/latest.pt --episodes 1 --mode mode --max-steps 1200 --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set hold --ic-index $i \
    --seed 0 --out $W/rs2_prep/smoke_full_$i --device cpu 2>&1 | grep -E "^\[eval\]|^ep[0-9]|Traceback|Error|assert" | tail -8
echo "# rc=${PIPESTATUS[0]}"
done
"$PY" - <<PY
import json
for p in ["$W/rs2_prep/smoke_place/metrics.json", "$W/rs2_prep/smoke_full_0/metrics.json", "$W/rs2_prep/smoke_full_1/metrics.json"]:
    m = json.load(open(p)); pe = m.pop("per_episode")
    print("==", p); print({k: v for k, v in m.items() if k not in ("stage_notes", "checkpoint")})
    for e in pe: print("  ep", e["ep"], "uid", e.get("uid"), "restored", e.get("restored_uid"), "frame", e.get("entry_frame"), e["outcome"], "honest", e.get("outcome_honest"), "steps", e["steps"], {k: v for k, v in e["stages"].items() if v})
d = json.load(open("$W/rs2_prep/smoke_dump.json")); print("dump entries", len(d), [(x["uid"], round(x["grip_cmd"], 3), round(x["grip_cmd_raw"], 3), x["grip_units"], x["bank_version"]) for x in d])
PY
echo "# $(date -Is) PREP-DONE"
