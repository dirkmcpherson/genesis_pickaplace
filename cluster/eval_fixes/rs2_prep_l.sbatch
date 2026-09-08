#!/usr/bin/env bash
# amendment (l) smoke: does slide_success actually fire, and by which route? (a) end-to-end episodes that reached the
# proxy terminal in the record (dHfull_all s3 rnd30 ep0 = nested, ep9 = contact without nested), (b) a whole slide-phase
# cell (scope=contact, holdE_contact, 11 entries) where EVERY episode terminates on the first contact frame.
#SBATCH -J rs2_prep_l
#SBATCH -p batch,preempt
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 4
#SBATCH --mem=16g
#SBATCH -t 0-02:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -uo pipefail
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/r2d_venv/bin/python; B=$W/phase_banks
export TORCHINDUCTOR_CACHE_DIR=$W/inductor_cache/${SLURM_JOB_ID}
export GENESIS_PICKAPLACE_ROOT=$W/gp_root R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
mkdir -p $W/rs2_prep_l; cd $W/r2dreamer_fix
echo "# $(date -Is) slide smoke; can_env $(md5sum $W/gp_root/baselines/genesis_can_env.py | cut -c1-12) eval $(md5sum eval_genesis.py | cut -c1-12)"
RUNF=full_r2d_state_dHfull_all_bnormclampS8ent5_s3
for i in 0 9; do
  echo "# $(date -Is) e2e rnd30 ic-index $i (record: ep0 nested, ep9 contact-not-nested)"
  "$PY" eval_genesis.py --checkpoint $W/runs/$RUNF/latest.pt --episodes 1 --mode mode --max-steps 1200 \
      --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set rnd --ic-index $i --seed 0 \
      --out $W/rs2_prep_l/e2e_$i --device cpu 2>&1 | grep -E "^\[eval\]|^ep[0-9]|Traceback|Error" | tail -4
done
RUNC=s2_r2d_contact_state_dDP_bnormclamp1ent5_n11_s0
echo "# $(date -Is) slide phase: holdE_contact (11 entries), scope=contact"
"$PY" eval_genesis.py --checkpoint $W/runs/$RUNC/latest.pt --episodes 1 --mode mode --max-steps 600 --ic-set all \
    --entry-bank $B/holdE_contact.json --seed 0 --out $W/rs2_prep_l/slide_holdE --device cpu 2>&1 | grep -E "^\[eval\]|Traceback|Error" | tail -3
"$PY" - <<'PY'
import json, glob, os
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"
for p in sorted(glob.glob(f"{W}/rs2_prep_l/*/metrics.json")):
    m = json.load(open(p)); pe = m.pop("per_episode")
    print("==", os.path.basename(os.path.dirname(p)), "scope", m["scope"], "| slide_success", m.get("slide_success"),
          "| routes", m.get("slide_routes"), "| nested_honest", m.get("nested_honest"), "| nested_proxy", m.get("nested_proxy"),
          "| outcomes_honest", m.get("outcomes_honest"))
    for e in pe:
        st = e.get("stages") or {}
        print("   ep", e["ep"], e["outcome"], "steps", e["steps"], "| slide", st.get("slide_success"), "route", e.get("slide_route"),
              "| contact", st.get("contact"), "nested(proxy)", st.get("nested"), "honest", st.get("nested_honest"))
PY
echo "# $(date -Is) SLIDE-SMOKE-DONE"
