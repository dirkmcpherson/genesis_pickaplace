#!/bin/bash
# Robomimic leg: everything between "environments built" and "smokes" (brief deliverables 2-5), CPU only, in order:
#   G0 (stops the chain if it fails) -> bank_can50 -> arm manifests + masked copies -> conversions (rlpd/r2d for every
#   arm; lerobot for the three primary arms + MH300) -> random-policy negative control (G2a).
# Runs on the login node under nohup (MuJoCo is cheap; the lerobot writer is the slow part) or as a CPU sbatch job.
#   ssh pax 'nohup bash $LAB/genesis_pickaplace/cluster/robomimic/prep_data.sh > $LAB/robomimic_data/logs/prep.log 2>&1 &'
set -uo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}; PY=$LAB/robo_venv/bin/python
export GENESIS_PICKAPLACE_ROOT=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace} PYTHONUNBUFFERED=1 MUJOCO_GL=egl OMP_NUM_THREADS=2
B=$GENESIS_PICKAPLACE_ROOT/baselines/robomimic; D=$LAB/robomimic_data
LEROBOT_ARMS=${LEROBOT_ARMS:-"PH200 MH200 MG200s MH300"}
echo "# prep_data $(date -Is) host=$(hostname)"
if [ ! -f $D/g0_report.json ]; then
  $PY $B/g0_replay.py --hdf5 $D/v1.5/can/ph/low_dim_v15.hdf5 --n 10 --out $D/g0_report.json || true
fi
G0V=$(python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print(r["verdict"], r["n_pass"], "/", r["n"], "flag-agree", r.get("n_success_flag_agree"))' $D/g0_report.json)
echo "== G0: $G0V"
case "$G0V" in PASS*) ;; *) [ -n "${G0_OVERRIDE:-}" ] || { echo "G0 FAILED by the registered clause -- stopping (plan §5). Set G0_OVERRIDE=<reason> to build the data anyway (disclosed)."; exit 3; }
  echo "== G0 OVERRIDE: '$G0_OVERRIDE' -- continuing with data prep; every downstream artifact carries this disclosure" ;; esac
[ -f $D/bank_can50.npz ] || $PY $B/make_bank.py --out $D/bank_can50.npz || exit 1
[ -f $D/arms/arms_index.json ] || $PY $B/make_arms.py --seed 0 || exit 1
for ARM in PH200 MH200 MG200s MH300 MGall PH200pb; do
  [ -d $D/arms/$ARM ] || { echo "   (arm $ARM not built -- skipped)"; continue; }
  T="rlpd,r2d"; case " $LEROBOT_ARMS " in *" $ARM "*) T="rlpd,r2d,lerobot" ;; esac
  [ -f $D/arms/$ARM/r2d/repeat.json ] && T=${T/rlpd,r2d/}; T=${T#,}
  [ -f $D/arms/$ARM/rlpd/manifest.json ] && [ -f $D/arms/$ARM/r2d/repeat.json ] && [[ "$T" != *lerobot* || -f $D/arms/$ARM/lerobot/meta/info.json ]] && { echo "== $ARM converted, skip"; continue; }
  echo "== convert $ARM targets=$T $(date -Is)"; $PY $B/convert_arms.py --arm $ARM --targets "$T" || exit 1
done
[ -f $D/eval_random_bank50/metrics.json ] || $PY $B/eval_random_robosuite.py --out $D/eval_random_bank50 --seed 0 || exit 1
$PY - "$D" <<'PY'
import json, sys, glob, os
d = sys.argv[1]
print("== ARM SUMMARY (rows after the cut / tapes):")
for m in sorted(glob.glob(f"{d}/arms/*/manifest.json")):
    j = json.load(open(m)); parts = []
    for t in ("rlpd", "r2d", "lerobot"):
        f = {"rlpd": "rlpd/manifest.json", "r2d": "r2d/repeat.json", "lerobot": "lerobot/robomimic_source.json"}[t]
        p = os.path.join(os.path.dirname(m), f)
        if os.path.exists(p):
            k = json.load(open(p)); parts.append(f"{t}={k.get('n_transitions', k.get('total_rows', k.get('n_frames')))}")
    print(f"   {j['arm']}: tapes {j['n_tapes']} (success {j['n_success']}) rows {j['rows_after_cut']} | " + " ".join(parts))
r = json.load(open(f"{d}/eval_random_bank50/metrics.json")); print(f"== RANDOM-POLICY control: {r['n_success']}/{r['episodes']} (registered G2a: <= 2/50)")
PY
echo "# prep_data end $(date -Is)"
