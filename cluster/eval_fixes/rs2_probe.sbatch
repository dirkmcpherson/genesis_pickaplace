#!/usr/bin/env bash
# Element-wise first-steps diff of ep0 (dHfull_all s3, rnd index 0) between the CURRENT trees and the (j)+(l)-reverted
# copies made by rs2_isolate.sbatch ($W/rev). Answers: same inputs? same actions? from which step do they differ?
#SBATCH -J rs2_probe
#SBATCH -p batch,preempt
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 4
#SBATCH --mem=16g
#SBATCH -t 0-01:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -uo pipefail
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/r2d_venv/bin/python
export TORCHINDUCTOR_CACHE_DIR=$W/inductor_cache/${SLURM_JOB_ID} MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
CK=$W/runs/full_r2d_state_dHfull_all_bnormclampS8ent5_s3/latest.pt; IC=$LAB/genesis_pickaplace/baselines/eval_ics.json
mkdir -p $W/rs2_iso
GENESIS_PICKAPLACE_ROOT=$W/gp_root     "$PY" $W/probe_first_steps.py $W/r2dreamer_fix     $CK $IC rnd 0 $W/rs2_iso/probe_current.json 12 2>&1 | grep -E "probe|Traceback|Error" | tail -3
GENESIS_PICKAPLACE_ROOT=$W/rev/gp_root "$PY" $W/probe_first_steps.py $W/rev/r2dreamer_fix $CK $IC rnd 0 $W/rs2_iso/probe_reverted.json 12 2>&1 | grep -E "probe|Traceback|Error" | tail -3
"$PY" - <<'PY'
import json, numpy as np
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"
a = json.load(open(f"{W}/rs2_iso/probe_current.json")); b = json.load(open(f"{W}/rs2_iso/probe_reverted.json"))
print("obs_space current :", a["obs_space"]); print("obs_space reverted:", b["obs_space"])
print("missing/unexpected:", a["missing"], a["unexpected"], "|", b["missing"], b["unexpected"])
s0a = np.array(a["rows"][0]["state"]); s0b = np.array(b["rows"][0]["state"])
print(f"RESET state max|diff| = {np.max(np.abs(s0a - s0b)):.3e}")
for i in range(1, min(len(a["rows"]), len(b["rows"]))):
    ra, rb = a["rows"][i], b["rows"][i]
    da = np.max(np.abs(np.array(ra["action"]) - np.array(rb["action"])))
    ds = np.max(np.abs(np.array(ra["state"]) - np.array(rb["state"])))
    print(f"step {ra['step']:2d}: max|dAction| {da:.3e}  max|dState| {ds:.3e}  r {ra['reward']} vs {rb['reward']}  flags {ra['flags']} | {rb['flags']}")
    if da > 0 or ds > 0:
        print("   FIRST DIVERGENCE HERE"); break
PY
echo "# $(date -Is) PROBE-DONE"
