#!/usr/bin/env bash
# Is the full-scope episode-order effect an incomplete reset or amplification? Same node, one job: dump the complete
# post-reset(254) env state solo vs after a full uid252 episode, then K decisions each, and diff field by field.
#SBATCH -J rs2_resetprobe
#SBATCH -p preempt,batch
#SBATCH --qos=preempt
#SBATCH -N 1
#SBATCH -n 4
#SBATCH --mem=16g
#SBATCH -t 0-02:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%N_%j.out
set -uo pipefail
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/r2d_venv/bin/python
export TORCHINDUCTOR_CACHE_DIR=$W/inductor_cache/${SLURM_JOB_ID} MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export GENESIS_PICKAPLACE_ROOT=$W/gp_root R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
CK=$W/runs/full_r2d_state_dHfull_all_bnormclampS8ent5_s0/latest.pt
echo "# $(date -Is) NODE=$(hostname) CPUTot=$(nproc)"
"$PY" $W/reset_probe.py $W/r2dreamer_fix $CK solo $W/rs2_iso/reset_solo.json 20 2>&1 | tail -2
"$PY" $W/reset_probe.py $W/r2dreamer_fix $CK seq  $W/rs2_iso/reset_seq.json  20 2>&1 | tail -2
"$PY" - <<'PY'
import json, numpy as np
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"
a = json.load(open(f"{W}/rs2_iso/reset_solo.json")); b = json.load(open(f"{W}/rs2_iso/reset_seq.json"))
pa, pb = a["post_reset"], b["post_reset"]
print("\n=== POST-RESET STATE: solo vs after-a-uid252-episode (same node, same code) ===")
diffs = []
for k in sorted(set(pa) | set(pb)):
    va, vb = pa.get(k), pb.get(k)
    if isinstance(va, list) and isinstance(vb, list) and len(va) == len(vb):
        d = float(np.max(np.abs(np.array(va, float) - np.array(vb, float)))) if va else 0.0
        print(f"  {k:26s} max|diff| = {d:.3e}")
        if d > 0: diffs.append((k, d))
    else:
        same = va == vb
        print(f"  {k:26s} {'same' if same else f'DIFFER: {va} vs {vb}'}")
        if not same: diffs.append((k, f"{va} vs {vb}"))
print(f"\nFIELDS DIFFERING AFTER RESET: {len(diffs)}")
for k, d in diffs: print(f"   {k}: {d}")
print("\n=== first {} decisions after that reset ===".format(min(len(a['steps']), len(b['steps']))))
first = None
for i in range(min(len(a["steps"]), len(b["steps"]))):
    sa, sb = a["steps"][i], b["steps"][i]
    da = np.max(np.abs(np.array(sa["action"]) - np.array(sb["action"])))
    ds = np.max(np.abs(np.array(sa["state"]) - np.array(sb["state"])))
    print(f"  t{sa['t']:2d}: max|dAction| {da:.3e}  max|dState| {ds:.3e}  r {sa['reward']} vs {sb['reward']}")
    if first is None and (da > 0 or ds > 0): first = i
print(f"first diverging decision: {first}")
print("VERDICT: " + ("INCOMPLETE RESET (post-reset state already differs)" if diffs else
      ("HIDDEN STATE (post-reset identical, trajectories diverge)" if first is not None else "CLEAN (identical)")))
PY
echo "# $(date -Is) RESETPROBE-DONE"
