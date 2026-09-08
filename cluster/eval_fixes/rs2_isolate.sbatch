#!/usr/bin/env bash
# Why does dHfull_all s3 rnd30 ep0 no longer reproduce (record: nested, 32 steps)? Three arms, ONE node, one job:
#   A1/A2  current trees, run TWICE  -> is the pipeline even deterministic on a fixed node? (Genesis CPU is
#          node/thread dependent; the contact_push agent measured up to 62% of episodes differing in step count)
#   B      (j)+(l) reverse-applied copy (contact_push left in) -> do MY edits move it?
# Each arm dumps the first 12 (state, action, reward, flags) tuples (probe_first_steps.py) AND runs the full episode,
# so an input/action difference at step 0 is distinguishable from chaotic amplification of a tiny state difference.
#SBATCH -J rs2_isolate
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --nice=2000
#SBATCH --exclude=pax077
#SBATCH -N 1
#SBATCH -n 4
#SBATCH --mem=16g
#SBATCH -t 0-02:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -uo pipefail
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/r2d_venv/bin/python
export TORCHINDUCTOR_CACHE_DIR=$W/inductor_cache/${SLURM_JOB_ID} MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
RUN=full_r2d_state_dHfull_all_bnormclampS8ent5_s3; CK=$W/runs/$RUN/latest.pt; IC=$LAB/genesis_pickaplace/baselines/eval_ics.json
echo "# $(date -Is) host=$(hostname) cpus=$SLURM_CPUS_ON_NODE"
rm -rf $W/rev $W/rs2_iso; mkdir -p $W/rev $W/rs2_iso
cp -al $W/gp_root $W/rev/gp_root; cp -a $W/r2dreamer_fix $W/rev/r2dreamer_fix
for f in baselines/genesis_can_env.py baselines/rl/full_env.py; do rm -f $W/rev/gp_root/$f; cp $W/gp_root/$f $W/rev/gp_root/$f; done
python3 $W/rev_apply.py --r2d $W/rev/r2dreamer_fix --gp $W/rev/gp_root --dir $W
python3 -m py_compile $W/rev/r2dreamer_fix/eval_genesis.py $W/rev/gp_root/baselines/genesis_can_env.py && echo REV-COMPILE-OK
echo "# current  can_env $(md5sum $W/gp_root/baselines/genesis_can_env.py | cut -c1-12)  eval $(md5sum $W/r2dreamer_fix/eval_genesis.py | cut -c1-12)"
echo "# reverted can_env $(md5sum $W/rev/gp_root/baselines/genesis_can_env.py | cut -c1-12)  eval $(md5sum $W/rev/r2dreamer_fix/eval_genesis.py | cut -c1-12)"
echo "### first-steps probes (same node)"
GENESIS_PICKAPLACE_ROOT=$W/gp_root     "$PY" $W/probe_first_steps.py $W/r2dreamer_fix     $CK $IC rnd 0 $W/rs2_iso/probe_current1.json 12 2>&1 | grep -E "probe|Traceback|Error" | tail -3
GENESIS_PICKAPLACE_ROOT=$W/gp_root     "$PY" $W/probe_first_steps.py $W/r2dreamer_fix     $CK $IC rnd 0 $W/rs2_iso/probe_current2.json 12 2>&1 | grep -E "probe|Traceback|Error" | tail -3
GENESIS_PICKAPLACE_ROOT=$W/rev/gp_root "$PY" $W/probe_first_steps.py $W/rev/r2dreamer_fix $CK $IC rnd 0 $W/rs2_iso/probe_reverted.json 12 2>&1 | grep -E "probe|Traceback|Error" | tail -3
"$PY" - <<'PY'
import json, numpy as np, itertools
W = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"
P = {n: json.load(open(f"{W}/rs2_iso/probe_{n}.json")) for n in ("current1", "current2", "reverted")}
for n, p in P.items():
    print(f"{n:10s} missing {p['missing']} unexpected {p['unexpected']} obs_space {p['obs_space']}")
def cmp(x, y, lx, ly):
    a, b = P[x], P[y]
    s0 = np.max(np.abs(np.array(a["rows"][0]["state"]) - np.array(b["rows"][0]["state"])))
    print(f"\n--- {lx} vs {ly}:  RESET state max|diff| = {s0:.3e}")
    for i in range(1, min(len(a["rows"]), len(b["rows"]))):
        ra, rb = a["rows"][i], b["rows"][i]
        da = np.max(np.abs(np.array(ra["action"]) - np.array(rb["action"])))
        ds = np.max(np.abs(np.array(ra["state"]) - np.array(rb["state"])))
        tag = "  <== FIRST DIVERGENCE" if (da > 0 or ds > 0) else ""
        print(f"  step {ra['step']:2d}: max|dAction| {da:.3e}  max|dState| {ds:.3e}{tag}")
        if da > 0 or ds > 0: break
cmp("current1", "current2", "current run 1", "current run 2 (SAME code, same node)")
cmp("current1", "reverted", "current", "(j)+(l) reverted")
PY
echo "### full episode under each tree"
for arm in current reverted; do
  gp=$W/gp_root; r2=$W/r2dreamer_fix; [ $arm = reverted ] && { gp=$W/rev/gp_root; r2=$W/rev/r2dreamer_fix; }
  echo "# $(date -Is) === full episode: $arm ==="
  cd $r2
  GENESIS_PICKAPLACE_ROOT=$gp "$PY" eval_genesis.py --checkpoint $CK --episodes 1 --mode mode --max-steps 1200 \
    --ic-file $IC --ic-set rnd --ic-index 0 --seed 0 --out $W/rs2_iso/ep0_$arm --device cpu 2>&1 \
    | grep -E "loaded checkpoint|^ep0|^\[eval\] 1 episodes|Traceback|Error" | tail -4
done
echo "# RECORD (2026-09-05 23:19): nested, 32 steps, picked+contact+nested"
echo "# $(date -Is) ISOLATE-DONE"
