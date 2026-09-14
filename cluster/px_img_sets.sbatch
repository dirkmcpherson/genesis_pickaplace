#!/usr/bin/env bash
# RENDERED-IMAGE demonstration sets for the PIXEL world model, built ON THE CLUSTER on a 64/64-core
# node (PHASE_PLAN amendment (af) arm 3: the nested_ramp attribution control needs `_rnrh_img` sets;
# the coordinator's brief of 2026-09-13). Same recorder as lane PX-1 (`relabel_reward.py --images`,
# paper/PX_IMAGE_DEMOS_2026-09-12.md): one termination-suppressed re-execution per tape with the dv3
# camera rig built, ladder applied as it runs, `image` column (T,64,64,6) uint8 rendered by
# GenesisCanEnv.rig_obs() -- the SAME call the online adapter makes.
#
#   GENESIS_PICKAPLACE_ROOT=$LAB/gp_px sbatch --exclude=<non-64/64 nodes> cluster/px_img_sets.sbatch
#   env: SETS="dHfull_all dDPfull_first"  LADDER=nested_ramp  TIP_GUARD=not_in_hand  PROCS=8
#        DEMO_ROOT=$W/demos_state_full   REQUIRE_CORES=64 REQUIRE_LOGICAL=64   FORCE=1 (rebuild)
#        VERIFY_AGAINST_SUFFIX=_rnrh  (the state-only set OF RECORD whose reward column this build must reproduce)
#
# VERIFICATION (in-job, independent of the builder's own assertions), per set:
#   tape count == source; action AND state columns byte-identical to the SOURCE on every tape;
#   reward column, Sigma reward, end reasons and `home` count identical to the set of record
#   ($DEMO_ROOT/<set>$VERIFY_AGAINST_SUFFIX, built from the pax146 stage records); image column
#   non-blank on EVERY frame of EVERY tape. A set that fails is renamed *_UNVERIFIED so no launcher
#   can pick it up, and the job exits 1.
#SBATCH -J px_img_sets
#SBATCH -p batch
#SBATCH --qos=normal
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH -t 0-04:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -euo pipefail
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
PY=${PY:-$LAB/r2d_venv/bin/python}
DEMO_ROOT=${DEMO_ROOT:-$W/demos_state_full}
SETS=${SETS:-"dHfull_all dDPfull_first"}
LADDER=${LADDER:-nested_ramp}
TIP_GUARD=${TIP_GUARD:-not_in_hand}
case "$TIP_GUARD" in grip|not_in_hand) ;; *) echo "FATAL: TIP_GUARD must be grip | not_in_hand (got $TIP_GUARD)"; exit 2 ;; esac
PROCS=${PROCS:-8}
SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}
REQUIRE_CORES=${REQUIRE_CORES:-64}
REQUIRE_LOGICAL=${REQUIRE_LOGICAL:-64}
VERIFY_AGAINST_SUFFIX=${VERIFY_AGAINST_SUFFIX:-_rnrh}

: "${GENESIS_PICKAPLACE_ROOT:?set GENESIS_PICKAPLACE_ROOT to the code tree this build must use (no default)}"
[ -f "$GENESIS_PICKAPLACE_ROOT/baselines/rl/relabel_reward.py" ] || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT is not a genesis_pickaplace tree"; exit 2; }
grep -q -- "--images" "$GENESIS_PICKAPLACE_ROOT/baselines/rl/relabel_reward.py" || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT/relabel_reward.py has no --images (lane PX-1, a083664)"; exit 2; }
cd "$GENESIS_PICKAPLACE_ROOT"
for G in FULLENV_REWARD_X FULLENV_EPISODE_RECORD; do
  if [ -n "$(eval echo \"\${$G:-}\")" ]; then echo "FATAL: legacy gate set ($G); this tree has no gates"; exit 2; fi
done
export GENESIS_PICKAPLACE_ROOT GENESIS_SIM_VARIANT=$SIM_VARIANT R2D_SIM_VARIANT=$SIM_VARIANT PYTHONUNBUFFERED=1 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl

# ---- hardware assertion: BOTH counts (cluster/ln_r2_milestone_eval.sbatch's rule) ----
PHYS=$(lscpu -p=Core,Socket 2>/dev/null | grep -v '^#' | sort -u | wc -l)
LOGI=$(grep -c '^processor' /proc/cpuinfo)
MODEL=$(awk -F': ' '/^model name/{print $2; exit}' /proc/cpuinfo)
ISA=$(grep -qm1 ' avx512f ' /proc/cpuinfo && echo avx512 || echo avx2)
echo "[hw] host=$(hostname) physical=$PHYS logical=$LOGI isa=$ISA model=$MODEL"
[ "$REQUIRE_CORES" = 0 ] || [ "$PHYS" = "$REQUIRE_CORES" ] || { echo "FATAL: $PHYS physical cores != REQUIRE_CORES=$REQUIRE_CORES -- refusing to build a set on the wrong hardware class"; exit 3; }
[ "$REQUIRE_LOGICAL" = 0 ] || [ "$LOGI" = "$REQUIRE_LOGICAL" ] || { echo "FATAL: $LOGI logical processors != REQUIRE_LOGICAL=$REQUIRE_LOGICAL (SMT) -- refusing"; exit 3; }
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor -- refusing to build"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free"
echo "[tree] $GENESIS_PICKAPLACE_ROOT ($(git -C "$GENESIS_PICKAPLACE_ROOT" describe --always --dirty 2>/dev/null || echo 'no git'))"
# the suffix relabel_reward.py ASSERTS against the ladder + guard (+ `_img` under --images)
SFX=$("$PY" -c "import sys; sys.path.insert(0,'baselines/rl'); import relabel_reward as r; print(r.LADDER_SUFFIX['$LADDER'] + ('h' if '$TIP_GUARD'=='not_in_hand' else '') + '_img')")
echo "[plan] sets='$SETS' ladder=$LADDER tip_guard=$TIP_GUARD suffix=$SFX procs=$PROCS verify_against=<set>$VERIFY_AGAINST_SUFFIX"

RC=0
for S in $SETS; do
  IN=$DEMO_ROOT/$S; OUT=$DEMO_ROOT/${S}${SFX}; REF=$DEMO_ROOT/${S}${VERIFY_AGAINST_SUFFIX}
  [ -f "$IN/repeat.json" ] || { echo "FATAL: $IN has no repeat.json"; exit 2; }
  [ -f "$REF/repeat.json" ] || { echo "FATAL: no set of record at $REF to verify against"; exit 2; }
  if [ -e "$OUT" ]; then
    [ -n "${FORCE:-}" ] || { echo "FATAL: $OUT exists (set FORCE=1 to rebuild)"; exit 2; }
    echo "# FORCE: removing $OUT"; rm -rf "$OUT"
  fi
  echo "=================================================================="
  echo "== $(date -Is) BUILD $IN -> $OUT  ladder=$LADDER tip_guard=$TIP_GUARD --images procs=$PROCS"
  T0=$(date +%s)
  "$PY" baselines/rl/relabel_reward.py --in "$IN" --out "$OUT" --ladder "$LADDER" --tip-guard "$TIP_GUARD" \
        --sim-variant "$SIM_VARIANT" --images --procs "$PROCS"
  echo "# build wall $(( $(date +%s) - T0 )) s"
  # stamp the hardware into the set itself (same as cluster/ladderN_sets.sbatch)
  "$PY" - "$OUT" "$(hostname)" "$PHYS" "$LOGI" "$ISA" "$MODEL" "${SLURM_JOB_ID:-none}" <<'PY'
import json, os, sys
out, host, phys, logi, isa, model, jid = sys.argv[1:8]
for name in ('repeat.json', 'manifest.json'):
    p = os.path.join(out, name); m = json.load(open(p))
    m['relabel_node'] = dict(host=host, cores=int(phys), logical=int(logi), isa=isa, cpu_model=model, slurm_job=jid)
    json.dump(m, open(p, 'w'), indent=1)
print('stamped node into', out)
PY
  # ---- INDEPENDENT verification against the SOURCE and the set OF RECORD ----
  if "$PY" - "$IN" "$OUT" "$REF" <<'PY'; then echo "# VERIFY-OK $OUT"; else RC=1; echo "# VERIFY-FAILED $OUT -> renamed ${OUT}_UNVERIFIED"; mv "$OUT" "${OUT}_UNVERIFIED"; fi
import glob, json, os, sys
import numpy as np
src, out, ref = sys.argv[1:4]
fs = sorted(glob.glob(os.path.join(src, '*.npz')))
rep_o, rep_r = json.load(open(out + '/repeat.json')), json.load(open(ref + '/repeat.json'))
man_o, man_r = json.load(open(out + '/manifest.json')), json.load(open(ref + '/manifest.json'))
bad = []
n_act = n_rew = n_img = 0
for f in fs:
    b = os.path.basename(f); g, h = os.path.join(out, b), os.path.join(ref, b)
    if not os.path.exists(g): bad.append((b, 'MISSING in build')); continue
    if not os.path.exists(h): bad.append((b, 'MISSING in set of record')); continue
    zs, zo, zr = np.load(f, allow_pickle=True), np.load(g, allow_pickle=True), np.load(h, allow_pickle=True)
    if np.array_equal(zs['action'], zo['action']) and np.array_equal(zs['state'], zo['state']) and str(zo['rz_actions_sha256']) == str(zr['rz_actions_sha256']):
        n_act += 1
    else:
        bad.append((b, 'action/state differ from source or sha differs from record'))
    if zo['reward'].shape == zr['reward'].shape and np.array_equal(np.asarray(zo['reward'], np.float64), np.asarray(zr['reward'], np.float64)):
        n_rew += 1
    else:
        bad.append((b, f"reward column differs from set of record (sum {float(zo['reward'].sum()):.4f} vs {float(zr['reward'].sum()):.4f})"))
    im = zo['image']
    if im.shape == (len(zo['reward']), 64, 64, 6) and im.dtype == np.uint8 and bool((im.reshape(len(im), -1) != 0).any(1).all()):
        n_img += 1
    else:
        bad.append((b, f'image column: shape {im.shape} dtype {im.dtype} blank frames {int((~(im.reshape(len(im), -1) != 0).any(1)).sum())}'))
n = len(fs)
so, sr = float(rep_o['total_reward']), float(rep_r['total_reward'])
eo, er = man_o.get('end_reasons'), man_r.get('end_reasons')
print(f"  tapes: source {n}, build {len(glob.glob(out + '/*.npz'))} (n_written {rep_o['n_written']}), record {len(glob.glob(ref + '/*.npz'))}")
print(f"  action+state identical to source AND action sha == record: {n_act}/{n}")
print(f"  reward column identical to set of record: {n_rew}/{n}")
print(f"  image column (T,64,64,6) uint8 non-blank on every frame: {n_img}/{n}")
print(f"  Sigma reward: build {so:.6f} vs record {sr:.6f} (diff {so - sr:+.2e})")
print(f"  end_reasons: build {eo} vs record {er} -> {'identical' if eo == er else 'DIFFER'}")
print(f"  home: build {(eo or {}).get('home')} vs record {(er or {}).get('home')}")
print(f"  build manifest: ladder {man_o.get('ladder')} tip_guard {man_o.get('tip_guard')} images {man_o.get('images')} state_only {rep_o.get('state_only')} node {rep_o.get('relabel_node')}")
print(f"  build stamp: {man_o.get('ladder_stamp')}")
for b, why in bad[:20]: print('  BAD', b, why)
ok = (not bad and n_act == n and n_rew == n and n_img == n and len(glob.glob(out + '/*.npz')) == n == int(rep_o['n_written'])
      and abs(so - sr) < 1e-6 and eo == er and rep_o.get('images') == 'rendered' and rep_o.get('state_only') is False)
print('  VERDICT:', 'OK' if ok else 'FAILED')
sys.exit(0 if ok else 1)
PY
done
echo "# DONE px_img_sets rc=$RC $(date -Is)"
exit $RC
