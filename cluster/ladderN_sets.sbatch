#!/usr/bin/env bash
# Ladder N demonstration sets, RECORD-ONCE / SCORE-MANY (PHASE_PLAN amendment (aa) + revision 1).
#
# Two phases in one job, on ONE 64-core node, so every set this batch trains on comes from the
# same hardware class and the same re-execution:
#
#   PHASE 1 (record)  one termination-suppressed re-execution per tape ->
#                     $RECORDS/<set>/<uid>.npz, a per-env-frame stage record.
#                     LADDER- AND GUARD-INDEPENDENT by construction (relabel_reward.record_one):
#                     nothing stops, and the record holds both guards' INPUTS.
#   PHASE 2 (build)   every ladder applied OFFLINE to those records, in numpy, no Genesis ->
#                     $DEMO_ROOT/<set><suffix>.  Verified equal to direct re-execution
#                     (relabel_reward --verify-against; 146/146 on the census, LADDER_N_DEMO_CHECK §1).
#
# Why not cluster/relabel_e2e_sets.sbatch: that one re-EXECUTES per ladder (~35 min per set per
# ladder = ~3.5 h for the six sets here) and has no record phase. Recording once and scoring three
# ladders offline costs one re-execution and minutes per ladder, and -- the reason that matters --
# all six sets are then provably the SAME simulation, so a difference between two of them can only
# be the reward column.
#
# usage:
#   GENESIS_PICKAPLACE_ROOT=$LAB/gp_ladderN TIP_GUARD=not_in_hand \
#     sbatch -J ln_sets --exclude=<the 33 non-64-core batch nodes> cluster/ladderN_sets.sbatch
# env: SETS="dHfull_all dDPfull_first"  LADDERS="nested_ramp nested_sparse staged"
#      TIP_GUARD=grip|not_in_hand (REQUIRED)   FAR_RELEASE=0|1 (default 0; amendment (aa): OFF)
#      RECORDS=$W/stage_records_2026-09-11    DEMO_ROOT=$W/demos_state_full   PROCS=8
#      REQUIRE_CORES=64   FORCE=1 (rebuild existing sets)   FORCE_RECORDS=1 (re-record)
#SBATCH -J ln_sets
#SBATCH -p batch
#SBATCH --qos=normal
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH -t 0-08:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%j.out
set -euo pipefail
LAB=/cluster/tufts/shortlab/jstale02
W=$LAB/wm_fix_2026-09-03
PY=${PY:-$LAB/r2d_venv/bin/python}
DEMO_ROOT=${DEMO_ROOT:-$W/demos_state_full}
RECORDS=${RECORDS:-$W/stage_records_2026-09-11}
SETS=${SETS:-"dHfull_all dDPfull_first"}
LADDERS=${LADDERS:-"nested_ramp nested_sparse staged"}
# A set IS its ladder AND its tip guard: the guard decides where every tape's episode ends and
# therefore what it is paid after that point. Never defaulted (amendment (aa)).
TIP_GUARD=${TIP_GUARD:?set TIP_GUARD (grip | not_in_hand) -- a demo set carries the guard that built it}
case "$TIP_GUARD" in grip|not_in_hand) ;; *) echo "FATAL: TIP_GUARD must be grip | not_in_hand (got $TIP_GUARD)"; exit 2 ;; esac
FAR_RELEASE=${FAR_RELEASE:-0}
case "$FAR_RELEASE" in 0|1) ;; *) echo "FATAL: FAR_RELEASE must be 0 or 1 (got $FAR_RELEASE)"; exit 2 ;; esac
PROCS=${PROCS:-8}
SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}
REQUIRE_CORES=${REQUIRE_CORES:-64}

: "${GENESIS_PICKAPLACE_ROOT:?set GENESIS_PICKAPLACE_ROOT to the code tree this build must use (no default: a builder that silently takes \$PWD is how the two learners ended up on different ladders)}"
[ -f "$GENESIS_PICKAPLACE_ROOT/baselines/rl/relabel_reward.py" ] || { echo "FATAL: $GENESIS_PICKAPLACE_ROOT is not a genesis_pickaplace tree"; exit 2; }
cd "$GENESIS_PICKAPLACE_ROOT"
for G in FULLENV_REWARD_X FULLENV_EPISODE_RECORD; do
  if [ -n "$(eval echo \"\${$G:-}\")" ]; then echo "FATAL: legacy gate set ($G); this tree has no gates"; exit 2; fi
done
export GENESIS_PICKAPLACE_ROOT GENESIS_SIM_VARIANT=$SIM_VARIANT PYTHONUNBUFFERED=1 MUJOCO_GL=egl

# ---- hardware assertion: the class this job LANDED on, not the one it asked for ----
CORES=$(grep -c '^processor' /proc/cpuinfo)
MODEL=$(awk -F': ' '/^model name/{print $2; exit}' /proc/cpuinfo)
ISA=$(grep -qm1 ' avx512f ' /proc/cpuinfo && echo avx512 || echo avx2)
echo "[hw] host=$(hostname) cores=$CORES isa=$ISA model=$MODEL"
[ "$CORES" -eq "$REQUIRE_CORES" ] || { echo "FATAL: $CORES cores != REQUIRE_CORES=$REQUIRE_CORES -- refusing to build a set on the wrong hardware class"; exit 3; }

# ---- disk guard (registered 150 GB floor, 2026-09-07 filesystem-full incident) ----
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: /cluster/tufts/shortlab free ${FREE_GB:-?} GB < 150 GB floor -- refusing to build"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free"
echo "[tree] $GENESIS_PICKAPLACE_ROOT ($(git -C "$GENESIS_PICKAPLACE_ROOT" describe --always --dirty 2>/dev/null || echo 'no git'))"
echo "[plan] sets='$SETS' ladders='$LADDERS' tip_guard=$TIP_GUARD far_release=$FAR_RELEASE records=$RECORDS"

# ================= PHASE 1: the stage records (one re-execution per tape) =================
mkdir -p "$RECORDS"
for S in $SETS; do
  IN=$DEMO_ROOT/$S; REC=$RECORDS/$S
  [ -f "$IN/repeat.json" ] || { echo "FATAL: $IN has no repeat.json"; exit 2; }
  N_SRC=$(ls "$IN"/*.npz | wc -l)
  N_REC=$(ls "$REC"/*.npz 2>/dev/null | wc -l || echo 0)
  if [ "$N_REC" -eq "$N_SRC" ] && [ -z "${FORCE_RECORDS:-}" ]; then
    echo "== $(date -Is) records for $S already complete ($N_REC/$N_SRC) -- skipping (FORCE_RECORDS=1 to redo)"
    continue
  fi
  [ -n "${FORCE_RECORDS:-}" ] && { echo "# FORCE_RECORDS: removing $REC"; rm -rf "$REC"; }
  echo "=================================================================="
  echo "== $(date -Is) RECORD $IN -> $REC  (termination suppressed, $PROCS procs)"
  "$PY" baselines/rl/relabel_reward.py --in "$IN" --records-out "$REC" \
        --tip-guard "$TIP_GUARD" --sim-variant "$SIM_VARIANT" --procs "$PROCS"
done

# ================= PHASE 2: score every ladder offline from those records =================
for L in $LADDERS; do
  # the suffix relabel_reward.py ASSERTS against the ladder (LADDER_SUFFIX / TIP_GUARD_SUFFIX there)
  case "$L" in staged) SFX=_rz ;; sparse) SFX=_rs ;; nested_ramp) SFX=_rnr ;; nested_sparse10) SFX=_rns10 ;;
                nested_sparse) SFX=_rns ;; *) echo "FATAL: ladder $L"; exit 2 ;; esac
  if [ "$FAR_RELEASE" = "1" ]; then SFX="${SFX}f"; FAR_FLAG="--far-release"; else FAR_FLAG=""; fi
  case "$TIP_GUARD" in not_in_hand) SFX="${SFX}h" ;; esac
  for S in $SETS; do
    IN=$DEMO_ROOT/$S; OUT=$DEMO_ROOT/${S}${SFX}; REC=$RECORDS/$S
    if [ -e "$OUT" ]; then
      [ -n "${FORCE:-}" ] || { echo "FATAL: $OUT exists (set FORCE=1 to rebuild)"; exit 2; }
      echo "# FORCE: removing $OUT"; rm -rf "$OUT"
    fi
    echo "=================================================================="
    echo "== $(date -Is) BUILD $IN -> $OUT  ladder=$L tip_guard=$TIP_GUARD far_release=$FAR_RELEASE (offline, from $REC)"
    "$PY" baselines/rl/relabel_reward.py --in "$IN" --out "$OUT" --from-records "$REC" \
          --ladder "$L" ${FAR_FLAG:+$FAR_FLAG} --tip-guard "$TIP_GUARD" --sim-variant "$SIM_VARIANT"
    # Stamp the hardware into the set itself (same as cluster/relabel_e2e_sets.sbatch): a reader
    # must see WHICH machine class produced the reward column without going back to the Slurm log.
    "$PY" - "$OUT" "$(hostname)" "$CORES" "$ISA" "$MODEL" "${SLURM_JOB_ID:-none}" <<'PY'
import json, os, sys
out, host, cores, isa, model, jid = sys.argv[1:7]
for name in ('repeat.json', 'manifest.json'):
    p = os.path.join(out, name)
    m = json.load(open(p))
    m['relabel_node'] = dict(host=host, cores=int(cores), isa=isa, cpu_model=model, slurm_job=jid)
    json.dump(m, open(p, 'w'), indent=1)
print('stamped node into', out)
PY
    # ---- INDEPENDENT action-fidelity check (not the builder's own assertion) ----
    # relabel_reward asserts the action sha256 per tape as it writes; this re-opens both sets
    # afterwards and compares them again, so a set that passed its own assertions and was then
    # corrupted (or written from the wrong source) still cannot pass unnoticed.
    "$PY" - "$IN" "$OUT" <<'PY'
import glob, hashlib, os, sys
import numpy as np
src, out = sys.argv[1], sys.argv[2]
def sha(a): return hashlib.sha256(np.ascontiguousarray(np.asarray(a, np.float32)).tobytes()).hexdigest()
def actions(z):
    return np.asarray(z['action'], np.float32)[1:] if 'action' in z.files else np.asarray(z['actions_delta'], np.float32)
fs = sorted(glob.glob(os.path.join(src, '*.npz')))
bad, n, rew_same = [], 0, 0
for f in fs:
    g = os.path.join(out, os.path.basename(f))
    if not os.path.exists(g):
        bad.append((os.path.basename(f), 'MISSING')); continue
    zs, zo = np.load(f, allow_pickle=True), np.load(g, allow_pickle=True)
    a, b = actions(zs), actions(zo)
    if sha(a) != sha(b):
        bad.append((os.path.basename(f), f'action sha {sha(a)[:12]} != {sha(b)[:12]}'))
    else:
        n += 1
    ks = 'reward' if 'reward' in zs.files else 'rewards'
    if np.array_equal(np.asarray(zs[ks], np.float32), np.asarray(zo[ks], np.float32)):
        rew_same += 1
print(f'ACTION-SHA {os.path.basename(out)}: {n}/{len(fs)} tapes byte-identical to {os.path.basename(src)}; '
      f'{rew_same}/{len(fs)} tapes have an UNCHANGED reward column')
if bad:
    for b in bad[:10]: print('  BAD', b)
    sys.exit('FATAL: action streams are not identical to the source')
PY
  done
done
echo "# DONE $(date -Is)"
