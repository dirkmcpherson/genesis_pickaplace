#!/usr/bin/env bash
# D5 demo-set relabel by RE-EXECUTION (LADDER_UNIFY_BRIEF D5, PHASE_PLAN amendment (z)).
#
# Builds the four end-to-end demonstration sets the unified-ladder pilot trains on:
#   dHfull_all_rz / dDPfull_first_rz   (staged: picked 1 / placed_v2 1 / contact_push 2 / slide_success 4)
#   dHfull_all_rs / dDPfull_first_rs   (sparse: nested_v2 1)
# by re-executing each source tape through FullTaskEnv(scope='full', ladder=...) -- the SAME
# code path training uses -- so tape reward == env reward by construction. The action stream
# is asserted byte-identical (sha256) to the source, per tape and over the set; only the
# reward column changes.
#
# WHY THE HARDWARE IS PINNED. Re-execution is a simulation, and full-scope outcomes on this
# cluster track the machine class: Lane 2's 6-tape dry run on an off-cluster box saw the CAN
# diverge 78 mm and 155 mm from the recording on 2 of 6 tapes while the ARM tracked to
# 0.012 rad. The project's convention is 64-core nodes (CLAUDE.md `_mixedcore` history;
# core count and instruction set are perfectly collinear here). This job therefore asserts
# the core count IT ACTUALLY LANDED ON, read from /proc/cpuinfo -- never a Slurm feature
# label, which lies on this cluster (pax001 advertises `broadwell` and is Cascade Lake).
# Every tape's `can_dev_max_m` is recorded so a reader can check the fidelity rather than
# take the pin on trust.
#
# usage:
#   GENESIS_PICKAPLACE_ROOT=$LAB/gp_unified sbatch -J lz_relabel \
#     --exclude=<non-64-core nodes> cluster/relabel_e2e_sets.sbatch
# env: SETS="dHfull_all dDPfull_first"  LADDERS="staged sparse"  PROCS=8  TIP_GUARD=grip|not_in_hand
#      REQUIRE_CORES=64  DEMO_ROOT=$W/demos_state_full  FORCE=1 (overwrite an existing set)
#SBATCH -J lz_relabel
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
SETS=${SETS:-"dHfull_all dDPfull_first"}
LADDERS=${LADDERS:-"staged sparse"}   # or: nested_ramp nested_sparse (+ FAR_RELEASE=1)
# PHASE_PLAN amendment (aa): REQUIRED, no default. A set is defined by its ladder AND its tip
# guard; a set built under one and trained under the other is the two-ladder confound again.
TIP_GUARD=${TIP_GUARD:?set TIP_GUARD (grip | not_in_hand) -- a demo set carries the guard that built it}
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
for L in $LADDERS; do
  # the suffix relabel_reward.py asserts against the ladder (LADDER_SUFFIX there)
  case "$L" in staged) SFX=_rz ;; sparse) SFX=_rs ;; nested_ramp) SFX=_rnr ;; nested_sparse10) SFX=_rns10 ;;
                nested_sparse) SFX=_rns ;; *) echo "FATAL: ladder $L"; exit 2 ;; esac
  [ "${FAR_RELEASE:-0}" = "1" ] && SFX="${SFX}f" && FAR_FLAG="--far-release" || FAR_FLAG=""
  # PHASE_PLAN amendment (aa): the TIP GUARD is part of what a set IS -- it decides where each
  # tape's episode ends and therefore what it is paid after that point -- so it is required
  # here too, and it appends 'h' to the suffix (relabel_reward.TIP_GUARD_SUFFIX).
  case "$TIP_GUARD" in grip) ;; not_in_hand) SFX="${SFX}h" ;;
                *) echo "FATAL: TIP_GUARD must be grip | not_in_hand (got $TIP_GUARD)"; exit 2 ;; esac
  for S in $SETS; do
    IN=$DEMO_ROOT/$S; OUT=$DEMO_ROOT/${S}${SFX}
    [ -f "$IN/repeat.json" ] || { echo "FATAL: $IN has no repeat.json"; exit 2; }
    if [ -e "$OUT" ]; then
      [ -n "${FORCE:-}" ] || { echo "FATAL: $OUT exists (set FORCE=1 to rebuild)"; exit 2; }
      echo "# FORCE: removing $OUT"; rm -rf "$OUT"
    fi
    echo "=================================================================="
    echo "== $(date -Is) relabel $IN -> $OUT  ladder=$L tip_guard=$TIP_GUARD procs=$PROCS"
    "$PY" baselines/rl/relabel_reward.py --in "$IN" --out "$OUT" --ladder "$L" ${FAR_FLAG:+$FAR_FLAG} \
          --tip-guard "$TIP_GUARD" --sim-variant "$SIM_VARIANT" --procs "$PROCS"
    # Stamp the hardware into the set itself: a reader must be able to see WHICH machine
    # class produced the reward column without going back to the Slurm log.
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
  done
done
echo "# DONE $(date -Is)"
