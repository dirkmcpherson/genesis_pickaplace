#!/bin/bash
# END-TO-END (full-task) evaluation cells for one finished RLPD or DP run (PHASE_PLAN amendment (n), 2026-09-07);
# the re-runnable eval stage of cluster/sbatch_{rlpd,dp}_e2e.sh (in job) and the post-hoc path (rnd300, reels).
#   shared-process cells:   fresh_eval_<set>_<mode>/metrics.json          (the PHASE_RESULTS §5.1 protocol)
#   isolated cells:         fresh_eval_<set>_<mode>_iso/metrics.json      (one FRESH PROCESS per start, merged)
#   sac: MODES="sample mode"; dp: MODES="sample" (a diffusion policy has no deterministic mode)
#
# WHY BOTH (coordinator 2026-09-07, two findings this evening):
#   (1) NODE SENSITIVITY -- the same checkpoint/IC/mode/seed/horizon gives `nested`@32 on pax109 and `timeout`@300 on
#       pax001/pax030/pax154; each node is self-consistent, nodes disagree, and the mechanism is chaotic amplification
#       over 300 decisions of contact-rich physics (agreement to 1e-9 through decision 12). Short phase episodes are
#       immune. The axis is MACHINE SIZE: physical core count, NOT the instruction set and NOT the node name. The
#       AVX2-vs-AVX-512 attribution is WITHDRAWN (coordinator, 2026-09-07 late): a 40-core Broadwell and a 64-core
#       Sapphire Rapids agree bit-for-bit ACROSS the ISA boundary while a 36-core Broadwell disagrees with the 40-core
#       Broadwell on the SAME ISA; all 53 same-core-count comparisons are bit-identical and every one of the 19
#       differing pairs has a 36-core machine on exactly one side. eval_e2e.py reads /proc/cpuinfo (never a Slurm
#       feature label, which mislabels a Cascade Lake node as `broadwell`) and stamps cpu_model, physical cores,
#       sockets, logical CPUs, task affinity, thread count and isa on EVERY episode. REQUIRE_CORES=<n> and/or
#       THREADS=<n> make a cell REFUSE to run on the wrong configuration -- and unlike a node pin, they can be
#       satisfied on ANY machine of the right size, which is what keeps the CPU-only pass ordinary parallel work.
#   (2) ORDER DEPENDENCE -- in the shared-process protocol state leaks between full-scope episodes (uid 254 scores
#       r=1.0 standalone but r=3.0 as episode 2 after uid 252). The shared cells inherit it BY DESIGN, because that is
#       the protocol the world-model rows of PHASE_RESULTS §5.1 were produced under and this table has to sit beside
#       them; the `_iso` cells are the corrected measurement. Producing both costs one extra eval pass and lets the
#       coordinator/user pick without a re-run -- and without either choice being made silently.
# Usage (env):
#   KIND=sac|dp CKPT=<rlpd_final.zip | .../pretrained_model> OUT=<run dir> ARM=<dH|dDP> SEED=<n> \
#   [SETS="hold15 rnd30 spots60"] [MODES="sample mode"] [ISO=1] [ISO_SETS="rnd30 spots60"] \
#   [REQUIRE_CORES=<n>] [THREADS=<n>] [REQUIRE_ISA=avx2|avx512] \
#   [ROLE=preview|record] [CELL_DIR=""|rec] \
#   [SIM_VARIANT=gc_kp4_riser3_shelf6] [EVAL_SEED=0] [VIDEO_SETS="rnd30"] [LIMIT=n] [REDO=0] [PAR=3] \
#   bash cluster/e2e_eval_cells.sh
# IC sets (amendment (n)): hold15 = baselines/eval_ics.json:hold (15 training starts -- in-distribution, NOT held
# out, REVIEW_GUIDE 8.7); rnd30 = eval_ics.json:rnd (the random box, out-of-distribution); spots60 =
# eval_ics_spots60.json:spots60 (amendment (k), the in-training-distribution test set); rnd300 =
# eval_ics_rnd300.json:rnd (post hoc, 300 starts -- hours per cell, never in job).
# Rules: a cell whose metrics.json exists is kept (resume-safe; REDO=1 wipes the listed cells first). Videos are
# written ONLY for the sets named in VIDEO_SETS and never for `_iso` cells (2026-09-07 disk incident).
set -uo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
: "${KIND:?KIND sac|dp}"; : "${CKPT:?CKPT}"; : "${OUT:?OUT}"; : "${ARM:?ARM}"; : "${SEED:?SEED}"
SETS=${SETS:-"hold15 rnd30 spots60"}
SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}; EVAL_SEED=${EVAL_SEED:-0}; PAR=${PAR:-3}
VIDEO_SETS=${VIDEO_SETS:-"rnd30"}
ISO=${ISO:-1}; ISO_SETS=${ISO_SETS:-"rnd30 spots60"}; REQUIRE_ISA=${REQUIRE_ISA:-}
REQUIRE_CORES=${REQUIRE_CORES:-}; THREADS=${THREADS:-}
# ROLE: 'preview' (default, so the in-job evaluation of an unpinned training job can never be mistaken for a number)
# vs 'record', which eval_e2e.py refuses without --require-isa. CELL_DIR puts the pinned pass's cells in their own
# subdirectory of the run dir, so a preview cell and a cell of record never share a path.
ROLE=${ROLE:-preview}; CELL_DIR=${CELL_DIR:-}
CELLS_ROOT="$OUT"; [ -n "$CELL_DIR" ] && CELLS_ROOT="$OUT/$CELL_DIR"
if [ "$ROLE" = record ] && [ -z "$REQUIRE_CORES" ] && [ -z "$THREADS" ]; then
  echo "FATAL: ROLE=record needs REQUIRE_CORES and/or THREADS -- a cell of record is pinned by construction, and the"
  echo "       axis is machine size / thread count (the AVX2-vs-AVX-512 attribution is withdrawn)."; exit 1
fi
mkdir -p "$CELLS_ROOT"
if [ -z "${MODES:-}" ]; then MODES="sample mode"; [ "$KIND" = dp ] && MODES="sample"; fi
[ "$KIND" = dp ] && case " $MODES " in *" mode "*) echo "FATAL: dp has no mode cell"; exit 1;; esac
if [ "$KIND" = sac ]; then [ -f "$CKPT" ] || { echo "FATAL: checkpoint $CKPT missing"; exit 1; }
else [ -d "$CKPT" ] || { echo "FATAL: DP checkpoint dir $CKPT missing"; exit 1; }; fi

ic_file_of() { case "$1" in
    hold15|rnd30) echo baselines/eval_ics.json ;;
    spots60)      echo baselines/eval_ics_spots60.json ;;
    rnd300)       echo baselines/eval_ics_rnd300.json ;;
    *) echo "" ;; esac; }
ic_set_of()  { case "$1" in
    hold15) echo hold ;; rnd30) echo rnd ;; spots60) echo spots60 ;; rnd300) echo rnd ;; *) echo "" ;; esac; }
n_starts_of() {   # number of starts in a set, after LIMIT
  python3 - "$(ic_file_of "$1")" "$(ic_set_of "$1")" "${LIMIT:-0}" <<'PY'
import sys, os
sys.path.insert(0, os.path.join(os.environ['GENESIS_PICKAPLACE_ROOT'], 'baselines'))
from make_eval_ics import episodes_from_file
n = len(episodes_from_file(sys.argv[1], sys.argv[2]))
lim = int(sys.argv[3])
print(min(n, lim) if lim > 0 else n)
PY
}

for S in $SETS; do
  [ -n "$(ic_file_of "$S")" ] || { echo "FATAL: unknown IC set '$S' (hold15|rnd30|spots60|rnd300)"; exit 1; }
  [ -s "$(ic_file_of "$S")" ] || { echo "FATAL: IC file $(ic_file_of "$S") missing"; exit 1; }
done
if [ "${REDO:-0}" = 1 ]; then for S in $SETS; do for M in $MODES; do rm -rf "$CELLS_ROOT/fresh_eval_${S}_${M}" "$CELLS_ROOT/fresh_eval_${S}_${M}_iso"; done; done; fi
CPU_MODEL=$(awk -F': ' '/^model name/{print $2; exit}' /proc/cpuinfo 2>/dev/null)
CPU_ISA=$(awk '/^flags/{if ($0 ~ /avx512f/) print "avx512"; else if ($0 ~ /avx2/) print "avx2"; else print "pre-avx2"; exit}' /proc/cpuinfo 2>/dev/null)
CPU_CORES=$(python3 -c "
import re
t=open('/proc/cpuinfo').read()
c=re.search(r'^cpu cores\s*:\s*(\d+)', t, re.M)
s=len(set(re.findall(r'^physical id\s*:\s*(\d+)', t, re.M))) or 1
print((int(c.group(1))*s) if c else t.count('processor	'))" 2>/dev/null)
echo "== e2e_eval_cells kind=$KIND ckpt=$CKPT out=$OUT cells=$CELLS_ROOT role=$ROLE arm=$ARM seed=$SEED sets='$SETS' modes='$MODES' iso=$ISO iso_sets='$ISO_SETS' video='$VIDEO_SETS' variant=$SIM_VARIANT eval_seed=$EVAL_SEED par=$PAR node=$(hostname) cores=${CPU_CORES:-?} isa=${CPU_ISA:-?} cpu='${CPU_MODEL:-?}' require_cores='${REQUIRE_CORES}' threads='${THREADS}' require_isa='${REQUIRE_ISA}' $(date)"

eval_one() {   # $1 out dir, $2 set, $3 mode, $4 '' | ic-index, $5 '' | --video
  local D=$1 SET=$2 MODE=$3 IDX=$4 VID=$5
  local XF=(); [ -n "$IDX" ] && XF=(--ic-index "$IDX")
  local VF=(); [ -n "$VID" ] && VF=(--video)
  local LF=(); [ -n "${LIMIT:-}" ] && LF=(--limit "$LIMIT")
  local IF=(); [ -n "$REQUIRE_ISA" ] && IF=(--require-isa "$REQUIRE_ISA")
  [ -n "$REQUIRE_CORES" ] && IF+=(--require-cores "$REQUIRE_CORES")
  [ -n "$THREADS" ] && IF+=(--threads "$THREADS")
  IF+=(--role "$ROLE")
  mkdir -p "$D"
  python baselines/eval_e2e.py --kind "$KIND" --checkpoint "$CKPT" --ic-file "$(ic_file_of "$SET")" \
      --ic-set "$(ic_set_of "$SET")" --out "$D" --mode "$MODE" --seed "$EVAL_SEED" --max-steps 1200 \
      --sim-variant "$SIM_VARIANT" --arm "$ARM" --tag "${SET}_${MODE}" "${XF[@]}" "${VF[@]}" "${LF[@]}" "${IF[@]}" \
      > "$D/eval.log" 2>&1
}

# ---- pass 1: the shared-process cells (the PHASE_RESULTS §5.1 protocol) ----------------------------
for SET in $SETS; do
  for MODE in $MODES; do
    D="$CELLS_ROOT/fresh_eval_${SET}_${MODE}"
    if [ -f "$D/metrics.json" ]; then echo "# cell $SET $MODE exists, kept"; continue; fi
    VID=""; case " $VIDEO_SETS " in *" $SET "*) VID=1;; esac
    while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 5; done
    ( eval_one "$D" "$SET" "$MODE" "" "$VID"
      echo "# cell $SET $MODE rc=$? $(date -Is)"
      grep -E "^\[eval-e2e\] [0-9]+ episodes|FATAL|Traceback|Error" "$D/eval.log" | tail -3 ) &
  done
done
wait

# ---- pass 2: the ISOLATED cells -- one fresh process per start, then merge -------------------------
if [ "$ISO" = 1 ]; then
  for SET in $ISO_SETS; do
    case " $SETS " in *" $SET "*) ;; *) continue;; esac
    N=$(n_starts_of "$SET"); [ -n "$N" ] && [ "$N" -gt 0 ] || { echo "FATAL: could not count starts for $SET"; continue; }
    for MODE in $MODES; do
      D="$CELLS_ROOT/fresh_eval_${SET}_${MODE}_iso"
      if [ -f "$D/metrics.json" ]; then echo "# iso cell $SET $MODE exists, kept"; continue; fi
      echo "# iso cell $SET $MODE: $N fresh processes, one per start $(date -Is)"
      for K in $(seq 0 $((N - 1))); do
        [ -f "$D/ep$K/metrics.json" ] && continue
        while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 5; done
        ( eval_one "$D/ep$K" "$SET" "$MODE" "$K" "" ) &
      done
      wait
      python3 baselines/merge_e2e_iso.py --cell "$D" --n "$N" || echo "# iso cell $SET $MODE MERGE-FAILED $(date -Is)"
    done
  done
fi

python3 - "$CELLS_ROOT" "$KIND" "$ARM" "$SEED" "$SETS" "$MODES" "$ISO_SETS" "$ISO" "$ROLE" <<'PY'
import json, os, sys
out, kind, arm, seed, sets, modes, iso_sets, iso, role = sys.argv[1:10]
def row(suffix, keep):
    parts = []
    for s in sets.split():
        if suffix and s not in iso_sets.split():
            continue
        for m in modes.split():
            f = os.path.join(out, f'fresh_eval_{s}_{m}{suffix}', 'metrics.json')
            tag = f'{s}_{"S" if m == "sample" else "M"}'
            if os.path.exists(f):
                d = json.load(open(f)); n = int(d['episodes']); c = d['stage_counts']
                nodes = ','.join(d.get('nodes') or [d.get('node', {}).get('hostname', '?')])
                nodes += '/' + ','.join(d.get('isa_classes') or ['?'])
                parts.append(f'{tag}=slide{c["slide_success"]}/{n}[p{c["picked"]},pv2{c["placed_v2"]},c{c["contact"]},'
                             f'nH{c["nested_honest"]},nP{c["nested_proxy"]}]@{nodes}')
            elif keep:
                parts.append(f'{tag}=—')
    return parts
lines = ['E2E-HEADLINE learner=%s arm=%s seed=%s role=%s key=slide_success protocol=shared ' % (kind, arm, seed, role) + ' '.join(row('', True)) + f' out={out}']
if iso == '1':
    p = row('_iso', True)
    if p:
        lines.append('E2E-HEADLINE learner=%s arm=%s seed=%s role=%s key=slide_success protocol=ISOLATED ' % (kind, arm, seed, role) + ' '.join(p) + f' out={out}')
for l in lines:
    print(l)
open(os.path.join(out, 'E2E_HEADLINE.txt'), 'w').write('\n'.join(lines) + '\n')
PY
echo "== e2e_eval_cells done $(date)"
