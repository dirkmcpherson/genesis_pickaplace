#!/bin/bash
# RLPD round-robin launcher: one seed per GPU, env-var driven. Post-E3-demo-rng code
# (>= 2fbed2a REQUIRED — the demo buffer ignores --seed before that commit).
#
# 08-23 REWRITE for the FINAL round robin (paper/PREREG_final_round_robin_2026-08-23.md):
#   * ONE CLOCK: ACTION_REPEAT=4, TRAIN_HORIZON=1200 sim steps (300 decisions), STEPS counted
#     in DECISIONS (BUDGET_UNIT=decisions); EVAL_HORIZON=1200 for every learner.
#   * ONE HARNESS: every eval goes through cluster/eval_sweep.sh (fresh process per episode,
#     CPU, the shared baselines/eval_ics.json: sel=15 selection uids, hold=15 confirmation
#     uids, rnd=30 support ICs). K=5 archived checkpoints (<OUT>/ckpt_20..ckpt_100, written
#     by train_rlpd.py) are each scored on `sel`; the best is CONFIRMED on hold+rnd; the final
#     checkpoint is also reported on hold+rnd. Headline per seed = selected-on-hold+rnd.
#   * REWARD=sparse|dense (dense = potential shaping, --pick-shaping on; demo half shaped too
#     when the demos carry eef_pos -- train_rlpd's --demo-shaping follows --pick-shaping).
#   * DEMO_FORMAT=native (contract-v1 tapes under DEMO_ROOT/<ARM>, manifest-gated) or legacy
#     (the 08-19 dirs, re-encoded in-job -- kept ONLY for pilots/controls).
#   * REG_KNOBS carry action_repeat, train_horizon, eval_horizon, budget_unit, reward, demo_sha,
#     demo_format; registry check+REGISTER happen at job START (a crashed run still leaves its
#     line; the TOCTOU window of check-then-register-after-training is gone).
#   * node class recorded (SLURM nodelist + hostname) in every eval json and the .out header.
#
# --- Submit (from the genesis_pickaplace checkout root, AFTER git pull) -----------
#   for S in $(seq 0 9); do ARM=dH   SEED=$S REWARD=sparse sbatch cluster/sbatch_rlpd.sh; done
#   for S in $(seq 0 9); do ARM=dH   SEED=$S REWARD=dense  sbatch cluster/sbatch_rlpd.sh; done
#   ... same for ARM=dDP, ARM=dR2D; secondary arms ARM=dDPfails / ARM=dR2DDPfails (sparse).
#   # pilots / legacy controls at the old clock:
#   ARM=dH SEED=0 DEMO_FORMAT=legacy ACTION_REPEAT=1 TRAIN_HORIZON=900 WAVE=pilot sbatch ...
#
# Env vars:
#   ARM            dH | dDP | dR2D | dDPfails | dR2DDPfails   (native, DEMO_ROOT/<ARM>)
#                  legacy (DEMO_FORMAT=legacy): dH dR2D dDP dHpruned dDPsucc dDPtiptrunc dR2Dfails
#   SEED           required
#   STEPS          100000   (DECISIONS = SB3 timesteps; x ACTION_REPEAT sim steps)
#   ACTION_REPEAT  4        TRAIN_HORIZON 1200 (sim steps)   EVAL_HORIZON 1200 (sim steps)
#   REWARD         sparse|dense   (PICK_SHAPING=on is accepted as an alias for dense)
#   DEMO_FORMAT    native|legacy  (default native)   DEMO_ROOT baselines/matched_v1 (native)
#   WAVE           final    (run-name / out-dir tag: {ARM}_RLPD-{WAVE}_s{SEED}[-dense])
#   IC_FILE        baselines/eval_ics.json
#   DUPLICATE_OK   <reason> (bypass a REGISTRY-REFUSE on an exact repeat)
#   CONDA_ENV      /cluster/tufts/shortlab/jstale02/condaenv/genesis
#   DRYRUN=1       print the plan and exit before any conda/module/training call
#
# DATA (rsync ONLY, never git): the native sets are built by baselines/make_matched_sets.py
# from record_demos.py tapes (contract v1) and rsynced to $DEMO_ROOT/<ARM>/ with their
# manifest.json; the legacy dirs are the 08-19 ones (see git history of this header).
#
# Stages: provenance gates -> RUN_REGISTRY check+register -> train (K=5 ckpts) -> selection
# sweep (5 x sel) -> confirmation (selected on hold+rnd; final on hold+rnd) -> one
# SWEEP-HEADLINE line -> best-effort wandb summary push (sweep/<tag>/<set>).
#
# --- SMOKE PLAN --------------------------------------------------------------------------
# 1. DRYRUN per ARM (no GPU): for A in dH dDP dR2D; do ARM=$A SEED=0 DRYRUN=1 \
#      GENESIS_PICKAPLACE_ROOT=$PWD bash cluster/sbatch_rlpd.sh; done
#    PROOF: "[dry] ..." plan lines incl. the registry, train and eval_sweep commands; exit 0.
# 2. One short real seed: ARM=dH SEED=0 STEPS=2000 sbatch cluster/sbatch_rlpd.sh
#    PROOF: REGISTRY-OK + REGISTRY-REGISTERED at the top, "[cfg] RLPD ...", ckpt_20..ckpt_100
#    dirs under OUT, five "SWEEP-RESULT ... tag=ckpt_XX sel=a/15" lines, the confirmation
#    SWEEP-RESULT lines (tag=selected, tag=final), ONE "SWEEP-HEADLINE arm=dH seed=0 ..."
#    line, "JOB DONE".
# 3. Re-submit identical -> "REGISTRY-REFUSE" before conda (exit 2); DUPLICATE_OK=x proceeds.

#SBATCH -J rlpd
#SBATCH -p gpu,preempt
#SBATCH --requeue
#SBATCH --gres=gpu:1
#SBATCH --constraint="l40s|a100|l40|h200"
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH --time=0-12:00:00
#SBATCH --output=rlpd_%j.out
#SBATCH --error=rlpd_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1

ARM=${ARM:?set ARM (dH, dDP, dR2D, dDPfails, dR2DDPfails; legacy arms need DEMO_FORMAT=legacy)}
SEED=${SEED:?set SEED}
STEPS=${STEPS:-100000}
ACTION_REPEAT=${ACTION_REPEAT:-4}
TRAIN_HORIZON=${TRAIN_HORIZON:-1200}
EVAL_HORIZON=${EVAL_HORIZON:-1200}
DEMO_FORMAT=${DEMO_FORMAT:-native}
DEMO_ROOT=${DEMO_ROOT:-baselines/matched_v1}
WAVE=${WAVE:-final}
SIM_VARIANT=${SIM_VARIANT:-base}
IC_FILE=${IC_FILE:-baselines/eval_ics.json}
BUDGET_UNIT=decisions
# REWARD (sparse|dense); PICK_SHAPING=on|off kept as an alias for back-compat
REWARD=${REWARD:-}
PICK_SHAPING=${PICK_SHAPING:-}
if [ -z "$REWARD" ]; then
  case "${PICK_SHAPING:-off}" in on) REWARD=dense ;; off) REWARD=sparse ;;
    *) echo "FATAL: PICK_SHAPING must be on|off, got '$PICK_SHAPING'"; exit 1 ;; esac
fi
case "$REWARD" in
  sparse) PICK_SHAPING=off ;; dense) PICK_SHAPING=on ;;
  *) echo "FATAL: REWARD must be sparse|dense, got '$REWARD'"; exit 1 ;;
esac
case "$DEMO_FORMAT" in native|legacy) ;; *) echo "FATAL: DEMO_FORMAT must be native|legacy"; exit 1 ;; esac

# ---- demo dir resolution + provenance gates ------------------------------------------
NEED_MANIFEST=""
if [ "$DEMO_FORMAT" = native ]; then
  case "$ARM" in
    dH|dDP|dR2D|dDPfails|dR2DDPfails|dHHfails|dR2DR2Dfails) ARM_DEMO=$DEMO_ROOT/$ARM; PAT='^[0-9]{6}\.npz$' ;;   # dHHfails/dR2DR2Dfails: same-source fails arms (make_samesource_fails_arm.py)
    *) echo "FATAL: ARM=$ARM is not a native (contract v1) arm: dH dDP dR2D dDPfails dR2DDPfails"; exit 1 ;;
  esac
else
  case "$ARM" in
    dH)       ARM_DEMO=baselines/episodes_pick_phase_all;       PAT='^[0-9]{3}\.npz$' ;;
    dR2D)     ARM_DEMO=baselines/episodes_champion_pick;        PAT='^1[0-9]{5}\.npz$' ;;
    dDP)      ARM_DEMO=baselines/m1all_harvest;                 PAT='^[0-9]{6}\.npz$' ;;
    dHpruned) ARM_DEMO=baselines/episodes_pick_phase_dppruned;  PAT='^[0-9]{3}\.npz$' ;;
    dDPsucc)     ARM_DEMO=baselines/m1all_harvest_succ;     PAT='^1[0-9]{5}\.npz$'; NEED_MANIFEST=succ ;;
    dDPtiptrunc) ARM_DEMO=baselines/m1all_harvest_tiptrunc; PAT='^[0-9]{6}\.npz$';  NEED_MANIFEST=tiptrunc ;;
    dR2Dfails)   ARM_DEMO=baselines/episodes_champion_pick_plus_dpfails; PAT='^[15][0-9]{5}\.npz$'; NEED_MANIFEST=r2dfails ;;
    *) echo "FATAL: ARM=$ARM (legacy arms: dH dR2D dDP dHpruned dDPsucc dDPtiptrunc dR2Dfails)"; exit 1 ;;
  esac
fi
if [ -n "$NEED_MANIFEST" ]; then
  python3 - "$ARM_DEMO" "$NEED_MANIFEST" <<'PY' || exit 1
import json, sys, os
d, mode = sys.argv[1], sys.argv[2]
m = os.path.join(d, 'manifest.json')
if not os.path.exists(m):
    print(f'FATAL: {d}/manifest.json missing -- build it with: python baselines/make_dDPsucc.py --mode {mode}'); sys.exit(1)
j = json.load(open(m))
ok = j.get('mode') == mode and j.get('source_name') == 'm1all_harvest' and j.get('builder') == 'baselines/make_dDPsucc.py'
n = len([f for f in os.listdir(d) if f.endswith('.npz')])
if not ok or n != j.get('n_kept'):
    print(f'FATAL: {m} does not describe a make_dDPsucc.py --mode {mode} build of m1all_harvest '
          f'(mode={j.get("mode")} source={j.get("source_name")} n_kept={j.get("n_kept")} vs {n} npz on disk)'); sys.exit(1)
print(f'PROVENANCE-OK {d}: {mode} build of m1all_harvest, {n} npz, sha {j.get("content_sha256","")[:12]}')
PY
fi

# stale-env case-guard: a stale exported DEMO_DIR/DEMO/DATASET must not override the ARM
for _v in DEMO_DIR DEMO DATASET; do
  _val="${!_v:-}"
  if [ -n "$_val" ] && [ "$_val" != "$ARM_DEMO" ]; then
    echo "FATAL: env $_v=$_val is exported and does not match ARM=$ARM's demo dir ($ARM_DEMO)."
    echo "       A stale 'export $_v=...' from another submission is the usual cause: unset $_v"
    exit 1
  fi
done
if [ -n "${DRYRUN:-}" ] && [ ! -d "$ARM_DEMO" ]; then
  echo "[dry] NOTE: demo dir $ARM_DEMO is absent on this box -- the dir/pattern/manifest/sha gates below would FATAL at a real run"
  DEMO_SHA="dry-missing"
else
[ -d "$ARM_DEMO" ] || { echo "FATAL: demo dir $ARM_DEMO missing (rsync it; gitignored)"; exit 1; }
ls "$ARM_DEMO" | grep -E '\.npz$' | head -5 | grep -qE "$PAT" || {
  echo "FATAL: $ARM_DEMO contents do not match $ARM provenance pattern $PAT"; exit 1; }

# native sets: contract-v1 manifest gate + demo sha (content sha from the manifest if the
# builder recorded one, else sha256 of the sorted filename list -- printed either way)
DEMO_SHA=$(python3 - "$ARM_DEMO" "$DEMO_FORMAT" "$ARM" <<'PY'
import json, os, sys, hashlib
d, fmt, arm = sys.argv[1:4]
files = sorted(f for f in os.listdir(d) if f.endswith('.npz'))
m = os.path.join(d, 'manifest.json'); j = json.load(open(m)) if os.path.exists(m) else {}
if fmt == 'native':
    if not j:
        print(f'FATAL: {d}/manifest.json missing -- native arms are built by baselines/make_matched_sets.py', file=sys.stderr); sys.exit(1)
    c = str(j.get('contract') or j.get('tape_contract') or '')
    if c != 'v1':
        print(f'FATAL: {m} contract={c!r} != v1 (not a record_demos.py / make_matched_sets.py product)', file=sys.stderr); sys.exit(1)
    msv = str(j.get('sim_variant') or 'base')
    import os as _os
    if msv != _os.environ.get('SIM_VARIANT', 'base'):
        print(f'FATAL: {m} sim_variant={msv} but this run has SIM_VARIANT={_os.environ.get("SIM_VARIANT", "base")} (world mismatch)', file=sys.stderr); sys.exit(1)
    if j.get('n_kept') is not None and int(j['n_kept']) != len(files):
        print(f'FATAL: {m} n_kept={j["n_kept"]} != {len(files)} npz on disk', file=sys.stderr); sys.exit(1)
sha = j.get('content_sha256') or j.get('sha256')
src = 'manifest' if sha else 'filenames'
if not sha:
    sha = hashlib.sha256('\n'.join(files).encode()).hexdigest()
print(f'DEMO-SHA {arm} {fmt} n={len(files)} sha={sha[:16]} ({src})', file=sys.stderr)
print(sha[:16])
PY
) || exit 1
fi

SHAPE_SUFFIX=""; RUN_SUFFIX=""; PICK_SHAPING_FLAG=()
if [ "$REWARD" = dense ]; then SHAPE_SUFFIX="_dense"; RUN_SUFFIX="-dense"; PICK_SHAPING_FLAG=(--pick-shaping on); fi
# no silent defaults (audit W2): demo shaping follows the reward condition (native tapes carry eef_pos),
# phi(terminal)=0 explicit, terminal guard explicit
DEMO_SHAPING=$([ "$REWARD" = dense ] && echo on || echo off)
PICK_SHAPING_FLAG+=(--demo-shaping "$DEMO_SHAPING" --pick-shaping-terminal-zero on --demo-terminal-guard on --sim-variant "$SIM_VARIANT")
DEMO_FORMAT_FLAG=(); [ "$DEMO_FORMAT" = native ] && DEMO_FORMAT_FLAG=(--demo-format native)
OUT=baselines/rl/checkpoints/rlpd_${WAVE}_${ARM}_s${SEED}${SHAPE_SUFFIX}
RUN_NAME="${ARM}_RLPD-${WAVE}_s${SEED}${RUN_SUFFIX}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"

TRAIN_ARGS=(--steps "$STEPS" --scope pick --action-mode delta_joint --delta-ref target
  --action-repeat "$ACTION_REPEAT" --train-max-steps "$TRAIN_HORIZON" --eval-max-steps "$EVAL_HORIZON"
  --gamma "${GAMMA:-0.998}" --backup-entropy off --per-member-ln off --pick-hold-reward off
  --utd "${UTD:-10}" --ensemble-size 10 --subset-size 2 --demo-batch 128   # GAMMA/UTD knobs: N18 critic-divergence fix factorial (08-28)
  --demo-dir "$ARM_DEMO" --out-dir "$OUT" --run-name "$RUN_NAME" --project genesis_paper
  --seed "$SEED" --device cuda "${PICK_SHAPING_FLAG[@]}" "${DEMO_FORMAT_FLAG[@]}")

# semantic knobs for the RUN_REGISTRY identity key
REG_KNOBS=(steps="$STEPS" budget_unit="$BUDGET_UNIT" scope=pick action_mode=delta_joint delta_ref=target
           action_repeat="$ACTION_REPEAT" train_horizon="$TRAIN_HORIZON" eval_horizon="$EVAL_HORIZON"
           gamma="${GAMMA:-0.998}" backup_entropy=off per_member_ln=off pick_hold_reward=off utd="${UTD:-10}"
           ensemble_size=10 subset_size=2 demo_batch=128 reward="$REWARD" pick_shaping="$PICK_SHAPING"
           demo_format="$DEMO_FORMAT" demo_sha="$DEMO_SHA" wave="$WAVE"
           demo_shaping="$DEMO_SHAPING" pick_shaping_terminal_zero=on demo_terminal_guard=on sim_variant="$SIM_VARIANT")

SWEEP_COMMON=(--ic-file "$IC_FILE" --max-steps "$EVAL_HORIZON" --arm "$ARM" --seed "$SEED" --reward "$REWARD"
              --wandb-run "$RUN_NAME" --wandb-project genesis_paper)

if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] ARM=$ARM SEED=$SEED STEPS=$STEPS($BUDGET_UNIT) ACTION_REPEAT=$ACTION_REPEAT TRAIN_HORIZON=$TRAIN_HORIZON EVAL_HORIZON=$EVAL_HORIZON REWARD=$REWARD DEMO_FORMAT=$DEMO_FORMAT WAVE=$WAVE"
  echo "[dry] ARM_DEMO=$ARM_DEMO DEMO_SHA=$DEMO_SHA OUT=$OUT RUN_NAME=$RUN_NAME IC_FILE=$IC_FILE NODE=$NODE_CLASS"
  echo "[dry] registry check:    python cluster/run_registry.py check    --script sbatch_rlpd.sh --arm $ARM --seed $SEED --demo-dir $ARM_DEMO --registry cluster/RUN_REGISTRY.jsonl ${REG_KNOBS[*]}"
  echo "[dry] registry register: python cluster/run_registry.py register --script sbatch_rlpd.sh --arm $ARM --seed $SEED --demo-dir $ARM_DEMO --registry cluster/RUN_REGISTRY.jsonl ${REG_KNOBS[*]} --stage start"
  echo "[dry] train: python baselines/rl/train_rlpd.py ${TRAIN_ARGS[*]}"
  echo "[dry] selection: for C in $OUT/ckpt_20 .. ckpt_100: bash cluster/eval_sweep.sh sac <C>/*.zip $OUT/sweep/<C> --sets sel --ckpt-step <C> --tag <C> ${SWEEP_COMMON[*]}"
  echo "[dry] confirmation: bash cluster/eval_sweep.sh sac <SELECTED>.zip $OUT/sweep/selected --sets hold,rnd --tag selected ${SWEEP_COMMON[*]}; same for <FINAL> --tag final"
  echo "[dry] headline: SWEEP-HEADLINE arm=$ARM seed=$SEED reward=$REWARD repeat=$ACTION_REPEAT selected=<C> sel=a/15 hold=b/15 rnd=c/30 final_hold=d/15 final_rnd=e/30"
  exit 0
fi

echo "== RLPD $RUN_NAME start $(date) node=$NODE_CLASS host=$(hostname) demo=$ARM_DEMO sha=$DEMO_SHA"

# ---- RUN_REGISTRY: check AND register at job START (crashed runs still leave a line) ----
if [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ]; then   # skip on preemption requeue (same logical run)
  python3 cluster/run_registry.py check --script sbatch_rlpd.sh --arm "$ARM" --seed "$SEED" \
    --demo-dir "$ARM_DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}"
  python3 cluster/run_registry.py register --script sbatch_rlpd.sh --arm "$ARM" --seed "$SEED" \
    --demo-dir "$ARM_DEMO" --registry cluster/RUN_REGISTRY.jsonl "${REG_KNOBS[@]}" --stage start
fi

module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"

# sb3 is pip-only in this env; install once, guarded. NEVER conda install here.
python -c 'import stable_baselines3' 2>/dev/null || pip install --no-input 'stable-baselines3==2.8.0'

# git gate: demo-RNG fix must be present
git merge-base --is-ancestor 2fbed2a HEAD || {
  echo 'FATAL: checkout predates the demo-RNG fix (2fbed2a). git pull first.'; exit 1; }

python baselines/rl/train_rlpd.py "${TRAIN_ARGS[@]}"

# ---- K=5 archived checkpoints (train_rlpd.py writes <OUT>/ckpt_20..ckpt_100, each one
# .zip + its .action_mode.json sidecar); fallback = rlpd_final.zip only (short smokes or a
# trainer predating the archive) -- loudly.
FINAL_CK=$OUT/rlpd_final.zip
[ -f "$FINAL_CK" ] || { echo "FATAL: no final checkpoint at $FINAL_CK"; exit 1; }
mapfile -t CK_DIRS < <(ls -d "$OUT"/ckpt_*/ 2>/dev/null | sort -V)
CANDS=()   # "tag|zip"
for D in "${CK_DIRS[@]}"; do
  Z=$(ls "$D"/*.zip 2>/dev/null | head -1)
  [ -n "$Z" ] && CANDS+=("$(basename "$D")|$Z")
done
if [ "${#CANDS[@]}" -eq 0 ]; then
  echo "WARN: no archived ckpt_* dirs under $OUT -- selection degenerates to the final checkpoint only"
  CANDS+=("final|$FINAL_CK")
fi
SW=$OUT/sweep; mkdir -p "$SW"
set +e   # evals must never fail an already-trained job; every stage reports its rc
BEST_TAG=""; BEST_ZIP=""; BEST_N=-1
for C in "${CANDS[@]}"; do
  TAG=${C%%|*}; Z=${C#*|}
  bash cluster/eval_sweep.sh sac "$Z" "$SW/$TAG" --sets sel --ckpt-step "$TAG" --tag "$TAG" "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/$TAG.log"
  N=$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); s=d["sets"]["sel"]; print(s["picked"] if s["n_present"]==s["n_expected"] else -1)' "$SW/$TAG/sweep.json" 2>/dev/null || echo -1)
  echo "SELECT-RESULT arm=$ARM seed=$SEED reward=$REWARD ckpt=$TAG sel=$N/15"
  if [ "$N" -ge "$BEST_N" ]; then BEST_N=$N; BEST_TAG=$TAG; BEST_ZIP=$Z; fi   # ties -> later checkpoint
done
FINAL_TAG=${CANDS[-1]%%|*}; FINAL_ZIP=${CANDS[-1]#*|}
echo "== selected $BEST_TAG (sel=$BEST_N/15); final=$FINAL_TAG"
bash cluster/eval_sweep.sh sac "$BEST_ZIP" "$SW/selected" --sets hold,rnd --ckpt-step "$BEST_TAG" --tag selected "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/selected.log"
if [ "$BEST_ZIP" = "$FINAL_ZIP" ]; then
  cp "$SW/selected/sweep.json" "$SW/final_sweep.json" 2>/dev/null
  FINAL_SWEEP="$SW/selected/sweep.json"
else
  bash cluster/eval_sweep.sh sac "$FINAL_ZIP" "$SW/final" --sets hold,rnd --ckpt-step "$FINAL_TAG" --tag final "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/final.log"
  FINAL_SWEEP="$SW/final/sweep.json"
fi
python3 - "$SW/selected/sweep.json" "$FINAL_SWEEP" "$ARM" "$SEED" "$REWARD" "$ACTION_REPEAT" "$BEST_TAG" "$BEST_N" "$FINAL_TAG" "$RUN_NAME" "$NODE_CLASS" <<'PY'
import json, sys
sel_j, fin_j, arm, seed, reward, rep, best, best_n, final_tag, run_name, node = sys.argv[1:12]
def rd(p, s):
    try:
        r = json.load(open(p))['sets'][s]; return f"{r['picked']}/{r['n_present']}" + ('' if r['n_present'] == r['n_expected'] else f"(exp{r['n_expected']})")
    except Exception as e:
        return 'MISSING'
line = (f'SWEEP-HEADLINE arm={arm} seed={seed} reward={reward} repeat={rep} selected={best} sel={best_n}/15 '
        f'hold={rd(sel_j,"hold")} rnd={rd(sel_j,"rnd")} final={final_tag} final_hold={rd(fin_j,"hold")} final_rnd={rd(fin_j,"rnd")} node={node}')
print(line)
try:
    import wandb
    api = wandb.Api(timeout=60)
    runs = list(api.runs('jambotime/genesis_paper', filters={'display_name': run_name}))
    if runs:
        run = sorted(runs, key=lambda x: x.created_at)[-1]
        run.summary['sweep/headline'] = line; run.summary['sweep/selected_ckpt'] = best
        run.summary['sweep/node'] = node; run.summary.update()
        print(f'wandb: pushed sweep/headline to {run.id}')
    else:
        print(f'WARN: wandb headline push skipped -- no run named {run_name!r}')
except Exception as e:
    print(f'WARN: wandb headline push failed ({type(e).__name__}: {e}) -- SWEEP-HEADLINE line is authoritative')
PY
echo "JOB DONE $(date)"
