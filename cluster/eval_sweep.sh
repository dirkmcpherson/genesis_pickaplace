#!/bin/bash
# ONE evaluation harness for every learner checkpoint (PREREG_final_round_robin_2026-08-23 §5).
#
#   bash cluster/eval_sweep.sh KIND CKPT OUTDIR [options]
#     KIND    sac | dp            (wandb_eval.py --kind; r2d/dv3 use their own eval scripts
#                                  with the SAME --ic-file / horizon / fresh-process rule)
#     CKPT    sac: path/to/model.zip (sidecar path/to/model.action_mode.json REQUIRED)
#             dp : path/to/pretrained_model dir (sidecar <dir>/../dp_sidecar.json REQUIRED)
#     OUTDIR  per-episode json/log/video dir; sweep.json + SWEEP-RESULT line written here
#   options (all optional):
#     --sets sel,hold,rnd   which IC sets of the shared file to run (default: all three)
#     --ic-file F           default baselines/eval_ics.json
#     --max-steps N         default 1200 (SIM steps) -- the protocol horizon, passed EXPLICITLY
#     --arm A --seed S --ckpt-step T --reward R --tag TAG   provenance, recorded in every json
#     --wandb-run NAME --wandb-project PROJ   best-effort summary push: sweep/<tag>/<set>
#     --max-jobs J          concurrent worlds (default: SLURM_CPUS_PER_TASK/2, min 1)
#   env:
#     SIDECAR_OPTIONAL=1    allow a legacy checkpoint without sidecar (prints the defaults it
#                           assumes LOUDLY; never for the block of record)
#     GENESIS_PICKAPLACE_ROOT   repo root (default: cwd)
#
# RULES (the confound protections this script exists for -- CRITIQUE R6/E1-E5):
#   * ONE FRESH PROCESS PER EPISODE; the SIM is CPU; the POLICY is CPU for sac and GPU (if visible)
#     for dp (POLICY_CUDA=0 forces CPU); <= 1 world per 2 cores.
#   * ONE horizon for every learner (1200 sim steps), ONE IC file for every learner.
#   * repeat / action mode / delta ref come from the checkpoint SIDECAR -- never a default.
#   * denominators asserted against the IC file; a missing episode is reported MISSING, never 0.
#   * resume-safe: an existing per-episode json is not recomputed.
# Output: OUTDIR/<set>_<k>.json (wandb_eval result), OUTDIR/sweep.json, and exactly one line
#   SWEEP-RESULT kind=.. arm=.. seed=.. ckpt=.. tag=.. sel=a/15 hold=b/15 rnd=c/30 missing=..
set -eo pipefail
trap 'rc=$?; [ $rc -ne 0 ] && echo "[sweep] EXIT rc=$rc at line $LINENO (cmd: $BASH_COMMAND)"' EXIT
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1

KIND=${1:?usage: eval_sweep.sh KIND CKPT OUTDIR [options]}; CKPT=${2:?CKPT}; OUTDIR=${3:?OUTDIR}; shift 3
SETS="sel,hold,rnd"; ICF=baselines/eval_ics.json; MAXS=1200; ARM=""; SEED=""; CSTEP=""; REWARD=""; TAG=""
WRUN=""; WPROJ=""; MAXJ=""
while [ $# -gt 0 ]; do
  case "$1" in
    --sets) SETS=$2; shift 2 ;;
    --ic-file) ICF=$2; shift 2 ;;
    --max-steps) MAXS=$2; shift 2 ;;
    --arm) ARM=$2; shift 2 ;;
    --seed) SEED=$2; shift 2 ;;
    --ckpt-step) CSTEP=$2; shift 2 ;;
    --reward) REWARD=$2; shift 2 ;;
    --tag) TAG=$2; shift 2 ;;
    --wandb-run) WRUN=$2; shift 2 ;;
    --wandb-project) WPROJ=$2; shift 2 ;;
    --max-jobs) MAXJ=$2; shift 2 ;;
    *) echo "FATAL: unknown option $1"; exit 1 ;;
  esac
done
case "$KIND" in sac|dp) ;; *) echo "FATAL: KIND must be sac|dp (got $KIND)"; exit 1 ;; esac
[ -f "$ICF" ] || { echo "FATAL: IC file $ICF missing (python baselines/make_eval_ics.py)"; exit 1; }
if [ "$KIND" = sac ]; then [ -f "$CKPT" ] || { echo "FATAL: checkpoint $CKPT missing"; exit 1; }
else [ -d "$CKPT" ] || { echo "FATAL: DP checkpoint dir $CKPT missing"; exit 1; }; fi
mkdir -p "$OUTDIR"
if [ -z "$MAXJ" ]; then
  # allocated CPUs: SLURM_CPUS_ON_NODE covers both -c N and -n N submissions (sbatch_rlpd uses -n 8,
  # which leaves SLURM_CPUS_PER_TASK unset -> the old fallback gave max_jobs=1 in the 08-23 smoke)
  _CPUS=${SLURM_CPUS_ON_NODE:-${SLURM_CPUS_PER_TASK:-${EVAL_CPUS:-$(nproc 2>/dev/null || echo 2)}}}   # EVAL_CPUS: explicit override for non-slurm shells
  MAXJ=$(( _CPUS / 2 )); [ "$MAXJ" -ge 1 ] || MAXJ=1
fi
echo "[sweep] cpus=$_CPUS -> max_jobs=$MAXJ (SLURM_CPUS_ON_NODE=${SLURM_CPUS_ON_NODE:-unset} SLURM_CPUS_PER_TASK=${SLURM_CPUS_PER_TASK:-unset})"

# ---- sidecar -> execution flags (no silent defaults) -----------------------------------
MODE_FLAGS=()
if [ "$KIND" = sac ]; then
  SC="${CKPT%.zip}.action_mode.json"
  if [ -f "$SC" ]; then
    read -r _AM _AR _DR < <(python3 - "$SC" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
missing = [k for k in ('action_mode', 'action_repeat', 'delta_ref') if k not in d]
if missing:
    print(f'FATAL: sidecar {sys.argv[1]} lacks {missing} -- a block-of-record checkpoint carries all three', file=sys.stderr); sys.exit(1)
print(d['action_mode'], int(d['action_repeat']), d['delta_ref'])
PY
) || exit 1
    MODE_FLAGS=(--action-mode "$_AM" --action-repeat "$_AR" --delta-ref "$_DR")
    echo "[sweep] sac sidecar $SC: action_mode=$_AM action_repeat=$_AR delta_ref=$_DR"
  elif [ -n "${SIDECAR_OPTIONAL:-}" ]; then
    MODE_FLAGS=(--action-mode delta_joint --action-repeat 1 --delta-ref target)
    echo "WARN: no sidecar at $SC; SIDECAR_OPTIONAL set -> assuming delta_joint / repeat 1 / target (LEGACY; not for the block of record)"
  else
    echo "FATAL: no sidecar at $SC (set SIDECAR_OPTIONAL=1 only for legacy checkpoints)"; exit 1
  fi
else
  SC="$(dirname "$CKPT")/dp_sidecar.json"
  if [ -f "$SC" ]; then
    _AR=$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); v=d.get("action_repeat"); print("" if v is None else int(v))' "$SC")
    if [ -z "$_AR" ]; then
      if [ -n "${SIDECAR_OPTIONAL:-}" ]; then _AR=1; echo "WARN: $SC has no action_repeat; SIDECAR_OPTIONAL -> 1 (legacy absolute pass-through)"
      else echo "FATAL: $SC has no action_repeat key (a block-of-record DP sidecar carries it)"; exit 1; fi
    fi
    MODE_FLAGS=(--action-repeat "$_AR")
    echo "[sweep] dp sidecar $SC: action_repeat=$_AR"
  elif [ -n "${SIDECAR_OPTIONAL:-}" ]; then
    MODE_FLAGS=(--action-repeat 1); echo "WARN: no dp sidecar at $SC; SIDECAR_OPTIONAL -> repeat 1 (LEGACY)"
  else
    echo "FATAL: no dp sidecar at $SC (set SIDECAR_OPTIONAL=1 only for legacy checkpoints)"; exit 1
  fi
fi

PROV=(); [ -n "$ARM" ] && PROV+=(--arm "$ARM"); [ -n "$CSTEP" ] && PROV+=(--ckpt-step "$CSTEP"); [ -n "$REWARD" ] && PROV+=(--reward "$REWARD")
echo "[sweep] kind=$KIND ckpt=$CKPT sets=$SETS ic_file=$ICF max_steps=$MAXS max_jobs=$MAXJ out=$OUTDIR tag=${TAG:-}"

# ---- one fresh process per episode ------------------------------------------------------
IFS=',' read -ra SETLIST <<< "$SETS"
for S in "${SETLIST[@]}"; do
  N=$(python3 -c 'import json,sys; print(len(json.load(open(sys.argv[1]))[sys.argv[2]]))' "$ICF" "$S")
  for k in $(seq 0 $((N - 1))); do
    J="$OUTDIR/${S}_${k}.json"
    [ -f "$J" ] && continue      # resume-safe
    while [ "$(jobs -rp | wc -l)" -ge "$MAXJ" ]; do sleep 2; done
    # per-episode seed = k: deterministic policies ignore it; sampled ones (DP) get a distinct,
    # reproducible draw per IC (act_selection is recorded in the json)
    # POLICY device: the SIM is always CPU (wandb_eval builds GenesisCanEnv(backend='cpu')); the
    # POLICY runs on the GPU when one is visible and KIND=dp (a 100-step DDPM query is ~6 s on
    # CPU vs ~0.1 s on GPU -- CPU DP evals would take hours per checkpoint). sac stays CPU.
    # POLICY_CUDA=0 forces CPU for everything (the 08-19 behaviour).
    # dp + POLICY_CUDA=1: leave CUDA_VISIBLE_DEVICES exactly as inherited (unset stays unset -- setting
    # it to "" HIDES the GPU, the 08-23 pilot-sweep bug); sac or POLICY_CUDA=0: hide the GPU explicitly.
    _ENV=(env); if [ "$KIND" != "dp" ] || [ "${POLICY_CUDA:-1}" != "1" ]; then _ENV=(env CUDA_VISIBLE_DEVICES=""); fi
    ( "${_ENV[@]}" python baselines/wandb_eval.py --kind "$KIND" --checkpoint "$CKPT" \
        --ic-file "$ICF" --ic-set "$S" --ic-index "$k" --max-steps "$MAXS" --seed "$k" \
        "${MODE_FLAGS[@]}" "${PROV[@]}" --record-dir "$OUTDIR/v_${S}_${k}" \
        --json "$J" --no-wandb > "$OUTDIR/${S}_${k}.log" 2>&1 ) &
  done
done
wait || true
set +e   # from here on nothing may silently abort the aggregation (08-23 smoke: the parent moved on
         # with no SWEEP-RESULT and no sweep.json -> selection read -1 for a complete sweep)
echo "[sweep] episodes done ($(ls "$OUTDIR"/*.json 2>/dev/null | wc -l) result files); aggregating"

# ---- aggregate: asserted denominators, missing never 0 ---------------------------------
python3 -u - "$OUTDIR" "$ICF" "$SETS" "$KIND" "$ARM" "$SEED" "$CSTEP" "$TAG" "$WRUN" "$WPROJ" "$CKPT" "$MAXS" <<'PY'
import json, os, sys, glob
out, icf, sets, kind, arm, seed, cstep, tag, wrun, wproj, ckpt, maxs = sys.argv[1:13]
ic = json.load(open(icf)); res = {}; parts = []; missing_all = {}
for S in sets.split(','):
    n_exp = len(ic[S]); picked = 0; present = 0; missing = []; eps = []
    for k in range(n_exp):
        f = os.path.join(out, f'{S}_{k}.json')
        if not os.path.exists(f):
            missing.append(k); continue
        d = json.load(open(f)); present += 1
        p = int(d.get('picked', round(d['metrics'].get('eval/picked', 0.0) * max(d['metrics'].get('eval/n', 1), 1))))
        picked += p
        eps.append(dict(k=k, picked=p, n_steps=d.get('n_steps'), action_repeat=d.get('action_repeat'),
                        action_mode=d.get('action_mode'), act_selection=d.get('act_selection'),
                        node=(d.get('node') or {}).get('hostname'), git=d.get('git')))
    res[S] = dict(n_expected=n_exp, n_present=present, picked=picked, missing=missing,
                  rate=(picked / present if present else None), episodes=eps)
    parts.append(f'{S}={picked}/{present}' + (f'(exp {n_exp})' if present != n_exp else ''))
    if missing: missing_all[S] = missing
summ = dict(kind=kind, arm=arm or None, seed=seed or None, ckpt=ckpt, ckpt_step=cstep or None, tag=tag or None,
            max_steps=int(maxs), ic_file=icf, sets=res, missing=missing_all)
json.dump(summ, open(os.path.join(out, 'sweep.json'), 'w'), indent=1)
print(f'SWEEP-RESULT kind={kind} arm={arm or "-"} seed={seed or "-"} ckpt={cstep or os.path.basename(ckpt)} '
      f'tag={tag or "-"} ' + ' '.join(parts) + f' missing={json.dumps(missing_all) if missing_all else "none"}')
if missing_all:
    print(f'SWEEP-MISSING {json.dumps(missing_all)} -- denominators above count PRESENT episodes only; '
          f'the cell is INCOMPLETE, never read a missing episode as 0')
if wrun and wproj:
    try:
        import wandb
        api = wandb.Api(timeout=60)
        runs = list(api.runs(f'jambotime/{wproj}', filters={'display_name': wrun}))
        if not runs:
            print(f'WARN: wandb push skipped -- no run named {wrun!r} in {wproj}')
        else:
            run = sorted(runs, key=lambda x: x.created_at)[-1]
            pre = f'sweep/{tag}/' if tag else 'sweep/'
            for S, r in res.items():
                run.summary[pre + S] = f"{r['picked']}/{r['n_present']}"
                run.summary[pre + S + '_rate'] = r['rate']
                run.summary[pre + S + '_missing'] = len(r['missing'])
            run.summary.update()
            print(f'wandb: pushed {pre}* to {run.id} ({wrun})')
    except Exception as e:
        print(f'WARN: wandb push failed ({type(e).__name__}: {e}) -- SWEEP-RESULT line is authoritative')
PY
