#!/usr/bin/env bash
# PHASE_PLAN amendment (af): the PIXEL world-model recipe on the cluster -- {r2dreamer chassis},
# env=genesis_full_pixel, two representation losses, 20 preempt GPUs, ONE job = ONE seed = ONE GPU
# (no packing: the preemption unit is one seed; a preempted job is requeued and restarts CLEAN).
#
#   DRYRUN=1 bash cluster/submit_px_batch.sh                       # print the 18 registered commands, submit nothing
#   bash cluster/submit_px_batch.sh                                # the (af) table, priority order, human/machine interleaved
#   REP=dreamer   ARM=dH SEED=4 bash cluster/submit_px_batch.sh    # exactly ONE job (sparse10)
#   REP=r2dreamer ARM=dM SEED=0 bash cluster/submit_px_batch.sh
#   REP=dreamer   ARM=dH SEED=0 LADDER=nested_ramp bash cluster/submit_px_batch.sh   # one ramp-control job
#   SMOKE=1 REP=dreamer ARM=dH SEED=9990 bash cluster/submit_px_batch.sh              # 20k online steps, hold15 in-job eval
#
# THE RECIPE (identical to the local (ad)/(ae) runs except the tree paths; PX_PIXEL_CONFIG_2026-09-12 §5):
#   train.py env=genesis_full_pixel seed=<s> env.steps=1000000 env.demo_dir=<set> env.ladder=<L>
#     env.far_release=false env.tip_guard=not_in_hand env.return_clamp=<max_return(L)> model.return_clamp=<same>
#     buffer.max_size=5e5 env.actor_dist=bounded_normal env.act_entropy=3e-5                      <- the launcher's own
#     model.rep_loss=<dreamer|r2dreamer> model.image_aug=shift4 env.state_slice=8               <- EXTRA_OVERRIDES, this script
#   env: R2_LONG_RUN=1 R2_MILESTONES=[500000,1000000] GENESIS_PICKAPLACE_ROOT=$LAB/gp_px R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
#        MUJOCO_GL=egl (launcher)  GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 (this script; the launcher exports R2D_SIM_VARIANT)
#   Everything passes through cluster/wmfix_full.sbatch (ENVCFG=genesis_full_pixel), which prints the resolved command,
#   derives the return clamp from the ladder, runs the demo gate (pixel sets must be `images: rendered`) and the
#   `[ladder]` stamp BEFORE training, and refuses the run if the log lacks a non-blank `[image] first ONLINE` stamp after.
#
# RUN DIR / JOB NAME: the launcher names a run full_r2d_state_<set>${TAG:+_$TAG}_s<seed>; the set alone does not say
# which representation loss trained it, and (af)'s two losses share a set, so TAG=<rep> is passed:
#   full_r2d_state_dHfull_all_rns10h_img_dreamer_s4     job px_dreamer_dH_s4
#   full_r2d_state_dDPfull_first_rnrh_img_dreamer_s0    job px_dreamer_ramp_dM_s0   (attribution control)
# The milestone sweep (cluster/ln_r2_milestone_sweep.sh) enumerates *_rns10h_img_* and *_rnrh_img_*; the eval
# sbatch reads the r2dreamer tree from each run's ladder_provenance.json (`r2dreamer_tree`).
#
# QOS: `-p gpu,preempt --qos=preempt` (the 20-GPU preempt allocation; user 2026-09-13), `--requeue` in the launcher.
set -euo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
GP=${GP:-$LAB/gp_px}
R2=${R2:-$W/r2dreamer_px}
DEMOS=${DEMOS:-$W/demos_state_full}
TIP_GUARD=not_in_hand
STEPS=${STEPS:-1000000}
MILES=${MILES:-'[500000,1000000]'}
Q=(-p gpu,preempt --qos=preempt --nice=0)
# PINS (optional, recommended for the submission of record): refuse if the trees are not at these revisions.
GP_PIN=${GP_PIN:-}; R2_PIN=${R2_PIN:-}

case "$GP" in *gp_ladderN*|*gp_ac*|*gp_aa4*|*gp_unified*|*gp_e2e*|*gp_root*) echo "FATAL: refusing the pinned/in-flight tree $GP"; exit 1;; esac
case "$R2" in *r2dreamer_ladderN*|*r2dreamer_unified*|*r2dreamer_fix*) echo "FATAL: refusing the pinned/in-flight tree $R2"; exit 1;; esac
[ -f "$GP/baselines/rl/full_env.py" ] || { echo "FATAL: $GP is not a genesis_pickaplace tree"; exit 1; }
[ -f "$R2/train.py" ] && [ -f "$R2/configs/env/genesis_full_pixel.yaml" ] || { echo "FATAL: $R2 is not a PIXEL-capable r2dreamer tree"; exit 1; }
grep -q 'ENVCFG must be' "$GP/cluster/wmfix_full.sbatch" || { echo "FATAL: $GP launcher predates the ENVCFG/pixel gate (2026-09-13)"; exit 1; }
grep -q 'r2dreamer_tree' "$R2/train.py" || { echo "FATAL: $R2/train.py does not stamp r2dreamer_tree into ladder_provenance.json"; exit 1; }
GPD=$(git -C "$GP" describe --always --dirty 2>/dev/null || echo 'no git'); R2D=$(git -C "$R2" describe --always --dirty 2>/dev/null || echo 'no git')
case "$GPD" in *-dirty) echo "FATAL: $GP is dirty ($GPD)"; exit 1;; esac
case "$R2D" in *-dirty) echo "FATAL: $R2 is dirty ($R2D)"; exit 1;; esac
[ -z "$GP_PIN" ] || case "$GPD" in *$GP_PIN*) ;; *) echo "FATAL: $GP is at '$GPD', not the pinned $GP_PIN"; exit 1;; esac
[ -z "$R2_PIN" ] || case "$R2D" in *$R2_PIN*) ;; *) echo "FATAL: $R2 is at '$R2D', not the pinned $R2_PIN"; exit 1;; esac
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor -- refusing to submit"; exit 1; }
echo "DISK-OK ${FREE_GB} GB free | gp $GP ($GPD) | r2 $R2 ($R2D) | steps $STEPS milestones $MILES | qos ${Q[*]}"

set_for() {   # arm ladder -> set name
  local arm=$1 ladder=$2 sfx
  case "$ladder" in nested_sparse10) sfx=_rns10h_img ;; nested_ramp) sfx=_rnrh_img ;; *) echo "FATAL: ladder must be nested_sparse10 | nested_ramp (got $ladder)" >&2; exit 1 ;; esac
  case "$arm" in dH) echo "dHfull_all$sfx" ;; dM) echo "dDPfull_first$sfx" ;; *) echo "FATAL: ARM must be dH | dM (got $arm)" >&2; exit 1 ;; esac
}
check_set() {   # set ladder: the set exists, is the RENDERED-image build of this ladder (the launcher's gate re-checks in-job)
  local s=$1 ladder=$2
  [ -f "$DEMOS/$s/repeat.json" ] || { echo "FATAL: $DEMOS/$s has no repeat.json"; exit 1; }
  python3 - "$DEMOS/$s" "$ladder" <<'PY' || { echo "FATAL: $DEMOS/$s is not the rendered-image $ladder set"; exit 1; }
import json, sys
d, L = sys.argv[1:3]; m = json.load(open(d + "/repeat.json")); r = m.get("relabel") or {}
assert r.get("ladder") == L and r.get("tip_guard") == "not_in_hand" and m.get("images") == "rendered" and m.get("state_only") is False, (r.get("ladder"), r.get("tip_guard"), m.get("images"), m.get("state_only"))
print(f"  set {d.split('/')[-1]}: {m['n_written']} tapes, ladder {r['ladder']}, tip_guard {r['tip_guard']}, images {m['images']}, total_reward {m['total_reward']}, built on {(m.get('relabel_node') or {}).get('host', (m.get('node') or {}).get('node', '?'))}")
PY
}

sub_px() {   # rep arm seed ladder
  local rep=$1 arm=$2 seed=$3 ladder=$4
  case "$rep" in dreamer|r2dreamer) ;; *) echo "FATAL: REP must be dreamer | r2dreamer (got $rep)"; exit 1 ;; esac
  local set; set=$(set_for "$arm" "$ladder")
  local short=""; [ "$ladder" = nested_ramp ] && short="ramp_"
  local tag=$rep steps=$STEPS miles=$MILES evalsets="hold rnd" name=px_${rep}_${short}${arm}_s${seed}
  if [ -n "${SMOKE:-}" ]; then tag=pxsmoke_$rep; steps=20000; miles='[20000]'; evalsets=hold; name=pxsmoke_${rep}_${short}${arm}_s${seed}; fi
  local run=$W/runs/full_r2d_state_${set}_${tag}_s${seed}
  [ -e "$run" ] && { echo "FATAL: $run exists -- refusing to submit $name over it"; exit 1; }
  check_set "$set" "$ladder"
  local cmd=(env R2_TREE=$R2 GENESIS_PICKAPLACE_ROOT=$GP GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6
             LADDER=$ladder TIP_GUARD=$TIP_GUARD ENVCFG=genesis_full_pixel TAG=$tag EVAL_SETS="$evalsets"
             R2_LONG_RUN=1 R2_MILESTONES="$miles"
             sbatch -J "$name" "${Q[@]}" "$GP/cluster/wmfix_full.sbatch" "$set" "$seed" "$steps"
             model.rep_loss=$rep model.image_aug=shift4 env.state_slice=8)
  if [ -n "${DRYRUN:-}" ]; then printf '%q ' "${cmd[@]}"; echo "   # $name -> $run"; return; fi
  "${cmd[@]}" | sed "s/$/  # $name -> $run/"
  printf '%s %q ' "$(date -Is)" "${cmd[@]}" >> "$W/runs/PX_SUBMISSIONS.log"; echo "  # $name" >> "$W/runs/PX_SUBMISSIONS.log"
}

if [ -n "${REP:-}${ARM:-}${SEED:-}" ]; then
  : "${REP:?REP=dreamer|r2dreamer}" "${ARM:?ARM=dH|dM}" "${SEED:?SEED=<int>}"
  sub_px "$REP" "$ARM" "$SEED" "${LADDER:-nested_sparse10}"
  exit 0
fi
[ -z "${SMOKE:-}" ] || { echo "FATAL: SMOKE=1 needs REP/ARM/SEED (one job)"; exit 1; }
# ---- THE (af) TABLE, in its registered priority order, human/machine interleaved ----------------
# 1. DV3 = rep_loss=dreamer, nested_sparse10, human s4-7 v machine s4-7 (8 GPUs; joins the local (ae) seeds 0-3)
for S in 4 5 6 7; do sub_px dreamer dH $S nested_sparse10; sub_px dreamer dM $S nested_sparse10; done
# 2. R2 = rep_loss=r2dreamer (the port's own contrastive loss, no decoder), nested_sparse10, s0-2 v s0-2 (6 GPUs)
for S in 0 1 2; do sub_px r2dreamer dH $S nested_sparse10; sub_px r2dreamer dM $S nested_sparse10; done
# 3. attribution control: DV3 losses, pixels, nested_ramp v2 (clamp 9), s0-1 v s0-1 (4 GPUs), sets *_rnrh_img
for S in 0 1; do sub_px dreamer dH $S nested_ramp; sub_px dreamer dM $S nested_ramp; done
# 4. RLPD pixels: 2 GPUs RESERVED, not submitted here (lane PXR-1; sub-amendment before its first job)
echo "# 18 jobs; 2 GPUs of the 20 reserved for (af) arm 4"
