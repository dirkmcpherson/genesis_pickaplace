#!/usr/bin/env bash
# MILESTONE EVALUATION SWEEP for the {r2dreamer} Ladder-N runs (PHASE_PLAN (aa) REVISION 3:
# "every milestones/online_*.pt of every ln_r2_* run (and each final latest.pt) is evaluated on
# rnd30 mode + hold15 mode (+ rnd30 sampled) by a sweep script, so the matched-milestone statistic
# exists for the world model").
#
# WHY IT EXISTS: cluster/wmfix_full.sbatch evaluates ONLY latest.pt, at the very END of the job
# (lines 153-165). Every milestone checkpoint the launcher writes is therefore unevaluated, and the
# audit (paper/AUDIT_STATISTICAL_SHOT_2026-09-12.md §VERDICT item 2) found the registered
# "matched milestone" rule unimplementable for the world model because of it.
#
#   bash cluster/ln_r2_milestone_sweep.sh              # copy + submit what is missing, then exit
#   DRYRUN=1 bash cluster/ln_r2_milestone_sweep.sh     # say what it would do, touch nothing
#   MAXJOBS=6 SWEEP_MODE=cpu64|gpu ...                 # knobs, below
#
# IDEMPOTENT, and safe to re-run after every milestone lands (there is no cron on this cluster):
#   * a (run, milestone) whose three cells all have metrics.json is skipped entirely;
#   * a partially-done one is resubmitted and the sbatch skips the cells it already has;
#   * a (run, milestone) that already has a PENDING/RUNNING job is skipped;
#   * the checkpoint copy is made once and verified against the milestone sidecar's own sha256.
# It NEVER writes into a run dir: it reads `milestones/*.pt`, `latest.pt`, `.hydra/`,
# `metrics.jsonl` and `step_contract.json`, and everything it produces lands under $CELLROOT.
#
# SCOPE: $W/runs/full_r2d_state_*_r{nrh,nsh,zh}_s9* -- the (aa) batch, the rev-2 extension and
# rev 3 as its runs appear. The pilot's `_rz_`/`_rs_` runs and the `*_lnsmoke_*` smokes are OUT of
# scope by construction (different guard / different ladder / 15k steps).
#
# HARDWARE, disclosed. SWEEP_MODE=cpu64 (default) submits to `-p batch --qos=normal` on nodes of
# 64 PHYSICAL cores, with the sbatch asserting the count itself. That is a deliberate change from
# Lane RC's GPU submission and the reasons are: (1) the r2dreamer adapter builds Genesis with
# backend="cpu" (envs/genesis.py:214) and the policy runs --device cpu, so no part of this
# evaluation uses a GPU; (2) at the time this sweep was written BOTH GPU allocations were at their
# cap (normal 10/10, preempt 20/20) with this project's own training runs, so a GPU sweep would
# have been paid for out of the batch it exists to read; (3) pinning the core count makes every
# cell hardware-homogeneous, which the unpinned GPU nodes are not, and full-scope outcomes track
# machine size in this project. SWEEP_MODE=gpu reproduces Lane RC's exact submission shape
# (-p gpu,preempt --qos=preempt --gres=gpu:1) for anyone who wants that comparison; do not MIX the
# two modes inside one table -- every cell stamps `node`/`cpu_model`/`ncpus_machine`, so check.
set -eo pipefail
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
GP=${GP:-$LAB/gp_ladderN}
R2=${R2:-$W/r2dreamer_ladderN}
CELLROOT=${CELLROOT:-$W/ln_milestone_cells}
SBATCH_FILE=${SBATCH_FILE:-$W/ln14_milestone_eval.sbatch}   # deployed OUTSIDE the pinned tree
MAXJOBS=${MAXJOBS:-6}
SWEEP_MODE=${SWEEP_MODE:-cpu64}
REQUIRE_CORES=${REQUIRE_CORES:-64}
EXCL64=${EXCL64:-$LAB/gp_dp_e2e/.excl64.txt}   # READ ONLY: "every node that is NOT 64 physical cores"

[ -f "$GP/baselines/rl/full_env.py" ] || { echo "FATAL: GP=$GP is not a genesis_pickaplace tree"; exit 1; }
[ -f "$R2/eval_genesis.py" ] || { echo "FATAL: R2=$R2 is not an r2dreamer tree"; exit 1; }
[ -f "$SBATCH_FILE" ] || { echo "FATAL: no sbatch at $SBATCH_FILE (scp cluster/ln_r2_milestone_eval.sbatch there)"; exit 1; }
FREE_GB=$(df -BG --output=avail /cluster/tufts/shortlab 2>/dev/null | tail -1 | tr -dc '0-9' || true)
[ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 150 ] || { echo "FATAL: free ${FREE_GB:-?} GB < 150 GB floor -- refusing"; exit 1; }
mkdir -p "$CELLROOT"

case "$SWEEP_MODE" in
  cpu64) [ -f "$EXCL64" ] || { echo "FATAL: no $EXCL64 for the node filter"; exit 1; }
         SUBMIT_ARGS=(-p batch --qos=normal --exclude="$(cat "$EXCL64")") ;;
  gpu)   SUBMIT_ARGS=(-p gpu,preempt --qos=preempt --gres=gpu:1 --constraint=l40s\|a100\|l40\|h200 --exclude=pax077)
         REQUIRE_CORES=0 ;;   # Lane RC's GPU nodes are not one core class; the cells stamp what they landed on
  *) echo "FATAL: SWEEP_MODE must be cpu64 | gpu (got $SWEEP_MODE)"; exit 1 ;;
esac

# ---- how many of this sweep's jobs are already in the queue -------------------------------
INFLIGHT_NAMES=$(squeue -u "$USER" -h -o "%j" | grep -c '^lnms_' || true)
INFLIGHT_NAMES=${INFLIGHT_NAMES:-0}
QUEUED=$(squeue -u "$USER" -h -o "%j" | grep '^lnms_' || true)
SLOTS=$(( MAXJOBS - INFLIGHT_NAMES ))
echo "DISK-OK ${FREE_GB} GB | mode=$SWEEP_MODE require_cores=$REQUIRE_CORES | lnms_ in queue: $INFLIGHT_NAMES | free slots: $SLOTS"

# ---- enumerate (run, milestone) pairs that need cells --------------------------------------
# python does the reading/copying/verifying; bash does the submitting.
PLAN=$(python3 - "$W" "$CELLROOT" "${DRYRUN:-}" <<'PY'
import glob, hashlib, json, os, shutil, sys
W, CELLROOT, DRYRUN = sys.argv[1], sys.argv[2], sys.argv[3]
CELLS = ("rnd30_mode", "hold15_mode", "rnd30_sample")

def sha256(p, buf=1 << 20):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(buf), b""):
            h.update(b)
    return h.hexdigest()

def last_step(run):
    """last `step` in metrics.jsonl, or None. Tail only: the file is large and append-only."""
    p = os.path.join(run, "metrics.jsonl")
    if not os.path.exists(p):
        return None
    with open(p, "rb") as f:
        f.seek(0, 2)
        f.seek(max(0, f.tell() - 200000))
        tail = f.read().decode("utf-8", "replace").splitlines()
    for line in reversed(tail):
        try:
            d = json.loads(line)
        except Exception:
            continue
        if "step" in d:
            return int(d["step"])
    return None

runs = []
for suf in ("rnrh", "rnsh", "rzh"):
    runs += glob.glob(os.path.join(W, "runs", f"full_r2d_state_*_{suf}_s9*"))
runs = sorted(r for r in runs if os.path.isdir(r) and "lnsmoke" not in os.path.basename(r))

plan = []
for run in runs:
    name = os.path.basename(run)
    cand = []                                              # (label, src_pt, sidecar_or_None)
    for pt in sorted(glob.glob(os.path.join(run, "milestones", "online_*.pt"))):
        lab = os.path.basename(pt)[:-3]                    # online_500000
        side = pt[:-3] + ".json"
        cand.append((lab, pt, side if os.path.exists(side) else None))
    # the FINAL checkpoint, only once training has reached its budget -- latest.pt is rewritten
    # periodically while a run is alive, so a copy taken earlier is a mid-run snapshot, not `final`
    sc = os.path.join(run, "step_contract.json")
    lp = os.path.join(run, "latest.pt")
    if os.path.exists(sc) and os.path.exists(lp):
        target = json.load(open(sc)).get("legacy_counter_target")
        ls = last_step(run)
        if target and ls is not None and ls >= int(target) - 5000:
            cand.append(("final", lp, None))
    for lab, src, side in cand:
        cell = os.path.join(CELLROOT, name, lab)
        if all(os.path.exists(os.path.join(cell, c, "metrics.json")) for c in CELLS):
            continue                                        # complete; nothing to do
        dst = os.path.join(cell, "latest.pt")
        if not DRYRUN:
            os.makedirs(cell, exist_ok=True)
            if not os.path.exists(dst):
                tmp = dst + ".part"
                shutil.copy2(src, tmp)
                os.replace(tmp, dst)
                hd = os.path.join(cell, ".hydra")
                os.makedirs(hd, exist_ok=True)
                for f in ("config.yaml", "overrides.yaml", "hydra.yaml"):
                    s = os.path.join(run, ".hydra", f)
                    if os.path.exists(s):
                        shutil.copy2(s, os.path.join(hd, f))
                for f in ("ladder_provenance.json", "step_contract.json"):
                    s = os.path.join(run, f)
                    if os.path.exists(s):
                        shutil.copy2(s, os.path.join(cell, f))
                got = sha256(dst)
                want = json.load(open(side))["sha256"] if side else None
                if want and got != want:
                    os.remove(dst)
                    print(f"SKIP\t{name}\t{lab}\tSHA-MISMATCH sidecar={want[:16]} copy={got[:16]}")
                    continue
                # training_step_at_copy is the RUN's step at copy time, NOT the checkpoint's;
                # a milestone's own step is in `sidecar`, and for `final` the two coincide.
                prov = dict(run=name, run_dir=run, milestone=lab, source=src,
                            source_mtime=os.path.getmtime(src), copied_at=__import__("time").time(),
                            sha256=got, sidecar_sha256=want,
                            sidecar=(json.load(open(side)) if side else None),
                            training_step_at_copy=last_step(run))
                json.dump(prov, open(os.path.join(cell, "provenance.json"), "w"), indent=1)
        print(f"NEED\t{name}\t{lab}")
PY
) || { echo "FATAL: enumeration failed"; exit 1; }

echo "$PLAN" | grep -v '^$' || true

SUB=0
while IFS=$'\t' read -r tag name lab rest; do
  [ "$tag" = "NEED" ] || continue
  JN="lnms_${name#full_r2d_state_}_${lab}"
  JN=${JN//online_/}
  if echo "$QUEUED" | grep -qx "$JN"; then echo "# in queue already: $JN"; continue; fi
  if [ "$SUB" -ge "$SLOTS" ]; then echo "# slot limit reached ($MAXJOBS) -- $name $lab deferred to the next sweep"; continue; fi
  CMD=(sbatch -J "$JN" "${SUBMIT_ARGS[@]}" "$SBATCH_FILE" "$name" "$lab")
  if [ -n "${DRYRUN:-}" ]; then printf '%q ' env CELLROOT="$CELLROOT" GP="$GP" R2="$R2" REQUIRE_CORES="$REQUIRE_CORES" "${CMD[@]}"; echo "   # $JN"; SUB=$((SUB+1)); continue; fi
  env CELLROOT="$CELLROOT" GP="$GP" R2="$R2" REQUIRE_CORES="$REQUIRE_CORES" \
      SBATCH_EXPORT=ALL "${CMD[@]}" | sed "s/$/  # $JN/"
  SUB=$((SUB+1))
done <<< "$PLAN"
echo "# submitted $SUB job(s); re-run this sweep after each milestone lands (it is idempotent)"
