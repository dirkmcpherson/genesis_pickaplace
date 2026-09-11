# Ladder N batch — job log and readouts (Lane 13, 2026-09-11)

Operational log for PHASE_PLAN amendment **(aa)** and its **REVISION 1** (`nested_ramp` v2).
The amendment is the registration (commits `266998a` + `dda07d0`, both BEFORE any Ladder-N job);
this file is the record of what was actually built, smoked and submitted. Every number carries
the command that produced it; every claim about a job carries its id.

Convention, per CLAUDE.md: results carry an explicit `{learner}` header.

Environment: `LAB=/cluster/tufts/shortlab/jstale02`, `W=$LAB/wm_fix_2026-09-03`,
local checkout `/home/j/workspace/genesis_pickaplace` on branch `ladder-unify-2026-09-11`.

---

## Step 1 — cluster state before anything was touched (2026-09-11 ~17:00 EDT)

    ssh jstale02@login.pax.tufts.edu 'squeue -u jstale02 -o "%.10i %.26j %.9T %.6M %.12q %.10P %R"'

| job | name | state | elapsed | QOS | partition | node |
|---|---|---|---|---|---|---|
| 3539253 | `lz_rl_sparse_dH_s945` | RUNNING | 8:21 | normal | gpu | pax049 |
| 3539254 | `lz_rl_sparse_dH_s946` | RUNNING | 8:08 | normal | gpu | pax049 |
| 3539255 | `lz_rl_sparse_dM_s965` | RUNNING | 7:21 | normal | gpu | pax051 |
| 3539256 | `lz_rl_sparse_dM_s966` | RUNNING | 7:20 | normal | gpu | pax105 |
| 3539261 | `lz_r2_sparse_dH_s945` | RUNNING | 7:09 | preempt | preempt | pax141 |
| 3539262 | `lz_r2_sparse_dH_s946` | RUNNING | **0:11** | preempt | preempt | pax113 |
| 3539263 | `lz_r2_sparse_dM_s965` | RUNNING | 7:09 | preempt | preempt | pax111 |
| 3539264 | `lz_r2_sparse_dM_s966` | RUNNING | 7:09 | preempt | preempt | pax112 |
| 3562861–63 | `dpRS_dHpruned_s105–107` | RUNNING | 1:37–2:03 | preempt | preempt | pax080/146 |

**GPU occupancy at this moment: normal 4 of 10, preempt 4 of 20** (the three `dpRS_*` are CPU).
So 6 normal GPU slots and ~16 preempt GPU slots are free for the 20 `ln_*` jobs.

`lz_r2_sparse_dH_s946` (3539262) was **preempted and requeued** — elapsed reset to 00:11 while its
three siblings are at 7:09. It did NOT die: `sacct` shows it `RUNNING 00:11:27 0:0`, not
`FAILED 2:0 00:00:00`, so amendment (z)'s requeue-guard fix held. It restarts clean from step 0
(logdir cleared by the guard), which costs ~7 h of its 4M-step budget, not the run.

Completed since 09-11 00:00 (`sacct -S 2026-09-11T00:00 -u jstale02 -X`):
all four `lz_rl_staged_*` (3539249–52, 4:27–5:18, `0:0`), all four `lz_r2_staged_*` (3539257–60,
3:49–4:45, `0:0`), all eight `dpP_e2e_dHpruned_s100–107` (3562638–45) and 29 of 32 `dpRS_*`
re-scores. **No job in the whole listing ended `FAILED 2:0 00:00:00`** (P6 of the pilot).
Two `dpP_e2e_dHpruned` rows (s106 3562644 at 00:00:06, s107 3562645 at 00:01:57) COMPLETED
almost instantly — that is the DP launcher's already-trained short-circuit, not a failure, and it
is Lane 10's to read, not mine.

Health checker (the 09-10 long-run batch, which the user cancelled 09-11 13:10):

    ssh … 'python3 $LAB/e2e_health.py'
    R2 run=0 pend=0 nokeys=0/32 | RLPD run=0 pend=0 nokeys=0/20 eps=23000 picked=11911
    disagree=0 young=0 | done=22 deaths=0 r2trained=29/32 benign_posttrain=0 preempted=0
    | disk=277G/OK | ALL-ENDED

Disk:

    df -BG /cluster/tufts/shortlab
    10.235.106.32:/projects/shortlab  1863G  1586G  277G  86%

**277 GB free against the registered 150 GB floor — DISK-OK.**

---

## Step 2 — the two trees of record for this batch

### `$LAB/gp_ladderN` (genesis_pickaplace)

    git clone -b ladder-unify-2026-09-11 https://github.com/dirkmcpherson/genesis_pickaplace.git \
      $LAB/gp_ladderN

HEAD **`c606b3ad`** = `known-good-2026-08-27-884-gc606b3ad`, which is the step-1 log commit on top
of the registration; `git merge-base --is-ancestor dda07d0 HEAD` → **YES**, so the tree contains
amendment (aa) and its revision 1. The three ladder-defining files and the relabel are byte-identical
to this checkout:

| file | sha256 |
|---|---|
| `baselines/rl/full_env.py` | `23fe428f222f1a3a…` |
| `baselines/genesis_can_env.py` | `40544bf73c8c69bd…` |
| `baselines/stage_predicates.py` | `a589b4f056322094…` |
| `baselines/rl/relabel_reward.py` | `8d7f8d705366bad3…` |

and so are the three launchers (`wmfix_full.sbatch` `872d0ed1…`, `sbatch_rlpd_e2e.sh` `030c3ea3…`,
`relabel_e2e_sets.sbatch` `deb263c5…`).

Note `genesis_can_env.py` carries the SAME hash as the pilot's stamp (`40544bf73c8c`) while
`full_env.py` and `stage_predicates.py` differ — Ladder N is entirely in those two files.

**The tree is PINNED here for the duration of the batch** (Lane 4's rule): pulling would change
`git describe` inside the `[ladder]` stamp, so jobs started after a pull would carry a different
`git=` suffix from jobs started before it, for no gain. Log commits after this point go to origin
only. If a CODE fix becomes necessary mid-batch, this log says which jobs ran which commit.

### `$W/r2dreamer_ladderN` (world-model port)

    rsync -a --exclude .venv --exclude .venv-ms --exclude runs --exclude wandb \
          --exclude '*.pt' --exclude __pycache__ --exclude r2dreamer.egg-info \
          ~/workspace/r2dreamer/ jstale02@login…:$W/r2dreamer_ladderN/
    # then, on the cluster:
    cd $W/r2dreamer_ladderN && git checkout -- .

653 MB, 757 files, 35 s. `git -C $W/r2dreamer_ladderN rev-parse --short HEAD` → **`0cf3d9e`**
("train.py: the step-accounting preflight compares env.steps with the loop's REAL starting
counter"), and `git status --porcelain --untracked-files=all` is **empty** — the cluster copy is
exactly the committed tree.

**Defect I made and fixed before it could matter:** `--exclude runs` is right for run OUTPUT but
`runs/` in this repo also holds seven TRACKED upstream launch scripts (`atari.sh`, `dmc.sh`, …), so
the first copy was missing tracked files (` D runs/*.sh` in `git status`). `git checkout -- .`
restored them; the status above is from after that. Nothing on our path reads `runs/*.sh` (our
logdir is `$W/runs`, a different place entirely), but a tree that does not equal its own HEAD is
exactly the provenance hole this branch exists to close.

Hashes verified against this box, after the checkout:

| file | sha256 (both machines) |
|---|---|
| `trainer.py` | `9a499792eb7a6850…` |
| `envs/genesis.py` | `2a2c0368836b44a0…` |
| `eval_genesis.py` | `b671550420269aad…` |
| `train.py` | `991535a16aad85a2…` |
| `demo_prefill.py` | `c59e031c838366f1…` |
| `configs/env/genesis_full_state.yaml` | `cfe8c088958a9cf5…` |

`tip_guard` appears 10× in `envs/genesis.py`, 8× in `train.py`, 3× in `eval_genesis.py` — Lane 12a's
wiring is present in the tree that will run.

**The launcher takes the world-model tree from a variable, not a hard-wire** (the Lane 4 trap):
`cluster/wmfix_full.sbatch:65` is
`R2_TREE=${R2_TREE:?set R2_TREE to the r2dreamer tree this run must use …}` — required, no default.
The old hardcoded `$W/r2dreamer_fix` survives only in the comment at line 62 explaining why it was
removed. Every submission below passes `R2_TREE=$W/r2dreamer_ladderN` explicitly, so no pilot tree
is touched.

Disk after both trees: **273 GB free** (the r2dreamer copy cost 4 GB, mostly `.git`).

---
