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
