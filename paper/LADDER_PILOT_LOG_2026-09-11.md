# Ladder-unification pilot — job log and readouts (Lane 4, 2026-09-11)

Operational log for PHASE_PLAN amendment **(z)**. The amendment is the registration; this file is
the record of what was actually submitted and what came back. Every number carries the command
that produced it, and every claim about a job carries its id.

Trees of record: `$LAB/gp_unified` @ `21c58b49` (branch `ladder-unify-2026-09-11`) and
`$W/r2dreamer_unified` @ `77b2c61`, with
`LAB=/cluster/tufts/shortlab/jstale02`, `W=$LAB/wm_fix_2026-09-03`.

---

## 1. Build and verification jobs (before the pilot)

| job | what | node | outcome |
|---|---|---|---|
| **3537411** | `cluster/relabel_e2e_sets.sbatch` — the four D5 sets | pax080, 64 cores, AVX-512, Xeon Gold 6438M | DONE 02:54, 4/4 sets, 34 min |
| **3537825** | {RLPD} staged smoke, GPU | — | cancelled; QOS `normal` GPU cap was full (14 of 10 in use by the live batch), re-run on CPU |
| **3537917** | {RLPD} staged smoke, CPU, 1000 decisions | pax145 | TRAIN-OK, eval cell written, stamp printed |
| **3538055** | {r2dreamer} staged smoke, 15k online steps | pax106 | FAILED — `KeyError: 'log_ep_record_valid'` 3 min into training (defect 5, fixed) |
| **3538260** | {RLPD} **sparse** smoke, CPU, 1000 decisions | pax161 | JOB DONE; sparse stamp + `max_return 1.0`; its eval cell exposed defect 6 |
| **3538337** | {r2dreamer} staged smoke, re-run after the fix | pax106 | see §3 |
| **3538555** | re-eval of the sparse smoke checkpoint | pax* | FAILED — `--wrap` job had no conda env on `PATH` (`ModuleNotFoundError: torch`); operator error, not a code defect |
| **3538603** | re-eval of the sparse smoke checkpoint, conda on `PATH` | pax* | see §3 |

Disk guard honoured throughout: 301–306 GB free against the registered 150 GB floor.

### 1.1 The four demonstration sets

    GENESIS_PICKAPLACE_ROOT=$LAB/gp_unified sbatch -J lz_relabel \
      --exclude=<the 33 non-64-core batch nodes> cluster/relabel_e2e_sets.sbatch

Counts, fidelity and the P3 discussion are in PHASE_PLAN amendment (z) §(z).3–§(z).5. Summary:

| set | tapes | Σ reward | top rung | `nested_v2` | `contact_push` |
|---|---:|---:|---:|---:|---:|
| `dHfull_all_rz` | 74 | 171 | `slide_success` 12 | 14 | 9 |
| `dDPfull_first_rz` | 72 | 183 | `slide_success` 13 | 15 | 13 |
| `dHfull_all_rs` | 74 | 14 | `nested_v2` 14 | 14 | 9 (logged) |
| `dDPfull_first_rs` | 72 | 15 | `nested_v2` 15 | 15 | 13 (logged) |

All four pass **both** launcher gates, checked before submission (the RLPD one by `DRYRUN=1`, the
world-model one by running its gate block directly):

    DEMO-SHA dH       n=74 sha=547e65f23e909015 total_reward=171.0 pick=65 nopick=9 decisions_p50=388   # dHfull_all_rz
    DEMO-SHA dH       n=74 sha=25a031cecdf1da58 total_reward=14.0  pick=65 nopick=9 decisions_p50=388   # dHfull_all_rs
    DEMO-SHA dDPfirst n=72 sha=7c7b9bf0c77967ac total_reward=183.0 pick=64 nopick=8 decisions_p50=600   # dDPfull_first_rz
    DEMO-SHA dDPfirst n=72 sha=ed22f3512acb2419 total_reward=15.0  pick=64 nopick=8 decisions_p50=600   # dDPfull_first_rs

with `one_per_ic_first=True` on both machine sets (inherited from the source manifest) and
`relabel_node.host=pax080` on all four.

---

## 2. The stamps (P1)

    [ladder] unified-2026-09-10 | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4 |
             max_return=8 | terminal=slide_success+tipped | shaping=off |
             full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7 |
             git=known-good-2026-08-27-815-gb89478d3-dirty

    [ladder] unified-2026-09-10 | ladder=sparse | nested_v2=1 |
             max_return=1 | terminal=nested_v2+tipped | shaping=off |
             full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7 |
             git=known-good-2026-08-27-815-gb89478d3-dirty

Identical between learners within a ladder, character for character, including the three file
hashes; between ladders they differ only in `ladder`, the rungs, `max_return` and `terminal` — the
four fields P1 allows. Read them with

    grep -h '^\[ladder\]' $LAB/gp_unified/e2e_rlpd_<jobid>.out $W/slurm/lz_*_<jobid>.out

---

## 3. Smoke outcomes

**{RLPD} staged — 3537917, CPU, 1000 decisions, `dHfull_all_rz`.** Complete end to end. Demo gate
`DEMO-SHA dH full-scope segments n=74 sha=547e65f23e909015 total_reward=171.0 pick=65 nopick=9
decisions_p50=388`; the `[ladder]` staged stamp printed twice (launcher and trainer, identical);
`TRAIN-OK … budget 1000 decisions reached (ckpt_100 at 1000)`; `fresh_eval_hold15_mode/metrics.json`
written carrying `ladder_provenance.ladder = staged`.

**{RLPD} sparse — 3538260, CPU, 1000 decisions, `dHfull_all_rs`.** Complete end to end. Demo gate
`n=74 sha=25a031cecdf1da58 total_reward=14.0`; `[ladder] … ladder=sparse | nested_v2=1 |
max_return=1 | terminal=nested_v2+tipped` with the same three file hashes as the staged stamp;
`[ladder] max_return 1.0`. Its eval stage FAILED first (`FATAL: checkpoint sidecar says
ladder='sparse' but --ladder is 'staged'`), which is defect 6 in amendment (z) §(z).10; after the
fix (`21c58b4`) the same checkpoint evaluated in job **3538603**:

    [eval-e2e] ladder 'sparse' from the checkpoint sidecar
    [eval-e2e] … ladder sparse {'nested_v2': 1.0} terminal ('nested_v2',)+tipped

and the cell's `metrics.json` carries `ladder_provenance.ladder = sparse`.

**{r2dreamer} staged — 3538337 (after 3538055 exposed defect 5), 15k ONLINE steps,
`dHfull_all_rz`, pax106.** Demo gate `[demo-gate] … 74 tapes, variant gc_kp4_riser3_shelf6,
stride 4, with_state, terminal 1.0, total_reward 171.0`; launcher and trainer stamps identical to
{RLPD}'s staged stamp, character for character; `[ladder] ladder=staged return_clamp=8.0
(env.return_clamp AND model.return_clamp)` and the trainer's independent check
`[ladder] return_clamp=8.0 (env and model agree)`; the corrected accounting line
`Step accounting [R2_LONG_RUN]: trainer starts at step 29406; env.steps=15000 ONLINE env steps ->
counter target 44406`; `# train rc=0`. Milestone checkpoint written at 15000 online steps,
**114 MB** — so the whole pilot's milestone footprint is ≈ 1.8 GB (8 staged + 16 sparse
checkpoints), not a disk concern.

Its END-OF-JOB evaluation then crashed — `TypeError: dict() got multiple values for keyword
argument 'slide_success'` at `eval_genesis.py` line 469, after 15 evaluated episodes. That is
defect 7 in amendment (z) §(z).10: `scope='full'` now names `slide_success` as its success key and
the summary splatted it into a `dict()` that already passed it explicitly. **All eight
{r2dreamer} pilot cells would have been lost.** Fixed in r2dreamer `197a1a3`, deployed to
`$W/r2dreamer_unified`, and verified by re-running the evaluator on this same checkpoint
(job **3539098**, 2 episodes, CPU).

**{r2dreamer} sparse — 3539030, 15k ONLINE steps, `dHfull_all_rs`, interactive QOS.**

    [demo-gate] .../dHfull_all_rs: 74 tapes, variant gc_kp4_riser3_shelf6, stride 4,
                with_state, terminal 1.0, total_reward 14.0
    [ladder] … | ladder=sparse | nested_v2=1 | max_return=1 | terminal=nested_v2+tipped | shaping=off |
             full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7
    [ladder] ladder=sparse return_clamp=1.0 (env.return_clamp AND model.return_clamp)
    [ladder] return_clamp=1.0 (env and model agree)      # the trainer's own independent check

This is the P1 evidence for the sparse world-model arm: the same three file hashes as every other
stamp, the ladder fields the only difference, and the clamp following the ladder ceiling on both
the env and the model.

**One provenance wrinkle, stated rather than smoothed over.** The four smokes did not all run at
the same commit: 3537917 / 3538260 / 3538337 at `b89478d3`, 3539030 at `21c58b49`. The difference
is `baselines/eval_e2e.py` (defect 6), which is not one of the three files in the stamp, so the
`git=` suffix differs (`…-815-gb89478d3-dirty` versus `…-816-g21c58b49-dirty`) while all four
fields P1 compares are identical. The pilot itself runs at one commit.

---

## 4. Pilot submissions

Submitted 2026-09-11 03:2x by `bash cluster/submit_lz_pilot.sh` from `$LAB/gp_unified` — one
command, whose text is the registered one (amendment (z) §(z).9). All 16 went in together; all 16
were PENDING with reason `QOSMaxGRESPerUser` at submission, which is expected and is the reason the
pilot is on QOS `normal` rather than `preempt`: the live 64-run batch holds 21 GPUs across both
QOSes (14 normal + 7 preempt against caps of 10 and 20), so the pilot starts as that batch drains
and never competes with it for the preempt allocation. The tree was fast-forwarded to the
registration commit `877eb2b` while all 16 were still pending, so every one of them stamps
`git=known-good-2026-08-27-818-g877eb2b6-dirty` and the three file hashes are unchanged
(`full_env=f9e9538d9fc6 genesis_can_env=40544bf73c8c stage_predicates=de4ffde57cd7`).

| job | name | learner | ladder | arm | seed | demo set | budget | checkpoints / milestones |
|---|---|---|---|---|---:|---|---|---|
| 3539249 | `lz_rl_staged_dH_s940` | {RLPD} | staged | human | 940 | `dHfull_all_rz` | 100k decisions | 0.4, 1.0 |
| 3539250 | `lz_rl_staged_dH_s941` | {RLPD} | staged | human | 941 | `dHfull_all_rz` | 100k | 0.4, 1.0 |
| 3539251 | `lz_rl_staged_dM_s960` | {RLPD} | staged | machine-first | 960 | `dDPfull_first_rz` | 100k | 0.4, 1.0 |
| 3539252 | `lz_rl_staged_dM_s961` | {RLPD} | staged | machine-first | 961 | `dDPfull_first_rz` | 100k | 0.4, 1.0 |
| 3539253 | `lz_rl_sparse_dH_s945` | {RLPD} | sparse | human | 945 | `dHfull_all_rs` | 250k | 0.16, 0.4, 1.0 |
| 3539254 | `lz_rl_sparse_dH_s946` | {RLPD} | sparse | human | 946 | `dHfull_all_rs` | 250k | 0.16, 0.4, 1.0 |
| 3539255 | `lz_rl_sparse_dM_s965` | {RLPD} | sparse | machine-first | 965 | `dDPfull_first_rs` | 250k | 0.16, 0.4, 1.0 |
| 3539256 | `lz_rl_sparse_dM_s966` | {RLPD} | sparse | machine-first | 966 | `dDPfull_first_rs` | 250k | 0.16, 0.4, 1.0 |
| 3539257 | `lz_r2_staged_dH_s940` | {r2dreamer} | staged | human | 940 | `dHfull_all_rz` | 1M online | 0.5M, 1M |
| 3539258 | `lz_r2_staged_dH_s941` | {r2dreamer} | staged | human | 941 | `dHfull_all_rz` | 1M online | 0.5M, 1M |
| 3539259 | `lz_r2_staged_dM_s960` | {r2dreamer} | staged | machine-first | 960 | `dDPfull_first_rz` | 1M online | 0.5M, 1M |
| 3539260 | `lz_r2_staged_dM_s961` | {r2dreamer} | staged | machine-first | 961 | `dDPfull_first_rz` | 1M online | 0.5M, 1M |
| 3539261 | `lz_r2_sparse_dH_s945` | {r2dreamer} | sparse | human | 945 | `dHfull_all_rs` | 4M online | 0.5M, 1M, 2M, 4M |
| 3539262 | `lz_r2_sparse_dH_s946` | {r2dreamer} | sparse | human | 946 | `dHfull_all_rs` | 4M online | 0.5M, 1M, 2M, 4M |
| 3539263 | `lz_r2_sparse_dM_s965` | {r2dreamer} | sparse | machine-first | 965 | `dDPfull_first_rs` | 4M online | 0.5M, 1M, 2M, 4M |
| 3539264 | `lz_r2_sparse_dM_s966` | {r2dreamer} | sparse | machine-first | 966 | `dDPfull_first_rs` | 4M online | 0.5M, 1M, 2M, 4M |

The exact commands are what `DRYRUN=1 bash cluster/submit_lz_pilot.sh` prints; the two forms are
in amendment (z) §(z).9. Run dirs: {RLPD}
`$LAB/gp_unified/baselines/rl/checkpoints/e2e/e2e_rlpd_<ARM>_s<seed>` with Slurm logs
`$LAB/gp_unified/e2e_rlpd_<jobid>.out`; {r2dreamer}
`$W/runs/full_r2d_state_<set>_s<seed>` with logs `$W/slurm/lz_r2_<ladder>_<arm>_s<seed>_<jobid>.out`.

**Pick up with** `squeue -u jstale02 -o "%.10i %.24j %.9T %.6M %R" | grep lz_` and, once a job
starts, `grep -h '^\[ladder\]' <its log>` to record its stamp here.

---

## 5. Readouts

*(milestone cells; `rnd30` mode with `nested_v2`, `slide_success`, `pushed`, `nested_honest`)*
