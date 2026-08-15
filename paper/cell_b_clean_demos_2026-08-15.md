# Cell B — ManiSkill-quality demos for OUR pick task (2026-08-15)

Author: Fable session (local box, branch `4dof-cartesian`). Simplified Technical
English: short sentences, active voice, one claim each.

Read with: `paper/RESULTS_MATRIX_2026-08-15.md` ("Positive controls"),
`FABLE_HANDOFF_2026-08-13.md` §12-§22, `dreamerv3-torch/MANISKILL_VS_GENESIS.md`.

---

## 0. What cell B is for

Our RLPD shows one signature across four independent waves: **~1 seed of 3 reaches
the >=3/15 bar, 5-7 picks per 45 demo-IC episodes at 100k**. Reward density (25x),
critic diversity (per-member LN), demo source (human vs DP-model), and action
reference (target vs measured) all failed to move it. Two explanations survive.

* **DATASET.** Our human demos are long, slow, teleoperated, and re-encoded from a
  different control space. ManiSkill's are short, in-sim, native-action, clean.
* **METHOD / TASK.** The task or our RLPD is the limit, whatever the demos look like.

Cell B builds the missing arm: **ManiSkill-quality demos for OUR task**, then trains
our RLPD on them. Same env, same code, same protocol — only the demo set changes.

---

## 1. PRE-REGISTERED BARS (written before any cell-B training)

Evaluation = the §4a-2 protocol: fixed checkpoint **100k decisions**
(`rlpd_100000_steps.zip`), **15 fresh processes** demo-IC per seed, explicit
`--action-mode delta_joint --action-repeat 1`, 400 env steps, picks read from
stdout, post-P2-fix code only. The peer session runs the sweep.

| verdict | rule |
|---|---|
| **DATASET problem** | **>= 2 of 3 seeds reach >= 3/15 demo-IC** |
| **METHOD / TASK problem** | **<= 1 of 3 seeds reaches >= 3/15** (the standard signature) |

Notes that are part of the registration, not caveats added later:

1. The demo-IC eval set is the SAME IC family the demos were harvested from. It is
   in-distribution by construction. Every other arm in the matrix is evaluated the
   same way, so the comparison is matched; the absolute number is not a
   generalization claim. (Random-IC is the generalization column, as always.)
2. `>= 3/15` keeps its §4a-2 rationale: 3 picks exceeds anything ever observed from
   contamination or borderline effects alone. It is not a binomial claim.
3. The bar is deliberately ASYMMETRIC in favor of "method problem". 2-of-3 seeds is
   strictly stronger than anything the four previous waves produced (each produced
   exactly 1-of-3). A 1-of-3 result on clean demos means clean demos changed nothing.
4. Registered expectation, stated before data (Fable): **no prediction.** Four waves
   of invariance point at "no effect"; the dataset gap measured in §4 below is the
   largest single delta anyone has changed in this arm. The two considerations
   oppose each other, and a coin-flip prediction adds nothing to the register.

---

## 2. The harvest

Teacher: `r2dreamer/runs/pick_delta25d4_s0/CHAMPION_1576820.pt` (0.91 sampled /
1.00 mode on demo ICs, hardened predicate). Loaded with the checkpoint path from
`r2dreamer/eval_genesis.py`, not a re-implementation. Every action-semantics
parameter is read from the run's OWN hydra config and asserted:
`action_mode=delta_joint`, `action_repeat=4`, `delta_cap=0.025`,
`delta_leash_mult=5`, `scope=pick`, `time_limit` protocol horizon 1200 sim steps.

Script: `baselines/harvest_champion_demos.py` (new).

**ICs.** The uid list is the HUMAN pick-phase demo set (`baselines/episodes_pick_phase`,
66 uids), intersected with the env placements map. **54 of the 66 are resettable**;
12 are the known unresettable uids (233, 259, 262, 266, 267, 275, 278, 301, 319, 321,
329, 333 — FABLE_HANDOFF §8 lists the same family). Model demos therefore occupy the
same ICs the human demos occupy, which is what makes the two sets comparable.

**Policy mode: MODE (deterministic).** Chosen, not defaulted: mode is the champion's
1.00 protocol, and on a FIXED IC a deterministic rollout is one tape — diversity comes
from the IC set, not from action noise. Repeating a mode rollout on the same IC adds
no information (the same class of null operation as the deterministic "x3" eval
retired in §5 of the handoff). The script ASSERTS `--attempts > 1` implies
`--mode sample`.

**Recording.** Per SIM step (not per decision):
`states (n,17)` = the obs BEFORE the step; `actions (n,7)` = the ABSOLUTE command the
env executed ([6 joint targets rad, grip 0..1]). Recorded by WRAPPING the inner
`FullTaskEnv.step`, so the delta integration has exactly one implementation (the
r2dreamer adapter's) and cannot drift from a copy. A per-sim-step tape is a STRIDE-1
tape: the teacher's repeat-4 window advances the target by `a*cap` on every sim step,
so the standard stride-1 encoder `delta_encode_transitions` (delta_ref='target')
re-derives the teacher's own per-step action. Verified: re-encoded actions clip at the
+-1 bound on **0.05%** of arm dims (human set: 0.91%; DP-model set: 2.48%). The
teacher's repeat-4 does not leak into the dataset.

**Truncation.** The pick-phase rule, reused from `baselines/make_pick_phase_datasets.py`:
first frame with `can_z > pick_z` AND `grip cmd > GRIP_CLOSED_FRAC`, keep one frame
past it (`end = k+2`). Same rule and same constants as the human arm — the fairness
requirement of the phase-1 comparison. `pick_z` is asserted against the live world.

**Filenames are ROLLOUT INDICES** (>=100000, the `m1all_harvest` convention). The IC
lives in `ic_uid`. This is deliberate: a uid-keyed model set is exactly the near-miss
recorded in FABLE_HANDOFF §3a, where DP-pruned HUMAN demos were nearly launched as the
model arm.

### Yield

| pass | policy | ICs | rollouts | kept | yield |
|---|---|---|---|---|---|
| 1 | mode | 54 resettable | 54 | 49 | 0.91 |
| 2 | sample | 17 (5 pass-1 failures first, then lowest uids) | 17 | 13 | 0.76 |
| 3 | sample, <=3 attempts | 4 | 4 | 4 | 1.00 |
| **total** | | | **75** | **66** | **0.88** |

Passes 2-3 exist ONLY to reach matched-N 66 with the human success set; the fill order
was fixed before running (the 5 failed ICs, then remaining ICs by ascending uid). 51 of
the 66 tapes sit on distinct ICs; 15 ICs carry a second (sampled) rollout. Pass 1
reproduced exactly (49/54, same 5 failures) when the harvest was re-run to add
full-length recordings — the pipeline is deterministic at this granularity.

The 5 pass-1 failures all TIPPED the can. Two tipped at decision 1 (uids 318, 234) —
those ICs start the can in a state the tip rule fires on almost immediately.

---

## 3. Census — SHORTNESS IS THE HEADLINE

`baselines/episodes_champion_pick/` (66 npz, 1.2 MB, RSYNC ONLY — gitignored).

| set | eps | transitions | rewarded | density | length min/med/max |
|---|---|---|---|---|---|
| **champion clean (cell B)** | **66** | **9,118** | **66** | **0.724%** | **97 / 131 / 218** |
| human `episodes_pick_phase_all` (the dH RLPD arm) | 91 | 83,465 | 66 | 0.079% | 172 / 746 / 3695 |
| human `episodes_pick_phase` (success only, matched N) | 66 | 55,863 | 66 | 0.118% | 172 / 560 / 3695 |
| DP-model `m1all_harvest` (the dDP RLPD arm) | 93 | 69,935 | 63 | 0.090% | 223 / 680 / 1200 |
| ManiSkill PickCube motionplanning (the MS control's demos) | 50 | 3,440 | 50 | 1.454% | mean 68.8 |

**Median tape length 131 frames vs the human arm's 746 — 5.7x shorter (4.3x against
the success-only human set at matched N).** The instruction to say so loudly if the
median were NOT well under the human ~660 does not fire: it is far under.

Reward density rises **9.2x** (0.079% -> 0.724%) and lands **within 2x of ManiSkill**
(1.45%). Expected rewarded rows per 128-demo minibatch: 0.10 -> 0.93.

The teacher reaches the hardened pick in **26 / 35 / 57 decisions** (min/median/max) =
104-228 sim steps. Its own mode eval during this session: **8/8 picked, mean 35
decisions** (videos: `scratchpad/cellb/champion_demo_videos/`).

Encoded action distribution: mean |a| 0.273, p99 0.854, max 1.000, clip 0.05%. 33% of
consecutive encoded deltas differ by < 1e-3 — the residual imprint of the teacher's
repeat-4 windows, softened by the leash. The ABSOLUTE command tape has zero repeated
frames (the target advances every sim step), so nothing downstream sees a degenerate
piecewise-constant signal.

### The confound this arm necessarily carries

Short demos mean a SMALL demo buffer: 9,118 transitions vs 83,465. Density and buffer
size move together and cannot be separated inside one arm — the same structure the
hold-reward arm carried (§18: density AND tape set both moved). State it in the
caption. If cell B ignites, "clean demos" is the claim, not "density alone".

---

## 4. VERIFIED-HARVEST GUARDS (project standard)

### (a) Negative control — PASS

Random uniform `[-1,1]^7` decisions, identical pipeline, identical ICs, identical keep
rule, `--teacher random`: **0 keeps in 54 rollouts** (51 timeout, 3 tipped).
`baselines/episodes_champion_negctl/`. The keep predicate cannot manufacture demos.

### (b) Open-loop verify — PASS (4/5), with the measurement's own limit recorded

`sacfd_delta_gate.py --demo-dir baselines/episodes_champion_pick_full --uids
104005 100008 102005 101005 105002 --delta-ref target --action-repeat 1`

The 5 gated rollouts sit on the 5 frozen human GATE_UIDS ICs (308, 325, 297, 326, 265),
so the model gate and the human gate are matched at the IC level.

* TENSOR EQUALITY: RLPD's `DemoData` == encoder output, bitwise. PASS.
* OPEN-LOOP REPLAY: **4/5 re-earn the demonstrated lift. GATE PASS.** The one miss
  (102005) peaks at 0.1495 vs `pick_z` 0.1505 — **1.0 mm short**, the flake tolerance
  MIN_PASS=4 exists for.

**Recorded honestly: the gate on the TRUNCATED training tapes fails 3/5 (target ref)
and 2/5 (measured ref).** That result is a property of the MEASUREMENT, and here is the
evidence, not an assertion: replaying each truncated tape and then HOLDING its last
action 60 more steps lifts the can to **0.36 / 0.46 / 0.36 / 0.35 / 0.42** — every one
of the 5 far above `pick_z`. The replay reproduces the grasp; it is a few frames behind
in time, and a pick-phase tape ends 2 frames past the geometric grant with 1-7 mm of
margin. `sacfd_delta_gate`'s own GATE_UIDS note already documents this for the HUMAN
set ("MARGINAL-TERMINAL ... fast demos lose that razor-thin margin"), which is why the
human gate uses 5 hand-picked gentle demos. The champion is fast by construction.

The guard therefore runs on the UNTRUNCATED recordings, which is what the guard is for:
it verifies the RECORDING (state/action alignment, encoder-env agreement). The training
tapes are **bitwise prefixes of the verified recordings, asserted 66/66** by
`--merge`. The decision to gate on the full tape was made after seeing the truncated
result; it is recorded here as post-hoc, with the +60-hold evidence that motivated it.

* Same-session regression of the FROZEN human gate (no new flags,
  `episodes_pick_phase_all`, same box, same code): **GATE PASS 4/5**, uid 326 the
  miss — the exact episode §9 recorded as flipping after the P2 reset fix. So under
  the identical guard, **model 4/5 and human 4/5**, on tapes of 117-130 frames versus
  336-877. Backward compatibility of the two gate changes is confirmed by this run.
* Population check (all 66 full recordings, same script, `--all-demos`, stride-1,
  target ref, env's OWN hardened stage grants): **35/66 = 53% re-earn >= picked**
  (34 picked + 1 placed). The project's human baseline under the identical measurement
  is **34/72 = 47%** (FABLE_HANDOFF §7). The clean set is at least as replay-faithful
  as the human set, at 1/5 the length. Neither number is a training-validity claim:
  RLPD never replays demos open-loop. The recorded (s, a, s') tuples are exact by
  construction — in-sim, closed-loop, zero replay gap.

### (c) Provenance — PASS

Every npz carries: `source='r2dreamer_champion_harvest'`, `teacher_ckpt` (absolute
path), `teacher_kind`, `act_mode` (mode|sample), `delta_ref='target'`,
`delta_scale=0.025`, `teacher_action_repeat=4`, `tape_stride=1`, `ic_uid`,
`git_genesis`, `git_r2dreamer`, and a `schema` string. Episodes format:
`states / actions / n / uid(=rollout index) / ic_uid / label='success' /
stage='picked'`. `manifest.json` carries every rollout, kept or not, with its outcome.

`DemoData` load check on the full set: n=9,118, rewarded=66, act_dim=7, obs_dim=17.

---

## 5. Tooling changes (shared files — announced)

`baselines/rl/sacfd_delta_gate.py`, two additive changes; the frozen human gate is
byte-identical when the new flags are absent:

1. `--uids` / `--min-pass` — the gate's uid list was hardcoded, so a harvested set
   (rollout-index stems) could not be gated at all.
2. `_ic_uid(path, fallback)` — a tape is now replayed from the IC it was RECORDED at:
   `ic_uid` when the tape stamps one (harvests), the stem otherwise (every human set).
   Without this the gate raised `KeyError: 104005` from `genesis_can_env.reset`. Loud,
   and only loud because the harvest stamps its IC — a set that did not would have
   been silently replayed from the wrong placement.

`.gitignore`: `baselines/episodes_champion*/` (datasets travel by rsync only).

---

## 6. THE WAVE — launch-ready, NOT launched

**Why not launched:** the MS positive control is running on this box —
`MS_RLPD-ctl_s{0,1,2}`, 3 seeds, 300k, started 10:56, PIDs 2736886 / 2736983 / 2737166.
Per the work order, deliver launch-ready while it runs. GPU headroom exists (1.8 GB of
12 GB, ~28% util), and 6 concurrent RLPD trainers are proven capacity (ar8 wave), so
launching alongside is defensible — but it would slow the control that must resolve
first (RESULTS_MATRIX "What must resolve before a full run", item 1). Owner's call.

**Naming hazard, flagged before launch:** the work order specifies run names
`dH_RLPD-clean_s{0,1,2}`. In this project `dH` means HUMAN demos. These demos come from
the r2dreamer champion — a THIRD source, neither `dH` nor `dDP`. Shipping them under a
`dH_` name re-creates the §3a confusion in the wandb table. Recommended rename:
`dR2D_RLPD-clean_s{k}`. The commands below use the specified names; change the two
`--run-name` strings if the owner agrees.

```bash
cd /home/j/workspace/genesis_pickaplace
for S in 0 1 2; do
  nohup ./.venv-eval/bin/python baselines/rl/train_rlpd.py \
    --steps 100000 \
    --scope pick \
    --action-mode delta_joint \
    --delta-ref target \
    --action-repeat 1 \
    --gamma 0.998 \
    --backup-entropy off \
    --per-member-ln off \
    --pick-hold-reward off \
    --utd 10 --ensemble-size 10 --subset-size 2 --demo-batch 128 \
    --demo-dir baselines/episodes_champion_pick \
    --out-dir baselines/rl/checkpoints/rlpd_clean_s$S \
    --run-name dH_RLPD-clean_s$S \
    --project genesis_paper \
    --seed $S --device cuda \
    > scratchpad/cellb/rlpd_clean_s$S.log 2>&1 &
done
```

Every semantic flag is passed EXPLICITLY (the silent-default rule). The config is the
nb wave's, one variable changed: `--demo-dir`.

Expected startup lines to check before walking away (they are the wave's own gate):

* `[demos] encoder=delta_encode_transitions delta_ref=target cap=0.025 leash=0.125`
* `[demos] 66 eps -> 9118 transitions in the IMMUTABLE demo buffer ..., 66 rewarded`
* `[cfg] RLPD | ... backup_entropy=off per_member_ln=off ... q_watchdog=2.00`

If the transition count is not 9,118 or the rewarded count is not 66, stop: the demo
dir is wrong.

**Post-training:** §4a-2 sweep (15 fresh processes demo-IC + 15 random-IC per seed at
`rlpd_100000_steps.zip`), then score against the §1 bars. Eval videos to the user.

---

## 7. What cell B can and cannot conclude

* **>= 2/3 seeds at the bar** -> the human dataset was a real limit on this arm. It
  does NOT identify WHICH property (length, density, buffer size, and native-action
  encoding all moved together, by design).
* **<= 1/3 seeds** -> five independent manipulations have now failed to move a
  constant. The dataset explanation is spent, and the remaining candidates are the
  task (exploration reaching the grasp) and the budget (100k). This is the stronger
  paper result of the two, and it is why the bar is asymmetric.
* Power: at n=3 per arm and a ~1/3 base rate, a null means "no effect detectable at
  this n and budget", never "no effect". Same caveat as §17 of the handoff.
* Cell B is a POSITIVE CONTROL on the data axis. It shares the MS control's job:
  make the RLPD row credible. Neither run is a paper row on its own.

## 8. Artifacts

| what | where |
|---|---|
| harvest script | `baselines/harvest_champion_demos.py` |
| clean demo set (66) | `baselines/episodes_champion_pick/` — RSYNC ONLY |
| full recordings (66) | `baselines/episodes_champion_pick_full/` — RSYNC ONLY |
| negative control | `baselines/episodes_champion_negctl/` (0 keeps, manifest only) |
| harvest logs | `scratchpad/cellb/h2_p{1,2,3}_s*.log`, `negctl_s*.log` |
| gate logs | `scratchpad/cellb/gate_full_target.log`, `gate_champion{,_measured}.log` |
| population sweep | `scratchpad/cellb/sweep_full_s*.log` |
| teacher videos (8/8 picked) | `scratchpad/cellb/champion_demo_videos/` |
