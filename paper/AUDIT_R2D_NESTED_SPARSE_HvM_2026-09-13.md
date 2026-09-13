# Audit brief: {r2dreamer} `nested_sparse`, human v machine demonstrations (2026-09-13)

Written for an auditor with cluster access and no session context. Every number is from a file on the cluster;
the path is given beside it. `$LAB` = `/cluster/tufts/shortlab/jstale02`, `$W` = `$LAB/wm_fix_2026-09-03`.

## 0. The claim under audit

Under a sparse reward that pays +1 once for the full task (`home` = the can was carried to the shelf, set down,
slid to the goal can and left nested) and nothing else, a DreamerV3-style world-model learner seeded with **human**
demonstrations reached the full task on 3 of 4 seeds, and the same learner seeded with **machine** demonstrations
(a Diffusion Policy teacher's rollouts from the same starts) reached it on 0 of 4.

| arm | seeds | 4M rnd30 MODE `home` per seed | ignited | mean rate |
|---|---|---|---|---|
| human `dHfull_all_rnsh` | 955 956 957 958 | 20/30, 0/30, 16/30, 13/30 | 3/4 | 0.408 |
| machine `dDPfull_first_rnsh` | 975 976 977 978 | 0/30, (no 4M cell; 0/30 at 2M), 0/30, 0/30 | 0/4 | 0.000 |

Fisher exact two-sided on ignition 3/4 v 0/4: p = 0.143. **This contrast was not a registered prediction**
(§7.9); the registered extension (+4 v +4, PHASE_PLAN (aa) rev 5) is the confirmatory test.
Cells: `$W/ln_milestone_cells/full_r2d_state_<set>_s<seed>/online_4000000/rnd30_mode/metrics.json`.
Table: `python3 $W/ln14_milestone_table.py`. Curves: `HRI_results/curves/ladderN_2026-09-13/`.

## 1. Pipeline, end to end

```
real teleop bags (74 trials, 2024-12-18)        DP teacher trained on the PRUNED human set
        |  recovered can starts                          |  rolled out from 72 of those starts,
        |  (trial_placements.json)                       |  FIRST attempt per start kept
        v                                                v
  dHfull_all (74 tapes)                          dDPfull_first (72 tapes)          <- demos_state_full/, 2026-09-11
        |            re-executed once in the sim with no termination -> per-frame stage RECORDS
        v                                                v
  dHfull_all_rnsh (74)     relabel_reward.py --from-records --ladder nested_sparse --tip-guard not_in_hand
  dDPfull_first_rnsh (72)  (reward rewritten offline by the SAME accountant the env runs; actions untouched)
        |                                                |
        v                                                v
  r2dreamer train.py  env=genesis_full_state  env.ladder=nested_sparse  4M online sim steps, 4 seeds per arm
        |  demos prefill the replay buffer (FIFO 5e5 rows: ALL demo rows are evicted by 0.5M online steps, §7.1)
        v
  milestones online_{500000,1000000,2000000,4000000}.pt + latest.pt
        |
        v
  eval_genesis.py, fresh process per cell, CPU, 64-core node, mode actions, 30 random starts (rnd30) / 15 hold starts
        |
        v
  per-episode stage flags (picked, placed_v2, farside, slide_event, home, tipped) -> ln14_milestone_table.py
```

## 2. Demonstrations

**Human** (`$W/demos_state_full/dHfull_all`, then `_rnsh`): the 74 in-the-wild teleoperation trials of the
2024-12-18 session (Kinova gen3-lite, joystick, ROS bags; `inthewild_trials/`). Each trial's can start was
recovered by search (`CAN_STARTING_POSITION.md`; `can_pos_recovery/trial_placements.json`) and the trial's joint
trajectory re-executed in the simulator as delta-joint commands (contract v1, `action_repeat 4`, `delta_cap 0.025`,
`delta_ref target`). ALL 74 are kept, including the 10 that never pick. One human. Participant count is not
recorded anywhere (`paper/EARLY_TRIALS_ROTATION_2026-09-08.md`, "PARTICIPANTS"); "human demonstrations" here is
one operator over one afternoon.

**Machine** (`$W/demos_state_full/dDPfull_first`, then `_rnsh`): rollouts of a Diffusion Policy teacher
(`dp_phase/dHfull_pruned_DP_s0`) from 72 of the human starts. The teacher was trained on the PRUNED human set
(raw minus the 10 no-pick tapes, idle collapsed; verified 195/195 tapes, `paper/DP_E2E_PRUNED_LOG_2026-09-11.md`).
The set of record keeps the FIRST attempt per start (`--one-per-ic-first`, PHASE_PLAN (v)); the earlier best-of-3
set is not used here. Tapes are capped at 600 decisions.

**Relabelled sets** (`$W/demos_state_full/<set>/manifest.json`, built 2026-09-11 17:45/17:50 on pax146, job 3575953):

| | human `dHfull_all_rnsh` | machine `dDPfull_first_rnsh` |
|---|---|---|
| tapes | 74 | 72 |
| decisions | 29,221 | 36,834 |
| tapes that pick / place / reach farside / slide_event / **home** | 65 / 40 / 38 / 25 / **12** | 64 / 41 / 40 / 23 / **12** |
| end reasons | home 12, tipped 22, stream_exhausted 26, truncated 14 | home 12, tipped 23, truncated 27, stream_exhausted 10 |
| Σ reward under `nested_sparse` | 12.0 | 12.0 |
| Σ reward under the old staged ladder | 118.0 | 131.0 |
| actions sha256 (identical to the source set) | 77bc4875… | 671614c5… |

Both arms demonstrate the rewarded event exactly 12 times. The arms differ in tape length (machine tapes are
longer: 37,488 v 29,406 prefill rows) and in everything else that distinguishes a teleoperator from a policy
(idle fraction 0.365 human v 0.007 machine, `paper/PHASE_PLAN_2026-09-04.md` (n)).

How relabelling works: each tape was re-executed ONCE with no termination and every frame's stage predicates
recorded (`$W/stage_records_2026-09-11/<set>/`); `relabel_reward.py --from-records` runs the ladder accountant
over those records offline. Verified offline == direct on 146/146 tapes (`paper/LADDER_N_PILOT_LOG_2026-09-11.md`).

## 3. Environment and reward

- `FullTaskEnv(scope='full')`, `baselines/rl/full_env.py` in `$LAB/gp_ladderN` @ a40c8aa1 (sha256 of the three
  files that define the task are in every run's `ladder_provenance.json`: `full_env 23fe428f…`,
  `genesis_can_env 40544bf7…`, `stage_predicates a589b4f0…`).
- World `gc_kp4_riser3_shelf6` (`baselines/sim_variants.py`; kp `[800,800,600,400,240,240]`, riser 0.03, shelf 6;
  printed as `[sim-variant]` in every job log). Genesis 0.2.1.
- Observation: 17-d state = 6 joint positions, gripper motor (0–1), grip effort, can position (3), can quaternion
  (4), goal xy (2) (`genesis_can_env.py::_obs`). No images.
- Action: 6-d delta joint targets, cap 0.025 rad, plus gripper; `action_repeat 4` (one decision = 4 sim steps).
- Episode: 1200 sim steps = 300 decisions, or terminal.
- Ladder `nested_sparse` (`full_env.LADDERS`): `stage_reward={'home': 1.0}`, terminal on `home` and on `tipped`.
  `home` := `slide_event ∧ nested_v2`. `slide_event` := released, then tool on the far side of the can (reach
  0.10 m / 60° cone), then ≥ 1 cm goalward motion after the set-down latch. `nested_v2` := settled predicate
  (can within 0.081 m of the goal can, both upright, at rest) — NOT the withdrawn training proxy.
  `paper/NESTED_V2_PREDICATE_2026-09-10.md`, `paper/LADDER_N_DEMO_CHECK_2026-09-11.md` (fires on 13/13 human
  and 14/14 machine sim-slides, 0 false positives on the demo set).
- Tip rule: terminate, penalty 0, when tilt > 60° and the can is not in hand, sustained 4 frames
  (`tip_guard=not_in_hand`, `paper/TIP_RULE_2026-09-11.md`).
- Stamp every run prints and stores: `unified-2026-09-10 | ladder=nested_sparse | home=1 | max_return=1 |
  terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f`.

## 4. Learner

r2dreamer = a PyTorch port of DreamerV3 (`$W/r2dreamer_ladderN` @ 0cf3d9e; bundle
`cluster/bundles/r2dreamer_ladderN_2026-09-11.bundle`). Exact command (from `$W/slurm/ln_r2_sparse_dH_s955_3581574.out`):

```
$LAB/r2d_venv/bin/python train.py env=genesis_full_state seed=955 env.steps=4000000 \
  env.demo_dir=$W/demos_state_full/dHfull_all_rnsh env.ladder=nested_sparse env.far_release=false \
  env.tip_guard=not_in_hand env.return_clamp=1.0 model.return_clamp=1.0 buffer.max_size=5e5 \
  logdir=$W/runs/full_r2d_state_dHfull_all_rnsh_s955 env.actor_dist=bounded_normal env.act_entropy=3e-5
```

Config `configs/env/genesis_full_state.yaml` + `configs/model/size12M.yaml` + `_base_.yaml`: RSSM deter 2048 /
stoch 32×classes, units 256 (9.6M parameters), batch 16 × 64, `train_ratio 512`, `env_num 6`, `imag_horizon 15`,
`horizon 333` (discount 0.997), lr 4e-5, slow critic EMA 0.02, `actor_bc_lambda 0` (no BC term), actor
`bounded_normal`, entropy 3e-5, **return clamp 1.0** (λ-return targets capped at the ladder's max return —
`max_return` is derived from the ladder registry and the launcher refuses a mismatch: `[ladder] return_clamp=1.0
(env and model agree)`). Return normalisation: `ReturnEMA` scale = clip(p95 − p05, min 1.0) (`networks.py:390`).

Demonstrations enter ONLY as replay-buffer prefill: `Demo prefill: episodes 74, transitions_added 29406,
rewarded_terminals 0` (human) / `episodes 72, transitions_added 37488, rewarded_terminals 0` (machine), from
`console.log`. No re-injection, no duplication, no BC loss. See §7.1 for eviction.

Budget: 4,000,000 ONLINE sim steps after prefill (`step_contract.json`: `prefill_counter_origin 117624` human /
`149952` machine; milestones at 0.5/1/2/4M online). Seeds and jobs (`sacct`): human s955 3581574 (pax008),
s956 3581575 (pax105), s957 3596021 (pax007), s958 3596022 (pax011); machine s975 3581576 (pax049), s976 3581577
(pax049, CANCELLED at ~3.93M — §7.6), s977 3596023 (pax049), s978 3596024 (pax007). One GPU per job, QOS normal,
no packing. s955/956/975/976 are amendment (aa) rev 2 (submitted 2026-09-11 19:00); s957/958/977/978 are
(aa) rev 3 (submitted 2026-09-12 ~01:30). Same trees, same command template, same sets.

## 5. Evaluation

`$W/ln14_milestone_eval.sbatch` → `eval_genesis.py --checkpoint <milestone>.pt --episodes 30 --mode mode
--max-steps 1200 --ic-file $GP/baselines/eval_ics.json --ic-set rnd --seed 0 --device cpu` (and `hold 15 mode`,
`rnd 30 sample`). Fresh process per cell; 30 episodes sequentially in one process (the same shared-process
protocol as PHASE_RESULTS §5.1). Pinned to 64-core nodes (36-core AVX2 nodes reproduce differently,
`paper/CONTACT_PUSH_2026-09-07.md`); node and core count stamped in every `metrics.json`. The tree is taken from
the run's own `ladder_provenance.json` (fix c330e60 — before it, (ac) cells failed `unknown ladder`).
`eval_ics.json` md5 `fa19897e4637…` is identical in every tree. `rnd` = 30 random can starts; `hold` = 15 demo
starts (14/15 are training starts — NOT held out, `REVIEW_GUIDE_2026-09-07.md` §8). `mode` = the actor's mean
action; the world model still samples a latent inside `act`, so "mode" is not run-to-run deterministic in
general — but every `final` cell (in-job `fresh_eval_*`) reproduced its 4M milestone cell exactly.

Per-episode outcome = the sticky stage flags from ONE `end_of_episode()` call; `home` is the unified predicate,
never the proxy. The evaluator also writes one mp4 per episode (`ep<k>_rnd<k>_<outcome>.mp4`), which is what
the delivered reels are cut from.

## 6. Statistic

Per-seed `home` count on rnd30 MODE at the 4M milestone; ignition = ≥ 1 `home` in the cell. The pre-registered
rev-3 rule (PHASE_PLAN (aa) rev 3) compares RECIPES by ignition count ("ignited if ≥ 2 of 4 seeds per arm");
the human-v-machine comparison on this arm uses the same count and Fisher's exact test, plus exact permutation
on the per-seed rates once n = 8 v 8 (rev 5).

## 7. Threats to validity — status of each

1. **Demonstrations leave the buffer at 0.5M steps.** `buffer.max_size=5e5` rows FIFO; the prefill note says
   "all demo frames gone by 500000 online env steps". The config header still says "nothing is ever evicted",
   written for the 1M design. Ignition happens at 2M+, so for ≥ 87 % of training neither arm has any
   demonstration in replay; the arms differ only through what the first 0.5M induced in the world model, critic
   and actor. This does not invalidate the contrast but changes what it measures (a seeding effect, not
   continued imitation). Same for both arms.
2. **`is_terminal` as recorded.** Relabelled sets keep the tape's recorded terminal flag (`rewarded_terminals 0`),
   so the world model sees the demonstrated +1 as a non-terminal reward and can bootstrap past it. Symmetric.
   `LADDER_IMPL_NOTES §4`; accepted as second-order by the (ac) audit.
3. **Machine tapes are a distillation of the human tapes** (teacher trained on the pruned human set) and are
   longer (600-decision cap, 37,488 v 29,406 rows). Both arms demonstrate `home` 12 times. Not a matched-length
   design.
4. **Sample size.** n = 4 v 4; Fisher p 0.14; MDE on a rate difference at n = 4 is ≈ 0.4. Directional only.
5. **Bimodal outcomes.** Every un-ignited seed on either arm ends 4M with ZERO picks after 3–9/30 at 0.5–2M
   (`HRI_results/curves/ladderN_2026-09-13/README.md`). A seed mean mixes two populations; report ignition
   count and the ignited seeds' rate separately.
6. **s976 stalled** at ~3.93M (CG contact-solve stall, CONFOUNDS 85; 105 % CPU / 0 % GPU, cancelled). It has
   0.5/1/2M cells (all 0 `home`, 0 picks at 2M) and `latest.pt` at 3.93M, unscored. The 0/4 machine count
   treats it as un-ignited on its 2M cell.
7. **Checkpoint non-stationarity.** `paper/R2D_LIVE_VS_RELOAD_2026-09-12.md`: a single milestone can land in a
   trough. Mitigation here: the training curves (per-episode records, `records/`) agree with the cells — the
   three ignited seeds hold 0.6–0.9 training `home` over the last 1M steps; the un-ignited ones are at 0.
8. **Start-set issues.** 4 of the 30 `rnd` starts sit inside the shelf footprint so `placed_v2` fires at reset
   (CONFOUNDS 82, symmetric); `hold15` is not held out. `home` cannot fire at reset.
9. **Registration.** The RECIPE question (ramp v sparse) and the ignition rule were registered before
   submission ((aa) rev 2/3). The human-v-machine SOURCE contrast on `nested_sparse` was not a registered
   prediction of (aa); this document is the first place it is stated as a claim. (aa) rev 5 (+4 v +4, registered
   2026-09-13 before submission) is the confirmatory test with its prediction stated in advance.
10. **Trees.** All 8 runs: `$LAB/gp_ladderN` @ a40c8aa1 (git describes it `-dirty`: the tree carried the
    committed-but-uncommitted `r2dreamer` symlink/launch files at submission; the three task files' sha256 are
    stamped and identical across all 8 runs) and `$W/r2dreamer_ladderN` @ 0cf3d9e. Training nodes differ per
    seed (GPU nodes are not pinned); evaluation nodes are pinned to 64 cores.
11. **What the videos show.** `~/data/genesis_pickaplace/videos_sparse_2026-09-13/` on the pop-os box: `home`
    episodes are pick → carry → set-down beside the goal → push into contact; the same 8 random starts tip
    across all three ignited seeds (start geometry).

## 8. Reproduce

```
ssh pax; cd $W
python3 ln14_milestone_table.py | grep rnsh                    # every cell of record
cat runs/full_r2d_state_dHfull_all_rnsh_s955/ladder_provenance.json
grep -m1 "Demo prefill" runs/full_r2d_state_dHfull_all_rnsh_s955/console.log
sed -n 6p slurm/ln_r2_sparse_dH_s955_3581574.out                 # the exact train command
python3 -c "import json;print(json.load(open('demos_state_full/dHfull_all_rnsh/manifest.json'))['tapes_granting'])"
# re-score one cell (CPU, ~1 h):
RUN=full_r2d_state_dHfull_all_rnsh_s955 MS=online_4000000 CELLROOT=$W/audit_rescore sbatch ln14_milestone_eval.sbatch
```

## 9. Auditor checklist

- [ ] Both `_rnsh` manifests: `actions_sha256` equals the source set's (`demos_state_full/<src>/manifest.json`).
- [ ] All 8 `ladder_provenance.json`: identical `stage_reward`, `terminal_stages`, `tip_guard`, three sha256s.
- [ ] All 8 `[ladder] return_clamp=1.0 (env and model agree)` lines in the job logs.
- [ ] Every 4M cell's `metrics.json`: `cores 64`, `ladder_stamp` matches the run's, 30 episodes.
- [ ] Per-episode `home` implies `slide_event` and `nested_v2` in every cell (implication check).
- [ ] Training-record curves (`HRI_results/curves/ladderN_2026-09-13/records/`) agree with the cells per seed.
- [ ] Watch three `home` mp4s from different seeds and one from a different start; confirm push, not drop.
