# r2dreamer positive control — OUR fork on ManiSkill PickCube-v1

2026-08-15. Author: Fable (agent). Code: `r2dreamer/envs/maniskill.py`,
`r2dreamer/configs/env/maniskill_pickcube.yaml`, `r2dreamer/ms_control.py`,
`r2dreamer/eval_ms.py` (all new) + a 4-line generalization of
`r2dreamer/demo_prefill.py`; `cluster/install_r2dreamer_ms.sh`,
`cluster/sbatch_r2d_ms.sh` (new, in this repo).

## 1. Why

The r2dreamer arm ignites on the genesis pick task at **6/24 seeds = 0.25**
(95% CI [0.10, 0.47]; local 2/4, cluster wave-1 3/10, wave-2 1/10 —
RESULTS_MATRIX_2026-08-15 "Ignition statistics"). The user's concern, verbatim:

> "the ignition rate is 0.25 pooled — pretty sketchy. Same credibility question
> as RLPD."

Two hypotheses survive: **(a)** our fork has a defect, **(b)** the genesis task
is that hard for a world model at this budget. They are separated by running
*the same fork* on *the benchmark the reference implementation solves*:
ManiSkill PickCube-v1, where `~/workspace/dreamerv3-torch` ignites on
essentially every seed (~200 wandb runs Feb 27–Mar 2, takeoff ~110–137k env
steps, final eval 0.6–1.0 — `MANISKILL_VS_GENESIS.md`).

Pass ⇒ (a) is falsified; the fork reproduces a known-good result and 0.25
becomes a claim about the genesis task's exploration/credit landscape — which
is the paper's claim. Fail ⇒ the fork has a defect that the genesis work
masked, and finding it is worth more than the run costs.

This is a control on the **fork**, so r2dreamer's own algorithm defaults stand
and only the *task shaping* is mirrored from the reference. The boundary
between the two is drawn explicitly in §5.

## 2. Interpreter — a THIRD venv, and why not either existing one

| venv | python | key pins | verdict |
|---|---|---|---|
| `r2dreamer/.venv` (genesis arm) | 3.11.14 | gymnasium **1.2.0**, numpy **2.4.6**, genesis-world 0.2.1 editable | **rejected** — mani_skill pins `gymnasium==0.29.1` and `numpy<2.0.0`; installing it here downgrades both *under the live genesis arm*. numpy 2.4.6 is deliberate there (genesis upgraded past r2dreamer's 1.26 pin; `tools.py` carries the numpy-2 tensorboard shim). |
| `dreamerv3-torch/venv` (has mani_skill) | 3.10.12 | torch 2.9.1, gymnasium 0.29.1, mani_skill editable | **rejected** — r2dreamer requires python >=3.11,<3.12 and torchrl 0.9.2 / tensordict 0.9.1, none present; installing them would churn torch **while three MS RLPD trainers are 3 h into a 14 h run in that venv**. |
| **`r2dreamer/.venv-ms` (new)** | 3.11.14 | r2dreamer's own pyproject pins + ManiSkill's two | **chosen** — zero mutation of either working env; and it is *closer to stock r2dreamer* than `.venv` is (pyproject asks for numpy 1.26; `.venv` drifted to 2.4.6 because of genesis). |

Built pip-only by `r2dreamer/build_ms_venv.sh` (mirrored for the cluster in
`cluster/install_r2dreamer_ms.sh`); full `pip freeze` (124 packages) is kept at
`r2dreamer/.venv-ms/freeze.txt`. Exactly what it contains:

* r2dreamer's pyproject stack at its pins: `torch==2.8.0+cu126` (cu126 index,
  same wheel as `.venv`), `torchrl==0.9.2`, `tensordict==0.9.1`,
  `ruamel.yaml==0.17.4`, `moviepy==1.0.3`, `einops==0.3.0`,
  `hydra-core==1.3.2`, `tensorboard 2.21`, `setuptools==77.0.3`, `wandb`.
* ManiSkill's pins: `numpy==1.26.4`, **`gymnasium==0.29.1`**, `sapien==3.0.2`,
  `mplib==0.1.1`, `fast_kinematics==0.2.2`, `pytorch_kinematics==0.7.5`, plus
  h5py/scipy/dacite/trimesh/transforms3d/tyro/… .
* `mani_skill 3.0.0b21` installed **editable from the local fork**
  `~/workspace/ManiSkill` (`rai-inst/ManiSkill @ 22f39cf`) with
  `--no-deps --no-build-isolation` — the fork is what recorded the shipped
  demos and what every dv3 MS run used; `--no-deps` keeps it from re-resolving
  anything above. Editable installs write only into the new venv's
  site-packages (PEP 660 finder), so the shared source tree is untouched.

**Deviations from r2dreamer's pyproject: exactly two** — `gymnasium` 1.2.0 →
0.29.1 and `numpy` 1.26.0 → 1.26.4. r2dreamer touches gymnasium only through
`gym.Env`, `gym.Wrapper` and `gym.spaces` (grep: `envs/*.py` only), all
identical in 0.29; the direction of travel is toward the *more* permissive API
(1.0 removed `Wrapper.__getattr__`, 0.29 still has it). The gates in §6 are the
evidence that this holds.

Two install footguns, recorded: `setuptools>=81` removes `pkg_resources`, which
`sapien 3.0.2` imports at module load (fixed by r2dreamer's own
`setuptools==77.0.3` pin); and `torchrl`/`tensordict` pull numpy 2 unless the
ManiSkill pin is installed after them.

Nothing else changed: `.venv` still reports numpy 2.4.6 / gymnasium 1.2.0,
`dreamerv3-torch/venv` still reports numpy 1.26.4 / gymnasium 0.29.1 /
mani_skill 3.0.0b21, and `git status` in `~/workspace/ManiSkill` shows only the
pre-existing local modification.

## 3. Setup

| item | value | source of the choice |
|---|---|---|
| task | `PickCube-v1` | the benchmark the reference dv3 solves |
| obs | `rgb+state_dict`, image = base_camera RGB ++ hand_camera RGB → **(64,64,6) uint8** | reference wrapper verbatim (state_dict is requested only so the success predicate can read `obs['extra']`; the policy is **pixels-only**, `cnn_keys 'image'`, `mlp_keys '$^'`) |
| resize | one bilinear `F.interpolate` over all 6 channels, then `clamp(0,255).byte()` | reference verbatim — **the shipped demo tapes were encoded by this exact path**, so rendering natively at 64×64 would shift pixel statistics away from the demos |
| action | `pd_ee_delta_pos` → `Box(4,) [-1,1]` = dx,dy,dz,grip | the demos' native space; the reference recipe |
| robot | `panda_wristcam` | what every demo file and dv3 MS run used (ManiSkill warns it is not in PickCube's supported list and falls back to the `panda` config — benign, and identical in the reference) |
| reward | `force_sparse`: dense zeroed, **+100.0 on success** | reference (`c38qrqga` used force_sparse True; False also reproduces). Also makes the online reward function *identical* to the relabeled demos, and identical in shape to the genesis arm's terminal-sparse reward |
| success | **relaxed**: `is_grasped ∧ ‖obj→goal‖ < goal_thresh` (no static requirement), **terminates** the episode | the reference's `custom_success_check`; `native` (`is_obj_placed ∧ is_robot_static`) is computed every step and logged as `log_native_success` diagnostic |
| horizon | `time_limit 100` agent steps, truncation | reference (`stock ManiSkill registers PickCube at 50`) |
| backend | physx_cpu (num_envs=1 per worker), sapien GPU rendering, headless EGL | one ManiSkill world per r2dreamer spawn worker |
| throughput | **~105 env steps/s** per worker, random policy (measured) | gate (a) |

**Caveat that must travel with every number from this file** (identical to the
RLPD MS control's): the primary metric is the *relaxed* predicate, so it is
**not** directly comparable to a published PickCube success rate, which uses
the static requirement. It *is* directly comparable to the dv3 MS arm, which is
the point.

## 4. Demos — route and census

**Route chosen: (a) the reference teleop npz set**,
`dreamerv3-torch/demonstrations/maniskill/teleop` — the *same 10 human-teleop
episodes* the authoritative working run (`c38qrqga`) trained on. They already
carry everything r2dreamer's `demo_prefill` needs, in its exact conventions:
`image (T,64,64,6) uint8`, `action (T,4) float32` with a zero action on the
`is_first` frame (backward-looking convention), `is_first[0]`, `is_last[-1]`,
`is_terminal[-1]`.

Why not the motion-planning h5 route the RLPD control took: that control needed
**state** vectors and 50 episodes for a 50/50 demo/online replay split; here the
demos must carry **pixels** rendered by this exact camera pipeline, which the
teleop npz already are and the h5 is not (it stores raw 128×128 sensor data in
a different layout). The reference also establishes that 10 clean teleop
episodes suffice (a 1000-episode motion-planning set works too — demo count is
not the bottleneck). `ms_control.py demos --src …/motionplanning --limit 50`
builds the alternative set if it is ever wanted.

`ms_control.py demos` applies **the reference relabel verbatim**
(`dreamerv3-torch/dreamer.py`, `force_sparse` branch: `sparse =
zeros_like(reward); if reward[-1] >= 1: sparse[-1] = 100.0`) and writes
`dreamerv3-torch/demonstrations/maniskill_teleop_sparse100` (rsync-only data,
never git).

```
[demo census]
  source            : demonstrations/maniskill/teleop  (10 human teleop episodes)
  out               : demonstrations/maniskill_teleop_sparse100
  episodes          : 10          frames: 1068
  action_dim        : 4           image: (64, 64, 6) uint8
  lengths min/med/max: 83 / 106 / 133      (8 of 10 exceed the 100-step horizon)
  rewarded frames   : 10  => DENSITY 0.936%
  terminal reward   : +100.0 (relabeled; dense sums before relabel 26.2-65.2)
  verify            : all 10 load through demo_prefill._load_episode with
                      exactly one +100 terminal and is_terminal on the last frame
```

Two honest notes. (1) **8 of 10 demos are longer than the agent's own 100-step
horizon** (median 106) — the human demonstrator is slower than the budget the
agent gets. This is a property of the *reference* setup, not a defect
introduced here: the same 10 tapes, the same `time_limit 100`, produced the
authoritative working runs. Demo frames are dynamics/reward data for the world
model, not episodes the agent must fit inside its horizon. (2) Density 0.936%
is percent-level — 12× the genesis terminal-only arm (0.079%) and in the same
regime as the RLPD MS control (1.45%).

## 5. Ours vs theirs — where the fork stands, and what is switched off

Two boundaries matter. **A** — r2dreamer (this control) vs the dv3 reference on
ManiSkill: everything the fork *is* stays ours; task shaping is mirrored.
**B** — this control vs the genesis r2dreamer arm: plumbing carries over, every
genesis *recipe lever* is off.

### A. this control vs the dv3 reference (`c38qrqga`)

| dimension | dv3 reference | this control | adaptation? |
|---|---|---|---|
| task / obs / action / robot | PickCube-v1, 64×64×6 pixels-only, pd_ee_delta_pos(4), panda_wristcam | **same** | none — mirrored |
| reward | dense zeroed + **100** terminal, relaxed success terminates | **same** | none — mirrored |
| time_limit | 100 | **100** | none |
| action_repeat | 1 | **1** | none |
| batch × length | 16 × 64 | **16 × 64** (r2dreamer's own default) | coincides |
| train_ratio | 256 ⇒ **0.25 updates/env-step** | 256 ⇒ **0.25 updates/env-step** (`Every(1024/256 × repeat)`) | none — same intensity, same arithmetic |
| discount / imag_horizon / act_entropy | 0.997 / 15 / 3e-4 | `horizon 333` (=0.997) / 15 / 3e-4 — all `_base_` defaults | coincides |
| world model | dv3: **reconstruction decoder**, deter 512, stoch 32×32, units 512, cnn_depth 32 | r2dreamer: **decoder-free redundancy-reduced** (`rep_loss: r2dreamer`), size12M = deter 2048, stoch 32×16, units 256, depth 16 | **THE THING UNDER TEST** — ours stands |
| optimizer | adam, model_lr 1e-4, grad_clip 1000 | LaProp, lr 4e-5, AGC 0.3, warmup 1000 (r2dreamer defaults) | fork's own |
| actor dist | dv3 normal | `bounded_normal` (`_base_` default) | fork's own — **and a named suspect**: it tanh-bounds the *mean* only, so samples can leave [-1,1] and be stored raw (the genesis arm added `bounded_normal_clipped` for this). Left STOCK deliberately; if the control fails, this is suspect #1 and that is a finding |
| prefill | 2500 random env steps + demos in the ongoing buffer | demos only (no random prefill in r2dreamer) | structural |
| demo mechanism | episodes in `train_eps`, sampled by the dataset sampler | packed into `env_num` buffer columns via `add_chunk` | structural |
| pretrain | 100 | **100** (r2dreamer's stock default is 0; 100 mirrors the reference and is 0.13% of the run's updates) | mirrored |
| eval | 10 eps / 10k steps, in-loop | **10 eps / 10k steps, in-loop** | none |
| is_terminal at truncation | reference sets `is_terminal = done` (timeouts marked **terminal**) | **truncation is NOT terminal** (critic bootstraps through it) | **DELTA** — r2dreamer's own convention (`envs/genesis.py` + `wrappers.TimeLimit`); the fork's plumbing is what is under test. Also the more standard choice |
| env count | 1 | 4 spawn workers | throughput plumbing |
| budget | 2e5 steps (takeoff 110–137k) | **3.02e5** | more headroom, pre-registered |

### B. this control vs the genesis r2dreamer arm (levers OFF)

| genesis lever | genesis champion (`genesis_pick_v5d4c_delta`) | here | class |
|---|---|---|---|
| env adapter | `envs/genesis.py` | `envs/maniskill.py` | **plumbing — required** |
| demo prefill / `add_chunk` | on | **on** | **plumbing — required** |
| `storage_device: cpu` | cpu | **cpu** (12 GB of uint8 pixels cannot sit on a 12 GB card) | **plumbing** |
| `act_entropy` | 3e-5 | **3e-4** (stock) | recipe lever → OFF |
| `actor_dist` | `bounded_normal_clipped` | **`bounded_normal`** (stock) | recipe lever → OFF |
| `delta_cap` / `delta_leash_mult` / `action_mode` | 0.025 / 5 / delta_joint | n/a — `pd_ee_delta_pos` *is* the benchmark's native control | n/a |
| `reward_scale` | 100.0 (rescale lever) | **1.0** — the +100 is the *env's own* terminal reward here, not a rescale | recipe lever → OFF |
| `return_clamp` | 100.0 | **0.0** (off) | recipe lever → OFF |
| `demo_duplicate` / `demo_reinject_every` | 4 / 150k | **1 / 0** (off) | recipe levers → OFF |
| `actor_bc_lambda` | 0.0 | **0.0** | already off in both |
| `action_repeat` / `demo_downsample` | 4 / 4 | **1 / 1** | genesis time-compression, not needed at 100 steps |
| buffer eviction | FIFO bites (450k cap, 3M steps) | **nothing is ever evicted** (3.2e5 cap ≥ 3.02e5 steps) — same as the reference | consequence of the budget |

Net: the algorithm is r2dreamer's, unmodified and at its own defaults; the task
is ManiSkill's, shaped exactly as the reference shaped it.

## 6. Gates (all run before launch)

**(a) env adapter smoke** — `.venv-ms/bin/python -m envs.maniskill --steps 300`

* `action_space Box(-1,1,(4,),float32)`; obs keys exactly
  `{image, is_first, is_last, is_terminal, log_success, log_native_success}`,
  image `(64,64,6) uint8`, **no `reward` key** (ParallelEnv injects it) ✓
* 300 random steps: all images finite, log_* zero on every non-final step,
  3 episodes each truncating at exactly 100, **random-policy successes 0**
  (negative control) ✓
* reward range `[0.0, 0.0]` under force_sparse ✓
* throughput 103–106 env steps/s ✓

**(a2) success termination fires on a demo tape** — `ms_control.py replay-gate`.
ManiSkill motion-planning tapes stepped through `ManiSkillPixels` from each
episode's recorded reset seed:

```
[replay gate] 5/5 demo action-tapes terminate in relaxed success
   traj_0 seed=0 actions=74 solved_at=69 reward_sum=100.0 rewarded_frames=1
   traj_1 seed=1 actions=74 solved_at=70 reward_sum=100.0 rewarded_frames=1
   traj_2 seed=2 actions=50 solved_at=46 reward_sum=100.0 rewarded_frames=1
   traj_3 seed=3 actions=86 solved_at=79 reward_sum=100.0 rewarded_frames=1
   traj_4 seed=4 actions=76 solved_at=72 reward_sum=100.0 rewarded_frames=1
```

5/5, each paying exactly one +100 and terminating, with `is_terminal`,
`is_last` and `log_success = 1.0` on that frame. **These `solved_at` values are
identical to the RLPD MS control's independently-written gate** (69/70/46/79/72
in `paper/ms_positive_control_2026-08-15.md` §5) and to the first-relaxed index
recomputed straight from the recorded h5 states — three independent
implementations agreeing on the same frame.

> **This gate caught a real bug.** The first run reported `solved_at`
> 56/41/36/46/65 — every tape succeeding ~13 steps early. Cause:
> `np.linalg.norm(_np(obj_to_goal_pos)[0])` computed **|dx|** instead of
> **‖(dx,dy,dz)‖** (the `[0]` was inherited from the reference wrapper, whose
> helper returned an unflattened `(1,3)` batch row). The random-policy negative
> control could never have caught it — a random policy never grasps. Fixed and
> re-gated.

**(b) demo census** — printed in §4. 10 eps / 1068 frames / 10 rewarded
terminals / **0.936%** density / terminal value exactly +100, verified through
the same loader training uses.

**(c) train smoke** — `train.py env=maniskill_pickcube … env.steps=2500
env.env_num=2 env.eval_episode_num=2 trainer.pretrain=10`:

* prefill reported `episodes 10, frames_raw 1068, rewarded_terminals 10,
  terminal_reward_values [100.0], transitions_added 1074, padding 6` and step
  accounting `trainer starts at step 1074; env.steps=2500 -> 1426 online` ✓
* model built from the MS spaces: encoder CNN `{'image': (64,64,6)}`, actor
  shape 4, 11.84 M parameters ✓
* in-loop eval ran and logged `episode/eval_success` and
  `episode/eval_native_success` (**step-0 negative control: 0.00 success**,
  eval_length 100) ✓
* ran to completion, exit 0: 2500 env steps, **360 updates**, no NaN, no
  traceback. World-model losses moving (`loss/dyn` 9.1 → 3.3, `loss/rep`
  9.1 → 3.3, `loss/rew` 5.5 → 3.6) ✓
* **demo frames reach the batches**: `train/data/reward_frames 12`,
  `train/data/reward_sum 1200` (= 12 × 100) — the +100 terminals are sampled
  and `ret_replay_max` is exactly 100.0 ✓
* **checkpoint round-trip**: `latest.pt` (145 MB, agent + optims, `step 2500`)
  loads in a **fresh process** through `eval_ms.py`; 166/166 agent tensors
  finite, zero NaN; 3 deterministic episodes on the pinned eval ICs →
  **0.00 success** (the expected negative control at 2500 steps) ✓
* throughput **15–16 env steps/s at env_num=2 while the three RLPD MS trainers
  saturate the box** — a contended floor, not a forecast; the run is
  update-bound (0.25 updates/env-step) and each worker alone renders at ~105
  steps/s.
* **Measured, and worth recording against the bar**: `train/action_min -4.2`,
  `train/action_max 3.9`. `bounded_normal` really does sample outside [-1,1];
  ManiSkill's controller clips on execution while the buffer and imagination
  keep the raw sample — the exact pathology the genesis arm patched with
  `bounded_normal_clipped`. It is left **stock on purpose** (§5A): if this
  control fails, that is suspect #1 and a finding about the fork rather than
  about the genesis task.

**(d) genesis paths untouched** — `git status` in `~/workspace/r2dreamer` shows
the ManiSkill files as ADDITIONS; the only edit to an existing file is
`demo_prefill.py`, whose genesis behavior is byte-identical:
`_load_episode` gained an `act_dim=7` default (the old hardcoded `7`), the
action dim is now derived from the live agent (`agent.act_dim` = 7 for genesis,
4 here), and a `suite == "maniskill"` branch selects the log-key set. Every
genesis env config, `envs/genesis.py`, `dreamer.py`, `trainer.py`, `buffer.py`
and `train.py` are unmodified by this work. The addition to `envs/__init__.py`
is a new `elif suite == "maniskill"` branch that no genesis config can reach.
(Note the whole genesis port is itself uncommitted upstream, so "zero diff"
here is shown functionally rather than by `git diff` against HEAD.)

Positive evidence, run in the **genesis** venv (`.venv`, untouched):

* `./.venv/bin/python train.py env=genesis_pick_v5d4c_delta --cfg job` resolves
  and exits 0 — the champion genesis config still composes.
* a `genesis_pick_pruned_delta25` demo loads through
  `demo_prefill._load_episode` with the **default** `act_dim=7` and identical
  shapes/dtypes; passing `act_dim=4` raises — the new guard fires on a
  mismatched demo set instead of silently loading one (the silent-default bug
  family this project keeps paying for).

## 7. PRE-REGISTERED BAR

> Registered 2026-08-15, **before any training run past the 2500-step smoke**
> and before any eval beyond the step-0 negative control.
>
> **The r2dreamer fork is VALIDATED iff ≥2 of 3 seeds reach eval success
> ≥ 0.50 on PickCube-v1 by 300k env steps.**
>
> * eval = in-loop `trainer.eval`, **deterministic (actor mode)**, n = 10
>   episodes.
> * **Decision points are the 50k multiples** (50k, 100k, …, 300k): 6 points.
>   The trainer also evaluates every 10k for curve resolution; those extra
>   points are **monitoring only** and do not count toward the bar (30 draws of
>   n=10 against a 0.5 threshold would inflate a null past 60%).
> * "reach" = at any decision point ≤ 300k, not necessarily the last one
>   (post-ignition decay and checkpoint bistability are documented r2dreamer
>   failure modes and would be reported as such, not hidden).
> * A seed that lands within one episode of the bar (0.4 or 0.5 at n=10) is
>   re-measured in a **fresh process** from its archived checkpoint with
>   `eval_ms.py --episodes 20 --mode mode`; the fresh number is the one
>   reported.
> * Basis: the reference implementation solves this exact task, from these
>   exact 10 demos, with a pixels-only policy, at takeoff 110–137k env steps
>   and final eval 0.6–1.0, across ~200 runs. 300k is ~2.2× the reference
>   takeoff in updates (75k updates at 0.25/step vs 28–34k at takeoff).
>
> **Outcomes.** Pass ⇒ hypothesis (a) is falsified: the fork reproduces a
> known-good result on a published benchmark, and the genesis 0.25 ignition
> rate becomes evidence about the *task*, which is the paper's claim. Fail
> (0–1 seeds) ⇒ the defect is in our fork, not in the genesis task, and every
> r2dreamer row must be reported as unvalidated until it is found; the named
> suspects, in order, are (1) `bounded_normal`'s unclamped samples, (2) the
> lambda-return explosion / checkpoint bistability already diagnosed on
> genesis, (3) the decoder-free representation loss itself. A 2/3 pass with
> wide variance is a pass by the letter and will be reported with its variance.

**Protocol caveats, stated up front.**

1. In-loop eval shares the training process. Acceptable here for the same
   reason as the RLPD MS control: the P2 cross-episode-contamination finding
   was a *genesis-env* finding, and ManiSkill resets a fresh scene state each
   episode. The confirmation path is fresh-process anyway.
2. The eval IC set is **not** pinned across checkpoints in the in-loop path
   (`trainer.eval` resets unseeded), so successive points differ in ICs as well
   as in policy — n=10 with a 95% CI of roughly [0.19, 0.81] at 0.5. The
   fresh-process confirmation (`eval_ms.py`) *does* pin ICs (seeds 1e6 …,
   disjoint from training) and is the tie-breaker.
3. Primary metric = the relaxed predicate (§3 caveat): comparable to the dv3 MS
   arm, **not** to published static-requirement PickCube numbers.

## 8. Launch-ready — NOT LAUNCHED

The local GPU is owned by the three MS RLPD trainers (~14 h, ~10 GB free) and
cell B is queued behind them, so no long run was started. Only the gates above
touched the GPU (peak ~3.5 GB for 4 ManiSkill worker envs + an 11.8 M-parameter
model; ~450–600 MB per sapien worker — **budget this**: `env_num 4` +
`eval_episode_num 10` = 14 worker contexts ≈ 7–8 GB, which needs the card to
itself).

### Local route (run when the GPU frees)

```bash
cd /home/j/workspace/r2dreamer
for S in 0 1 2; do
  MUJOCO_GL=egl nohup setsid ./.venv-ms/bin/python train.py \
    env=maniskill_pickcube seed=$S \
    env.demo_dir=/home/j/workspace/dreamerv3-torch/demonstrations/maniskill_teleop_sparse100 \
    env.steps=3.02e5 env.env_num=4 env.eval_episode_num=10 \
    buffer.max_size=3.2e5 trainer.pretrain=100 trainer.save_every=5e4 \
    logdir=runs/ms_pickcube_s$S > runs/ms_pickcube_s$S.log 2>&1 &
done
```

Run them **sequentially** (or with `env_num 2` / `eval_episode_num 5`) if the
card is shared — three concurrent seeds is ~21–24 GB of sapien contexts alone.
Read the result with
`grep eval_success runs/ms_pickcube_s*/console.log` or in tensorboard
(`episode/eval_success`); `sync_runs_to_wandb.py --runs-dir runs --only
ms_pickcube_s0` uploads a run.

### Cluster route

```bash
# 1. rsync (all three are UNCOMMITTED or gitignored -- git cannot deliver them)
rsync -av --exclude runs/ --exclude '.venv*' --exclude wandb/ \
  <devbox>:~/workspace/r2dreamer/            /cluster/tufts/shortlab/$USER/r2dreamer/
rsync -av --exclude '.git/' \
  <devbox>:~/workspace/ManiSkill/            /cluster/tufts/shortlab/$USER/ManiSkill/
rsync -av <devbox>:~/workspace/dreamerv3-torch/demonstrations/maniskill_teleop_sparse100/ \
  <cluster>:~/…/dreamerv3-torch/demonstrations/maniskill_teleop_sparse100/

# 2. build the venv (pip-only; NOT ~/r2d_venv -- see cluster/install_r2dreamer_ms.sh header)
bash cluster/install_r2dreamer_ms.sh ~/r2d_ms_venv

# 3. submit 3 seeds
cd /cluster/tufts/shortlab/$USER/genesis_pickaplace
for S in 0 1 2; do SEED=$S sbatch cluster/sbatch_r2d_ms.sh; done
```

`install_r2dreamer_ms.sh` ends with a **render probe** (`python -m
envs.maniskill --steps 120`), which is the cluster-only risk worth naming:
sapien rasterizes through **Vulkan**, which the genesis arm never exercised. If
it fails, the ICD is the first thing to check
(`VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json`). PickCube needs no
downloaded assets (`~/.maniskill/data` does not exist on the dev box either) —
only the package's own panda URDF.

The sbatch carries the house guards: preempt/requeue with warm restart,
double-submission claim file, background wandb sync, checkpoint archiving at
every `save_every` (so the 50k decision points survive for `eval_ms.py`), and a
final fresh-process n=20 deterministic eval.

## 9. Files

| file | role |
|---|---|
| `r2dreamer/envs/maniskill.py` | the adapter + gate (a) (`python -m envs.maniskill`) |
| `r2dreamer/configs/env/maniskill_pickcube.yaml` | the config, with the lever boundary documented in its header |
| `r2dreamer/ms_control.py` | `demos` (relabel + census, gate b) and `replay-gate` (gate a2) |
| `r2dreamer/eval_ms.py` | fresh-process confirmation eval (the bar's tie-breaker) |
| `r2dreamer/envs/__init__.py`, `r2dreamer/demo_prefill.py` | the only edits to existing files (additive; genesis behavior byte-identical) |
| `cluster/install_r2dreamer_ms.sh`, `cluster/sbatch_r2d_ms.sh` | the cluster route |
| `dreamerv3-torch/demonstrations/maniskill_teleop_sparse100/` | the demo set (rsync only, never git) |

r2dreamer is a **non-git-synced working tree** (deployment is rsync-only); the
work is committed there locally anyway for the record.
