# Notes for Fable — 2026-07-30

Handoff of open problems. Ordered by how much they block the ouroboros experiment.
Everything here is reproducible on the dev box (`.venv-eval`) unless noted.

---

## 1. verify-rejection in the harvester — LIKELY RESOLVED, needs confirmation

**Symptom.** Cluster negative control, random teacher, 50 rollouts:
`kept 3/50, 7 rejected by verify` — "KEPT-by-rollout but FAILED verify".

`harvest_ai_demos.verify()` resets the env to the *same* IC and replays the
*recorded* actions open-loop. Genesis is deterministic (measured: 0.00 mm cross-env
spread in the batched parity test), so replay should reproduce the rollout exactly.
7/50 disagreeing means one of:

- **(a) the sim is not deterministic across `reset()`** — some state (contacts,
  solver warm-start, RNG) survives a reset and perturbs the replay;
- **(b) state/action recording is misaligned** — the recorded `a_i` is not the action
  actually applied at `s_i` (an off-by-one in `rollout()`);
- **(c) the successes were spurious** — transient predicate firings that a replay
  doesn't reproduce because they depended on a fleeting configuration.

**Why it matters.** `--verify` is the *anti-manufacturing guard* for the entire
ouroboros. If it rejects genuine successes we lose data and bias the harvest; if it
*passes* things it shouldn't, generated datasets are contaminated. Either way every
generational result inherits the doubt.

**What I already did (2026-07-30, commit f44fe71).** Added `MIN_KEEP_FRAMES=100` —
a 15-frame "pick" appeared in that run, 5 steps after reset, when a genuine pick
takes ~600 frames (median demo pick frame 666).

**UPDATE (same day, after the fix):** the cluster negative control re-ran at
`kept 0/50 (0%), 0 rejected by verify` — down from 3 kept / 7 verify-rejected. So on
current evidence the rejections were **case (c)**: transient predicate firings on
implausibly short episodes, which replay does not reproduce. Cases (a) and (b) are
*not* implicated, and the bisect below is now a **confirmation** task rather than a
hunt. Still worth doing once against a REAL (non-random) teacher harvest, because a
random teacher may simply never produce the long marginal episodes where a genuine
misalignment would show: check `rejected_by_verify` in the first real harvest's
manifest, and only escalate to the state-by-state bisect if it is non-zero.

**Suggested experiment.**
1. Instrument `rollout()` to also record the *post-step* state, then replay and
   compare state-by-state. First divergence index localises it: divergence at step 0
   ⇒ reset non-determinism; divergence at the first *action* ⇒ off-by-one.
2. Run the same IC + same action sequence twice through fresh envs in one process,
   and again in two separate processes. Same-process agreement with cross-process
   disagreement points at warm-start/solver state.
3. Check whether rejections correlate with episode length (short ⇒ case (c)) now
   that the min-frames guard is in.
4. `rejected_by_verify` and `rejected_too_short` are both in every harvest
   `manifest.json` — compare across a real (non-random) teacher harvest.

Relevant: `baselines/harvest_ai_demos.py` (`rollout`, `verify`), and the historical
note in `CLAUDE.md` about `env.step`'s `max_steps` firing `_nested()` mid-tape — the
same class of bug (hidden state surviving into a supposedly clean replay).

---

## 2. UNSOLVED: cartesian BC fails even in-distribution

**2026-07-31 PIVOTAL UPDATE — the confound was the DATA SOURCE, not the
representation.** The 2×2's own control cell settled it: `jobs_jact` (joint obs +
joint actions, the exact format of the 0.67–0.80 positive control) trained on
`episodes_cartesian_dual` reads **0.13/0.00/0.00 picked** on the cluster and
**0.07 in a local replication** — dataset provenance alone moves joint DP from
0.73 to 0.07. The dual source replays `{uid}_cartesian.npy` (bag `joint_pos`
streams) rather than the proven vel-cmd replay; measured differences: ~20% more
frames per demo with an INCONSISTENT time base (ratio 1.02–1.34 across demos),
and smoother actions (measured positions, 0.0022 vs 0.0026 rad/step commanded).
Consequences:
- Every `episodes_cartesian*` dataset shares this replay lineage, so the entire
  "cartesian BC fails" ledger below is confounded — the six killed hypotheses
  were all tested WITHIN the weak source.
- The 2×2 as built cannot answer obs-vs-action. Rebuild path: re-collect the dual
  representation DURING the proven vel-cmd replay (read `w['eef']` per frame in
  the episodes_all collector; derive abs6 as pose-reached-at-i+1), then re-emit
  the four cells and rerun. Until then, no cell result is interpretable.
- The 12 cluster x2x2 trainings + evals measured the bad source; treat as void.

**The measurement.** Diffusion Policy, 100k steps, dual eval (demo ICs / random ICs):

| action space | in-dist | random |
|---|---|---|
| **joint (6 angles + grip)** | **0.67** | 0.13 |
| 4-DOF vel  (roll/yaw pinned) | 0.07 | 0.067 |
| 4-DOF delta                  | 0.00 | 0.00 (×3 seeds) |
| 4-DOF abs                    | 0.07 | 0.00 |
| **6-DOF abs (`abs6`)**       | **0.00** | 0.00 |
| **6-DOF delta (`delta6`)**   | **0.00** | 0.07 |
| ACT (4-DOF delta)            | 0.00 | 0.00 |

Joint works. *Every* end-effector encoding fails, including `abs6`, which is
FK-equivalent to absolute joint targets.

**Five hypotheses I tested and killed:**
1. *Delta control self-corrects* — **wrong**. Anchoring the setpoint to the measured
   pose bounds setpoint-vs-arm drift but does nothing about position error; a
   relative command is equally valid wherever the arm is. Rollout drifts 15 cm from
   the demo path by t=150.
2. *4-DOF cannot express the demos' wrist* — **real but not sufficient**. The teleop
   commanded only pitch (verified `wx=wz=0` in 25/25 demos); roll/yaw drifted through
   the real controller's null space up to 1.03/1.34 rad, and our env *pins* them.
   Giving full orientation control (`abs6`) did not fix BC.
3. *IK null-space wander* — **refuted**. Replaying a demo's own `abs6` actions puts
   the arm within 1–3.5° mean (max 10°) of the demo's joint configuration.
4. *Quaternion discontinuity in the obs* — **refuted**. 0 sign flips in 30 episodes.
5. *Command lead / feed-forward* — **refuted**. EE moves ~1.2 mm/step in both.

**What is verified working**, so the fault is narrow:
- Open-loop replay of derived demo actions picks 4/4 for `abs6` and `delta6`.
- The trained policy reproduces demo actions well *when fed demo states*
  (grip 0.80 vs 0.80; xyz cosine 0.66–0.84).
- Tracking is excellent in `abs6`: |tool − target| 0.4–1.35 cm, **0 IK jumps**.
- Closed-loop, the *commands themselves* diverge: 0.4 → 7.8 → 30 → 38 cm.

**So:** execution is fine, one-step prediction is fine, but the closed loop diverges
in EE space and not in joint space. The remaining difference between the working and
failing conditions is the **observation representation** (17-dim joint obs vs 18-dim
ee-centric obs) — which changed at the same time as the action space and has never
been isolated.

**Suggested experiment (the missing 2×2).** Train DP on:
`{joint obs, ee obs} × {joint actions, abs6 actions}`.
Harvested demos now carry **both** representations (`states_joint`/`actions_joint`
alongside the cartesian ones, commit a1dcd82), and `episodes_cartesian*` all derive
from the same tapes as `episodes_pick_pruned`, so all four cells are buildable
offline. If `joint obs + abs6 actions` works, the fault is the observation; if
`ee obs + joint actions` fails, likewise. That single experiment should end this.

---

## 3. dv3 reward hacking on the tip penalty

User observation (cluster, ~6 h in): runs go from "obviously knocking the can over"
to **0 reward** — i.e. the policy learned that *not engaging the can* (0) beats
engaging it (−0.5 tip penalty). Classic sparse-reward-plus-penalty local optimum: no
positive signal is reachable nearby, so avoidance dominates.

Suggested: set `TIP_PENALTY = 0.0` (keep the *termination*, drop the penalty) — the
user proposed exactly this early on and the evidence now supports it. Defined in
`baselines/rl/full_env.py` (`CartesianFullTaskEnv.TIP_PENALTY`) and mirrored in
`baselines/genesis_vec_env.py` (`C_TIP_PENALTY`) — **both must change together**, and
the demo converters stamp −0.5 too (`to_dreamer_demos_cartesian.py`,
`relabel_cartesian.py`), so demos and env must stay consistent or the world model
learns contradictory dynamics.

---

## 4. Silent-default bugs: five found this week, expect more

Every one produced *plausible but meaningless numbers* rather than a crash:

| default | consequence |
|---|---|
| `env.step(max_steps=1200)` | fired `_nested()` mid-tape; corrupted every demo >1200 frames |
| `PICK_CAP=300` | truncated closed-loop DP harvests (2.5% yield vs 33% eval rate) |
| `control='vel'` in both RL trainers | **every** "cartesian RL" run was velocity-mode |
| `--control` missing in the eval subprocess | delta policies evaluated under velocity semantics |
| `--control` missing in `harvest_ai_demos` | a delta/abs6 teacher would harvest garbage |
| grip column `a[:,4]` on 7-dim actions | found ×3 (audit caught the 3rd in `relabel_cartesian`) |
| 2×2 eval env keyed off the OBS half of the cell name | mixed cells would eval in the wrong env: `jobs_eact` runs abs6 targets through the joint env (dim-compatible!) → ~0.00 regardless of truth. Found 07-31, BEFORE any number was read (poisoned env had killed the evals). Fix: env follows the ACTION half; `wandb_eval --obs` supplies the state layout |

Pattern: a parameter added with a backward-compatible default whose default encodes a
*semantic* choice. Mitigation applied: every cartesian call site now passes `control=`
explicitly. **Worth a fresh sweep** — I found the last one by auditing rather than by
tripping over it, which suggests others are still latent.

---

## 5. Dataset integrity (fixed, but check anything old)

`convert_to_lerobot.py` never called `ds.finalize()` before 2026-07-25, so metadata
was truncated at interpreter exit (pyarrow already torn down). Result: datasets that
train fine on one `datasets` version and raise
`IndexError: Invalid key: 64 out of bounds for size 60` on another — corrupt
artifacts that only fail on *another machine*. The cluster smoke hit exactly this.

Converter now asserts `episodes-table rows == info.json total_episodes` (commit
f5419c2). Swept: `lerobot_dataset_{pick,pick_pruned,fulltask,v4}` were truncated (all
pre-fix); everything built after 07-25 is clean. `pick_pruned` rebuilt 66/66.

**Consequence worth remembering:** the headline **joint DP 0.67 in-dist was trained
on 60 of 66 demos**, not the full set. Valid, but mislabeled — re-run before it goes
in a paper.

---

## 6. Smaller concerns

- **`eval/*` changed meaning on 2026-07-30.** Before: random-IC (joint DP 0.13).
  After: demo-IC (joint DP 0.67). Same policy, 5× different number. Do not compare
  across that date. `eval_random/*` and `eval/gen_gap_picked` are still logged.
- **`delta6`'s dv3 path has never completed a run** — open-loop gated and
  compile-checked; the local smoke died on a GPU OOM from contention, not a code
  fault. Unverified at scale.
- **PNG-frame image datasets are less battle-tested than video.** It's the path where
  the missing-`mkdir` bug lived (fixed in the `genesis-fixes` lerobot branch). The
  cluster auto-selects PNG when FFmpeg is absent — run one image lineage through the
  smoke before trusting it in an overnight queue.
- **The ouroboros launcher has never completed a full generation** on the cluster.
  It has cleared: submission, preflight, training, eval, and the negative-control
  gate. Not yet cleared: harvest → convert → gen-1 submission.
- **Cartesian lineages cannot seed a chain** (teacher at 0.00 harvests nothing). The
  queue is joint-only until §2 resolves.
- **`no-pick` early-abort at 700 steps** (`NO_PICK_ABORT`) predates the pick-scope
  work and may be truncating slow-but-real picks in `--scope full` harvests.


---

## RESOLVED 2026-07-31: the dv3 cluster segfault saga

**Root cause:** `install_lerobot.sh`'s `conda install -c conda-forge ffmpeg` (ran
11:38 on 07-30) pulled conda-forge's libstdc++ into the env, breaking the C++
runtime genesis/taichi mesh loading was built against. Every genesis run after that
moment segfaulted at scene build on EVERY architecture; a meshless plane scene
passed, which misdirected the investigation.

**Fix:** `conda install --revision 0` + pip-only reinstall (installer no longer
touches conda, ever). `cluster/verify_env.sh` passes all 4 checks -- including the
full-world build, ON AN H200.

**Hypotheses burned en route (in order):** A100 architecture, code regression,
taichi cache contention, env-count, hardware-specific L40 issues. The A100 theory
was refuted by timeline, not by test -- all its evidence came from inside the
poisoned window. Retest A100 before excluding it.

**Lesson for the file:** when a system that worked stops working, list every
mutation to the ENVIRONMENT in the failure window before theorizing about
code/hardware. The 11:38 conda install was in `conda list --revisions` the whole
time.
