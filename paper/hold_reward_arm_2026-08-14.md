# REWARD-DENSITY arm (pick-hold reward) — build + gates

**Date:** 2026-08-14
**Lever:** per-step HOLD reward at pick scope, replacing the terminal-only +1.
**Why now:** with backup-entropy resolved (ignition restored, §12c) and per-member-LN a
formal null (§12e), reward density is the top remaining lever —
`paper/rlpd_literature_comparison_2026-08-13.md` RQ5 ranks it **#1** of the two
explanatory deltas vs published RLPD.
**Status:** code built + gated, NOT launched (GPU held by the 6 pair trainers).

---

## 1. Semantics

### The gap this closes

| | published RLPD-family | ours (before) | ours (hold reward) |
|---|---|---|---|
| sparse-manipulation reward | +1 **every** solved timestep (Adroit; Ball et al. 2023 return = % of steps solved, following Kostrikov 2022) | +1 **once**, terminal | +1 every held step, terminate at K consecutive |
| episode after first success | continues to horizon (Adroit) / terminates (ManiSkill) | terminates | terminates after K held steps |
| rewarded frames in the prior buffer | Adroit: tens per successful trajectory; HIL-SERL: ~200 classifier positives + interventions | **66 / 83,465 = 0.079%** | **1,966 / 100,148 = 1.96%** |
| expected rewarded transitions in a 128-sample demo minibatch | many | **0.10** | **2.5** |

Precedent, stated exactly (both are in-family, and we take the ManiSkill variant):

* **ManiSkill** pays +1 per *solved* step **and terminates on success** — reward density
  without an infinite hold-forever tail.
* **Sparse Adroit** (RLPD's own sparse-manipulation domain) pays +1 per solved step **to
  the horizon**, so the normalized score *is* the fraction of solved timesteps.

We adopt ManiSkill's shape (pay-per-held-step, then terminate) because our episode is
900 steps at 30 Hz: an Adroit-style pay-to-horizon would make the return dominated by
how long a hold is *maintained*, which is not the pick-scope question and would let a
policy farm return by freezing mid-lift. K = 25 consecutive held frames (~0.83 s at
30 Hz) is the terminal.

### The predicate (one definition, imported by both sides)

`full_env.pick_hold_held(can_z, grip_cmd, pick_z)` — **can above `pick_z` AND gripper
COMMANDED closed**:

* `pick_z` is **always the env instance's** `env.pick_z` (= `genv.w['pick_z']`);
  `train_rlpd` passes `env.pick_z` into the encoder. Never a literal.
* the closure threshold is `pick_env.GRIP_CLOSED_FRAC`, **imported** into `full_env`
  (`from pick_env import ... GRIP_CLOSED_FRAC ...`) and used only there.
* `train_sacfd_full` imports the predicate itself
  (`from full_env import FullTaskEnv, STAGE_REWARD, pick_hold_held`) and calls it on the
  demo arrays; `FullTaskEnv._step_once` calls the same function on
  `obs['state'][10]` and `a_phys[6]`.

So the env-side and demo-side rewards are **one implementation**, not two — which is the
structural reason gate (c) passes rather than a coincidence that gate (c) measures. The
repo's recurring bug family (grip column ×3 files, control mode ×3 boundaries) lived
precisely in re-implemented predicate math; there is no second copy to drift.

**Not the hardened predicate.** `genesis_can_env`'s `picked` additionally requires
`|eef − can| < PICK_EEF_DIST` sustained `PICK_SUSTAIN = 10` frames — a guard added
after the r2dreamer v4 policy learned to *whack* the can airborne with closed fingers.
Two reasons the hold reward uses the honest predicate instead:

1. **The demo tapes cannot compute the hardened one.** The recorded 17-dim state has no
   eef position; the offline relabeler would have to FK it, i.e. re-implement a second
   copy of the very predicate we just unified.
2. **K supplies the same anti-gaming sustain.** A batted can separates ballistically and
   falls back through `pick_z` within a few frames: it can collect a handful of +1s but
   never the K = 25-frame terminal, and 25 > PICK_SUSTAIN = 10. Whack-flinging is
   strictly worse than grasping under this reward.

### Indexing (a deliberate deviation from `relabel_full`)

The env pays step *i*'s reward from the state it **reached**, so the demo mirror is
`held[i] = pick_hold_held(s[i+1], a[i])`. `relabel_full` instead evaluates its pick proxy
at `s[i]` — one frame late relative to the env. The hold encoder uses the **env-exact**
convention, because the entire point of the lever is that the demo reward stream matches
the env reward stream frame for frame. Measured consequence in gate (c): env terminal
lands within **1–2 sim frames** of the demo's terminal index, and total reward matches
exactly.

### Episode/tape shape

* env: `+1` per held step; `_hold_run` resets on any non-held step; terminate
  (`terminated=True`, `info['pick_hold_done']`) when `_hold_run >= pick_hold_k`. The
  one-shot `STAGE_REWARD['picked']` grant is **suppressed** under the flag (paying both
  would re-import the terminal-only signal the lever exists to replace).
* demos: `+1` on every held frame, `done=True` on the frame where the run first reaches
  K, **all later frames dropped** — so the critic never bootstraps through a state the
  env would not have continued from, and the buffer is not padded with post-success
  transport.
* fail / no-pick demos (env-measured `stage` rank < picked): all-zero reward,
  non-terminal, **kept full length** — unchanged negatives.
* the tip rule and the nested proxy are untouched; neither can fire mid-hold (both need
  grip OPEN, which zeroes the run).

### Demo set

Requires **FULL-LENGTH** tapes. `episodes_pick_phase_all` (the dH set) is cut ~2 frames
past the lift and contains **no hold region at all** — 0/66 positives reach 25 consecutive
held frames. `train_rlpd` asserts `census['n_terminal'] > 0` and names the fix.

The arm uses `baselines/episodes_all`, verified to be a **strict superset** of the dH
tapes: for uids 308/325/297 the pick-phase file is a bitwise prefix of the
`episodes_all` file (states and actions). Same human demos, same recordings — the extra
frames are the hold region the lever needs, not different data.

---

## 2. Census (gate b)

`baselines/episodes_all`, `delta_ref=target`, K=25, pick_z=0.1505:

```
[hold-census] demos 91 = 78 >=picked + 13 negatives | skipped 0 short, 0 bad-t0
[hold-census] terminal (reached K consecutive held) 77 | held-but-never-K 1 | positive-with-zero-hold 0
[hold-census] rewarded frames 1966 / 100148 transitions (1.96%) | total reward 1966.0
[hold-census] per-demo rewarded-frame count (terminal demos): min 25 p25 25 med 25 p75 25 max 49
[hold-census] negatives paid 0 frames (MUST be 0); their proxy-positive frames suppressed by the stage gate: 0
```

Reading it:

* **1,966 rewarded frames vs 66 = 30×**, inside the pre-registered O(10–100×) band.
  Density **0.079% → 1.96% = 25×**; expected rewarded transitions per 128-sample demo
  minibatch **0.10 → 2.5**.
* **Negatives paid exactly 0** (the required sanity check), and on this set the stage gate
  did not even have to suppress anything: the 13 no-pick demos have **0** proxy-positive
  frames, so proxy and env-measured stage agree perfectly here. Nothing pathological.
* Per-demo distribution is **25 at every quartile** with max 49: the hold region, once it
  starts, runs straight through K, so nearly every demo contributes exactly K rewarded
  frames. The 49 is one demo with a ~24-frame blip before the sustained run (the env pays
  those blips too — same rule on both sides).
* Total transitions **100,148** (vs 180,761 for the untruncated tape set): dropping
  post-terminal frames removes the transport/place/nest tail, which is both correct
  (env terminates there) and a second density win.
* 1 positive demo holds but never reaches 25 consecutive frames — it contributes reward
  and no terminal, exactly as the env would truncate it.

Other sets, for the record (same command, different `--demo-dir`):

| set | ref | demos | terminal | rewarded frames | transitions | density |
|---|---|---|---|---|---|---|
| `episodes_all` | target | 91 (78+13) | 77 | 1966 | 100,148 | 1.96% |
| `episodes_delta_rerecord` | measured | 72 (61+11) | 59 | 1507 | 126,616 | 1.19% |
| `episodes_pick_phase_all` (dH) | target | 91 (66+25) | **0** | 132 | 83,465 | 0.16% |

The dH row is why the assert exists. **Note on `episodes_pick_phase_all`:** its 25
negatives include 9 `label=fail` demos whose honest proxy fires for 71–496 frames (total
2,264) while their recorded `stage` says `no-pick`; the stage gate suppresses all of them.
In `episodes_all` those same 9 uids carry `stage` ≥ picked (they *do* pick; the failure is
downstream), so they are positives there and are paid — which is `relabel_full`'s existing
stage-gated, label-agnostic convention, unchanged. Flagging it because it means the two
sets disagree about 9 demos' pick status; the hold arm inherits the `episodes_all` answer.

**Truncation confound (§12d) does not apply here.** The hold encoder pays *only* hold
reward — no `placed/contact/nested` grants — so running it on full-length tapes at pick
scope cannot leak downstream reward (contrast: `relabel_full` on `episodes_all` at pick
scope grants 189 rewarded / 254 total reward from downstream stages).

---

## 3. Gates

### (a) DEFAULT-PATH BYTE-IDENTITY — **PASS**

Demo tensors, all four existing encoders (stride-1 / repeat-4 × target / measured) plus
`relabel_full`, on **both** `episodes_pick_phase_all` and `episodes_all`, at scope `pick`
and `full`; obs, action, reward, next-obs, done arrays snapshotted before the change and
recomputed after:

```
arrays compared: 90 | mismatched: []
BYTE-IDENTITY PASS
```

(90 = 18 tensor sets × 5 arrays. This also covers the one refactor in the change: the two
stride-1 delta encoders now call the shared `_delta_actions` helper instead of carrying
their own copy of the delta math.)

300-step CPU smoke of `train_rlpd` **without** the flag, pre vs post:

* demo census line identical:
  `[demos] 91 eps -> 83465 transitions in the IMMUTABLE demo buffer (50% of every batch), 66 rewarded`
* sidecar: every pre-existing key present with the identical value; two keys **added**
  (`pick_hold_reward: "off"`, `pick_hold_k: 25`) — nothing removed or changed.
* `[cfg]` line gains `pick_hold_reward=off pick_hold_k=25 q_watchdog=2.00`; nothing else
  differs.
* **all 28 policy tensors of `rlpd_final.zip` are bitwise identical** pre vs post
  (sha256 per tensor) — the default training path is numerically unchanged, not merely
  similarly configured.

The pre-existing durable gate is also clean post-change:
`baselines/rl/sacfd_delta_gate.py` → `TENSOR-EQUALITY OK: 2538 transitions, 5 rewarded`,
`OPEN-LOOP REPLAY 4/5`, **GATE PASS** — its documented expected result, unchanged.

### (b) REWARD CENSUS — **PASS** (section 2). Nothing pathological; negatives paid 0.

### (c) ENV-DEMO CONSISTENCY — **PASS 3/3**

Open-loop replay of the hold-encoded demo actions through
`FullTaskEnv(scope='pick', action_mode='delta_joint', delta_ref='target',
pick_hold_reward=True, pick_hold_k=25)`. The action stream is the full-tape delta
encoding (identical prefix to the hold encoder's — asserted in the gate — extended past
the demo terminal so a replay lagging by a frame can finish its own run):

```
[gate-c] env FullTaskEnv pick_z=0.1505 cap=0.025 ref=target pick_hold_reward=True pick_hold_k=25
[gate-c] uid 308 DEMO : frames=406 (tape 1354) rewarded=25 total_reward=25.0 done_idx=405 done_count=1 trailing_run=25/25
[gate-c] uid 308 ENV  : steps=404 total_reward=25.0 terminated=True truncated=False hold_run_at_end=25 pick_hold_done=True first_held_step=380
[gate-c] uid 308 -> OK (|d_reward|=0.0 |d_frames|=2)
[gate-c] uid 325 DEMO : frames=608 (tape 1459) rewarded=25 total_reward=25.0 done_idx=607 done_count=1 trailing_run=25/25
[gate-c] uid 325 ENV  : steps=609 total_reward=25.0 terminated=True truncated=False hold_run_at_end=25 pick_hold_done=True first_held_step=585
[gate-c] uid 325 -> OK (|d_reward|=0.0 |d_frames|=1)
[gate-c] uid 297 DEMO : frames=380 (tape 1137) rewarded=25 total_reward=25.0 done_idx=379 done_count=1 trailing_run=25/25
[gate-c] uid 297 ENV  : steps=381 total_reward=25.0 terminated=True truncated=False hold_run_at_end=25 pick_hold_done=True first_held_step=357
[gate-c] uid 297 -> OK (|d_reward|=0.0 |d_frames|=1)
GATE-C PASS
```

Both sides: same K (25), same predicate (same function), total reward **exactly 25** on
each side, env terminal within 1–2 sim frames of the demo's. The residual 1–2 frames is
open-loop replay lag (the can crosses `pick_z` a frame late/early in the re-simulation) —
the same mm-margin effect `sacfd_delta_gate` already documents — not a semantic mismatch.

**Diagnostic worth keeping:** when the env was fed *only* the hold-truncated action list
(no extension), uids 325/297 finished at `hold_run = 24` with reward 24 — i.e. the replay
runs ~1 frame behind the tape. That is the correct behaviour of a faithful mirror under
sim lag, and it is why the gate extends the action stream rather than loosening the
criterion.

### (d) SMOKE with the flag ON — **PASS**

600-step CPU run, `--pick-hold-reward on --pick-hold-k 25 --demo-dir baselines/episodes_all`:
env asserts pass, census prints, 100,148 demo transitions / 1,966 rewarded loaded into the
immutable buffer, run completes clean in 41 s. `[cfg]` line:

```
[cfg] RLPD | E=10 Z=2 UTD=10 gamma=0.998 ent_coef=auto target_entropy=-3.5 demo_batch=128/256
      backup_entropy=off per_member_ln=off pick_hold_reward=on pick_hold_k=25 q_watchdog=48.82
      scope=pick action_mode=delta_joint delta_ref=target action_repeat=1 demo_dir=baselines/episodes_all
```

Because 600 steps sits below `learning_starts = 1000`, a **3,000-step** CPU run was added
so gradients actually flow (~2,000 steps × UTD 10 = ~20,000 critic updates; 5 m 14 s
wall while the 6 pair trainers held the machine). Result: exit 0, **no NaN** (all 28
policy tensors finite), **no `[Q-WATCHDOG]` trip**, no traceback. Critic values on uid
308's hold tape at that point: 10 members × 406 frames, mean **−0.48**, range
**[−0.78, 3.09]**, all finite — early-training scale, rising toward the ~24 fixed point,
nowhere near the 48.82 alarm. Reward scale on the data side is the census: 1,966 total
demo reward over 100,148 transitions, max 25 per episode.

### Negative controls on the new flag (all fail LOUD, as designed)

* `--pick-hold-reward on --demo-dir baselines/episodes_pick_phase_all` →
  `AssertionError: no demo ... shows 25 consecutive held frames -- this is a PICK-TRUNCATED set ...`
* `--pick-hold-reward on --scope full` → `AssertionError: pick_hold_reward is a scope=pick lever`
* `--pick-hold-reward on --action-repeat 4` →
  `AssertionError: no decision-level hold encoder yet -- run action_repeat=1 rather than silently striding the hold region`

---

## 4. Q-watchdog rescale (found while building; not in the original work order)

`RLPDSAC`'s watchdog warns at mean actor Q > 2.0 = "2× the max task return", which was
correct when the max return was 1. Under the hold reward the max **discounted** return is
`sum_{i<K} gamma^i` ≈ **24.4** at K=25 / gamma=0.998, so an unchanged threshold would
scream on a perfectly healthy critic — and a watchdog that cries wolf is exactly how the
entropy-backup explosion got waved off (audit §12). `train_rlpd` now passes
`q_watchdog = 2 * (1 - gamma^K)/(1 - gamma)` when the flag is on (**48.82** at the launch
config) and **exactly 2.0** when it is off, so the default path is unchanged. The value is
printed in the `[cfg]` line.

Downstream reading rule for this arm: **Q ≈ 25 is the healthy fixed point now, not a
pathology.** Compare Q against ~24.4, not against 1.

---

## 5. Launch command (3 seeds, nb config + hold reward)

Single-lever vs the nb wave: `--pick-hold-reward on` (+ the demo set it *requires*, since
the hold region does not exist in the truncated tapes). Everything else is the nb config
(`backup-entropy off`, `per-member-ln off`, stride-1, `delta_ref target`, gamma 0.998,
UTD 10, E=10, Z=2, demo-batch 128/256), 100k steps, matching §4a-2's protocol.

```bash
cd /home/j/workspace/genesis_pickaplace
for S in 0 1 2; do
  ./.venv-eval/bin/python baselines/rl/train_rlpd.py \
    --steps 100000 --scope pick --action-mode delta_joint --delta-ref target \
    --action-repeat 1 --gamma 0.998 --utd 10 \
    --backup-entropy off --per-member-ln off \
    --pick-hold-reward on --pick-hold-k 25 \
    --demo-dir baselines/episodes_all \
    --out-dir baselines/rl/checkpoints/rlpd_hold_dH_s$S \
    --run-name dH_RLPD-hold_s$S --project genesis_paper \
    --seed $S --device cuda \
    --eval-freq 25000 --eval-max-steps 400 \
    > /tmp/rlpd_hold_s$S.log 2>&1 &
done
```

**Verify before walking away** (the derived-script rule, §12e): each log must show
`pick_hold_reward=on pick_hold_k=25 q_watchdog=48.82` on the `[cfg]` line, a
`[hold-census] ... terminal ... 77 ...` block, and
`[demos] 91 eps -> 100148 transitions ... 1966 rewarded`.

**Pre-registered success criterion (§4a-2 protocol, same bar as the nb wave):**
≥1 seed ≥3/15 demo-IC picks at fixed 100k under the fresh-process protocol. Read it
against nb's `1/15 | 4/15 | 1/15`.

**Caveats to carry in any caption:**

* The arm changes **two things at once by necessity**: reward density *and* the demo tape
  set (`episodes_pick_phase_all` → `episodes_all`). They are not separable — the hold
  region only exists in the full tapes. The mitigation is that the sets are the *same
  recordings* (bitwise-prefix verified), and the additional frames are post-pick
  transport that the hold relabel then **drops** at the terminal; the buffer differs by
  the hold region plus longer negatives, not by different demonstrations.
* Returns are **not comparable** across arms: this arm's episode return is up to ~25.
  Compare **eval pick rate** only.
* Eval is untouched: `wandb_eval` measures the honest pick and ignores the new sidecar
  keys.

---

## 6. Files changed

* `baselines/rl/full_env.py` — `pick_hold_held()` (shared predicate);
  `FullTaskEnv(pick_hold_reward=False, pick_hold_k=25)`; per-step hold reward +
  K-consecutive termination in `_step_once`; `_hold_run` reset in `reset`/`reset_to`;
  docstring semantics + ManiSkill/Adroit precedent.
* `baselines/rl/train_sacfd_full.py` — `_delta_actions()` (shared delta math, now used by
  the two stride-1 encoders); `relabel_hold_region()`; `print_hold_census()`;
  `hold_region_encode_transitions()`.
* `baselines/rl/train_rlpd.py` — `--pick-hold-reward {on,off}` (default off),
  `--pick-hold-k` (default 25); preconditions asserted; env wired + post-construction
  asserts; hold encoder selected; census printed + truncated-set assert; Q-watchdog
  rescaled; `[cfg]` line and checkpoint sidecar record both fields.

**Boundary line for the ledger:** the pair wave (`dH/dDP_RLPD-pair_s*`) trained on
pre-hold-reward code; its snapshots were authored by old-code processes (sidecar
`git: 398e078`).
