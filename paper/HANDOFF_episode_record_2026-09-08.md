# Implementation brief: per-episode stage record for full-scope runs

*Self-contained. Written for an agent with code and cluster access. Registered as PHASE_PLAN amendment (w). Half of it is already committed; the remaining two wires are specified below.*

## The problem

In `scope='full'`, the environment's per-step stage flags are written **only when an episode terminates inside the adapter**. A horizon truncation happens outside it, so a truncated episode logs all zeros even though it picked. Measured on `dHfull_all` seed 3: **1,198 of 2,911 episodes all-zero, every one exactly the horizon length (300), and 608 of them had scored.** Pick reads 0.480 by flag against 0.688 by score.

The consequence is that the accumulated reward became the only truncation-proof channel, so **we can only observe what the reward pays for**. That is why full-scope learning curves exist for pick, contact and the nested proxy but **not for placement or slide**: the `placed` rung uses the stale release predicate that is essentially never granted, `placed_v2` exists only as a flag, and slide is not a rung at all.

**The goal is to observe placement and accepted-slide acquisition without changing the reward to make them observable.** The reward ladder must not be touched — runs must stay comparable with the existing arms.

## What is already done (commit `7096fe6`, do not redo)

`baselines/rl/full_env.py` now emits the sticky cumulative record from the **single exit path both termination and truncation pass through**, at the end of `step()`:

```python
if (terminated or truncated) and EPISODE_RECORD:
    info = dict(info)
    info['episode_end'] = True
    for _stage in ('picked', 'placed_v2', 'contact',
                   'contact_push', 'slide_success', 'nested'):
        info['ep_' + _stage] = bool(_stage in self._granted)
```

`self._granted` was already sticky and cumulative — a stage enters it the first time the env's own predicate flips and never leaves. It was simply never emitted. Module-level gate, **default OFF**:

```python
EPISODE_RECORD = os.environ.get('FULLENV_EPISODE_RECORD', '') == '1'
```

## What remains — two wires, to be done TOGETHER

### Wire 1: the adapter must surface these as `log_`-prefixed transition keys

The trainer logs episode scalars with this loop (see `paper/wm_fix_r2dreamer_base_uncommitted_2026-09-04.diff:499-501`):

```python
for key in trans.keys():
    if key.startswith("log_"):
        self.logger.scalar(f"episode/train_{key[4:]}", float(trans[key][i, 0]))
```

It reads the **transition** dict, not `info`, and only keys beginning with `log_`. So the keys committed above **currently reach nothing**. The genesis adapter (`envs/genesis.py`, cluster-side; not present in the local checkout) must copy them onto the transition at episode end as `log_ep_picked`, `log_ep_placed_v2`, `log_ep_contact`, `log_ep_contact_push`, `log_ep_slide_success`, `log_ep_nested`. They will then appear in `metrics.jsonl` as `episode/train_ep_*`.

### Wire 2: the launcher must enable it

Export `FULLENV_EPISODE_RECORD=1` in the sbatch script for the runs that should carry it. Without this the gate stays closed and nothing changes.

## Constraints that must be respected

- **Do not change the reward.** No rung added, removed or reweighted. Observability must not be bought by changing what the agent optimises.
- **Logging only.** Advance no simulation, mutate no state, consume no randomness, swallow exceptions so it cannot kill a run. Scalars only — no containers, which some loggers cannot serialise.
- **Settle-dependent predicates must not run inline during training.** The settled nested predicate and accepted-slide need extra simulation; computing them mid-training would change what is being measured. Evaluate them in evaluation or from a deferred pass. Note `slide_success` in `_granted` is the *within-episode* grant, not the settled one.
- **Do not edit shared code while an experiment is queued against it.** End-to-end jobs are queued now. Editing the adapter mid-flight puts a code difference *between the arms being compared* — this has already happened once today with a settle flag and was caught only by chance. Apply both wires when the long runs are configured, with the gate on for those runs only.

## Verification before trusting any output

1. Run one short full-scope job with the gate on; confirm `episode/train_ep_*` keys appear in `metrics.jsonl`.
2. Confirm a **truncated** episode reports the stages it reached — take an episode of exactly the horizon length with score ≥ 1 and check `ep_picked` is true. This is the specific bug being fixed.
3. Assert **strictly harder stages are subsets of easier ones**. Three identical ignition steps across nested stages is what exposed the original defect; the check is in `HRI_results/curves/learning_curves.py`.
4. Confirm the gate off reproduces existing behaviour exactly.

## Then

`HRI_results/curves/learning_curves.py` gains a full-scope stage set beyond `SCORE_THRESHOLDS`: read `episode/train_ep_<stage>` directly instead of thresholding `episode/score`, which removes the reward-ladder dependency entirely and yields acquisition curves for pick, placement and accepted slide on one axis.
