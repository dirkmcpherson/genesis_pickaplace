# Episode-record implementation, 2026-09-08

Implements [the handoff](HANDOFF_episode_record_2026-09-08.md), amendment (w),
in an isolated R2Dreamer bundle for future full-scope runs. No long training run
was submitted. Existing experiment launchers, queued jobs, datasets, evaluation
cells and the local committed `baselines/rl/full_env.py` were not changed.

## Changes

- The real timeout belongs to R2's outer `TimeLimit`; its inner `FullTaskEnv`
  uses `max_steps=10**9`. The logging hook therefore runs after the outer wrapper
  marks the episode final. It copies an available inner episode record, otherwise
  reads the same sticky `_granted` set without stepping or modifying it.
- `FULLENV_EPISODE_RECORD=1` adds six scalar `log_ep_*` transition columns and
  `log_ep_record_valid`. The existing trainer consumes transition keys, not
  `info`. Missing records remain invalid rather than becoming negative outcomes.
- Demonstration prefill gets zero placeholders for the new replay columns.
  These do not assert demonstration outcomes. A gated trainer tail flush also
  records a completion at the exact budget boundary, which the original
  next-iteration logger misses.
- The new launcher enables the record and sets `R2D_EOE=0`, pins the frozen w3
  task, checks source hashes and demonstration metadata, and refuses existing
  output paths or less than 10 GiB free space. The gate comparison uses EOE=0
  on both sides; this does not establish equivalence to historical EOE=1 runs.
- `HRI_results/curves/learning_curves.py --episode-record` reads full-scope
  episode records directly, with no score fallback. It validates actual
  predicate implications and includes seed IDs and per-bin seed counts.
  Equal first-crossing bins are allowed: nested predicates can legitimately
  cross within the same bin. Existing CSVs and figures were not regenerated.

The cluster's live GP source did not contain the local `EPISODE_RECORD` commit.
The outer-boundary fallback handles that source as well; no shared environment
patch or reward change was needed. Python sources are copied into the bundle;
non-code GP inputs are linked, so this is not a fully copied dependency archive.

## Scope of the measurements

`placed_v2` can now be observed even when it earns no reward. `nested` remains
the training proxy and is exported as `nested_proxy` in the curve data.
`slide_success` remains the withdrawn within-episode diagnostic, exported as
`slide_within_episode_diagnostic`. **This implementation does not deliver an
accepted-Slide acquisition curve.** That requires a validated deferred or
evaluation measurement; no settling continuation runs inline here.

No RLPD, DV3, robomimic, model, reward, or action-control code was changed by
this handoff implementation. The added fields are excluded from the encoder;
the frozen full-state decoder selects only `state`.

## Verification and deployment

Eight adapter/trainer tests pass locally and with the cluster dependencies.
They cover true termination, outer timeout, unrewarded Place, gate-off parity
against the original adapter, non-full scope, logging exceptions, RNG/state
preservation and exact-budget flushing. Five curve-reader tests pass, including
invalid-record rejection, predicate checks and byte-identical legacy CSVs.
Both launch scripts pass shell syntax checks.

The real-Genesis smoke uses a scripted demonstration prefix followed by holding
the can, with the actual R2 `OnlineTrainer`, logger, demo prefill and replay
storage. It performs no optimization. This is an instrumentation test, not a
learner-performance result or a multi-seed scientific comparison.

The first diagnostic, job `3388885`, failed because the test harness passed a
NumPy quaternion where the reset helper expects a list. Its outputs remain
under `smoke_01`. The corrected harness is used by job `3388950` under
`smoke_02`. The corrected diagnostic passed: non-record transition hashes were
bit-identical across the two processes. Each executed two 300-decision episodes;
gate-off logged one, retaining the original missing-final-flush behavior, and
gate-on logged both with valid picked records. Both prefilled 171 transitions.
The shared non-record transition SHA-256 is:

```
48276d2245eb4ccc15331c57c8cee007580928c7e610ca20cee23665f4fb6f75
```

This demonstrates trajectory parity for this scripted CPU fixture, not a claim
of identical optimization over arbitrary GPU training runs. Evidence, gate-on
metric rows, job accounting and the final manifest are saved in
[cluster_smoke_3388950.json](../cluster/episode_record/verification/cluster_smoke_3388950.json).

The gate-on run's first 300-decision timeout has `episode/score=1`, legacy
`episode/train_picked=0`, new `episode/train_ep_picked=1` and
`episode/train_ep_record_valid=1`. Thus the new path reaches the actual logger
on the specific timeout case that the old flags lose.

Final bundle on `pax`:

```
/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/episode_record_2026-09-08/bundle_v2
```

Its 86 R2 files and 155 GP Python files have identical hashes to the simulator
smoke bundle. The corrected smoke harness also matches. The original four R2
files targeted by the patch and all copied GP Python sources still match their
source hashes. Final manifest SHA-256:

```
b695fced52878e2bcfd45a734c536e51981084093940e06e97884f0eee2c646d
```

Source, preparation/launch instructions, tests and measurement caveats:
[cluster/episode_record/README.md](../cluster/episode_record/README.md).
The initial `bundle`, failed diagnostic and corrected diagnostic are preserved
alongside `bundle_v2`. Future training should use `bundle_v2/launch.sbatch`;
the example budget in the README is not an approved experiment design.
