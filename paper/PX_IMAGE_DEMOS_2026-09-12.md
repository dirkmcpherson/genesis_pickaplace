# Rendered-image human demonstration set `dHfull_all_rns10h_img` (lane PX-1, 2026-09-12)

**What this is.** The human end-to-end set `dHfull_all_rns10h` (ladder `nested_sparse10`, tip guard
`not_in_hand`, world `gc_kp4_riser3_shelf6`) rebuilt with a **rendered** `image` column in place of the
all-zero placeholder every demonstration set on disk carries, so a **pixel** DreamerV3 / r2dreamer run can
prefill from it. Nothing else about the set changes: the action streams, the state column, `is_first` /
`is_last` / `is_terminal`, the reward column, the grants and the terminal decisions are identical to
`dHfull_all_rns10h` (verified tape for tape, §3).

Set: `/home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img` (local only — datasets never
travel by git).

## 1. Why the placeholder was there, and why `--from-records` cannot fix it

`baselines/rl/relabel_reward.py` has two paths. The **re-execution** path (`--in/--out`) runs each tape's
action stream through `FullTaskEnv` and writes what the env paid. The **offline** path (`--from-records`)
replays a per-frame *stage record* in numpy, with no Genesis and no simulation, and copies every non-reward
column from the source tape. A stage record holds poses, contacts and stage flags — **it holds no pixels** —
so `--from-records` can only copy the source set's `image` column, and every source set was built state-only
(`to_dreamer_native.py --state-only` writes a zero `(T,64,64,6)` placeholder). That is why `_rns10h`,
`_rnrh`, `_rnsh` and the cluster sets all carry an all-zero image column. Pixels can only come from a real
re-execution with the camera rig built.

## 2. `--images` (new, default off)

Added to the re-execution path only; `--from-records`, `--records-out` and `--verify-against` refuse it.

* The env is built with `camera_rig=True` (asserted after construction), so `genv.rig_obs()` exists.
* `genv.rig_obs()` is captured **once after the reset and once after every decision** → `(T,64,64,6)` uint8,
  `T` identical to the placeholder it replaces and to the `state` column, asserted per tape. Frame `t` is
  therefore the observation the backward-shifted `action[t]` led *into* — the demo format
  (`to_dreamer_native.py` docstring, `r2dreamer/demo_prefill.py`) and the adapter's own convention.
* It is the **same call at the same point in the loop** the world-model adapter makes online
  (`~/workspace/r2dreamer/envs/genesis.py` renders `self._env.genv.rig_obs()` into `obs["image"]` before each
  decision): topB overhead RGB ++ through-gripper wrist RGB, concatenated on the channel axis. Demo and
  online frames are the same observation function, which is what the prefill needs — it mixes them in one
  buffer.
* **Termination is suppressed for the render** (`env.never_terminate`, the mechanism `--records-out` already
  uses) so the *whole* action stream renders. This is not cosmetic: a relabelled tape keeps all `T` rows —
  only the reward column is cut at the terminal, the state column is not, and `demo_prefill._load_episode`
  asserts `image.shape == (T,64,64,6)` and loads every row — so an image column that stopped at the terminal
  would contradict its own state column on the **2 656 of 29 221 decisions (9.1 %)** that follow one
  (35 of 74 tapes terminate before their stream ends). Suppression also makes the physics identical to the
  stage-record path the `_r*` sets of record were scored from: every decision runs its full `action_repeat`.
* The **reward, grant and end-reason bookkeeping is cut at the terminal exactly as without the flag** — the
  loop sets a `stopped` latch at the first `terminated or truncated` and scores nothing after it; the
  per-episode tracker columns (`rz_slide_gain_m`, `rz_release_dist_m`, `rz_release_far`) are snapshotted at
  that point rather than at the end of the render.
* Manifest gains `images: rendered` (vs `inherited` for every other build) plus
  `image_nonzero_frac_min` / `image_mean_min`; each tape gains `rz_images`, `rz_image_nonzero_frac`,
  `rz_image_mean`.
* The `--out` suffix gate accepts `<ladder suffix>_img` under `--images`, so the launcher still reads the
  ladder off the set name.
* **Default off**: a run without `--images` builds the env exactly as before and takes the same branch it
  always took (the `break` at the terminal), so existing set builds are unchanged.

## 3. Build and verification

```bash
export GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace
export GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
~/workspace/genesis_sim2real/venv/bin/python baselines/rl/relabel_reward.py \
  --in  /home/j/data/genesis_pickaplace/demos_state_full/dHfull_all \
  --out /home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img \
  --ladder nested_sparse10 --tip-guard not_in_hand \
  --sim-variant gc_kp4_riser3_shelf6 --images --procs 8
```

Node: pop-os, AMD Ryzen 9 5950X, 32 cores, AVX2 (stamped in the manifest). Wall **543.6 s on 8
processes**; summed single-process simulate+render time **2 736 s over 74 tapes = 37 s/tape median
(0.094 s per decision, two 64×64 renders included), min 0 s (the 1-decision tape), max 60 s**. Set size
66.7 MB compressed (vs 3.3 MB for the state-only set).

### Identity with `dHfull_all_rns10h` (per tape, 74/74)

| check | result |
|---|---|
| tapes | **74** |
| Σ reward | **130.0** = 10 × 13 `home`, identical to `_rns10h` |
| reward column, element-wise | identical on **74/74** |
| `rz_end_reason` / `rz_end_decision` | identical on **74/74** |
| `rz_grants` (stage → decision) | identical on **74/74** |
| end reasons | `{home 13, tipped 26, stream_exhausted 23, truncated 12}` — identical |
| `n_pick` (repeat.json) | **65**, identical |
| action sha256 vs `dHfull_all` | identical on **74/74**; set-level `77bc4875…92913`, identical |
| `action` / `state` / `is_first` / `is_last` / `is_terminal` / `discount` / `logprob` columns | byte-identical to the source on 74/74 |
| `can_dev` max / p50 / >1 cm | 0.703 m / 9.15 mm / 36 tapes — identical to `_rns10h` |
| `tapes_granting` (all 14 rungs) | identical |
| env-code sha256 (`full_env` / `genesis_can_env` / `stage_predicates`) | `584bea6d9c6d` / `40544bf73c8c` / `a589b4f05632` — identical |

Only three manifest fields differ from `_rns10h`, all by construction: `images` (`rendered` vs absent),
the `git describe` inside `ladder_stamp` (this build is a later, dirty tree — `relabel_reward.py` is not
one of the three hashed env files, and all three hashes match), and `n_tapes_nested_honest` (see §4).

### The image column

| statistic | value |
|---|---|
| shape / dtype | `(T,64,64,6)` uint8, `T` = `state` rows, **asserted on 74/74** (T ranges 2…601) |
| nonzero fraction | min **1.0000**, median 1.0000, max 1.0000 — no tape has a black frame |
| frame mean (0–255) | min **99.39**, median 106.09, max 109.33 |
| top half (ch 0:3) mean | 103.12 … 104.36 |
| wrist half (ch 3:6) mean | 95.18 … 115.03 |
| distinct frames / T | min 0.883, median 0.986 — the column moves, it is not a held frame |

Contact sheet (8 frames, top camera | wrist camera, tagged with the decision each rung fired):
`can_pos_recovery/videos_ln_home_2026-09-12/px_contact_sheet_232.png` — tape
`genesis-100000-013-256.npz` = trial **uid 232** (mapped by matching decisions + grant decisions against
`can_pos_recovery/videos_home_2026-09-11/INDEX.md`; the segment tapes carry no `ic_uid`). It runs
reset → `picked` d70 → `placed_v2`/`released` d121 → `settled_after_release` d123 → `farside` d190 →
`slide_event` d194 → `home` d201, ending `home@202` of 255 decisions.

## 4. Deviations and things a reader should not assume

1. **`state_only` is overwritten, not inherited.** `repeat.json` inherits its source stamps verbatim by
   design (the 2026-09-09 `one_per_ic_first` defect). `state_only: true` means "the image column is a zero
   placeholder"; a rendered set falsifies it, so `--images` sets `state_only: false`, adds
   `images: "rendered"` and a `relabel.images` block (call, layout, `never_terminate_for_render`, measured
   minima). No launcher gates on `state_only` — `demo_prefill` reads only the stride from `repeat.json`.
2. **`n_tapes_nested_honest` is now `null`, not `0`, on every direct re-execution.** `nested_honest` comes
   from the single end-of-episode settle that only `--records-out` runs, so the direct path never computes
   it and `summarize` was counting an absent key as a count of zero — which reads as "no tape settles
   nested", while the stage records say **16 of these same 74 tapes do**. Fixed in `summarize`
   (pre-existing defect, not introduced here; it only became visible because this is the first direct
   re-execution built as a set of record). The set's own `manifest.json` / `repeat.json` were regenerated
   from the stored `per_tape` rows through the fixed `summarize` / `write_repeat_json` — a pure function of
   those rows, no re-simulation; every other field was asserted unchanged before the rewrite.
3. **This set is NOT on the hardware class of record.** It was built on pop-os (32-core, AVX2), like
   `_rns10h` itself. Genesis is not bit-identical across CPU classes and the local re-execution disagrees
   with the cluster's 64-core class on 6 of 74 tapes after contact (HANDOFF §1). The local `_rns10h` pays
   Σ 130 = 13 × `home`; the class-of-record set has 12. This image set reproduces the LOCAL set exactly,
   which is what a local pixel run needs; it is not a cluster set.
4. **Diagnostics that are not bit-equal, and why.** `rz_slide_gain_m` differs from `_rns10h` in the 5th
   decimal (e.g. 0.027922 → 0.027951 on uid 232) and `rz_can_dev_max_m` differs only in rounding (the
   offline path rounds to 6 dp, the direct path stores the full float). `_rns10h` was scored OFFLINE from a
   stage record — the tracker is fed per env frame and cut at the terminal frame, while the direct path
   snapshots the tracker after the terminal DECISION, which is up to three `action_repeat` sub-frames
   later. Sub-0.03 mm, in a diagnostic column, and it does not touch reward, grants or the terminal.
5. **`is_terminal` is still unchanged from the source**, so the paid `home` frame carries no terminal flag
   and the tape continues past it with zero reward. That is the `--cut-at-terminal` follow-up already
   recorded in HANDOFF §3e; the image column now covers that tail honestly (real frames, real states,
   zero reward) rather than pairing real states with black frames, but the underlying inconsistency for
   the continuation head is unchanged and is not this lane's to fix.
6. **The demo images are rendered by `rig_obs()` with `camera_rig=True`; the online adapter must be the
   same rig.** The channel order (top RGB first, wrist RGB second) and the 64×64 size come from
   `GenesisCanEnv.rig_obs()` itself, so demo and online frames agree by construction *as long as the
   online run uses the same env build*. A pixel run that changes the rig invalidates this set.
7. **Operational note (collision).** The first launch of this build (11:59:11) was stopped at ~12:02: a
   `train.py env=genesis_full_pixel` smoke started on the same box a minute later, and HANDOFF §3e's rule
   is that no multi-world job runs beside training. All 8 shard worlds were killed by PID, the output
   directory removed, and the build re-run from scratch at 12:14:35 with the box to itself. Nothing was
   written by the aborted run. The ~2 minutes of overlap is disclosed to whoever owns that smoke.
   `relabel_reward.py` uses the CPU backend and never appeared on the GPU.
