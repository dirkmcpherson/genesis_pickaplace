# PX-2 — pixel observations + image augmentation in the r2dreamer chassis (2026-09-12)

Lane PX-2, pop-os. Tree `~/workspace/r2dreamer` (git `main`, local only). Companion repo
`/home/j/workspace/genesis_pickaplace`, branch `ladder-unify-2026-09-11`.

**Task (user, 2026-09-12 ~11:30):** bring PIXEL observations and IMAGE AUGMENTATION into the
`{dv3 local}` arm (`model.rep_loss=dreamer` inside the r2dreamer chassis) and train
`nested_sparse10` with persistent world models. **The loss structure is unchanged** — no head is
detached, no loss term is added or removed. The augmentation is a data augmentation: the frame the
encoder sees is the frame the decoder reconstructs.

**Rule held throughout:** every option defaults to today's behaviour, so every existing run, every
existing eval cell and every existing checkpoint is byte-identical unless a config explicitly opts
in.

**Status: DONE.** Both smokes passed (§4), settling at **≈ 46.7 fps** against the state-only arm's
81.7. The real run was NOT launched — the coordinator registers and launches it. r2dreamer commit
`43a0e3c` (local `main`, no upstream).

---

## 1. What changed

| # | File | Change |
|---|---|---|
| 1 | `configs/env/genesis_full_pixel.yaml` (NEW) | `genesis_full_state.yaml` with exactly three differences: encoder/decoder `cnn_keys: 'image'` (was `'$^'`), `state_slice: 8`, `image_aug: shift4`. Everything else — ladder, `tip_guard`, `far_release`, `return_clamp`, action encoding, horizon, `train_ratio`, `env_num`, demo plumbing — is verbatim. |
| 2 | `envs/genesis.py` | `GenesisPick(..., state_slice=None)`. Applied in **`_state_vec`** — the single function every state observation goes through — and in `observation_space`'s declared dim. Raises on a slice outside `[1,17]` and on `state_slice` + `state_extra` together (`tool_goal` would re-add goal-derived columns). |
| 3 | `envs/__init__.py` | passes `state_slice=config.get("state_slice", None)` to the adapter. |
| 4 | `demo_prefill.py` | `add_demo_set` truncates the demo `state` column by the same `env.state_slice` and asserts the width; prints `[prefill] state_slice=N: demo state truncated 17 -> N dims`. Without this the buffer would mix a 17-dim demo row with an 8-dim online row (the first `extend` fixes the column width, so this is a hard failure, not a silent one). |
| 5 | `eval_genesis.py` | passes `state_slice=cfg.env.get("state_slice", None)` from the run's own `.hydra/config.yaml`, and prints it. The two pinned reset paths (`reset_to_uid`, `reset_to_ic`) bypass `adapter.reset` but both call `env._state_vec`, which is where the slice lives — so neither can bypass it. |
| 6 | `dreamer.py` | `model.image_aug: none \| shift4` (default `none`). New `Dreamer.apply_image_aug(data)`: replicate-pad 4 px, random-crop back to 64×64, **one integer offset per sequence** (`same_across_time=True`, `bilinear=False`), applied to `data["image"]` **in place** — the batch size is unchanged. Called in `Dreamer.update` between `preprocess` and `_cal_grad`, i.e. before the encoder. New metric `data/image_nonzero_frac`. |
| 7 | `trainer.py` | one-shot `[image]` stamp on the first ONLINE transition written to the buffer (shape/min/max/mean/nonzero-fraction/per-env max). |
| 8 | `train.py` | `[obs]` startup stamp (encoder + decoder key sets read from the BUILT modules, state dim, `state_slice`, `image_aug`); `+wm_init` now refuses a source that is missing world-model tensors this run needs, and its shape-mismatch message names the source and target shapes. |

### Why `state_slice`

`obs["state"]` is `[q(6), gripper motor, grip effort, can xyz, can quat(4), goal xy]`. A pixel run
that keeps all 17 dims can read the object and target pose straight out of the state vector and has
no reason to use the camera. `state_slice: 8` keeps the first eight columns — pure proprioception —
and drops can pose, can orientation and goal xy. `state_slice: null` keeps the full vector (pixels
+ full state), which is the second configuration smoked below.

### What `shift4` is, and what it is not

`shift4` reuses `Dreamer.random_translate`, the DreamerPro path's own shift, at `max_delta=4`,
`same_across_time=True`, `bilinear=False` — replicate-pad the 64×64×6 frame to 72×72×6 and take a
64×64 crop at a random integer offset in `[0,8]²`, the same offset for all T frames of a sequence so
the RSSM sees a temporally consistent view of a shifted camera. This is the standard DrQ shift.

It is **not** `augment_data`. `augment_data` (DreamerPro) *doubles* the batch (`cat([v, v])`) to
build two views for a contrastive objective; it is untouched and still only runs under
`rep_loss=dreamerpro`. `apply_image_aug` replaces the frames, so under `rep_loss=dreamer` the
reconstruction target is the same augmented frame the encoder was given — a data augmentation, not
a new loss term.

It is applied in `update()` rather than inside `_cal_grad` because `_cal_grad` is
`torch.compile(..., mode="reduce-overhead")`; drawing random integers inside a captured cudagraph
is exactly the kind of thing that silently freezes the offset. "Before the encoder" still holds —
the encoder is the first thing `_cal_grad` does.

---

## 2. Offline checks (no Genesis world)

`state_slice`, on the lazily-constructed adapter (no world build):

```
full  : obs dim (17,) vec [ 0.  1. ... 16.]
slice8: obs dim (8,)  vec [0. 1. 2. 3. 4. 5. 6. 7.]
raises on state_slice=0 / 18 / -1
raises on state_slice + state_extra='tool_goal'
default path unchanged: OK          <- _state_vec with no slice returns all 17 dims
```

`shift4`, on a Dreamer built over a fake `(64,64,6)` + `(8,)` obs space:

```
[image_aug] shift4 pad=4: max|aug-raw|=0.9981 frames_changed=1.000 raw_max=1.0000 ... [REAL frames: the augmented batch differs from the raw batch]
per-seq offsets (top-left codes): [[400.0 x6], [400.0 x6], [3.0 x6], [0.0 x6]]
constant across T within each sequence: True
varies across sequences: True
image_aug=none is a no-op: True          <- torch.equal(out, raw)
no-image batch passes through (state-only configs): dict_keys(['state'])
```

(The ramp image encodes the offset as `y*100 + x`, so `400` = shift (4, 0), `3` = shift (0, 3).)

Config resolution:

```
env=genesis_full_pixel  state_slice=8    -> model.image_aug='shift4'  enc.cnn='image' enc.mlp='state'
env=genesis_full_pixel  state_slice=null -> model.image_aug='shift4'  env.state_slice=None
env=genesis_full_state                   -> model.image_aug='none'    state_slice=None   (defaults unchanged)
```

---

## 3. `+wm_init` under the pixel config

`+wm_init=<ckpt>` (added 2026-09-12, local commit `066162a`) loads only `encoder./rssm./decoder./cont.`
and the frozen acting clones. Two failure modes were checked; both fire **before any Genesis world is
built** (the adapter's world build is lazy, so this costs seconds, not minutes):

1. **Shape/key mismatch** — the state-only checkpoint of record
   (`dv3e2e_ramp_resume_series/ck_1417866_hang/latest.pt`) against the pixel config:

```
FATAL: wm_init shape/key mismatch on 22 tensors (src shape -> this run's shape): [('encoder.encoders.0.layers.mlp_encoder_linear0.weight', (256, 17), 'ABSENT-IN-TARGET'), ...]. A state-only checkpoint cannot warm-start a pixel run (no CNN encoder/decoder weights, and a different state width under env.state_slice); train the first pixel world model fresh, then warm-start later pixel runs from IT.
```

2. **Silent partial load** (NEW check) — the pre-existing shape test only inspects tensors the
   *source* has, so a source that simply **lacks** CNN encoder/decoder weights would have loaded its
   subset and left the rest at fresh init, while `wm_init.json` recorded a successful load. That is
   now refused by name, with `+wm_init_allow_partial=true` as the explicit opt-in, and
   `wm_init.json` gained `world_model_tensors_in_run` / `absent_in_source` / `partial`.
   Exercised on a deliberately mutilated copy of smoke 1's checkpoint (19 `decoder._cnn.*`
   tensors deleted):

```
FATAL: wm_init source is MISSING 19 of this run's 136 world-model tensors, e.g. ['decoder._cnn.sp0.weight', 'decoder._cnn.sp0.bias', 'decoder._cnn.sp1.0.weight'] (modules: ['decoder._cnn']). Loading it would silently leave those at fresh init. Pass +wm_init_allow_partial=true ONLY if a partial warm start is what you intend (it is then recorded in wm_init.json).
```

3. **Positive path** — smoke 1's own `latest.pt` (a pixel checkpoint) into a fresh pixel run:

```
[wm_init] loaded 136 world-model tensors from .../smoke_px_20260912_115958_slice8/latest.pt (sha256 63b24c59719f); fresh: _frozen_actor, _frozen_reward, _frozen_slow_value, _frozen_value, _slow_value, actor, return_ema, reward, value; optimizer fresh
wm_init.json: "world_model_tensors_in_run": 136, "absent_in_source": 0, "partial": false
```

**136 vs the state-only config's 93**: the extra 43 tensors are the CNN encoder and CNN decoder, so
the pixel warm start really does carry the visual world model, not just the RSSM.

**Consequence for the plan:** the first pixel world model has to be trained fresh — no state-only
checkpoint can seed it. Later pixel runs warm-start from that one, and the load is now provably
complete (`partial: false`) rather than assumed.

---

## 4. Smokes

Both on the GPU, `env_num=6` (six Genesis worlds), ladder `nested_sparse10`, `tip_guard=not_in_hand`,
`return_clamp=10.0`, `actor_dist=bounded_normal`, `act_entropy=3e-5`, `rep_loss=dreamer`,
`image_aug=shift4`, `buffer.max_size=6e4`, 20 000 online env steps, demo set
`dHfull_all_rns10h` (**zero-image plumbing set** — PX-1's image set had not landed).
Script: `smoke_px.sh` in this session's scratchpad.

### Smoke 1 — `state_slice: 8` (proprioception + pixels)

`logdir /home/j/runs_dv3_local/smoke_px_20260912_115958_slice8`, exit 0 (ran the full 20 000
online steps; not killed by its timeout).

```
[sim-variant] gc_kp4_riser3_shelf6: kp=[800.0, 800.0, 600.0, 400.0, 240.0, 240.0] gc=1.0 riser=0.03      (x6, one per worker)
[ladder] unified-2026-09-10 | ladder=nested_sparse10 | home=10 | max_return=10 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-954-gd5520a2-dirty
[ladder] return_clamp=10.0 (env and model agree)
[obs] encoder cnn_keys=['image'] mlp_keys=['state'] shapes={'image': (64, 64, 6), 'state': (8,)}
[obs] decoder cnn_keys=['image'] mlp_keys=['state']
[obs] state_slice=8 state_dim=(8,)
[obs] image_aug=shift4 (pad 4 px, one offset per sequence)
[prefill] state_slice=8: demo state truncated 17 -> 8 dims
Demo prefill: {'episodes': 74, ..., 'transitions_added': 29406, 'rows_per_stream': 4901, ...}
Step accounting [R2_LONG_RUN]: prefill 29406 decisions; trainer starts at counter step 117624 (env frames); env.steps=20000 ONLINE env steps -> counter target 137624.
[image] first ONLINE transition: shape=(6, 64, 64, 6) dtype=torch.uint8 min=10 max=236 mean=107.36 nonzero_frac=1.0000 per_env_max=[236, 236, 236, 236, 236, 236]
[image_aug] shift4 pad=4: max|aug-raw|=0.0000 frames_changed=0.000 raw_max=0.0000 raw_nonblank_frac=0.000  [all-zero batch: a shift of a blank frame is a no-op by construction]
[image_aug] shift4 pad=4: max|aug-raw|=0.6588 frames_changed=0.001 raw_max=0.9255 raw_nonblank_frac=0.001  [REAL frames: the augmented batch differs from the raw batch]
```

`[image]` is the direct answer to "are real online frames reaching the buffer": all six workers
return a non-blank 64×64×6 frame (max 236, mean 107.4) on the first online transition. The two
`[image_aug]` lines are the augmentation assertion: on the all-zero demo batch the shift is
correctly a no-op; on the first batch containing real frames every non-blank frame changed
(`frames_changed` 0.001 == `raw_nonblank_frac` 0.001), with `max|aug-raw|` 0.66 on a [0,1] image.

| counter | `data/image_nonzero_frac` | fps |
|---|---|---|
| 117624 (trainer start) | — | 0.0 |
| 122640 | 0.0 | **14.4** |
| 127632 | 0.1 | **46.7** |
| 132624 | 0.1 | **46.3** |
| 137640 (done) | 0.2 | **45.5** |

**Settled ≈ 46 fps**, against **81.7 fps** state-only — pixels cost about 44 % of throughput, and
46 fps is comfortably above the 25 fps line. The 14.4 in the first window is `torch.compile` plus
about two minutes of contention with lane PX-1's eight-world image build, which overlapped the
start of this smoke; it is not the steady-state rate. `data/image_nonzero_frac` climbing 0.0 → 0.2
is the demo/online mix in the sampled batch: the demo tapes are blank, so this is the fraction of
sampled frames that are real online pixels.

### Smoke 2 — `state_slice: null` (full state + pixels), on the REAL-image demo set

Lane PX-1's `dHfull_all_rns10h_img` (74 npz, real `image` column) landed at 12:24, so this smoke used
it instead of the zero set. `logdir /home/j/runs_dv3_local/smoke_px_20260912_122449_slicenull`, exit 0.

```
[sim-variant] gc_kp4_riser3_shelf6: kp=[800.0, 800.0, 600.0, 400.0, 240.0, 240.0] gc=1.0 riser=0.03      (x6, one per worker)
[ladder] unified-2026-09-10 | ladder=nested_sparse10 | home=10 | max_return=10 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-954-gd5520a2-dirty
[ladder] return_clamp=10.0 (env and model agree)
[obs] encoder cnn_keys=['image'] mlp_keys=['state'] shapes={'image': (64, 64, 6), 'state': (17,)}
[obs] decoder cnn_keys=['image'] mlp_keys=['state']
[obs] state_slice=None state_dim=(17,)
[obs] image_aug=shift4 (pad 4 px, one offset per sequence)
Step accounting [R2_LONG_RUN]: prefill 29406 decisions; trainer starts at counter step 117624 (env frames); env.steps=20000 ONLINE env steps -> counter target 137624.
[image] first ONLINE transition: shape=(6, 64, 64, 6) dtype=torch.uint8 min=10 max=236 mean=107.36 nonzero_frac=1.0000 per_env_max=[236, 236, 236, 236, 236, 236]
[image_aug] shift4 pad=4: max|aug-raw|=0.8549 frames_changed=1.000 raw_max=1.0000 raw_nonblank_frac=1.000  [REAL frames: the augmented batch differs from the raw batch]
```

No `[prefill] state_slice=...` line, correctly — there is no slice under `state_slice: null`, and the
demo `state` column stays 17-dim. Only ONE `[image_aug]` line, also correctly: with a real-image demo
set the very first batch already carries non-blank frames, so the blank-batch case never arises, and
**every frame in the batch is changed by the shift** (`frames_changed` 1.000).

| counter | `data/image_nonzero_frac` | fps |
|---|---|---|
| 117624 (trainer start) | — | 0.0 |
| 122640 | 1.0 | **15.4** |
| 127632 | 1.0 | **46.6** |
| 132624 | 1.0 | **46.8** |
| 137640 (done) | 1.0 | **46.6** |

**Settled ≈ 46.7 fps** — indistinguishable from the `state_slice: 8` run. The state MLP's width is
not the cost; the CNN encoder/decoder is, and it is the same in both. `data/image_nonzero_frac` is
1.0 throughout because both the demo rows and the online rows now carry real frames.

### Both smokes, side by side

| | smoke 1 | smoke 2 |
|---|---|---|
| `state_slice` | 8 | null |
| `state_dim` | (8,) | (17,) |
| demo set | `dHfull_all_rns10h` (zero images) | `dHfull_all_rns10h_img` (real images) |
| encoder / decoder | cnn `['image']`, mlp `['state']` | same |
| `image_aug` on real frames | max\|aug-raw\| 0.6588, every non-blank frame changed | max\|aug-raw\| 0.8549, `frames_changed` 1.000 |
| first online frame | min 10, max 236, mean 107.4, all six workers non-blank | identical |
| settled fps | **45.5–46.7** | **46.6–46.8** |
| exit | 0 (full 20 000 online steps) | 0 (full 20 000 online steps) |

Reference: the state-only arm settles at **81.7 fps**. Pixels cost ~43 % of throughput on this box
and both configurations sit well above the 25 fps line, so the number of world models we can afford
is not changed by a factor that matters. The first 5 000-step window reads 14–15 fps in both runs —
that is `torch.compile` plus, in smoke 1, about two minutes of overlap with PX-1's eight-world image
build; it is not the steady-state rate.

---

## 5. How to launch

```bash
cd /home/j/workspace/r2dreamer
export R2_LONG_RUN=1 R2_MILESTONES='[500000,1000000,2000000]'
export GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
.venv/bin/python train.py \
  env=genesis_full_pixel seed=0 env.steps=2000000 \
  env.demo_dir=/home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img \
  env.ladder=nested_sparse10 env.far_release=false env.tip_guard=not_in_hand \
  env.return_clamp=10.0 model.return_clamp=10.0 \
  model.rep_loss=dreamer model.image_aug=shift4 env.state_slice=8 \
  env.actor_dist=bounded_normal env.act_entropy=3e-5 \
  buffer.max_size=5e5 logdir=<logdir>
```

Change `env.state_slice=8` to `null` for the pixels + full-state arm. The coordinator registers and
launches the real run; this lane did not.

**The demo set must carry real images.** `dHfull_all_rns10h`'s `image` column is all zeros (it was
built for the state-only arm) — it is a plumbing set only; training on it would teach the encoder
that the world is blank. Lane PX-1's `dHfull_all_rns10h_img` (74 npz, real images, landed 12:24) is
the set to use, and is what smoke 2 ran on.

**Buffer memory.** With a real image column, `buffer.max_size=5e5` holds 5e5 × 64×64×6 B ≈ 12.3 GB
of frames on the CPU storage device, plus ~10 GB of `stoch`/`deter`. The state-only runs already
paid the image cost (their zero column is stored too), so this is not new, but it is worth knowing
before two pixel runs are put on this box at once.

---

## 6. What was NOT done

- The real run was not launched (coordinator's call).
- No learning claim. 20 000 online steps is a plumbing smoke; nothing here says whether a pixel
  world model learns this task. Only that it is fed real pixels, augments them, and runs at 46.7 fps.
- `video_pred` now works under the pixel config (it needs a decoder, which `rep_loss=dreamer`
  builds), but `trainer.video_pred_log` is still `False` and was not exercised.
- `buffer.max_size` was 6e4 in both smokes to keep the smoke's RAM small. The real run's 5e5 is
  untested at this image width — see the memory note in §5.
- The `[ladder]` stamp in both smokes reads `-dirty`. That flag describes the **genesis_pickaplace**
  tree (the stamp's `git describe` comes from there, not from r2dreamer), which carried lane PX-1's
  in-flight edits plus untracked files at smoke time. The r2dreamer changes were committed after the
  smokes as `43a0e3c`; the two cosmetic edits made between smoke 1 and the commit (a comment reword
  and `set(bad)` in place of a rebuilt dict) do not change any stamp or behaviour.
