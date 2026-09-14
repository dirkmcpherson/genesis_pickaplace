# PX cluster deploy — the PIXEL world-model recipe on the cluster (lane PXC-1, 2026-09-13)

Lane PXC-1 (Fable), pop-os, for the head session. Cluster `login.pax.tufts.edu`, `LAB=/cluster/tufts/shortlab/jstale02`,
`W=$LAB/wm_fix_2026-09-03`. Repo branch `ladder-unify-2026-09-11`. Registration: PHASE_PLAN amendment (af)
(`7883021`, registered BEFORE any of this ran). **Nothing of the batch was submitted** — this lane built, smoked and
hands back `cluster/submit_px_batch.sh`; the head session registers the job ids and submits.

Every number below is from tool output. Sections marked PENDING are filled at the end of this lane's session.

## 1. Trees (new; no pinned tree was touched)

| tree | path | commit | how |
|---|---|---|---|
| genesis_pickaplace | `$LAB/gp_px` | `24a4da30` (`known-good-2026-08-27-1009-g24a4da30`, clean) | `git clone --branch ladder-unify-2026-09-11` from GitHub at `81c7166`, then fast-forwarded to `2830405` (launcher/sweep/eval/submit script) before the first smoke, and to `24a4da3` (**while both smoke jobs were running** — the diff is `submit_px_batch.sh` only, no file any job reads at run time; disclosed because the standing rule forbids moving a tree under a job; the smokes are this lane's own plumbing jobs, not runs of record; their preflight stamps read `2830405`, their in-job evals `24a4da30`). It does NOT carry `08aed34` (verifier wording in `px_img_sets.sbatch`) — pull it before the next set build, or leave it; it moves no further under any job of record |
| r2dreamer (pixel-capable) | `$W/r2dreamer_px` | `0b1b9d8` (clean) | `git clone cluster/bundles/r2dreamer_px_full_main_2026-09-12.bundle -b main` (= `43a0e3c`, lane PX-2's tree: `configs/env/genesis_full_pixel.yaml`, `state_slice`, `image_aug`, `+wm_init` all present), then ONE commit on top (§4) shipped as `cluster/bundles/r2dreamer_px_full_main_2026-09-13.bundle` |
| python | `$LAB/r2d_venv` (torch 2.8.0+cu126) | — | the venv every r2dreamer job of record uses |

Do-not-touch list honoured: `$LAB/gp_ladderN` (still `a40c8aa1`), `$W/r2dreamer_ladderN` (still `0cf3d9e`), `$LAB/gp_ac`,
`$LAB/gp_aa4`, `$LAB/gp_unified`, `$W/r2dreamer_unified`, `$W/gp_root`, `$LAB/gp_e2e`, `$W/r2dreamer_fix` — none opened
for writing; no job that is not this lane's was touched. The deployed `$W/ln14_milestone_sweep.sh` /
`$W/ln14_milestone_eval.sbatch` were **not** redeployed (§6).

## 2. Demonstration sets (rsynced, never by git)

`rsync -a` of `/home/j/data/genesis_pickaplace/demos_state_full/{dHfull_all_rns10h_img,dDPfull_first_rns10h_img}/` →
`$W/demos_state_full/` (152 files, 182 684 048 bytes, 8.9 s). Verified ON THE CLUSTER with `$W/px_verify_img_sets.py`
(read-only; `VERIFY-OK`):

| set | npz / `n_written` | Σ reward | `home` (end_reasons) | ladder / guard / images / state_only | action+state == source (5 sampled) | frames non-blank (3 sampled) | built on |
|---|---|---|---|---|---|---|---|
| `dHfull_all_rns10h_img` | 74 / 74 | **130.0** | **13** (tipped 26, stream_exhausted 23, truncated 12) | `nested_sparse10` / `not_in_hand` / `rendered` / `False` | 5/5 (`rz_actions_sha256` e.g. `2434a8df96829171` for uid 232's tape) | 1.0000, 1.0000, 1.0000 (mean 104.5–107.8, min 10, max ≤ 255) | pop-os, 32 cores, AVX2 |
| `dDPfull_first_rns10h_img` | 72 / 72 | **140.0** | **14** (truncated 26, tipped 20, stream_exhausted 12) | same; `one_per_ic_first: True` inherited | 5/5 | 1.0000 ×3 (mean 105.1–109.5) | pop-os, 32 cores, AVX2 |

Other gate fields, both sets: `with_state True`, `terminal_reward 1.0`, `reward_from_tape True`, `scope full`,
`sim_variant gc_kp4_riser3_shelf6`, `action_repeat 4`, state column `(T,17)`, image column `(T,64,64,6) uint8`;
filename sets identical to `$W/demos_state_full/{dHfull_all,dDPfull_first}`. Ladder stamp hashes
`full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632` = the cluster tree's.

**Disclosure carried from PX_IMAGE_DEMOS §4.3:** these are the LOCAL (pop-os, AVX2) re-executions — Σ 130 / 140 with
13 / 14 `home` — not the cluster's class-of-record `_rns10h` sets (Σ 120 / 120, 12 / 12 `home`, pax146 records). Their
action streams are identical; the reward column differs on 1 human and 2 machine tapes. Amendment (af) names exactly
these sets (and the `_rnrh_img` sets of §7 ARE built on the class of record).

## 3. Launcher: `cluster/wmfix_full.sbatch` (commit `2830405`)

`ENVCFG`, `RECIPE` and the hydra extras were already plumbed (`env=${ENVCFG:-genesis_full_state}`, `RECIPE` default
`env.actor_dist=bounded_normal env.act_entropy=3e-5`, `EXTRA_OVERRIDES=("$@")` after the three positionals, all reaching
the `CMD` array that is `printf %q`-ed into the log). Changes, all fail-closed, none relaxing a check:

1. `ENVCFG` is validated (`genesis_full_state | genesis_full_pixel`, else FATAL before the demo gate) and exported.
2. **Pixel demo gate** inside the existing `PYG` block: under `ENVCFG=genesis_full_pixel` the set's `repeat.json` must say
   `images: rendered` AND `state_only: false` (both written only by `relabel_reward.py --images`), and the first tape's
   `image` column must be `(T,64,64,6) uint8` with every frame non-blank — otherwise a pixel encoder is prefilled from
   the zero placeholder every state-only set carries. The strict checks (`with_state`, `terminal_reward 1.0`, stride 4,
   `sim_variant`, `n_written`, state `(T,17)`, set ladder == run ladder) are unchanged and passed on the `_img` sets
   without extension. The `[demo-gate]` line now prints the set's ladder and `images`.
3. `[envcfg]` line: env config, recipe, extra overrides, both trees with `git describe`.
4. After training, a pixel run must have a `[image] first ONLINE transition … nonzero_frac ≥ 0.99` stamp in its own
   log (else FATAL 3, no evaluation), beside the existing `[sim-variant]` check.
5. **Requeue keeps the interrupted attempt** (commit after the smokes, see §9): a requeued run still starts CLEAN, but
   the partial logdir is moved to `<logdir>.preempted<N>` instead of `rm -rf`. Found by smoke 3684501: preempted at
   20:44:00 on pax047 AFTER writing `milestones/online_20000.pt`, requeued at 20:46:08 on pax141 with restart 1 — and
   the old handler deleted the milestone with the logdir. Under (af)'s policy ("a seed preempted after a milestone keeps
   that milestone's cells") a milestone the hourly sweep had not yet copied would simply have vanished. The kept dir is
   not a run (the sweep skips `*.preempted*`); scoring its milestones is a deliberate, labelled staging
   (`$W/px_smoke_cell.py`-style). Disk: ~0.5 GB per kept pixel attempt (latest.pt 135–149 MB + milestones).

Memory `--mem=48g`, `-n 8`, `--requeue` + clean restart on `SLURM_RESTART_COUNT > 0`, `--constraint=l40s|a100|l40|h200`,
`--exclude=pax077`: unchanged. QOS/partition come from the submit script: `-p gpu,preempt --qos=preempt --nice=0`
(the sweep's `gpu` mode and the RLPD preempt jobs' shape). Measured on the QOS: `preempt` = `gres/gpu=20, cpu=1000`.

## 4. r2dreamer_px: one commit (`0b1b9d8`) — provenance records the tree

`train.py:_write_ladder_provenance` now writes into `ladder_provenance.json`: `r2dreamer_tree` (abs path of the tree
`train.py` ran from), `r2dreamer_git` (`git describe --always --dirty`), `env_config` (hydra's `env` choice),
`rep_loss`, `image_aug`, `state_slice`, `encoder_cnn_keys`, and prints one `[tree]` line. Nothing else changed; a
state-only run writes the same keys with its own values. The milestone eval reads `r2dreamer_tree` back (§6).

## 5. `cluster/submit_px_batch.sh` — the submit command of record (NOT run for the batch)

```
DRYRUN=1 bash cluster/submit_px_batch.sh                                  # print all 18, submit nothing
bash cluster/submit_px_batch.sh                                           # the (af) table: arm 1 (8) → arm 2 (6) → arm 3 (4), human/machine interleaved
REP=dreamer ARM=dH SEED=4 bash cluster/submit_px_batch.sh                 # ONE seed (sparse10)
REP=dreamer ARM=dM SEED=0 LADDER=nested_ramp bash cluster/submit_px_batch.sh   # ONE ramp-control seed (needs the _rnrh_img set, §7)
GP_PIN=24a4da30 R2_PIN=0b1b9d8 bash cluster/submit_px_batch.sh            # refuse unless the trees are at these revisions
```

What one job resolves to (this is the verbatim `printf %q` of the human dreamer smoke; the batch differs only in
`env.steps=1000000`, `R2_MILESTONES=[500000,1000000]`, `TAG=<rep>`, `EVAL_SETS="hold rnd"`, seed):

```
env R2_TREE=$W/r2dreamer_px GENESIS_PICKAPLACE_ROOT=$LAB/gp_px GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 LADDER=nested_sparse10 TIP_GUARD=not_in_hand ENVCFG=genesis_full_pixel TAG=pxsmoke_dreamer EVAL_SETS=hold R2_LONG_RUN=1 R2_MILESTONES=[20000] sbatch -J pxsmoke_dreamer_dH_s9990 -p gpu,preempt --qos=preempt --nice=0 $LAB/gp_px/cluster/wmfix_full.sbatch dHfull_all_rns10h_img 9990 20000 model.rep_loss=dreamer model.image_aug=shift4 env.state_slice=8
```

and the train command the launcher printed for it (log line 8, verbatim; `R2D_SIM_VARIANT`, `MUJOCO_GL=egl`,
`PYOPENGL_PLATFORM=egl` are exported by the launcher):

```
$LAB/r2d_venv/bin/python train.py env=genesis_full_pixel seed=9990 env.steps=20000 env.demo_dir=$W/demos_state_full/dHfull_all_rns10h_img env.ladder=nested_sparse10 env.far_release=false env.tip_guard=not_in_hand env.return_clamp=10.0 model.return_clamp=10.0 buffer.max_size=5e5 logdir=$W/runs/full_r2d_state_dHfull_all_rns10h_img_pxsmoke_dreamer_s9990 env.actor_dist=bounded_normal env.act_entropy=3e-5 model.rep_loss=dreamer model.image_aug=shift4 env.state_slice=8
```

Token for token the local (ad)/(ae) command (`~/runs_dv3_local/LAUNCH_px_sparse10_s0.txt`) except `seed`, `env.steps`,
`env.demo_dir`, `logdir` and argument order.

**Naming (a deliberate choice, flagged):** the launcher names a run `full_r2d_state_<set>${TAG:+_$TAG}_s<seed>`; the set
does not say which representation loss trained it and (af)'s two losses share a set, so the script passes
`TAG=<rep>`: run dirs `full_r2d_state_dHfull_all_rns10h_img_dreamer_s4`, `…_rns10h_img_r2dreamer_s0`,
`…_rnrh_img_dreamer_s0`; job names `px_<rep>_<arm>_s<seed>` and `px_<rep>_ramp_<arm>_s<seed>` for the control. The brief's
`full_r2d_state_<set>_rns10h_img_s<seed>` (no rep) would have let the two losses collide on one set+seed and hidden the
learner from the run dir (the 2026-09-09 reporting convention). Guards in the script: refuses the pinned trees by name,
dirty trees, an existing run dir, a set that is not the rendered-image build of the requested ladder, < 150 GB free;
optional `GP_PIN`/`R2_PIN`; appends every submission to `$W/runs/PX_SUBMISSIONS.log`.

## 6. Milestone sweep / eval — diffs prepared, tested with DRYRUN, NOT deployed

Repo copies (commit `2830405`); the deployed `$W/ln14_milestone_sweep.sh` (`d0175d58…`) and `$W/ln14_milestone_eval.sbatch`
(`c21782b0…`) were sha-identical to the repo copies at HEAD~ before this lane, so the diffs apply cleanly. **Deploy only on
the head session's word** (`cp cluster/ln_r2_milestone_sweep.sh $W/ln14_milestone_sweep.sh; cp cluster/ln_r2_milestone_eval.sbatch
$W/ln14_milestone_eval.sbatch; sha256sum both into the log`) — the other workstation runs that sweep hourly.

* `ln_r2_milestone_sweep.sh`: enumeration adds `full_r2d_state_*_{rns10h_img,rnrh_img}_*_s[0-9]*` (and the untagged
  spelling), any seed; excludes any run whose name contains `smoke` (was `lnsmoke`) or `.preempted` (§3 item 5);
  dedupes. Nothing else.
  **DRYRUN test** (fake `W` with three pixel run dirs + one smoke dir + one `_rns10h_s957`, real GP/R2/sbatch, touching
  nothing): `NEED` for all three pixel runs in interleaved order (`rnrh_img_dreamer_s0`, `dDPfull_first_rns10h_img_r2dreamer_s0`,
  `dHfull_all_rns10h_img_dreamer_s4`) and for `s957`; the `pxsmoke` dir NOT enumerated; 0 submitted (`lnms_ in queue: 6`, slots 0).
* `ln_r2_milestone_eval.sbatch`: resolves the r2dreamer tree per cell with the same precedence the GP tree already has —
  `R2_OVERRIDE` > the run's own `ladder_provenance.json` (`r2dreamer_tree`, copied into the cell by the sweep) >
  inherited/default `$W/r2dreamer_ladderN`; prints `[r2] tree for this cell`; FATAL if a `*_img_*` run resolves to a tree
  without `configs/env/genesis_full_pixel.yaml`; writes `trees.json` (both trees + `git describe` + where each came from)
  beside the cells; `CELL_SPECS="<set> <n> <mode>;…"` overrides the three (aa) cells (used for the smoke cell below).
  `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6` was already exported (line 81) — checked, unchanged.

## 7. `_rnrh_img` sets for (af) arm 3 — `cluster/px_img_sets.sbatch` (job 3684502, pax019: 64 physical / 64 logical, AVX-512, Xeon Gold 6438M)

Built on the cluster (never on the login node) with `$LAB/gp_px`'s `relabel_reward.py --in $W/demos_state_full/<set> --out
…<set>_rnrh_img --ladder nested_ramp --tip-guard not_in_hand --sim-variant gc_kp4_riser3_shelf6 --images --procs 8`, the
node asserted (physical AND logical 64, the milestone eval's rule), then verified in-job against BOTH the source set and
the class-of-record `_rnrh` set (pax146 records). The sbatch renames a set that fails any check to `*_UNVERIFIED` so no
launcher can pick it up.

### Human `dHfull_all_rnrh_img` (build wall 915 s on 8 procs)

| check | result |
|---|---|
| tapes | source 74, build 74 (`n_written` 74), record 74 |
| action + state byte-identical to the SOURCE, and `rz_actions_sha256` == the set of record | **74/74** — the physics is reproduced exactly (the state column carries the can pose) |
| image column `(T,64,64,6)` uint8, non-blank on every frame | **74/74** |
| end reasons | build `{home 12, tipped 22, stream_exhausted 26, truncated 14}` == record — **identical**; `home` **12 = 12** |
| grants (stage → decision), checked on 4 tapes | identical |
| Σ reward | build **204.941535** v record **204.675320** (**+0.266, +0.13 %**) |
| reward column identical to the set of record | 48/74 by exact float equality (the in-job verifier's first form); **68/74 at 1e-6** (`$W/px_reverify_rnrh_img.py`, the `08aed34` criterion) — the 6 that differ are ALL `home` tapes, ONE decision each = the terminal `home` decision, all positive: `100000-013` +0.0024 @202, `101000-023` +0.0100 @213, `104006-049` +0.0014 @498, `104007-050` +0.0765 @297, `105000-052` +0.1753 @184, `107001-066` +0.0006 @363 — summing to **+0.266215**, the whole Σ difference |
| ladder stamp | `unified-2026-09-10 \| ladder=nested_ramp \| picked=1 placed_v2=1 home=4 ramp:slide_gain_m=3/0.05m \| max_return=9 \| terminal=home+tipped \| … \| full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632` (the record's hashes) |
| verdict of the in-job verifier | **FAILED on the reward column → renamed `dHfull_all_rnrh_img_UNVERIFIED`** (fail-closed, as designed) |

**Where the reward differs, measured on four tapes (`$W/px_ramp_diff.py`):**

```
genesis-100004-017-388: sum build 9.000000 record 9.000000; differing decisions 0 (at 1e-9)   slide_gain 0.062185 v 0.062105   end home@333 both; grants identical
genesis-104007-050-391: sum build 8.533954 record 8.457499; differing decisions 1: [297]  t=297 build 4.128222 record 4.051766   slide_gain 0.042233 v 0.040958   end home@297 both
genesis-100000-013-256: sum build 7.557147 record 7.554742; differing decisions 1: [202]  t=202 build 4.011324 record 4.008919   slide_gain 0.025952 v 0.025912   end home@202 both
genesis-101002-025-435: sum build 2.184546 record 2.184546; differing decisions 0                 slide_gain identical             end stream_exhausted@434 both
```

So: (i) the "26/74 differ" count was my verifier's exact-float comparison over-counting sub-1e-9 rounding of the ramp
values — on those tapes the columns are equal at 1e-9 and the sums equal to 6 dp; (ii) the REAL differences are one
decision per tape — the terminal `home` decision — always positive, on the `home` tapes whose slide gain had not yet
saturated the 0.05 m span: the direct re-execution pays the ramp accrued during that decision's own `action_repeat`
sub-frames, while the records scorer cuts at the terminal FRAME. This is exactly the direct-vs-records convention
PX_IMAGE_DEMOS §4.4 recorded on `rz_slide_gain_m` for `nested_sparse10`, where it was diagnostic-only; under `nested_ramp`
the slide gain IS reward. Σ +0.266 over 74 tapes = the four `home` tapes' terminal decisions (+0.0765, +0.0100, +0.0024,
+0.0013 on the ones seen). `home` count, every grant, every end reason, every action and every state are those of the set
of record. The verifier now compares at 1e-6 and names the differing decisions (commit `08aed34`); it is still fail-closed.

**Decision for the head session (not taken here):** (a) accept the direct build as registered ((af) says "rendered with
`relabel_reward.py --images --ladder nested_ramp`, same recorder") with this +0.13 % terminal-decision disclosure —
`mv dHfull_all_rnrh_img_UNVERIFIED dHfull_all_rnrh_img`; or (b) splice: take the rendered image column from the direct build
onto the `_rnrh` set of record (every other column is byte-identical, so frame *t* is the render of the record's own
state *t*) — reward == record exactly, at the cost of a new splice script and its own verification. Neither was done.

### Machine `dDPfull_first_rnrh_img` (build wall 1135 s on 8 procs) — **VERIFIED, in place**

| check | result |
|---|---|
| tapes | source 72, build 72 (`n_written` 72), record 72 |
| action + state byte-identical to the SOURCE, and `rz_actions_sha256` == record | **72/72** |
| image column `(T,64,64,6)` uint8, non-blank on every frame | **72/72** |
| end reasons | build `{truncated 27, tipped 23, stream_exhausted 10, home 12}` == record — **identical**; `home` **12 = 12** |
| Σ reward | build **206.224628** = record **206.224628** (diff +0.000000) |
| reward column identical to the set of record | 48/72 by exact float equality (sub-1e-9 rounding of the ramp values); **72/72 at 1e-6** — no machine `home` tape pays anything extra at its terminal decision |
| in-job verdict / state now | the in-job verifier (exact-float form) renamed it `_UNVERIFIED`; the 1e-6 re-verification passes every check, so it was renamed back to **`$W/demos_state_full/dDPfull_first_rnrh_img`** (this lane's own build; the rename reverses the verifier's own action on the strength of the corrected criterion, both runs quoted above). `relabel_node` stamped: pax019, 64/64, avx512, Xeon Gold 6438M, job 3684502 |

So (af) arm 3 has its machine set; its human set exists as `dHfull_all_rnrh_img_UNVERIFIED` pending the decision above.
`submit_px_batch.sh … LADDER=nested_ramp ARM=dH` refuses until the human set carries its proper name.

## 8. Smokes (2 jobs, preempt QOS, 20 000 online steps, `EVAL_SETS=hold`)

Submitted with `SMOKE=1 REP=… ARM=… SEED=… bash cluster/submit_px_batch.sh` (the same script and launcher as the batch;
only `env.steps=20000`, `R2_MILESTONES=[20000]`, `TAG=pxsmoke_<rep>`, `EVAL_SETS=hold` differ).

| | smoke 1 | smoke 2 |
|---|---|---|
| job / name | **3684495** `pxsmoke_dreamer_dH_s9990` | **3684501** `pxsmoke_r2dreamer_dM_s9991` |
| rep loss / set | `dreamer` (= local {dv3}) / `dHfull_all_rns10h_img` | `r2dreamer` (contrastive, no decoder) / `dDPfull_first_rns10h_img` |
| node / GPU / partition | pax141 / NVIDIA A100-PCIE-40GB / preempt | pax047 / NVIDIA H200 NVL / gpu (QOS preempt) |
| run dir | `$W/runs/full_r2d_state_dHfull_all_rns10h_img_pxsmoke_dreamer_s9990` | `$W/runs/full_r2d_state_dDPfull_first_rns10h_img_pxsmoke_r2dreamer_s9991` |
| `Demo prefill` | `{'episodes': 74, 'episodes_unique': 74, … 'frames_raw': 29295, … 'transitions_added': 29406, 'rows_per_stream': 4901, …` | `{'episodes': 72, 'episodes_unique': 72, … 'frames_raw': 36906, … 'transitions_added': 37488, 'rows_per_stream': 6248, …` |
| `Step accounting` | prefill 29406 decisions; trainer starts at counter 117624; target 137624 | prefill 37488; starts 149952; target 169952 |
| `[sim-variant] gc_kp4_riser3_shelf6` | **6** occurrences (`grep -o`; the launcher's `grep -c` reads 4 because concurrent workers concatenated two stamps onto one line — a counting artefact, the `-ge 1` check passes) | **6** (launcher `grep -c` 5, same artefact) |
| `train rc` | 0 at 20:22:37 (job start 20:08:27) | 0 at 20:21:53 (start 20:09:13) |
| fps (`fps/fps` in metrics.jsonl, 5k-step windows) | 11.1 (compile), **55.0, 55.0, 52.3** | 13.8 (compile), **59.1, 59.5, 60.0**; requeued attempt on pax141/A100: 12.5, **61.9, 61.6, 61.5** |
| peak RSS at ~4 min (`sstat MaxRSS`) | 11.5 GB | 12.1 GB |
| milestone | `milestones/online_20000.{pt,json}` written | same |
| launcher post-train pixel check | passed (`[image]` nonzero_frac 1.0000 ≥ 0.99) | passed |

Stamps, **verbatim** (smoke 1 unless noted; smoke 2 differs only where shown):

```
[ladder] unified-2026-09-10 | ladder=nested_sparse10 | home=10 | max_return=10 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-1008-g2830405f
[ladder] tip_guard not_in_hand sustain 4 env frames
[ladder] ladder=nested_sparse10 tip_guard=not_in_hand return_clamp=10.0 (env.return_clamp AND model.return_clamp)
[demo-gate] pixel set: images=rendered state_only=False first tape 256 frames non-blank frac 1.0000 mean 107.43          (smoke 2: 601 frames … mean 105.08)
[tree] r2dreamer=$W/r2dreamer_px git=0b1b9d8 env_config=genesis_full_pixel rep_loss=dreamer image_aug=shift4 state_slice=8 encoder_cnn_keys='image'   (smoke 2: rep_loss=r2dreamer)
[ladder] return_clamp=10.0 (env and model agree)
[obs] encoder cnn_keys=['image'] mlp_keys=['state'] shapes={'image': (64, 64, 6), 'state': (8,)}
[obs] decoder cnn_keys=['image'] mlp_keys=['state']                                       (smoke 2: [obs] decoder cnn_keys=[] mlp_keys=[]  (no decoder: model.rep_loss is not 'dreamer'))
[obs] state_slice=8 state_dim=(8,)
[obs] image_aug=shift4 (pad 4 px, one offset per sequence)
[prefill] state_slice=8: demo state truncated 17 -> 8 dims
[sim-variant] gc_kp4_riser3_shelf6: kp=[800.0, 800.0, 600.0, 400.0, 240.0, 240.0] gc=1.0 riser=0.03      (x6)
[image] first ONLINE transition: shape=(6, 64, 64, 6) dtype=torch.uint8 min=10 max=236 mean=107.18 nonzero_frac=1.0000 per_env_max=[236, 236, 236, 236, 236, 236]     (identical in smoke 2)
[image_aug] shift4 pad=4: max|aug-raw|=0.8902 frames_changed=1.000 raw_max=1.0000 raw_nonblank_frac=1.000  [REAL frames: the augmented batch differs from the raw batch]
                                                                    (smoke 2: max|aug-raw|=0.8588 frames_changed=1.000 raw_max=0.9569 raw_nonblank_frac=1.000  [REAL frames: …])
```

Reading: the encoder is built over the image and the 8-dim proprioceptive state in BOTH losses; the augmentation
changes every frame of the first sampled batch (the demo sets carry real frames, so there is no blank-batch case);
every one of the six workers returned a non-blank first online frame (min 10, max 236 — the same numbers as the local
smokes in PX_PIXEL_CONFIG §4, so the cluster rig renders what pop-os renders). The two smokes ran on different GPU
classes (A100-40GB v H200) — the (af) disclosure "cluster GPU class and node per seed stamped" is met by the launcher's
first log line; the sim is CPU on both.

**In-job evals (launcher's `fresh_eval_hold15_{sample,mode}`, `preview` role, CPU on the training node) and the pinned
cells:** §9.

## 9. Results

### 9.1 Smoke evaluation cells — the pixel eval path, both in-job and pinned

| cell | job | node (phys/logical) | trees (from) | `[sim-variant]` | episodes / mode | `success_key` | result |
|---|---|---|---|---|---|---|---|
| smoke 1 in-job `fresh_eval_hold15_sample` (preview) | 3684495 | pax141 (32 logical, Xeon Gold 6226R — the GPU node) | gp_px `24a4da30` | yes | 15 / sample | `home` | 0/15 (15 `timeout`), rc=0 20:35:44 |
| smoke 1 in-job `fresh_eval_hold15_mode` (preview) | 3684495 | pax141 | same | yes | 15 / mode | `home` | rc=0 20:49:01; `# DONE` 20:49:01 |
| **smoke 1 pinned `hold15_mode`** (`$W/px_smoke_cells/…_pxsmoke_dreamer_s9990/online_20000/hold15_mode`) | **3684564** | **pax004 (64/64, Xeon Gold 6438M)** | `[gp] gp_px (from run provenance)`, `[r2] r2dreamer_px (from run provenance)`; `# gp tree describe …24a4da30`, `# r2 tree rev 0b1b9d8`; `trees.json` written | **1 occurrence** | 15 / mode | `home` | 0/15 (`outcomes_honest {timeout: 15}`), `mean_steps 300`; `metrics.json` stamps `ladder_stamp … ladder=nested_sparse10 \| home=10 \| max_return=10 …`, `node pax004`, `cpu_model`, `ncpus_machine 64`, `slurm_job 3684564` |
| **smoke 2 pinned `hold15_mode`** (`…_pxsmoke_r2dreamer_s9991/online_20000/hold15_mode`) | **3684565** | **pax002 (64/64, Xeon Gold 6438M)** | same resolution, same revs | **1** | 15 / mode | `home` | 0/15 (15 `timeout`), rc=0 20:50:02, `# DONE-MILESTONE … rc_all=0` |

The pinned cells were staged with `$W/px_smoke_cell.py` (a copy of the sweep's own copy step — the sweep excludes
`*smoke*` runs by design), sha256 of the copy == the milestone sidecar (`e681deb2d8356b4d`, `448bfdd5fc31997b`), and
submitted through the REPO eval sbatch (`$LAB/gp_px/cluster/ln_r2_milestone_eval.sbatch`, `CELL_SPECS="hold 15 mode"`,
`-p batch --qos=normal`, the sweep's 64/64 node filter, `CELLROOT=$W/px_smoke_cells`, **`R2` deliberately unset** so
the provenance branch is what ran). ~0 is the expected reading for a 20k-step checkpoint; the point is that a PIXEL
checkpoint (CNN encoder, `state_slice 8`) loads and scores under the pinned protocol with the right trees and stamps.
Wall: ~40 min per 15-episode MODE cell on the shared 64-core nodes (the in-job cells on the GPU node's CPUs took ~13 min).

### 9.2 The preemption, and what it changed

Smoke 2 (3684501, H200 pax047) finished training (rc=0 20:21:53, `milestones/online_20000.pt` + sidecar written) and
was **`CANCELLED AT 2026-09-13T20:44:00 DUE TO PREEMPTION`** during its in-job hold15 sample eval; Slurm requeued it
(`Restarts=1`), it restarted at 20:46:08 on pax141 (A100) with `# requeued (restart 1): clearing the partial logdir and
starting clean` — the OLD handler (the spooled script), which `rm -rf`-ed the logdir INCLUDING the milestone. Its pinned
cell (staged at 20:31 from the first attempt, sha-verified) was unaffected and scored (above). Consequence and fix: §3
item 5 (commit `9b50280`, unit-tested on a fake logdir: restart 1 → `<logdir>.preempted1` with its milestone intact,
restart 2 → `.preempted2`, the restart-0 "exists → FATAL" line untouched). Not exercised live — no second preemption
happened. Restart outcome of 3684501: the requeued attempt (pax141, A100-PCIE-40GB) re-ran the whole preflight (same stamps),
trained to budget again (`# train rc=0` 20:58:10; fps 12.5 compile, then **61.9 / 61.6 / 61.5** — the decoder-free
`r2dreamer` loss on the same A100 class where `dreamer` ran 52–55), passed the post-train pixel check, ran both in-job
hold15 evals (`sample` rc=0 21:11:14, `mode` rc=0 21:24:39) and printed `# DONE … 21:24:39`. So a preempted pixel job
completes on requeue; only the interrupted attempt's artefacts were lost, which §3.5 now prevents.

### 9.3 Job table

`sacct` returned no rows for this user from the login node (tried twice, with and without `-S`); states are from
`squeue` and the job logs.

| job | name | partition / QOS | node / GPU | state | timeline |
|---|---|---|---|---|---|
| 3684495 | `pxsmoke_dreamer_dH_s9990` | preempt / preempt | pax141 / A100-PCIE-40GB | completed, `# DONE` 20:49:01 | start 20:08:27; train rc=0 20:22:37; hold sample 20:35:44; hold mode 20:49:01 |
| 3684501 | `pxsmoke_r2dreamer_dM_s9991` | gpu,preempt / preempt | pax047 / H200 NVL → (preempted 20:44:00, restart 1) → pax141 / A100 | completed, `# DONE` 21:24:39 | start 20:09:13; train rc=0 20:21:53; preempted 20:44:00; restart 20:46:08; train rc=0 20:58:10; hold sample 21:11:14; hold mode 21:24:39 |
| 3684502 | `px_img_sets` | batch / normal | pax019 (64/64) | completed rc=1 (human set failed the exact-float check, machine set likewise; see §7 for the 1e-6 re-verification) | 20:09:13 → 20:43:28; human build 915 s, machine 1135 s |
| 3684564 | `pxsmk_eval_dHfull_all_rns10h_img_pxsmoke_dreamer_s9990` | batch / normal | pax004 (64/64) | completed, cell written | 20:23 → ~21:00 |
| 3684565 | `pxsmk_eval_dDPfull_first_rns10h_img_pxsmoke_r2dreamer_s9991` | batch / normal | pax002 (64/64) | completed, `rc_all=0` 20:50:02 | 20:23 → 20:50 |

Nothing of anyone else's was touched; `3684603 pxr_smoke_dH_s9991` seen in the queue is lane PXR-1's.

## 10. Not done / open

* **The 18-job batch: not submitted** (head session registers the job ids and submits; `submit_px_batch.sh` is the
  command of record; `GP_PIN=9b50280 R2_PIN=0b1b9d8` are the revisions of record for the trees as left).
* **Human ramp image set** is `dHfull_all_rnrh_img_UNVERIFIED` pending the §7 decision; (af) arm 3's human seeds cannot
  be submitted until it is renamed (the submit script refuses). The machine ramp set is in place and verified.
* **Deploying the sweep/eval diffs to `$W/ln14_*`: not done** (head session's approval; the other workstation runs the
  sweep hourly). Until deployed, the pixel runs of (af) get NO milestone cells and the (ac)-style `unknown ladder`-class
  failure recurs as "no cells at all" — deploy before the first 0.5M milestone (~2.5 h into a run at 55 fps).
* The requeue-preserve handler (§3.5) is unit-tested, not preemption-tested.
* The two `*.preempted*`/`_UNVERIFIED` conventions are new; nothing else reads them.
* fps was measured on two GPU classes (A100-40GB, H200) that the preempt allocation hands out — the (af) batch will
  land on whatever `-p gpu,preempt --constraint=l40s|a100|l40|h200` gives; per-seed GPU class is in each log's first line.
* `$W/px_verify_img_sets.py`, `$W/px_ramp_diff.py`, `$W/px_reverify_rnrh_img.py`, `$W/px_smoke_cell.py`: this lane's
  read-only helpers, left on the cluster; the two bundles `bundles_r2dreamer_px_full_main_2026-09-1{2,3}.bundle` under `$W`.
