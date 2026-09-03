# ADVERSARIAL CRITIQUE — post-maintenance launch plan (written 2026-08-25, pre-launch)

Reviewer stance: hostile reviewer of the LAUNCH PLAN, not of the code or of past results.
Question asked of every job: *what does this buy, and what does the allocation lose by
running it?* Read-only; nothing was launched, edited or ssh'd. Every claim below cites a
file:line, a manifest key, or a `.out` line from
`~/workspace/final_rr_artifacts_2026-08-24/`.

---

## VERDICT

**LAUNCH WITH THE LISTED CHANGES — but not in the proposed order, and not WAVE 1 or
WAVE 3 as specified.**

- WAVE 2 (r2d disentangle) is the right experiment and should be **first**, not second:
  it gates the corrected-world dR2D teacher *and* it is a precondition for interpreting
  WAVE 1's chosen horizon.
- WAVE 1 as specified is the most likely single way to waste the night: ~110 GPU-h on a
  configuration that has **never once ignited**, at a seed count (2) that **cannot reach
  significance even under the best possible outcome**, on a contrast that is confounded
  with a 3.4× reward-density change, and with a demo-exposure fraction ~17× smaller than
  the RLPD mediator it is meant to test. It becomes a good experiment with four changes
  (below) and the same job count.
- WAVE 3 is **not buildable tonight**: its arms B and C need a stride-1 *pixel* demo set,
  and contract-v1 tapes carry images only at the decision rate. Its launcher also does not
  read the knobs the plan passes it.
- WAVE 4 is the best value-per-GPU-hour item in the plan and should run tonight after a
  ten-line ARM-table edit, with two changes to how it is read.
- The biggest omission is not in any wave: **the corrected-world RLPD seed top-ups**
  (24 short jobs, ~72 GPU-h) that take the paper's emerging headline — the N6 source-parity
  **null** — from n=4 seeds to n=10. Nothing in the plan funds them, and they are the
  cheapest jobs in the queue.

---

## BLOCKERS (ranked: would waste GPU-hours or produce uninterpretable results)

### BL-1. WAVE 1 has no positive control at its own configuration, and the only run of that configuration is 0/2

The plan's rationale — "r2d ignites reliably in the old world (4/4 dense on dH, 08-19)" —
refers to a configuration that differs from WAVE 1 on **four** axes at once:

| axis | the 4/4 result (08-19, seeds 50-53) | WAVE 1 as proposed |
|---|---|---|
| `time_limit` | 400 sim steps (config native, `configs/env/genesis_pick_v5d4c_delta.yaml:34`) | **1200** (`cluster/sbatch_r2dreamer.sh:236`, forced by `DEMOSET=v2`) |
| demo dir | legacy `genesis_pick_pruned_delta25`, 67 eps, stride-1 re-windowed, grant-slack 48 (`sbatch_r2dreamer.sh:63-66`) | `matched_v2/r2d/<ARM>`, 56 eps, native stride-4, no slack |
| shaping γ | hard-coded **0.999** (patch `cluster/patches/r2dreamer_final_rr.patch`, `envs/genesis.py` hunk) | **0.997** (the fix) |
| checkpoint-scoring coverage | every `latest.pt` write, max taken (AUDIT_design_2026-08-22.md §4 shows ignition counts scale with coverage) | K=5 archive |

The **only** run of the post-fix dense configuration at `time_limit=1200` is the 08-24
corrected-world pilot: `outs/r2d_train_2830979.out:2724` `[eval] 15 episodes ... picked
0.00`, and `2830980.out:2493` the same. Both trained 2e6 steps, `episode/length 300.0`
throughout, one training pick in the whole run (`2830979.out:2675`). That failure is
confounded with the world change — which is exactly what WAVE 2 exists to disentangle —
but it means **the plan proposes to spend 8 jobs on a config whose only empirical datum is
a double zero, before running the 2 jobs that would tell it whether the config is viable.**

If dR2D-dense reads 0/2 in WAVE 1, the fails contrast is a floor effect and the wave
returns nothing. Note also §3's γ point below: the 0.999-vs-0.997 mismatch was a *distance-
proportional per-decision bonus* of ≈ (0.999−0.997)·φ·100 ≈ −0.1 to −0.2 per decision — a
genuine, non-policy-invariant "be near the can" incentive. It is entirely possible that the
fix removed the thing that produced 4/4.

**Change.** Run WAVE 2 first (or concurrently), and run the N5 cells at
`TIME_LIMIT=400` — the horizon at which the only r2d ignitions on this task were ever
observed. The dR2D tapes are 14-25 decisions long (`matched_v2/r2d/dR2D/repeat.json`:
`decisions_min 14 / median 18 / max 25`), i.e. comfortably inside a 100-decision horizon;
the 1200 horizon buys them nothing. Require the success-only arm to ignite in ≥2/4 seeds
before reading the fails contrast at all.

### BL-2. WAVE 1 at 2 seeds/arm cannot reach significance under *any* outcome

Fisher's exact on 2-vs-2 has minimum p = 0.167 — a perfect 2/2-vs-0/2 split is not
significant. 4-vs-4 gives p = 0.029. Given r2d's documented ignition variance (dense 4/4
on dH, sparse 8/34, dR2D pixel 0/3, "seed 1 never discovered in 1.9M steps",
`configs/env/genesis_pick_v5d4c_delta.yaml:80`), a 2-seed arm is a coin-flip readout.

For calibration, the RLPD effect being replicated is itself thin. From
`outs/rlpd_*.out` `SWEEP-HEADLINE` lines:

- dR2D sparse hold: 8, 14, 4, 7 /15 → **0.55**
- dR2DDPfails sparse hold: 3, 1, 5, 0 /15 → **0.15**
- dDP sparse hold: 3, 1, 6, 5 → 0.25; dDPfails: 2, 4, 5, 4 → 0.25

n=4 paired seeds, and the 0.55 is carried by one lottery seed (14/15). Replicating *that*
with n=2 in a noisier learner is not a decision cell.

**Change.** 4 seeds minimum on the decisive pair. Fund them by dropping the dDP/dDPfails
pair (BL-6) — same 8 jobs.

### BL-3. Three sbatch ARM tables reject the arms the plan wants to run

- `cluster/sbatch_r2dreamer.sh:196-199`: `case "$ARM" in dH|dDP|dR2D) ;; *) echo "FATAL:
  ARM=$ARM (must be dH, dDP, or dR2D)"; exit 1`. **`ARM=dDPfails` / `ARM=dR2DDPfails` die
  before the DRYRUN print block.** The `DEMOSET=v2` branch at :233 then resolves
  `matched_v2/r2d/$ARM`, which for the fails arms does not exist yet (the built dirs are
  `dH`, `dDP`, `dR2D` only — SESSION_LOG 07:23:29, job 2830902).
- `cluster/patches/dreamerv3-torch-genesis_final_rr.patch`, `sbatch_genesis_final_rr.sh`:
  `case "$ARM" in dH|dDP|dR2D) ;; *) echo "FATAL: ARM=$ARM"; exit 1`. POSTMAINT item 4
  loops `for A in dR2D dR2DDPfails dDP dDPfails dH` — 2 of 5 arms FATAL.
- `cluster/sbatch_dp.sh:218,224`: native arms are `dH|dDP|dR2D|dHunpruned`; the N7 arms
  `dHallpruned_1e3` etc. FATAL (POSTMAINT already flags this as STOP-AND-ASK at :115-117).

These are cheap edits, but they are edits to **provenance gates** during a block whose
PREREG §2 says "one commit hash for the block; a fix restarts that learner's block", and
PREREG §11 has a "Change" audit gate for exactly this. Do the edits, DRYRUN every new arm
(`DRYRUN=1 ARM=<new> ... bash cluster/sbatch_*.sh`), and log the amendment — do not
hand-edit on the cluster.

### BL-4. WAVE 3 arms B and C are not buildable from existing data (pixel demos do not exist at stride 1)

The plan's pre-launch item — "a stride-1 demo builder (derivable from the sim_states/
sim_actions sub-tape in contract-v1 tapes, no re-recording)" — is **false for dv3**.
`baselines/record_demos.py:28-30`:

```
images        (n+1,64,64,6) u8  rig (top ++ wrist) before each decision + final
sim_states    (m,17) f32  per-SIM-step sub-tape: obs before each sim step (m <= 4n)
sim_actions   (m,7)  f32  absolute command executed at each sim step
```

There are **no per-sim-step images**. dv3's genesis arm is a pixel learner
(`--configs genesis_pixels`; `demo_prefill._load_episode` asserts
`ep["image"].shape == (T,64,64,6)`). A stride-1 demo dir therefore requires a **re-record
at `action_repeat=1` with the camera rig**, not a derivation. (Repeating each decision's
image four times is not an option: it teaches the world model that the scene freezes for
three steps and jumps on the fourth — the exact "contradictory dynamics" the
`demo_downsample == action_repeat` assert exists to prevent, `demo_prefill.py:166-171`.)

Consequence: WAVE 3 cannot launch tonight. The re-record is CPU-cheap (the dH_w2 8-shard
run took under an hour, SESSION_LOG 07:35:11 → 08:15) but it also means B/C's demos are
**different tapes**, not the same tapes at a different stride — the human adapter's
arrival/dwell behaviour changes with the clock. That is a legitimate design (it is the
"native 30 Hz, demos exactly representable" design the user prefers) but it must be
disclosed, and it is a second variable in the A-vs-B contrast.

### BL-5. WAVE 3's launcher ignores the knobs the plan passes it, and B/C collide

In `sbatch_genesis_final_rr.sh` (patch text):

- `ACTREP=4` is **hard-coded**; there is no `ACTREP` env read. `EXTRA_ARGS` does not exist.
  Passing `ACTREP=1 EXTRA_ARGS="--discount 0.99925"` silently runs **arm A's configuration
  three times**, differing only in `STEPS`. This is precisely the silent-default/fallback
  bug family catalogued in `paper/AUDIT_silent_defaults_2026-08-17.md`.
- `TAG="final_${ARM}_DV3${SHAPED:+-dense}_s${SEED}"` and `RUNDIR="logs_cluster/
  genesis_final/${TAG}"` derive **only** from ARM/REWARD/SEED. `WAVE=n8A|n8B|n8C` is not
  read → all three arms write into the **same run directory** for a given seed. The
  checkpoint watcher and the eval sweep would clobber each other.
- Registry: `REG_KNOBS` carry `action_repeat`, `budget`, `demo_sha` but **not the
  discount**. Once ACTREP is wired, arms B and C differ *only* in `--discount` → identical
  full key → `run_registry.py check` exits 2 → "FATAL: registry refused this cell". One of
  B/C dies at job start.

POSTMAINT:82-86 already says "CHECK FIRST ... STOP-AND-ASK". The plan lists this as routine
pre-launch work; it is a launcher change plus a knob added to the registry key, and it must
be DRYRUN-verified for all three arms before any of them is submitted.

### BL-6. The WAVE 1 contrast confounds fail-tape content with a 3.4× reward-density dilution — and the WM's exposure to the mediator is ~17× smaller than RLPD's

Measured on the local copies of the v2 sets (decisions per tape from the npz; rows =
decisions+1 in dreamer format):

| arm | success decisions | fail decisions | **fail share** | rewarded rows / rows | **reward density** |
|---|---|---|---|---|---|
| dR2D | 954 (56 tapes, median 18) | — | 0 | 56 / 1010 | 5.5 % |
| dR2DDPfails | 954 | 2400 (8 tapes × 300) | **0.716** | 56 / 3418 | **1.6 %** (÷3.4) |
| dDP | 6909 (56 tapes, median 115) | — | 0 | 56 / 6965 | 0.80 % |
| dDPfails | 6909 | 2400 | **0.258** | 56 / 9373 | 0.60 % (÷1.34) |

Two consequences:

**(a) The contrast is not single-variable.** Adding the fails changes *both* the
off-manifold content and the reward density, by very different factors in the two arms
(3.4× vs 1.34×). N8 itself names reward-frames-per-batch as "the density lever
METHODOLOGY §6.5 calls load-bearing for this arm". A drop in dR2DDPfails is therefore
**not** evidence that "the WM suffers from DP failure tapes the way RLPD did"; it is
equally an instance of the density lever the project already believes in. The
pre-registered fix already exists — N5's "share-matched fails arm (subsample fail
transitions to ~28%)" — and it belongs *in* this wave, not after it.

**(b) The share is right; the exposure is not.** Good news for the design: r2dreamer's
buffer does sample per-transition. `buffer.py:16-22` uses `SliceSampler(num_slices=…,
traj_key="episode")` where `episode` is the **stream/column index**, not the demo
(`demo_prefill.py` docstring: "episode int32 = column index"), so slices are drawn
uniformly over storage positions and exposure is proportional to transitions — the same
mediator as RLPD's per-transition demo sampling, and the measured share (0.716) matches
N5's 74 %.

But the *fraction of gradient signal* is wildly different:

- RLPD: `demo_batch=128` of a 256 batch (`baselines/rl/train_rlpd.py:283,286`) → demos are
  **50 % of every batch**; fails ≈ 0.72 × 0.5 = **36 % of every gradient step**.
- r2dreamer: prefill is `demo_duplicate 4` × ~3.4k rows ≈ 13.7k transitions into
  `buffer.max_size=450000` (`sbatch_r2dreamer.sh:303`); observed on the live pilot,
  `outs/r2d_train_2830979.out:` `'frames_raw': 27940, 'transitions_added': 28014 …
  buffer.max_size=450000`. Once online data fills the ring, demos are **~3 % of the
  buffer** and the fails **~2 %**, refreshed every 150k steps.

**36 % vs 2 % is a 17× difference in exposure to the very quantity N5 identifies as the
mediator.** A null in the WM arm would then be substantially explained by dilution, not by
learner robustness — i.e. exactly the "uninformative null" the task asks about. This is
answerable cheaply: `sbatch_r2dreamer.sh` already exposes `DUPLICATE` and `BUFFER_MAX`
(:54, :303, passed only when set — the good silent-default discipline). Either run the pair
at an exposure-matched setting (e.g. `BUFFER_MAX=40000`, giving demos ≈ 34 % of the ring;
note the prefill assert `dup*raw_frames < 0.5*max_size` still holds at 13.7k < 20k), or —
minimum — **print the exposure arithmetic in the readout and never state the null without
it**.

### BL-7. `matched_w3/dR2D` is a byte-identical copy of `matched_w3/dDP` — and the launch list points at it

`matched_w3/MATCHED_SETS.json`:

```
"dDP":  {"content_sha256": "d3bf95f6af4475952f4aaefb96956c914ac763a9049f5b0788ce55f3dae4784a", "N": 58}
"dR2D": {"content_sha256": "d3bf95f6af4475952f4aaefb96956c914ac763a9049f5b0788ce55f3dae4784a", "N": 58}
```

Cause: `paper/SESSION_LOG_2026-08-23_cluster.md:139` — the w3 build ran
`make_matched_sets.py --dH dH_w2 --dDP dDP_w2 --dR2D baselines/demos_v1/dDP_w2`, because
`--dR2D` is `required=True` (`baselines/make_matched_sets.py:197`) and no new-world dR2D
teacher existed. `matched_w3/census.md` consequently **publishes a dR2D column that is a
duplicate of dDP** (identical transitions 7285, len p50 120, density 0.00796).

`paper/POSTMAINT_COMMANDS.md:37` says "use matched_w4 if item 2 ran, **else matched_w3**"
and :39-41 then loops `ARM=dR2D`. That path trains a labelled dR2D cell on dDP tapes and
**passes every gate**: the manifest contract is v1, the sha matches its own manifest, the
sim_variant matches, the arm name is free text and the registry keys on the fingerprint
(which is legitimately different from dDP's only in filenames, and here is the same set of
files). Nothing would flag it.

**Change before anything launches:** delete or rename `baselines/matched_w3/dR2D` (e.g.
`dR2D_PLACEHOLDER_DO_NOT_USE`), strike the dR2D column from `matched_w3/census.md`, and
make the POSTMAINT item-3 dR2D loop conditional on `matched_w4` existing.

---

## WARNINGS (launch, but change the readout or disclose)

**W-1. N7's `hold` is at the ceiling; its falsifier is untestable there.** dH_DP w2final
hold = 13,13,14,14,14 /15 (`DP-HEADLINE arm=dH seed=20..24`). N7's falsifier is
"dHallpruned ≥ dH", which cannot be observed against 14/15. Make **`rnd` the primary
readout for N7** (dH rnd = 18,19,18,14,13 /30 — room in both directions) and report hold as
the ceiling-limited secondary. Otherwise the pre-registration is half unfalsifiable.

**W-2. N7 should use seeds 20-22, not 30-32.** PREREG §7 analyses "paired by seed index";
`POSTMAINT:113` uses seeds 30-32 against a baseline run at 20-24, forcing an unpaired test
against a 5-seed row. Seeds 20,21,22 pair one-to-one and cost nothing (different ARM →
different registry key, no refusal).

**W-3. `make_pruned_bc_set.py` leaves a stale sub-tape.** The slicing loop only prunes
arrays whose leading dim is `n` or `n+1`; `sim_states`/`sim_actions` have leading dim `4n`
and are copied through **unpruned**. Harmless for DP (lerobot reads `states`/`actions`),
but it is a trap for any later stride-1 derivation from a pruned dir, and it inflates the
npz. Also: `manifest.sim_variant` is inherited from the source manifest — `matched_w3/dH`
carries `gc_kp4_riser3_shelf6`, so those DP jobs **must** set
`SIM_VARIANT=gc_kp4_riser3_shelf6` or `sbatch_dp.sh:300-303` FATALs. (POSTMAINT does; keep
it.)

**W-4. Reward-mass asymmetry in the dR2D dreamer dir.** `matched_v2/r2d/dR2D/repeat.json`:
`total_reward 64.0` for 56 episodes (vs `56.0` for dH and dDP) — the 8 tapes that land
picked+placed in one repeat-4 window pay +2 (already footnoted in the runbook). Constant
within the N5 pair, so harmless there; must not travel into any cross-source WM comparison
without the footnote.

**W-5. Reward-regime mismatch in the N5 comparison.** The RLPD numbers being replicated
(0.55 → 0.15) are **sparse**; WAVE 1 is **dense**. "Same data, different learner" is true;
"same reward function" is not. Either say so in one sentence in the claim, or add 2 sparse
seeds to the decisive pair (sparse r2d ignition is 8/34, so this is expensive — disclosure
is the honest cheap option).

**W-6. Old-world defensibility (both directions).** *For:* the RLPD half of N5 was measured
on exactly these `matched_v2` tapes, in this world; running the WM half elsewhere would
break the pairing; the corrected-world fails sets do not even exist
(`matched_w3/MATCHED_SETS.json: "fails_arm_n_fails": 0`), and building the corrected-world
version of N5 means re-running the RLPD fails arms too — several times the cost. *Against:*
`gc_kp4_riser3_shelf6` is the world of record by user decision (SESSION_LOG NOTE 08-24
~08:15), and N6 shows the learners are world-sensitive (RLPD 0.42 → 0.18 sparse at the same
budget). **Verdict: defensible as a mechanism result, in one clearly-scoped sentence
("characterised in the pre-correction world, on the same tapes as the RLPD measurement"),
and only if the corrected-world replication is named as future work.** What is *not*
defensible is POSTMAINT:57-59's plan to make this "the paper's headline" — an old-world,
n=2, dense-vs-sparse cross-learner comparison cannot carry a headline.

**W-7. `dv3_interrogate.py` is uncalibrated and its density metric does not exist.** Its
own header: "*** NOTHING HERE HAS BEEN RUN AGAINST A REAL dv3 CHECKPOINT ***", five
`API-GUESS` sites. `value_reach` returns `None` with a reason when the terminal value is
not positive (`analysis/dv3_interrogate.py:600-606`) — i.e. exactly the untrained/flat
case N8 predicts is likely, in which case all three arms report `None` and the wave's
*primary* readout is empty. Worse, the replay-density input it parses,
`train/data/reward_frames`, **does not exist in the dv3 tree** (`grep -rn "reward_frames"
~/workspace/dreamerv3-torch-genesis/*.py` → no hits), so §4 will report ABSENT. Spend ~1
GPU-hour running the tool against an existing checkpoint (the MS-HEAD control, or `rr_dH_s2`
@320k which N4 records as 5/6) **before** committing 75-100 GPU-h to a wave whose readout
it is. The rewarded-frames-per-batch number can be computed analytically from the demo dir
for free.

**W-8. WAVE 3's budget matching confounds updates with experience.** 250k decisions at
repeat 1 vs 1M sim steps at repeat 4 matches *updates* (train_ratio is per decision) but
gives B/C **one quarter of the environment experience and one quarter of the episodes**
(at `time_limit 1200` sim steps: 208 episodes vs 833). N8 prediction (2) — "B trails A
because rewarded frames per batch are ~4× lower" — is then not identified: 4× fewer
episodes explains it equally well. Either run B at both matchings (one extra job) or demote
prediction (2) to descriptive. Prediction (1) (value_reach in sim steps) is the sound half,
because the metric is explicitly clock-normalised (`value_reach_sim_steps =
value_reach_decisions × action_repeat`, asserted at :1355-1356).

**W-9. PREREG §3.3 caps fails at 30 % of *episodes*; the mediator is a *transition* share.**
`make_matched_sets.py:202 --fail-share 0.30` is an episode cap: 8/64 = 12.5 % of episodes
but **71.6 %** of transitions. The pre-registration's matching unit does not control the
quantity its own mechanism claim rests on (AUDIT_design_2026-08-22 §1 said this about
episode-matching generally; it bites hardest here). Report both numbers wherever the fails
arms appear.

**W-10. Sim-variant gate footguns (both benign, both fail fast).** (a) The `DEMOSET=v2`
provenance gate (`sbatch_r2dreamer.sh:239-255`) asserts the demo dir's `sim_variant` equals
`R2D_SIM_VARIANT`; `matched_v2` manifests carry **no** `sim_variant` key → resolve to
`base`. So WAVE 1 must **not** set `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6`, even though the
runbook's post-maintenance section says "every launch below MUST" — copy that line and the
job FATALs at start (intended, cheap). (b) `to_dreamer_native.py` writes
`sim_variant=(sorted(svs)[0] if 'svs' in dir() and svs else src_sv)` — `svs` is never
defined, so the value always comes from the source manifest. It happens to be correct; it
is dead code that will confuse the next reader.

**W-11. WAVE 2's train/demo horizon mismatch.** At `TIME_LIMIT=400` (100 decisions) the
`r2d_dH_w2` tapes have median ~120 decisions (`matched_w3/census.md`: tape len p50 110-120)
— most demos are longer than any online episode the agent will ever see. This was *also*
true of the 08-19 wave that ignited 4/4 (the legacy delta25 dir is ~159 decisions/episode
after downsampling), so it is not a blocker — but say it in the readout, because it is the
obvious reviewer question about a 400-step horizon.

**W-12. Seed hygiene.** PREREG §1: "new seed values never used before for that learner".
`RUN_REGISTRY.jsonl` shows r2dreamer has used dH 50-53, 99; dR2D 0, 40-43; legacy 60, 61.
WAVE 1 should therefore avoid 40-43 for its dR2D arms (the knobs differ, so the registry
would not refuse — this is a protocol rule, not a gate). 70/71 (WAVE 2) and dv3 40/41
(WAVE 3) are clean.

---

## Answers to the seven questions asked

1. **Does WAVE 1 answer N5?** Partly. *Ingestion works*: `to_dreamer_native.py` has no
   terminal-reward gate on zero-reward tapes (fails convert: reward all 0, `is_terminal[-1]
   = terminated[-1]`), and `demo_prefill._load_episode` asserts only `is_first[0]`,
   `is_last[-1]`, `action[0]==0` — all satisfied. Verified on the local fails: all 8 carry
   `images (301,64,64,6)`, `truncated[-1]=True`, `rewards.sum()=0`. The patched prefill's
   only new gate is `terminal_reward == 1.0` on the **dir stamp**, so a fails dir built with
   `--terminal-reward 1` loads. *Sampling is analogous*: `SliceSampler` over packed streams
   is per-transition, and the measured fail share (0.716) matches RLPD's 74 %. **But** the
   *exposure* differs 17× (36 % of every RLPD batch vs ~2 % of the WM buffer, BL-6), and the
   harm channel differs in kind (RLPD's actor/critic loss runs directly on demo transitions —
   the bootstrapping loop N1 describes; dreamer's critic trains on *imagined* rollouts and
   demo transitions enter only through the world model and the start-state distribution).
   So a **null is informative only if the exposure arithmetic is reported alongside it**, and
   a **drop is confounded with the 3.4× density dilution**. n needed: **4 seeds minimum**
   (2v2 cannot be significant); 6 would be comfortable.
2. **Old world defensible?** Yes as a scoped mechanism result, no as a headline — see W-6.
3. **Does the γ fix change what "dense" means?** Yes, materially. The 4/4 ran with
   shaping γ = 0.999 against an agent discount of 0.997; that mismatch is a
   distance-proportional bonus of order 0.1-0.2 reward per decision at `reward_scale 100` —
   a real "approach the can" incentive, not a rounding error. The fix removes it. The
   within-condition-contrast defence is **sufficient for the fails contrast itself** (both
   arms share the reward), but **insufficient for the wave as planned**, because it does not
   protect against the floor: if the post-fix dense recipe does not ignite, there is no
   contrast to be within. Hence BL-1's requirement of an ignition anchor.
4. **Is WAVE 3 worth 102 GPU-h at 2 seeds?** Not as specified: it cannot be built (BL-4),
   its launcher ignores its knobs (BL-5), its instrument is uncalibrated and its density
   input does not exist (W-7), and its budget control confounds updates with experience
   (W-8). `value_reach` is a *sound idea* — clock-normalised to sim steps by construction —
   but it is self-normalised against the model's own terminal value and returns `None` on a
   flat head, so on an arm that has never confirmed ignition the modal outcome is three
   `None`s. Calibrate first (1 GPU-h), then decide.
5. **Ordering / opportunity cost.** Wrong order. WAVE 2 gates WAVE 1's horizon *and* the
   corrected-world dR2D teacher (the missing third source of the world of record) — it must
   go first. What is being crowded out and should not be: the **RLPD corrected-world seed
   top-ups** (`POSTMAINT:47-50`, dH/dDP × sparse/dense × seeds 24-29 = 24 jobs at ~3 h,
   `sbatch_rlpd.sh:70` allows 12 h). The paper's emerging headline (N6: source parity) is a
   **null asserted at n=4 seeds per cell** with per-seed hold counts spanning 0-10/15; going
   to n=10 is the single largest increase in claim strength per GPU-hour available, and the
   jobs are short enough to backfill around the long r2d/dv3 runs. Cut WAVE 3 entirely
   tonight; halve WAVE 1 (drop the dDP pair) and spend the freed jobs on seeds.
6. **Failure modes that would waste the night.** Ranked: ARM-table FATALs (BL-3, three
   scripts); missing `matched_v2/r2d/{dDPfails,dR2DDPfails}` dirs; the dv3 launcher silently
   ignoring `ACTREP`/`EXTRA_ARGS` and three arms sharing one RUNDIR (BL-5); the B/C registry
   full-key collision (BL-5); the non-existent stride-1 pixel demos (BL-4); the
   `matched_w3/dR2D` placeholder (BL-7); `R2D_SIM_VARIANT` set against old-world v2 dirs
   (W-10a, fails fast); unbuilt lerobot datasets for the N7 arms (`sbatch_dp.sh:334-337`
   FATALs if `$DEMO_ROOT/$ARM/lerobot` is absent — POSTMAINT builds them, keep that order).
   Wall-clock: `--time` is generous everywhere (r2d/dv3 2 days, DP 14 h, RLPD 12 h); nothing
   is under-budgeted. No registry collision exists for the proposed r2d/dv3 seeds *except*
   the B/C case above.
7. **Pre-registration violations.** (a) PREREG §1 seed-disjointness for r2d dR2D 40-43
   (W-12). (b) PREREG §7's paired-by-seed analysis is broken by N7's seeds 30-32 (W-2).
   (c) N7's falsifier is unfalsifiable on the ceilinged `hold` split (W-1). (d) N5's own
   "share-matched fails arm" is registered as a follow-up but the design cannot be read
   without it (BL-6) — promote it. (e) PREREG §2's "one commit hash for the block; a fix
   restarts that learner's block" is violated by editing three launchers mid-block; PREREG
   §11's Change-audit gate applies. (f) PREREG §8's r2d pilot bar ("≥1 seed ignites by 1M
   with best-ckpt ≥0.5, else keep 400") was **failed** by the 08-24 pilot (0/2 at 2e6) —
   the amendment it prescribes is *keep 400*, which WAVE 1 as proposed does not do.

---

## REVISED WAVE PLAN

Assumes ~13 concurrent GPUs. "Tonight" = the first slot; jobs listed in submission order.

| # | wave | jobs | GPU-h | one-line justification |
|---|---|---|---|---|
| **R1** | **r2d disentangle** — corrected world, native `TIME_LIMIT=400`, `DEMO_DIR=demos_v1/r2d_dH_w2`, `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6`, seeds 70,71 (POSTMAINT item 1, unchanged) | **2** | ~24 | Gates the corrected-world dR2D teacher *and* tells WAVE D which horizon to run; the PREREG §8 amendment already prescribes falling back to 400. |
| **R2** | **RLPD corrected-world seed top-ups** — `ARM ∈ {dH,dDP} × REWARD ∈ {sparse,dense} × SEED 24-29`, `DEMO_ROOT=matched_w3`, `SIM_VARIANT=gc_kp4_riser3_shelf6` (POSTMAINT item 3, second loop) | **24** | ~72 | Takes the paper's emerging headline (N6 source parity, a NULL) from n=4 to n=10 seeds; shortest jobs in the queue; zero new code; backfills the GPUs R1 leaves idle. |
| **R3** | **N7 action-density control** — 4 pruned variants × seeds **20,21,22**, `DEMO_ROOT=matched_w3`, `SIM_VARIANT=gc_kp4_riser3_shelf6`; primary readout **`rnd`** | **12** | ~36 | Pre-registered, cheap, and blocks any "demo quality" claim; needs only the `sbatch_dp.sh:218` ARM-table edit + DRYRUN. |
| **R4** | **N5 WM decision cells** — r2dreamer, dense, `matched_v2` (old world, `R2D_SIM_VARIANT` unset), arms **dR2D vs dR2DDPfails only**, **4 seeds** (80-83), `TIME_LIMIT` per R1's answer (400 unless R1 says 1200 is fine); gate: read the contrast only if dR2D ignites ≥2/4 | **8** | ~110 | The pinned N5 question, at the minimum n that can be significant, with an ignition anchor built in; dropping the dDP pair costs nothing (RLPD found 0.25 vs 0.25 there and the density change is only 1.34×). |
| **R5** | **N5 share-matched fails arm** (subsample fail *transitions* to ~26 %), 4 seeds — *only if R4's pair reads* | 4 | ~55 | Promotes N5's own registered follow-up into the identification, separating "off-manifold content" from "reward-density dilution" (BL-6). |
| **R6** | **N8 clock/horizon** — DEFERRED. Tonight: (i) ~1 GPU-h calibrating `dv3_interrogate.py` on an existing dv3 checkpoint; (ii) CPU re-record of dH at `action_repeat=1` **with the rig** to create the stride-1 pixel set; (iii) wire `ACTREP`/`EXTRA_ARGS`/`WAVE` + `discount` into `sbatch_genesis_final_rr.sh` and its `REG_KNOBS`; then relaunch as A/B/C with **sim-step-matched** budgets as the primary and update-matched as a secondary | 0 tonight | ~1 | The arms as specified are unbuildable and the launcher ignores their knobs; the instrument has never seen a real checkpoint. One GPU-hour of calibration decides whether 75-100 are worth spending at all. |
| **R7** | **Pre-launch, zero GPU** — rename `matched_w3/dR2D` to `dR2D_PLACEHOLDER_DO_NOT_USE`, strike its census column, make POSTMAINT item 3's dR2D loop conditional on `matched_w4`; build `matched_v2/r2d/{dDPfails,dR2DDPfails}` with `to_dreamer_native --terminal-reward 1`; add the fails arms to `sbatch_r2dreamer.sh:197` and `sbatch_genesis_final_rr.sh`; DRYRUN every new ARM | 0 | 0 | Every one of these is a job that would otherwise FATAL at start or, worse, silently train the wrong cell. |

**Totals tonight:** 46 GPU jobs, ~240 GPU-h (R1-R4), vs the original plan's 28 jobs /
~275 GPU-h — more jobs, less compute, and every job has a readable outcome.

**What the plan gets right and should not be second-guessed:** choosing r2dreamer over dv3
for the N5 cell (dv3 has no confirmed ignition, N4, and its launcher has never run these
arms); running the WM cell on the *same* v2 tapes the RLPD numbers were measured on;
keeping the fails arms out of the BC rows (N7's "BC only" warning is correctly propagated
into `make_pruned_bc_set.py`'s manifest `bc_only=True`); declining the gripper-model change
and the full-task/place expansion; and treating WAVE 2 as a single-variable disentangle of
the 08-24 pilot's three simultaneous changes — that experiment is exactly right, it is just
in the wrong position in the queue.
