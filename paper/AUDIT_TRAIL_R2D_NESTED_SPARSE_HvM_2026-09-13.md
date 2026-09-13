# AUDIT TRAIL — {r2dreamer, 17-d state, `nested_sparse` +1, 4M} human v machine (2026-09-13)

Audit lane (Fable), pop-os, read-only, branch `ladder-unify-2026-09-11` @ 99a766e. **No cluster access**: every finding is
from repository artefacts (`paper/`, `HRI_results/curves/ladderN_2026-09-13/`, `git log`), the local r2dreamer tree
(`~/workspace/r2dreamer` @ 43a0e3c, same `trainer.py`/`demo_prefill.py` semantics as 0cf3d9e), and python over the
per-episode records. Nothing that opens a Genesis world was run. Object under audit:
`paper/AUDIT_R2D_NESTED_SPARSE_HvM_2026-09-13.md` ("the brief"). Convention: every number is **{r2dreamer}, e2e**.

**Counts: 3 BLOCKING · 8 SHOULD FIX · 8 NOTE.** Verdict in one line: the per-seed numbers reproduce from the repo's own
cells, but the result is an unregistered n = 4 v 4 ignition contrast (p = 10/70 = 0.143) whose arms are confounded with
the training node, whose brief mis-states when the demonstrations leave replay by 4×, and whose generality is already
contradicted by the project's own +10/pixel runs — it is a direction rev 5 tests, not a paper finding.

## BLOCKING

### B1. Not supportable as a source finding today; and rev 5 reproduces the node confound by design
Registered predictions of (aa): P-aa-2…7 (`PHASE_PLAN_2026-09-04.md:1431-1443`), none a source contrast; the readout
clause says "NOT a source comparison" (:1446-1447). Rev 2 (:1508-1555, commit 5dcd380 22:24 09-11) and rev 3
(:1576-1594, 4cf58af 00:30 09-12) register the RECIPE rule (ignited = ≥ 2 of 4 seeds per arm); the first registered
human-v-machine prediction on the sparse arm is rev 5 P-aa5-1/2 (:1870-1874). So the brief's "not a registered
prediction" (§7.9) is accurate. Statistic: 3/4 v 0/4 → Fisher two-sided 0.143 = exact permutation on per-seed rates
(20,0,16,13 v 0,0,0,0: 10 of 70 splits, floor 2/70). **Rev 5's packs are one-per-ARM** (:1856-1858; the launcher's
once-only demo gate binds one set per job, :1773-1775), so each pack's node hosts one arm only — the confirmatory is
built with the same arm↔node structure as the original (S1). Fix before submission (or as a registered addendum):
mixed-arm packs (launcher change) OR both arms pinned to one node set (`-w`/`--exclude` identical for both arms), and
NodeList recorded in the log beside the seed.

### B2. The brief's demo-eviction claim is off by 4×; §7.1's "seeding effect" reading rests on it
`trainer.py:137` `step = replay_buffer.count() * action_repeat` and `:201` (one row per env per decision): a replay ROW is
one decision = 4 env frames. `buffer.max_size=5e5` rows therefore holds 2.0M online env steps, and the prefill's own
`eviction_note` (`demo_prefill.py:337-339`, "online env steps") mislabels rows as env steps. Correct figures: first demo
row evicted at online row 470,594 (human) / 462,512 (machine) = **1.88M / 1.85M online env steps**, all demo rows gone
at **2.0M** for both. The config comment agrees (`configs/env/genesis_full_state.yaml:11`: "5e5 rows > demo rows +
1M/4 online rows"). Consequences: (i) demonstrations are in replay for HALF the budget (19–23 % of rows at 0.5M, ~10 % at
1M, ~6 % at 1.9M), not "≥ 87 % of training with no demonstration"; (ii) the 2M→4M ignition window opens exactly when
the demos leave; (iii) every un-ignited seed's residual picking dies within ~0.1M of its arm's eviction (records:
machine last rolling-100 `picked` ≥ 0.05 at 1.76M / 1.79M / 1.81M / 1.95M; human s956 at 2.31M). Brief §7.1, the curves
README ("Demonstrations leave the buffer at 0.5M"), rev 5's closing paragraph (:1883-1884) and commit d7718ed's message
all carry the error. Fix: correct all four; re-read §7.1 as "the arms diverge while the demonstrations are present".

### B3. The brief omits the project's own registered counter-context, so "candidate for the paper" over-reads it
(a) Amendment (ac) (:1609-1617) registered the +1 recipe as possibly "below this learner's normalisation floor"
(`ReturnEMA` scale = clip(p95 − p05, min 1.0)); the sparse-v-sparse10 comparison exists for exactly that reason and is
not in the brief. (b) Local (ae) runs — same sets re-rendered, `nested_sparse10`, pixels + proprio, 1M steps
(`AE_PIXEL_HUMAN_VS_MACHINE_2026-09-13.md:16-36`): machine s0 0.125, s1 **0.371** (= human s0), ignition 2/2 v 2/2,
machine s1's 0.7M rnd30 cell 20/30 — machine demonstrations DO seed the slide under +10. (c) PAPER_PLAN H4 ("world
models benefit equally from all sources") is a REGISTERED prediction recorded FAILED with a directional human preference
(`HRI_results/REGISTRATIONS.md:22,74-79`); a human-favouring WM result is a second failure of a registered null, and
must be written as such, never as a fresh discovery. Fix: add (a)–(c) to the brief's §7; scope every sentence to the
recipe (see "the one sentence" below).

## SHOULD FIX

### S1. Training node is confounded with arm (brief §4 lists it, §7.10 does not name it)
Machine: s975/s976/s977 on **pax049**, s978 on pax007; human: pax008/pax105/pax007/pax011 (brief :131-133; rev-3 starts
`LADDER_N_PILOT_LOG:1075-1076`). What the repo knows: pax049 = 64 cores, 8 GPUs, ran 8 WM jobs at load 138 (2×
oversubscribed) in August (`SESSION_LOG_2026-08-23_cluster.md:182`, `sbatch_r2dreamer_pack.sh:11`); pax011 = 96 cores
(`TIP_RULE:415`); pax008 = H200 (`WM_FIX_LOG:224`); pax007 = the node excluded as "bad" on 09-01/02 (`CONFOUNDS.md:18`
row 10, CUDA "device busy"); pax105 unknown. CPU model / SMT of the GPU nodes: cluster-only (`$LAB/gp_e2e/hw_map.json`
covers `-p preempt`). Documented routes by which a node can move a TRAINING run: rollouts under load feed the replay
buffer (`CONFOUNDS.md:40` row 32), episode outcomes depend on machine class (`cluster/hw_probe.sh:5-9`), SMT-on cells
read `picked` ≈ 0 for a seed at 8/30–16/30 elsewhere (`LN_R2_MILESTONE_CELLS:186-192`; re-score outcome cluster-only).
Evidence it is probably not decisive: the one node hosting both arms (pax007) reads the same direction (human s957
16/30 v machine s978 0/30; early `picked` 0.145 v 0.072), and all four machine seeds diverge from 0–0.5M on. Fix: B1's
scheduling change plus a 2 v 2 cross-over (machine on a human node, human on pax049) if rev 5 lands 0/4 again.

### S2. s976 is counted un-ignited on a 2M cell, but this set shows 2M→4M ignition
s958 (human): rnd30 MODE `picked` 1/30 and hold15 0/15 at 2M, `home` 13/30 at 4M (`eval_cells_r2dreamer.tsv`). A 2M
cell is therefore not evidence about 4M. The training record supports the s976 call independently (0 `home` in 3,973
episodes to 3.93M; `picked` 0.00 over its last 1M, n 683), which the brief should cite. Fix: score `latest.pt` @ 3.93M
(one CPU-hour, the brief's own §8 command) as the seed's terminal cell and state how it enters rev 5's pooled 8 v 8.

### S3. CONFOUNDS 85 is mis-cited for the s976 stall
Row 85 (`CONFOUNDS.md:126`) is the replay diagnostic on uid 233 (CG cap exits in 2.42 % of substeps); it records no
training stall and no node. "CG contact-solve stall, 105 % CPU / 0 % GPU" is a cluster-side observation with no row of
its own. Fix: give it a row (node, step, symptom, what was checked), and cite that.

### S4. Rev 5's registration time is 3 h before its commit, in the same commit as the brief
PHASE_PLAN says "registered 2026-09-13 ~14:30 BEFORE submission" (:1843); the text first exists in d7718ed at **17:32**,
the commit that also adds the brief. Whether the four packs were submitted before 17:32 is cluster-only (`sacct
--format=Submit`). Same class as REGISTRATIONS item 7 (dv3 G3). Fix: record the Submit timestamps beside rev 5.

### S5. Registration-trail mislabel: s955/956/975/976 are the (aa) batch, not rev 2
Jobs 3581574–77 are in the (aa) submission table (`LADDER_N_PILOT_LOG:538-541`, registered aac4131 13:55, submitted
~18:50 09-11, commit 9ccabe9). Rev 2 (22:24) is the ramp-4M extension only (:668-712). Brief §4 (:134-135) and rev 5
(:1848) say "rev 2 + rev 3". Fix the labels; the timeline is otherwise clean (rev 3 registered 00:30, submitted 00:38).

### S6. Set asymmetries, ranked by what could produce 0/4 under +1 with tip termination (question C)
Records, arm-pooled: **early `tipped` is equal** (0–0.5M: human 0.422 v machine 0.412; episode length 215–226 v
196–220 decisions), so tip-starvation is rejected as the mechanism. **`picked` differs from the start**: 0–0.5M 0.153 v
0.059; 0.5–1M 0.193 v 0.015; 1–2M 0.466 v 0.029 — while demonstrations are still in replay. Ranking:
1. **Demonstration content** (the treatment, with its registered sub-confounds): idle 0.365 v 0.007
   (`HRI_results/DEMO_SETS_2026-09-11.md:27,48`), DP-smooth v joystick actions. The idle-collapsed human control that
   amendment (n) requires before the word "source" has never been run for the world model (S8).
2. **Rows beyond the online horizon**: 27 v 14 tapes truncated at the 300-decision cap, machine median tape 601 (the
   harvest cap) v human 389, prefill 37,488 v 29,406 rows; post-terminal rows are KEPT at zero reward
   (`PHASE_PLAN:1409-1412`) → ~8k more zero-reward rows the online policy can never produce, and a lower reward density
   (12/37,488 v 12/29,406). Cheap control: prefill with tapes cut at the horizon. Not named in the brief.
3. `is_terminal` never set on `home` rows (symmetric, `LADDER_IMPL_NOTES:348,409`) — second order, but it interacts
   with 2 (the +1 is followed by long zero-reward continuations, longer on the machine tapes).
4. Eviction timing: symmetric to within 0.03M (B2); it ends the un-ignited seeds' residual picking but does not
   cause the 0–1M divergence.
5. Σ staged 118 v 131 — irrelevant here (both Σ 12 under `nested_sparse`).

### S7. Internal inconsistencies in the brief
(i) §2 table "decisions 29,221 / 36,834" v §4 prefill "29,406 / 37,488" for the same sets, unreconciled (185 / 654 rows).
(ii) §7.5 "every un-ignited seed ends 4M with ZERO picks after 3–9/30 at 0.5–2M": s976 reads `picked` 0/30 at every
milestone, all three cells. (iii) §5 "the same shared-process protocol as PHASE_RESULTS §5.1" — true, but hold15 is
14/15 training starts (§5 says so) and the ignited seeds score 15/15 there; only rnd30 is out-of-support.

### S8. The attributing control is missing for the world model
PHASE_PLAN (n) (:412) makes an idle-collapsed human set the control before "source" is used; DP has `dHfull_pruned`
runs, the world model has none under any nested ladder. Two seeds of `dHfull_pruned`-relabelled `nested_sparse` would
decide whether item 1 above is "human" or "idle".

## NOTE

N1. **Verified from local artefacts:** brief §0 per-seed 4M rnd30 MODE `home` (20, 0, 16, 13 v 0, —, 0, 0) =
`eval_cells_r2dreamer.tsv`; every `final` cell equals its 4M cell count-for-count across DIFFERENT 64-core nodes (s955
pax146 v pax031, s975 pax019 v pax146, s977 pax002 v pax019; 7 runs × 3 cells) — evaluation is node-stable within the
class; §7.7 last-1M training `home` 0.81 / 0.81 / 0.61 for s955/957/958, 0 for the rest; `records/INDEX.tsv` s976
last_step 3,931,560; Fisher 0.143; prefill rows (`LADDER_N_PILOT_LOG:763-768`); trees/stamps (:1084-1093).
N2. **Checkpoint robustness (question D):** ignition is monotone in this set — human `home` per seed at 0.5/1/2/4M:
s955 0/0/8/20, s956 0/0/0/0, s957 0/0/2/16, s958 0/0/0/13; machine 0 at every milestone. A 2M read would be 2/4 v 0/4,
a 4M read 3/4 v 0/4; no human seed ignited and then collapsed. Training records: first `home` episode s955 0.35M,
s957 0.41M, s958 1.26M, s956 1.05M (one episode, never again); rolling-100 `home` ≥ 0.5 at 1.94M / 2.65M / 3.18M.
N3. **Bimodality is a state, not a fate:** s958 was pick-collapsed at 2M (1/30) and ignited by 4M; the machine seeds
peak at rolling `picked` 0.11–0.16 and never establish it. Rev 5's bimodality check (:1879-1881) is descriptive only.
N4. Rev 3's "exact permutation p and the MDE" (:1589) does not say which contrast; in context it is the per-arm
recipe read. Rev 5 states its prediction and disconfirm branches before the data (:1870-1878) but no interim-look
rule and no rule for the original s976 in the pooled 8 v 8 (S2).
N5. Rev 4 (ramp, :1837-1841) already registers a source-ignition prediction on the OTHER arm; its 8 v 8 read is the
nearest same-learner replication of direction and should be reported beside rev 5.
N6. (ae)'s P-ae-1 (both arms ignite ≥ 3/4 under +10/pixels) and rev 5's P-aa5-1 (machine ≤ 1/4 under +1/state) can both
hold; if they do, the finding is "sparse +1 is a marginal-ignition regime in which source decides who climbs out", and
the recipe, not the source, is the headline.
N7. Two of the eight training nodes are non-64-core classes (pax011 96 c; pax008 H200 host unknown); the project's
class rule (`hw_probe.sh:5-9`) was established for evaluation, never for training rollouts.
N8. Brief §7.11's local video path `~/data/genesis_pickaplace/videos_sparse_2026-09-13/` does not exist on this box.

## Records table (training rollouts, sampled actions, own starts; 500k-online-step bins; p = picked, t = tipped, h = home)

| seed | 0.5M | 1M | 1.5M | 2M | 2.5M | 3M | 3.5M | 4M |
|---|---|---|---|---|---|---|---|---|
| H s955 | p.15 t.44 h0 | p.29 t.46 h0 | p.46 t.45 h.05 | p.84 t.40 h.33 | p.90 t.29 h.56 | p.92 t.15 h.76 | p.95 t.15 h.79 | p.96 t.10 h.84 |
| H s956 | p.12 t.38 h0 | p.07 t.34 h0 | p.17 t.42 h0 | p.14 t.43 h0 | p.07 t.41 h0 | p.01 t.24 h0 | p0 t.22 h0 | p0 t.18 h0 |
| H s957 | p.16 t.43 h0 | p.13 t.48 h0 | p.31 t.54 h0 | p.65 t.50 h.06 | p.81 t.43 h.18 | p.92 t.24 h.58 | p.95 t.15 h.81 | p.96 t.14 h.82 |
| H s958 | p.18 t.43 h0 | p.29 t.46 h0 | p.33 t.50 h0 | p.69 t.44 h.03 | p.78 t.53 h.11 | p.81 t.46 h.26 | p.89 t.31 h.52 | p.91 t.20 h.71 |
| M s975 | p.05 t.37 h0 | p.02 t.32 h0 | p.06 t.33 h0 | p.03 t.34 h0 | p0 t.36 h0 | p0 t.09 h0 | p0 t.02 h0 | p0 t.04 h0 |
| M s976 | p.06 t.40 h0 | p.01 t.33 h0 | p.01 t.42 h0 | p.02 t.28 h0 | p0 t.15 h0 | p0 t.05 h0 | p0 t.07 h0 | (3.93M) p0 |
| M s977 | p.05 t.48 h0 | p.02 t.41 h0 | p.03 t.44 h0 | p.04 t.40 h0 | p0 t.17 h0 | p0 t.07 h0 | p0 t.08 h0 | p0 t.07 h0 |
| M s978 | p.08 t.40 h0 | p.01 t.48 h0 | p.02 t.50 h0 | p.03 t.39 h0 | p0 t.16 h0 | p0 t.13 h0 | p0 t.13 h0 | p0 t.10 h0 |

Source: `HRI_results/curves/ladderN_2026-09-13/records/r2dreamer__*_rnsh__s*.csv`, online step = `step` −
prefill origin (117,624 human / 149,952 machine). Un-ignited seeds drift to 250–525-decision episodes (timeouts), not
to short tipped episodes.

## The one sentence supportable today

"Under one recipe — {r2dreamer}, 17-d state observations, a +1 terminal on `home` with tip termination, 4M online
steps — three of four human-seeded seeds and none of four machine-seeded seeds reached `home` on random starts at the
4M milestone (20/30, 16/30, 13/30 v 0/30 ×3, one machine seed unscored past 2M; unregistered, exact p = 0.14, machine
seeds 3/4 on one training node); the same demonstration sets under a +10 terminal with pixel observations ignite on
both arms (2/2 v 2/2 at 1M locally), so the contrast is recipe-specific and is being tested by a registered 4 v 4
extension, not reported as a source effect."

## What would make the human-v-machine-under-sparse claim

1. Rev 5 lands with node balance (B1/S1) and its P-aa5-1/2 met → pooled 8 v 8, exact permutation, MDE stated.
2. The idle-collapsed human control (S8) and the horizon-cut prefill control (S6 item 2), 2 seeds each, do not close
   the gap.
3. The (ac) sparse10 4 v 4 (state, +10) read beside it: if machine ignites there, the sentence is about the +1 regime.
4. s976's terminal cell scored (S2); the four "cluster-only" checks below done and cited.
5. Framing: an H4-registered-null failure replicated in a second recipe, with the (ae) pixel null/near-null beside it.

## The single most important confound the brief does not name

The **training-node imbalance** (3/4 machine seeds on pax049, 0/4 human), because it is the only nuisance variable that
is systematically unbalanced across arms, is documented as able to change rollout outcomes and hence replay content,
and is reproduced by rev 5's per-arm packing — so the confirmatory as registered cannot remove it. The records make it
unlikely to be decisive (S1), but "unlikely" is not a control. The most important unnamed THREAT to the claim, as
opposed to the contrast, is B3: the +1 recipe sits at the learner's normalisation floor and the same sets ignite on
both arms at +10.

## Cluster-only (cannot be verified from this box)

- Both `_rnsh` manifests: `actions_sha256` = source, `tapes_granting` 12/12, end-reason counts, the 29,221/36,834 v
  29,406/37,488 reconciliation (S7).
- All 8 `ladder_provenance.json` + `[ladder]` stamp lines + `return_clamp=1.0 (env and model agree)`; 4M `metrics.json`
  `cores 64` / `ladder_stamp`; the per-episode `home ⇒ slide_event ∧ nested_v2` implication.
- `sacct` NodeList for all 8 (brief's list) and Submit times of the rev-5 packs v commit d7718ed 17:32 (S4).
- pax049 / pax007 / pax008 / pax011 / pax105 CPU model, SMT and load during the runs; the SMT-cell re-score outcome.
- The s976 stall diagnosis (105 % CPU / 0 % GPU) and its `latest.pt` @ 3.93M (S2, S3).
- The `home` mp4s (N8) — push v drop by eye.
