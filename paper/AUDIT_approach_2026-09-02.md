# AUDIT — approach and algorithms (2026-09-02)

Scope: the design, the three learners, the demo pipeline, the eval harness and the statistics behind the
corrected-world "triple gradient" (ADVISOR_BRIEF §1–§2) and the blocks now running (A27/A28/A31/A32/A33).
Read-only. Scratch computation: `/tmp/.../scratchpad/interaction.py` (interaction test, power). Findings 10–27
come from three parallel code audits (matching; eval/selection; RLPD+WM incl. the `cluster/r2dreamer_port.tar.gz`
port); ✔ = re-checked against source by the lead auditor. Extends, does not repeat, AUDIT_zero_action/sources/results.

## Verdicts

| claim | verdict | findings |
|---|---|---|
| DP indifferent, pruned sets (+0.06, p 0.04) | SOUND WITH CAVEATS | machine arm is self-distillation (1); Holm per PREREG §7 gives p≈0.08 (5) |
| RLPD corrected-world null (−0.02) | SOUND WITH CAVEATS | power 12–21 % for Δ 0.10–0.15 (6); carried by 2 collapsed seeds, alive-conditional +0.06 (7); v2 re-eval partition/crash asymmetries (23,24); divergence statistic not a max (27) |
| WM prefers human (+0.25; ignition 7/8 v 3/8) | FLAWED AS STATED | BEST over ~28 snapshots vs 5 (10); under the clamp the actor gets no grasp credit, ignition = discovery under a saturated critic (11); demo rows unshaped vs online shaped, arm-asymmetric (12); train horizon 400 < median demo pick time (13); raw pair carries 14 over-horizon human tapes (14); A27 is not a replication and runs the mis-set critic (15,16) |
| Cross-learner gradient | FLAWED | no statistic, interaction p 0.20–0.25, ordering non-monotone (2); A33 comparator is pruned, not raw (17) |
| Matching: same ICs, same N | SOUND (frozen) / CAVEATS (v2) | horizon (14), `--allow-short` (19), attempts 8 v 1 (20), fingerprints (21) |
| One eval harness | SOUND WITH CAVEATS | crashed episodes leave the denominator (23); partitions (24); IC files (25); action mode (26) |
| Numbers-of-record pipeline | FLAWED | canonical table mislabels the world of every running block (3); Bayes script default reproduces the withdrawn number (4) |
| Robomimic positioning | FLAWED | misstates the prior (28) |

## Findings (ranked within section)

### Design and statistics

1. **HIGH — The machine arm is a self-distillation of the human arm and the paper does not say so.** dDP = rollouts
   of a DP trained on the *same* pruned human tapes at the *same* ICs, success-filtered (A31; `cluster/a31_chain/shard.sh:13`).
   DP-on-dDP is self-distillation; indifference is the expected outcome. Robomimic's MG came from independently
   trained RL agents. Fix: state the question as *does re-executing human demonstrations through a policy trained on
   them change their value to a learner?*; position against re-execution/augmentation pipelines, not robomimic's MG.

2. **HIGH — The gradient claim has no statistic and is non-monotone.** Δ rnd: DP +0.06, RLPD −0.02, WM +0.25; the
   claimed order "DP < RLPD < WM in use beyond imitation" is not what the data show. Source×learner interaction by
   permutation within learner (1e5 reps): Δ_WM−Δ_RLPD +0.27, p 0.20; Δ_WM−Δ_DP +0.19, p 0.26; spread p 0.25. The rows also
   differ in checkpoint rule, reward, human tape format, n, eval action mode (26), selection IC file (25) and train
   horizon (13). Fix: per-learner results + the interaction p; delete "scales with".

3. **HIGH ✔ — `analysis/results_table.py world_of()` labels the world by seed range and mislabels every running block.**
   Seeds 50-59 → old (v2 w3 waves `v2fullPw3` at s50-57 match neither `_W3` nor `g99w3`), 60-67 (A33) → `?`, 90-93 (A27)
   → old, 94-139 → `?`. The brief's rule is that numbers of record come from this script. Fix: parse `sim_variant`/
   DEMO-SHA from each `.out` (launchers print both); drop the seed heuristic; self-test on one `.out` per wave.

4. **HIGH ✔ — `analysis/bayes_triple_2026-09-01.py` reproduces the withdrawn number by default.** `--s84-best-rnd`
   defaults to `None` (`:18`), DATA is hand-typed (`:22-31`); run as the brief documents it prints the n=7v8 WM row
   ("superseded, do not quote"). It also writes to a dead session's scratch path (`:100`). Eleventh silent default.
   Fix: s84=1 hard-coded or read from RESCORE-RESULT lines; flag required; output beside the figure.

5. **MED — Registered multiplicity rule not applied.** PREREG §7 (`:194-196`): two primary contrasts per learner → Holm.
   DP rnd p 0.042 with hold flat → Holm p ≈ 0.08: not significant under the paper's own plan. §7 also says *paired*
   permutation; `analysis/stats.py:77-93` is unpaired (correct — seed index pairs nothing); say which was used.

6. **MED — Power overstated.** Seed SD (rnd): DP 0.06, RLPD 0.26, WM 0.31. Power at n=8/arm for Δ 0.10/0.15/0.25: RLPD
   0.12/0.21/0.49, WM 0.10/0.17/0.37. A27 at n=12 and the observed 0.8-SD effect ≈ 0.50, not "~80 %" (A27 `:466`).
   RLPD's CrI [−0.25,+0.20] does not exclude the old-world +0.21. Fix: every null carries its detectable-effect bound.

7. **MED — Bimodal outcomes summarised by means; the WM effect is ignition-only.** RLPD LAST rnd dH 19,1,19,19,1,21,20,19
   vs dDP 18,19,17,19,20,0,20,11: alive-conditional 0.65 vs 0.59 (+0.06). WM BEST rnd conditional on ignition (BEST hold
   ≥ 8): dH 0.67 (6) vs dDP 0.69 (3) — igniting machine seeds do at least as well; the +0.25 is entirely the ignition
   rate. **CORRECTION (09-02 pm, analysis/DECOMPOSITION_2026-09-02.md):** the registered criterion BEST hold ≥ 8/15 applied
   to the per-seed list of record gives 6/8 v 3/8 (Fisher 0.315), not the 7/8 v 3/8 (Fisher 0.119) quoted in RESULTS §3.1,
   the brief and bayes_triple; 7/8 v 3/8 is what BEST rnd ≥ 8/30 gives. Thresholds 6–9 on hold → 6v4/6v3/6v3/6v3. A27's
   co-primary is registered on a criterion that does not produce the split it cites. Fix: one decomposition for all
   learners — alive/ignited fraction (Fisher), rnd | alive, and selected/LAST/BEST-of-5 — a 3×3 table; RESULTS must
   state which criterion the 7/8 was counted under.

8. **MED — WM predictions of record were disconfirmed, then reversed post hoc.** PREREG P-R2D (`:184-187`) registered
   "4/4 ignition on dH extends to dDP"; PAPER_PLAN H4 (`:39`) registered indifference. Observed 3/8. A27's reversed
   prediction and its threshold were registered after the n=8 readout (A27 `:463-464`); A16 after s30-32 of its own 8
   seeds (RESULTS `:226-231`); A29 after two seeds. Fix: each amendment lists seeds already seen; confirmatory
   statistics on unseen seeds; write the WM story as "registered indifference failed; reversal under test".

9. **MED — rnd-30 has been the decision set for weeks** (γ restart, clamp verdicts, pruning, recipes). It is an
   in-support sample (`baselines/ic_sampling.py:14-31`: uniform xy in the demo bounding box + 1 cm, goal fixed), not
   "off-demo-support" (A23 `:426`). Fix (CPU, ~1–2 h): `rnd2` from a new RNG seed, scored once on the checkpoint of
   record of every arm; report both; correct A23's wording.

### World-model learner and its blocks

10. **HIGH ✔ — WM BEST is the max over every `latest.pt` write (~28 snapshots); DP/RLPD select over exactly 5.**
    `cluster/sbatch_r2dreamer.sh:455-487` scores every mtime change (15 sampled episodes, seed 0, `2>/dev/null`, ERR
    excluded); `:538` takes the max of the whole `ckpt_scores.tsv`. DP/RLPD: `dp_select_confirm.sh:27`,
    `train_rlpd.py:171-173,441-462`. Max of ~28 noisy estimates vs max of 5 inflates BEST hold/rnd and the ignition
    count; K varies per run (missed polls, warm restarts). The 20/40/60/80/100 % snapshots are kept (`:489-490`), so
    repairable without retraining. Fix: re-select among the 5 fraction checkpoints via `cluster/r2d_rescore.sh`.

11. **HIGH — Under the clamp the actor receives no grasp credit; ignition = discovery under a saturated critic.**
    Port (`cluster/r2dreamer_port.tar.gz`): clamp applied to the λ-return target before ReturnEMA (`dreamer.py:499-506`,
    replay `:551-553`); scale = max(1, p95−p5) of the *clamped* returns (`networks.py:397-405`); adv = (min(ret,100)−v)/scale.
    With v≈100 everywhere the pick's advantage over hovering is (1−γ)·100 ≈ 0.3 — the critic residual. Policy loss =
    logπ·adv + 3e-5·H (`:511-512`; DV3 default 3e-4). So 7/8 v 3/8 measures how often reach-only + sampling noise +
    demo-learned contact dynamics yields sustained grasps — a demo-dynamics-coverage effect, not value learning.
    Contradicts RESULTS §3.2 / CONFOUNDS row 13 "ignition asymmetry unaffected". Fix: caption as such; no WM source
    claim until both arms run under A32's scale; add an entropy 3e-4 leg to A32.

12. **HIGH — WM demo rows are unshaped while online rows are shaped ×100.** `to_dreamer_native.py:65-72` refuses
    non-sparse rewards; port `demo_prefill.py:59-67` scales the terminal only; online shaping lives in the adapter
    (`cluster/patches/r2dreamer_final_rr.patch:42-53`). The reward head sees contradictory targets at similar states,
    each demo tape under-reports return by −φ(s₀)·100 ≈ 60–100. RLPD closed exactly this with `--demo-shaping`
    (`train_rlpd.py:161,205-208`); the WM did not. The magnitude scales with demo rows, which differ by arm in A27
    (dHv2raw w3 14,323 rows p50 146 vs dDPv2 ≈8k, ×4 duplication). Fix: shape demo rewards from recorded `eef_pos` as
    `train_sacfd_full.py:494-501` does, or run sparse-both; report demo share of the buffer per arm.

13. **HIGH ✔ — WM trains at 400 sim steps (100 decisions), is evaluated at 1200, and the median demo picks later
    than 100 decisions.** `submit_learners.sh:30,35` (A27) and every W3 block (`SESSION_LOG:220,308,409`) pass
    `TIME_LIMIT=400` (`sbatch_r2dreamer.sh:188`); RLPD trains at 1200 (`sbatch_rlpd.sh:82`); demo picks sit at p50
    107–146 decisions (V2_BUILD `:323`). PREREG §2 (`:52`) fixes 1200; §8 (`:209`) allows 400 "with amendment";
    METHODS_draft `:365`: "amendment not formally logged [CHECK 21]". Fix: log it; disclose; report A32 returns per
    400-step episode; a TL-1200 WM pilot if any GPU remains.

14. **HIGH ✔ — Success-horizon asymmetry inside the raw v2 pair (RLPD-v2, A27).** 14/66 w3 dHv2raw tapes were
    re-collected with `--max-sim-steps 12000` (V2_BUILD `:298-312`; 1253–4974 sim steps) while every dDPv2 tape is
    capped at 1200 (`record_demos.py:99,220`; `shard.sh:13` passes no cap). Human "success" = pick within ~3000
    decisions, machine = within 300; the teacher's slow successes are labelled failures; 14 human tapes cannot be
    executed inside any learner's horizon, and at γ 0.99 their terminal reward is worth < 0.05 at the start state.
    Not on CONFOUNDS. Fix: row; report the raw-pair contrasts on the per-IC subset excluding those 14 ICs (both
    arms) or re-harvest dDPv2 at 12000 with the same attempts; add `max_sim_steps` to the census.

15. **MED — A27 is not a replication of the frozen WM block.** Pool 58→66, human format pruned→raw (the frozen WM
    block used `matched_w3/dH`, pruned), plus (12) and (14). Three explanations compete if it fails. Fix: pre-state
    the replication pattern now; keep a pruned-dHv2 WM leg (4 seeds) in reserve.

16. **MED — A27 (24 runs × 3M) is queued under the critic A32 declares mis-set**, and A32 runs dH only. Whatever A27
    finds is a claim about a saturated-critic learner (11). Recommendation (user decision, cost ≈ A27's budget):
    gate A27 on A32's first readout and run the source pair under the surviving config; if A27 stays, add dDP arms
    to A32 regardless of dH endpoint health.

17. **MED — A33's human arm is PRUNED** (`matched_w3/dH`, N=58; METHODS `:231`) while CONFOUNDS row 4 says "RLPD/WM
    raw". A33's dense-vs-sparse comparator is A20 (pruned), not the raw v2 RLPD. Fix: name the comparator in the
    readout; correct row 4 (frozen WM block and A33 are pruned; only v2 RLPD and A27 are raw).

### Demo pipeline

18. **LOW — `--verify` (open-loop replayability) applies to machine harvests only** (`record_demos.py:723`); ≤ 2
    tapes affected. Disclose.

19. **MED ✔ — `--allow-short` fill is unconstrained and not recorded.** `make_v2_matched.py:86-88` fills from all
    leftover successes regardless of `ic_uid` (pool = raw∪pruned union, `select.sh:32-34`); `build.sh:13-14` reruns
    with `--allow-short` automatically on the shortfall FATAL. Manifest records `short_ics` (`:91`) but not the fill.
    Did not fire for w3 (66/66). Fix: restrict to base ICs; write `fill_ics`; FATAL on N mismatch unless flagged.

20. **MED ✔ — Attempts/verify asymmetry.** Machine: `--attempts 8 --mode sample --verify` = best-of-8 success-
    conditioned draws; human: attempts forced to 1 (`record_demos.py:804`), never verified, at ICs that were searched
    to make open-loop replay succeed (July load-flip finding → marginal grasps). Grasp-margin asymmetry is not on
    CONFOUNDS. Fix: first-attempt yield per IC beside every dDP result; a per-tape margin stat in the census.

21. **MED ✔ — Fingerprints do not cover what is loaded.** WM tapes: `to_dreamer_native.py:156-157` writes `src_sha`/
    `src_manifest_sha`, no `content_sha256`, so `sbatch_r2dreamer.sh:332` falls through; DP/RLPD gates print the
    manifest's self-reported sha (`sbatch_dp.sh:306-309`, `sbatch_rlpd.sh:175-179`) and never recompute; DP trains on
    `$ARM/lerobot` gated by fps/count only (`sbatch_dp.sh:330-331`). Fix: recompute + compare in every gate.

22. **LOW — "dDPpruned" tapes are never pruned**: dDPv2/dDPv2p are one harvest with two IC multisets
    (`build.sh:11-14`). Rename dDP(raw-base)/dDP(pruned-base).

### Eval harness and selection

23. **HIGH ✔ — Crashed episodes leave the denominator.** `cluster/eval_sweep.sh:144-157`: `rate = picked/present`,
    headline `c/28(exp30)`; `results_table.py:39` takes 28 as denominator; incomplete sel sweeps → −1 and skipped
    (`sbatch_rlpd.sh:269-271`, `dp_select_confirm.sh:27`). Crash rate is node-dependent (`rlpd_select_confirm.sh:52`,
    full node-local /tmp on batch nodes). Fix: no headline unless present == expected, or missing = 0, stated.

24. **MED ✔ — Arms scored on different partitions and load.** Human RLPD-v2 re-scores on `-p batch` CPU nodes
    (`rlpd_select_confirm.sh:41`); machine dDPv2 s50-57 in-job on `-p gpu` (`submit_learners.sh:22`); WM selection
    evals run concurrently with two trainings on 16 cores (`sbatch_r2dreamer_pack.sh:23,42`), DP/RLPD selection is
    fresh-process post-training. Node class is recorded per eval (PREREG §5). Fix: stratify v2 RLPD by node class;
    re-score both arms on one partition if it moves.

25. **MED ✔ — Learners select on different IC files in v2/w3.** WM defaults to the frozen `eval_ics.json`
    (`sbatch_r2dreamer.sh:245`, `r2d_rescore.sh:21`); DP/RLPD v2 use `eval_ics_v2_w3.json` with sel ⊂ hold = all 66
    training ICs (`make_eval_ics_v2.py`). rnd is byte-identical across files (verified); "hold" and "ignition ≥ 8/15"
    are defined only on the frozen file. Fix: state A27 ignition uses frozen hold-15; report v2 hold as hold∖sel.

26. **LOW — Action-selection regime differs per learner**: RLPD deterministic (`wandb_eval.py:236`), DP sampled with
    fixed seed (`:108`), WM sampled seed 0 for selection and confirm (`sbatch_r2dreamer.sh:484,536`; `r2d_rescore.sh`
    passes no `--mode`). Disclosed (PREREG §5) but a stochastic-vs-deterministic asymmetry inside a pick-rate
    comparison. Fix: one deterministic re-score of the WM/DP checkpoints of record.

27. **MED — "Divergence (max critic loss ≥ 1)" is a scraped sample, not a max.** Watchdog is print-only
    (`rlpd_sac.py:312-324`; cannot bias an arm — good). The statistic greps SB3's `critic_loss` (one sample per dump,
    summed over E=10 members, `rlpd_sac.py:275-276`; `harvest_readout.sh:13`; hard-coded `make_figs_2026-08-31.py:42-50`);
    threshold 1 = 0.1/member and is not comparable under dense reward. Fix: per-member loss normalised by target
    variance, logged as a running max; re-derive the Fisher rows from it.

### Prior work

28. **MED — The robomimic paragraph misstates the prior** (`DRAFT_robomimic_positioning_2026-09-02.md:12-17`): MG was
    not "worst across learners" — BC/BC-RNN dropped sharply on MG while BCQ stayed high on Lift/Can, i.e. a
    source×learner interaction was already published. Verify against the robomimic tables; then position this paper
    as re-finding learner dependence under matched ICs with a self-distilled generator, plus the coverage measurement.

## Verified sound (do not re-audit)
RLPD demo buffer symmetric between arms except action provenance (tapes end at the env terminal, r = min(r,1),
shaping γ = learner γ on env and demos, φ(term)=0, no success-leaking field; `train_sacfd_full.py:494-540`,
`train_rlpd.py:223-229,306-311`). Recorder writes identical keys/dtypes for both arms (`record_demos.py:275-368`).
Success criterion identical for DP/RLPD (sustained held-can, `genesis_can_env.py:262-265`, horizon 1200). sel and
rnd disjoint by construction; rnd byte-identical across IC files. `stats.py` Welch/permutation correct (selftest).
Ignition threshold robust 6–10. Watchdog cannot alter training.

## Unverifiable here (cluster paths that settle them)
- K per WM run, restarts: `$LAB/r2dreamer/runs/*_s{80..87,90..101,130..139}/ckpt_scores.tsv`.
- Whether `EXTRA="env.return_clamp=2000"` reached A32's train.py through the pack launcher: `grep -a return_clamp r2d_train_*_s130.out`.
- Which running jobs loaded the 14 over-horizon tapes: `baselines/demos_v2/dHv2raw/manifest_shard*_base100200.json`, `matched_w3/dHv2raw/manifest.json`.
- `eval_genesis.py` `--max-steps` units, default mode, pick predicate: `$LAB/r2dreamer/eval_genesis.py`; `R2D-EVAL-PROTOCOL` in `r2d_pack_*.out`.
- Recipe parity dHv2raw s60-67 vs dDPv2 s50-57: cluster `cluster/RUN_REGISTRY.jsonl` (local copy empty), `[cfg] RLPD` lines.

## What would make the result valid and useful (ordered by cost)
1. **Today, CPU:** fix (3), (4); WM BEST re-selected among the 5 fraction checkpoints, fresh-process (10); the 3-learner
   × {alive rate, rnd | alive, selected/LAST/BEST-5} table (7); Holm + interaction p (2,5); `rnd2` once per arm (9);
   stratify v2 RLPD by node (24); CONFOUNDS rows for (11,12,13,14,17,20,23,25,27).
2. **Before any A27/A28 readout:** log the TIME_LIMIT amendment (13); fix ignition's IC file in writing (25); define the
   over-horizon exclusion for the raw pair (14); decide (16); name A33's comparator (17).
3. **Writing:** machine arm = policy re-execution (1); drop "scales with" (2); WM claim = ignition probability under a
   saturated critic and BEST-of-5 (7,10,11); every null carries its detectable-effect bound (6); fix robomimic (28).
4. **If GPU remains after A32:** the source pair under the surviving clamp config with shaped demo rows (11,12,16) is
   the one WM experiment that would turn "directional" into a claim.
