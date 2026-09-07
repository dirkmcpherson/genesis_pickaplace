# Adversarial review — statistics and registration integrity (2026-09-07)

*Mandate: `~/wm_fix_2026-09-03/agent_brief_adversarial_common.md`, topic (1). Targets of record:
`paper/REVIEW_GUIDE_2026-09-07.md` (RG), `paper/PHASE_RESULTS_2026-09-05.md` (PR), `paper/PHASE_PLAN_2026-09-04.md` (PP),
`paper/MORNING_TABLE_2026-09-04.md` (MT), `paper/DP_PRUNED_GAP_2026-09-07.md`, `paper/ROBOMIMIC_PLAN_2026-09-05.md`.
Everything below was recomputed: permutation tests re-run from the published per-seed lists with an independent
implementation (`~/wm_fix_2026-09-03/adv_review/perm.py`), per-episode `metrics.json` and `sweep.json` read on the
cluster, registration times read from `git log --date=iso-local` and `sacct -X --format=Submit`. Nothing was trained,
submitted, or modified; the only files written are under `~/wm_fix_2026-09-03/adv_review/` and this document.*

---

## S1 — findings that change a number or a claim of record

### S1-1. RG §3 puts the RLPD pick null in the "≈ 0.1 precision or better, counts matched" bucket. Its MDE is 0.345 and its counts are not matched. **CONFIRMED**

RG:73 (§3.1): *"Established at ≈ 0.1 precision or better: no source effect for the world model at pick (≈ 0.08), place
(≈ 0.13), contact (≈ 0.12) and carrycontact (≈ 0.03), **and for RLPD at pick**, with demonstration counts matched."*
RG:74 (§3.2): *"the RLPD null is between the two."* RG §2.1's table gives an MDE/CI for the world-model row only.

Recomputed with RG's own definitions (two-sample t, df = 14, 80 % power at α .05) from the per-seed counts in
`CROSS_LEARNER_CONDITIONS_2026-09-03.md`:68,74-75,81 — human dHv2raw LAST rnd30 `[23,20,18,20,18,2,24,19]` vs machine
dDP `[18,19,17,19,20,0,20,11]`:

| | Δ | p | sd_h | sd_m | **MDE** | CI |
|---|---|---|---|---|---|---|
| RLPD pick, as reported | +0.083 | 0.485 ✓ | 0.227 | 0.230 | **0.345** | ±0.245 |
| RLPD pick, both dead seeds dropped | +0.086 | 0.139 | 0.079 | 0.105 | 0.151 | ±0.108 |
| end-to-end picked (RG calls this "the weakest null") | −0.037 | 0.643 | 0.159 | 0.125 | 0.216 | ±0.154 |

The RLPD pick null is the **weakest** null in the project, not "between" the 0.1 and 0.2 buckets: its MDE is 1.6× the
end-to-end one. It would not have detected the +0.307 robomimic effect that RG §5 calls a falsifier firing. Separately,
its counts are **not** matched: 66 human tapes / 14,389 rows vs 58 machine tapes / 7,343 rows (measured, see S2-6), so
the clause "with demonstration counts matched" is false for this row as well.

*Scenario:* a reviewer reads §3.1 and concludes RLPD's source-indifference is established to ±0.1; it is established to
±0.25 at best. **Smallest fix:** move the RLPD row to §3.2 (or a new "≈ 0.35" line), print its MDE/CI in the §2.1 table
like the WM row, and drop "with demonstration counts matched" or qualify it to the WM/DP rows.

### S1-2. RG §2.1 attributes "the only significant source effect in the project" to the matched 66-v-58 pick comparison. It belongs to the unmatched 106-tape all-data arm. **CONFIRMED**

RG:22, inside §2.1 ("Pick stage … 66 human raw vs 58 machine tapes on the same starts"): *"Also of record: the world
model's in-distribution deficit for human demos (holdv2 p 0.035, all-demo p 0.044) — small, significant, and the only
significant source effect in the project."* RG:79 (§3.3) repeats it as "the WM holdv2/all-demo deficit".

Those two p-values are `dHv2all` (the **106-tape all-data arm**: 66 successes + 40 failure/no-pick tapes) vs dDP.
Source: MT:124, which states them correctly inside the dHv2all paragraph. Reproduced exactly (cluster
`morning_table.py runs` and independently):

| contrast (MODE) | Δ | p |
|---|---|---|
| dHv2all (106) vs dDP — the cells RG quotes | holdv2 −0.053 / alldemo −0.047 | **0.035 / 0.044** |
| **dHv2raw (66, the §2.1 arm) vs dDP** | holdv2 −0.017 / alldemo −0.017 | **0.106 / 0.238** |

The arm that produces the significance differs from the machine arm in *count* (106 v 58) **and** *composition* (40
failure tapes), i.e. it is RG §2.2's own "unmatched by design" condition. It cannot be read as a demonstration-*source*
effect; it is a data-composition effect. The matched human arm's in-distribution cells are null.

*Scenario:* the paper reports "the world model learns human demonstrations slightly worse in distribution" as the one
positive source finding, sourced to a comparison that never showed it. **Smallest fix:** move the sentence to §2.2 and
rewrite as "the all-data human arm (106 tapes, 40 of them failures) trails the machine arm in distribution (holdv2
MODE p 0.035, alldemo MODE p 0.044); the matched 66-tape human arm does not (p 0.106 / 0.238)". Then §2.1's "the only
significant source effect in the project" has no referent (see S1-3).

### S1-3. Every p < 0.05 human-vs-machine result in the Genesis leg is deterministic-action-only and vanishes under sampled actions — and the user asked for sampled actions on 2026-09-07, after all of them were known. **CONFIRMED**

RG:11 fixes the statistic as "deterministic actions ('mode'; sampled actions reported alongside)".
`WM_FIX_LOG_2026-09-03.md`:1436 (entry *2026-09-07 15:45*): *"User answers (15:20): … sampled actions for RLPD and
r2dreamer"*. Every readout in RG §2 predates 15:20. Recomputed both modes for every cell where both exist:

| cell | MODE Δ / p | SAMPLE Δ / p |
|---|---|---|
| **dHv2all v dDP holdv2** | −0.053 / **0.035** | −0.042 / **0.139** |
| **dHv2all v dDP alldemo** | −0.047 / **0.044** | −0.042 / **0.118** |
| **contact holdE (h11 v m11)** | +0.057 / **0.026** | +0.023 / **0.467** |
| WM pick rnd30 | +0.008 / 0.875 | −0.017 / 0.641 |
| place polE matched-39 | +0.056 / 0.227 | +0.014 / 0.743 |
| place polE unmatched-63 | −0.007 / 0.878 | −0.013 / 0.748 |
| place polE-dDP | −0.039 / 0.374 | −0.039 / 0.434 |
| contact polE_contact matched | −0.009 / 0.841 | −0.021 / 0.644 |
| carrycontact polE | +0.011 / 0.348 | +0.006 / 0.660 |
| e2e picked / contact / nested rnd30 | −0.037 / +0.042 / −0.029 | −0.012 / +0.088 / +0.021 |
| repeat-1 rnd30 | −0.008 / 1.000 | +0.017 / 0.829 |
| robomimic RLPD MH v MG | +0.307 / 0.008 | +0.290 / 0.011 |

**No null flips** (largest SAMPLE |Δ| anywhere is e2e contact +0.088, still inside the registered 0.10 margin, p 0.232).
**All three "significant" cells flip.** The robomimic positive is mode-robust. The contact holdE cell is the clearest:
88/88 vs 83/88 deterministic (p 0.026) but 88/88 vs 86/88 sampled (p 0.467) — a two-episode difference on eleven starts.

*Scenario:* the statistic of record is switched to sampled per the user's request and the paper's two positive
in-distribution findings silently disappear, or worse, the mode version is kept because it is the one that is
significant. **Smallest fix:** RG §2.5 and §2.1 must say in-line that these differences exist only under deterministic
actions; and if SAMPLE becomes the statistic of record, RG §2.1/§2.5/§3.3 need rewriting to "no source difference in
this project reaches α 0.05 under the registered statistic".

---

## S2 — findings that weaken a claim

### S2-4. Every MDE/CI in RG §2 is computed over evaluation entries that both arms fail 100 % of the time; the pick MDE is understated by 30 %. **CONFIRMED**

RG:11 defines MDE from "the observed per-seed spread". Entries on which *both* arms score 0 in every seed contribute
nothing but denominator: they shrink the rate spread and therefore the apparent MDE. Counted from `per_episode`
records on the cluster:

| cell | entries dead for BOTH arms | MDE as printed | MDE on the discriminating entries | Δ as printed → restated |
|---|---|---|---|---|
| **WM pick rnd30** | **7 / 30** (ICs 2, 5, 6, 10, 24, 25, 26) | 0.078 | **0.101** | +0.008 → +0.011 |
| place polE (matched) | 12 / 148 | 0.132 | 0.144 | +0.056 → +0.061 |
| carrycontact polE | 21 / 148 (incl. the 5 restore failures) | 0.032 | 0.037 | +0.011 → +0.013 |
| contact polE_contact | 2 / 160 | 0.116 | 0.117 | — |
| e2e picked / nested | 4 / 30, 8 / 30 | 0.216 / 0.152 | 0.249 / **0.207** | — |

RG §3.1's headline "world model at pick (≈ 0.08)" is really ≈ 0.10 on the 23 starts where a difference is possible;
the DP_PRUNED_GAP doc makes exactly this argument for DP and RG adopts it there but not here. Two further honesty
caveats on the same table: the MDE uses a two-sample-*t* approximation while the test of record is a permutation test,
and it is estimated from 14 df, so its own 95 % interval is roughly [0.72, 1.55] × the printed value — quoting
"MDE 0.032" to three decimals is more precision than the estimate has.
**Fix:** footnote the dead-entry share per cell (or print the stratified MDE) and give MDEs to two significant figures.

### S2-5. The place symmetry control was never run on the matched-N arm — i.e. not on the number of record. **CONFIRMED**

PR §2.x evaluates "the same 16 checkpoints" on the machine-policy bank; those 16 are dH s0–7 + **dDP(63)** s0–7.
Directory listing on the cluster: `runs/s2_r2d_place_state_dDP_bnormclamp1ent5_n39_s*` contain only
`fresh_eval_{holdE,polE}_{sample,mode}` — there is **no** `fresh_eval_polEdDP_*` cell for any matched-39 seed.
The control that does exist measures a home-field effect worth ≈ 0.03 in the contrast: on the human bank the machine
arm scores 0.709 and the human arm 0.703; on the machine bank 0.720 and 0.681. The place number of record (+0.056,
human ahead) is measured on the bank that favours the human arm; the same swing applied would leave ≈ +0.02.
RG §2.4's table marks the symmetry row "(63 demos)" but nowhere says the matched pair has no symmetry control.
**Fix:** one sentence in RG §2.4 / PR §2.x, or a CPU post-hoc re-score of the eight n39 checkpoints on
`phase_banks/polE_place_dDP.json` (no training).

### S2-6. Carrycontact — "the tightest null in the project" — has no symmetry control at all, and its entry bank is human-policy-generated. **CONFIRMED**

PR §4 and RG §2.6 report Δ +0.011 with MDE 0.032 on `polE` = the 148 pick-grant entries produced by the eight
**human-arm** phase-1 pick policies (PR §6 item 5 discloses this for the place phase; RG §2.6 and §4 do not mention it
for carrycontact). No `polEdDP` cell exists for any `s2_r2d_carrycontact_state_*` run. The entry-source asymmetry
measured on place (≈ 0.03) is the same size as the entire claimed precision of this cell (0.032), so "a true difference
above ≈ 0.03 would have shown" cannot be asserted without the symmetric bank.

### S2-7. The matched-N rule can only ever cut the machine arm, and it matches tapes while rows differ by up to 1.96×. **CONFIRMED**

PP amendment (e): *"the machine set subsampled UNIFORMLY (seed 0, without replacement; `matched_n.py`) to the human
tape count … The human count is never increased."* `$W/matched_n.py` implements exactly that (sorted glob, `default_rng(0)`,
`rng.choice(..., replace=False)`, kept files listed in `repeat.json` as `subsample_kept`) — the implementation is
faithful and reproducible. Two consequences the docs do not state:

1. **Asymmetric by construction.** The rule has no branch for the case where the *human* arm has more data, which is
   exactly the headline pick comparison (66 v 58 tapes). PP (e)'s own audit table calls it "count ≈ matched, machine
   fewer" and no matched pick arm was ever built. RG §3.1 nevertheless says the pick nulls hold "with demonstration
   counts matched".
2. **Matched on tapes, not rows.** Measured over `demos_state/*` on the cluster:

| comparison | human tapes / rows | machine tapes / rows | row ratio |
|---|---|---|---|
| pick (headline, WM + RLPD) | 66 / **14,389** | 58 / **7,343** | 1.96× human |
| contact "matched" (n11) | 11 / **989** | 11 / **580** | 1.71× human |
| place "matched" (n39) | 39 / 4,304 | 39 / 4,766 | 1.11× machine |
| carrycontact "matched" (n21) | 21 / 2,705 | 21 / 3,087 | 1.14× machine |

   `matched_n.py` records `decisions_min` / `decisions_median` but no row total, and no results table prints rows.
   For r2dreamer the demos are prefill rows in a 5e5 buffer and count against the step budget (MT §3.2 discloses this
   for pick: 57,936 vs 29,904 prefill rows ⇒ the human arm gets ≈ 3 % fewer online steps) — rows, not tapes, are the
   natural unit for an off-policy learner. The place phase itself proved counts matter (+0.062 for 24 extra tapes), so
   a 1.7–2.0× row asymmetry inside a cell labelled "matched" is not negligible.

**Fix:** add a rows column to every matched table; state "the human arm is never subsampled" as a disclosed asymmetry;
drop "with demonstration counts matched" from RG §3.1's RLPD/WM pick claim or add "tape counts 66 v 58, rows 1.96×".

### S2-8. No multiplicity control anywhere; three significant cells is exactly the yield of ~60 tests under a global null. **CONFIRMED**

Counted p-values in the five docs of record: PR 27, MT 29, PP 11, RG 6, RESULTS_WM 9 = 82 (the human-vs-machine subset
is ≈ 60: `phase_table.py`:54-60 prints five cells per machine arm per phase, `full_table.py`:49-54 twelve per wave,
`morning_table.py`:52-56 about twenty). Below 0.05: dHv2all holdv2 0.035, dHv2all alldemo 0.044, contact holdE 0.026
(all three mode-only, S1-3), plus the n = 2 pilot cells 0.044 / 0.022 and DP-selected 0.041. The expected number of
p < 0.05 at 60 tests under a global null is 3. The only Holm adjustment in the corpus is MT:16 (DP selected
0.041 → 0.082). Nulls are not harmed by multiplicity, but the two cells RG asserts as *findings* are.
RG is also internally inconsistent: §2.1 calls holdv2/alldemo "the only significant source effect in the project"
while §2.5 calls contact holdE "the project's other significant difference".
**Fix:** a one-line multiplicity statement in RG §2 ("k human-vs-machine tests were run; no correction is applied;
the p < 0.05 cells are at or below the family-wise expectation") and drop the "only significant" phrasing.

### S2-9. The robomimic falsifier is quoted without the CI that RG demands of every null, and the registered threshold Δ ≥ 0.15 is not excluded from below. **CONFIRMED**

RG:11 defines MDE/CI for the whole document; RG §5 (the falsifier that "has FIRED") gives Δ +0.307, p 0.008 and no
MDE/CI. Recomputed from `ROBOMIMIC_LOG_2026-09-06.md`:158 per-seed lists (MH200 `[27,44,30,1,20,9,24,27]` vs MG200s
`[10,2,2,13,6,12,9,5]`, of 50): **MDE 0.295, CI ±0.210 ⇒ 95 % interval [0.098, 0.517]**. The registered falsifier is
"Δ ≥ 0.15 with p < 0.05" (ROBOMIMIC_PLAN §5); Δ ≠ 0 is established, Δ ≥ 0.15 is not. The study's MDE ≈ its observed
effect, which is the classic winner's-curse regime (MH s3 = 1/50 is a dead seed inside the arm). **Fix:** print the CI
next to the Δ and say the falsifier's *threshold* is met by the point estimate, not by the interval.

### S2-10. DP_PRUNED_GAP's recommended stratification raises the DP effect size by 50 %, and RG adopts the recommendation without the new number. **CONFIRMED**

Verified from the DP sweep records (`baselines/outputs/dp_w2final/{dH,dDP}_DP_s2?/sweep/{final,100000}/sweep.json`,
per-episode `k`/`picked`): for **every** seed of both arms the live-20 count equals the all-30 count, i.e. the ten dead
ICs contribute exactly zero. Hence the permutation p is invariant (0.123, reproduced), but

| DP pruned-human vs machine | Δ | MDE | CI |
|---|---|---|---|
| over all 30 rnd starts (RG §2.1 table) | +0.053 | 0.091 | ±0.065 |
| over the 20 in-support starts (the adopted reporting change) | **+0.080** | 0.136 | ±0.097 |

RG §2.1 states "Reporting change to adopt: stratify rnd30 by support" but keeps +0.053 in the table. DP is the learner
the paper wants to call source-*sensitive*, so its effect size is load-bearing. **Fix:** print both, or the stratified
one, with the note that p is unchanged.

Two smaller arithmetic points in the same document, both checked against its own §3.1 table: the per-IC columns sum
correctly (dH 156, dDP 140, dHv2 121, RLPD 144, WM 148) and "9 of the 10 dead ICs have x ≥ 0.524" and "no live IC has
x ≥ 0.52" are both exactly right. But "RLPD reaches 5 … of those ten" (RG §2.1) uses an unstated ≥ 2/8 threshold — at
≥ 1/8 RLPD reaches **7** of the ten (k = 6 and 26 at 1/8). DP_PRUNED_GAP §3.1 hedges this correctly ("and 6, 26 at
≤ 1/8"); RG drops the hedge.

---

## S3 — fragility / protocol drift

### S3-11. Three of the five amendments were committed within a minute of the runs they register; two document self-timestamps disagree with the machine record. **CONFIRMED**

| amendment | doc's own registration time | git commit (`--date=iso-local`) | first job Submit (sacct) | verdict |
|---|---|---|---|---|
| (c′) repeat-1 | "09-05 09:55" | 73cc1d3 **09:50:44** | 3290344–51 **09:54:51** | commit 4 min before submit ✓ |
| (d) end-to-end | "09-05 10:05" | 31cb560 **09:59:16** | 3290391–98 **10:03:01** | commit 3.7 min before submit ✓ — but the doc's own stated time (10:05) is *after* the submit |
| (e) matched-N | "registered 11:25 … Submitted 11:25" | 5469e2a **11:18:37** | 3291435–58 **11:18:20** | **commit 17 s AFTER submit**; the stated 11:25 is 7 min after the real submit |
| (f) polE_contact | "09-05 13:35" | 3a04d4f **13:26:53**, cap fixed 3926ae2 **13:48:25** | dumps 13:27:32, capped merge resubmitted **13:48:43** (COMMANDS.log:301) | ✓ by 18 s on the cap |
| (d) wave 2 | — | de05900 **09-06 22:36:40** | 3337021–28 **22:35:07** | commit 93 s after submit |

No prediction was demonstrably written after a result — the runs in every case take hours, and the (e)/(f)/wave-2 gaps
are seconds. But the docs' self-reported registration times are not the machine record and in two cases postdate the
submission they claim to precede. **Fix:** put the commit hash and its `iso-local` time in each amendment header, and
stop hand-writing registration clock times.
*Note for the brief:* PHASE_PLAN has **no amendments (g) or (h)**. Both are named only as delegated future work in
WM_FIX_LOG:1432 (contact_push → (g)) and :1445 (DP/RLPD place → (h)); neither is registered yet.

### S3-12. The 4 v 4 → 8 v 8 trigger is ambiguous and was satisfied on the exact boundary, after the wave-1 direction was known. **CONFIRMED**

PP (d) registers **P1** as "picked ≥ 0.5 on rnd30 MODE **in ≥ 3/4 seeds per arm**" and the wave-2 trigger as
"**both arms ≥ 0.5 picked on rnd30 MODE**". Wave 1 machine per-seed = `[14,10,22,14]`/30 ⇒ **1/4** seeds ≥ 0.5, arm
mean **60/120 = exactly 0.500**. Under the P1 reading the trigger fails; under the arm-mean reading it passes by zero
episodes. PR §5 says "met by the letter (0.533 / 0.500)" — an honest phrasing of a post-readout disambiguation that
went the permissive way, at the moment wave 1 showed human ahead at every stage.
Mitigating and verified: the trigger is *effect-independent* (learnability, not sign), it was committed 09-05 09:59
before wave 1 ran, and pooling wave 1 + wave 2 is not budget-mixing — all 16 end-to-end runs reached 1.998–2.000 × 10⁶
steps (`metrics.jsonl` last `step`), even though wave-1 jobs 3290391–98 exited `FAILED` (the in-job eval stage, not
training). **Fix:** one line in PR §5.1 saying which reading was used and that the alternative reading would not have
triggered wave 2.

### S3-13. Amendment (f)'s registered covariate is never reported. **CONFIRMED**

PP (f): "The bank mixes entries produced by human-arm and machine-arm place policies (8 + 8 checkpoints), so it is
symmetric by construction; **the per-source split is reported as a covariate**." The bank *is* symmetric — verified:
`phase_banks/polE_contact.json`, 160 entries, 80 with a `source_run` from a `dH` place checkpoint and 80 from a `dDP`
one. But no per-source result split appears in PR §3 or RG §2.5. It is a cheap CPU re-tabulation of existing
`per_episode` records and it is the direct symmetry check that place got and contact did not.

### S3-14. `p = 1.000` on the repeat-1 pilot is arithmetically forced and carries no evidence. **CONFIRMED**

PP (c′) verdict / PR §6 / RG §2.3: rnd30 MODE 4 v 4, human `[22,18,18,19]` vs machine `[18,20,19,21]`. The pooled total
(155) is odd and the groups are equal, so every one of the 70 splits has |Δ| ≥ 0.25 = |observed| — p = 1.000 is the only
attainable value. The informative statistic there is the CI (±0.093), which RG §2.3 does print. **Fix:** a footnote, so
"p = 1.000" is not read as unusually strong agreement.

---

## S4 — hygiene

- **S4-15.** RG §2.4's row label "unmatched machine (63 demos) | — | 0.709 | −0.006 | 0.878" puts the *unmatched*
  secondary in the same table as the matched primary with a blank human column; a reader can take 0.709 as the machine
  number of record (it was, until 09-06). Adding "(secondary, superseded)" costs one word.
- **S4-16.** PR has **two sections numbered "## 3"** (Negative controls, and Contact after release) and **two numbered
  "## 6"** (Asymmetries, and Clock control). RG §6 item 1 points at "PHASE_RESULTS §2.y, §3, §4, §5.1"; "§3" is
  ambiguous.
- **S4-17.** `phase_table.py`:6-9 rounds a rate back into a count (`int(round(float(p) * episodes))`). It is exact for
  every cell checked, but it is a lossy round-trip through a float rate where the evaluator already has the integer;
  a future cell with a large denominator could round the wrong way. `full_table.py` does it correctly (counts from
  `per_episode`).

---

## Claims that survived

Each of these I tried to break and could not; the check is named.

1. **The permutation test is implemented correctly.** `phase_table.py`:44-50, `full_table.py`:41-47,
   `morning_table.py`:45-50 are the same routine: complete enumeration of all C(n_a+n_b, n_a) label assignments,
   absolute-difference-of-means statistic, ties admitted (`>= |obs| − 1e-12`), correct for unequal group sizes
   (n = len(a); complements are of the right size). Minimum attainable p is 2/12870 at 8 v 8, 2/70 = 0.029 at 4 v 4
   (matches PP (c′)/(d)), 1/45 = 0.022 at 2 v 8 (matches MT:77 — correct, because at unequal n the arm-swap is not in
   the enumeration).
2. **Every published p and Δ reproduces.** Independent re-run over 20 cells (`~/wm_fix_2026-09-03/adv_review/repro.txt`):
   WM pick 0.875, WM pick SAMPLE 0.641, RLPD 0.485, DP 0.123, repeat-1 1.000, place matched 0.227, place unmatched
   0.878, polE-dDP 0.374, contact matched 0.841, contact m25 0.711, contact holdE 0.026, carrycontact 0.348,
   e2e picked/contact/nested 0.643/0.639/0.621, robomimic 0.008, dHv2all holdv2/alldemo 0.035/0.044 — all to 3 dp.
   Every per-seed list sums to its published numerator and every Δ equals count-difference ÷ denominator.
3. **The MDE/CI formula reproduces exactly** and is the standard one:
   MDE = (t_{.975,df} + t_{.80,df})·s_p·√(2/n), CI = t_{.975,df}·s_p·√(2/n) — 0.078/±0.055 (pick), 0.132/±0.094 (place),
   0.116/±0.082 (contact), 0.032/±0.023 (carrycontact), 0.216/0.239/0.152 (e2e), 0.128/±0.093 (repeat-1). No cell is
   mis-transcribed. (Its *interpretation* is what S2-4 challenges, not its arithmetic.)
4. **`restore_failed` really is counted as failure, and it is symmetric.** `runs/s2_r2d_carrycontact_state_dH_…_s0/
   fresh_eval_polE_mode/metrics.json`: `contact` = 0.78378… = 116/148 with `Counter({'contact': 116, 'timeout': 27,
   'restore_failed': 5})` — the rate denominates over all 148. The **same five entries** (uids 900002, 900007, 900055,
   900058, 900126) fail in **all 34** carrycontact polE cells: both arms, both action modes, all eight seeds. The
   "143 restorable" column is arithmetically consistent (955/1144 = 0.835, 942/1144 = 0.823) and does not move the
   conclusion (Δ +0.011 → +0.013). RG §4 item 5's "≈ 3 % of entries" = 5/148 = 3.4 % ✓.
5. **`polE_contact` is genuinely symmetric.** `phase_banks/polE_contact.json`: 160 entries, exactly 80 sourced from
   `dH` place checkpoints and 80 from `dDP` — the registered 10-per-checkpoint × 16 construction, as (f) states.
6. **The end-to-end 8 v 8 pool does not mix training budgets** despite wave 1's `FAILED` job state: all 16 runs'
   `metrics.jsonl` end at 1,997,632–1,999,992 steps against a 2e6 budget.
7. **`matched_n.py` is a faithful uniform subsample**: sorted file list, `np.random.default_rng(0)`,
   `choice(..., replace=False)`, kept basenames written to `repeat.json` (`subsample_kept`), reward and tape counts
   re-derived from the copied files — reproducible and auditable. (What it does *not* record is rows: S2-7.)
8. **The (d) wave-2 trigger predates wave 1** (commit 09-05 09:59:16 vs submit 10:03:01) and is written to be
   independent of the observed difference ("not by the sign of any difference") — the ambiguity in S3-12 is about
   *which* learnability threshold, not about optional stopping on the effect.
9. **ROBOMIMIC amendment A2 was registered before any control run**: commit 10e5662 09-07 13:26:29; no `MGall`,
   `MG718s` or `ext300k` job exists in `sacct` (the only robomimic jobs today are the cancelled 10:26 mis-fire and the
   48-run 10:53 primary matrix).
10. **The contact holdE p = 0.026 is the right number for its data**: the actual machine-11 holdE MODE per-seed counts
    are `[11,11,10,10,10,10,11,10]` (83/88) against a human arm at `[11]×8`, which gives exactly 330/12870 = 0.0256.
    (That it is mode-only is S1-3; the arithmetic is right.)

---

## Out of scope (S1 seen while checking, for the eval/env reviewer)

`runs/s2_r2d_carrycontact_state_dH_…_s0/fresh_eval_polE_mode/metrics.json` shows `stages.picked` = 0.966 at the entry
state and `mean_steps` = 36.5 of a 600-step horizon, with the first episode granting `contact` in 8 decisions. If the
carrycontact entry states are already within a few decisions of the goal, the 0.80 rate and the 0.032 MDE describe a
near-trivial phase, which is a different reason to distrust "the tightest null in the project" than the one in S2-6.
Worth a look by whoever owns the entry-restore and stage-predicate mandate.

---

## Summary (≤ 15 lines)

1. **S1** RG §3 files the RLPD pick null under "≈ 0.1 precision, counts matched". Its MDE is **0.345** (CI ±0.245) — the
   weakest null in the project, weaker than the end-to-end one — and its counts are 66 v 58 tapes / 1.96× rows.
2. **S1** RG §2.1's "only significant source effect in the project" (holdv2 p 0.035, all-demo p 0.044) belongs to the
   **106-tape all-data arm**, not the matched 66-v-58 pick comparison, whose own cells are p 0.106 / 0.238.
3. **S1** All three p < 0.05 Genesis cells are **deterministic-action-only**: under sampled actions they read 0.139,
   0.118 and 0.467. No null flips. The user asked for sampled actions on 09-07 15:20, after every readout.
4. **S2** MDEs are computed over entries both arms always fail (pick 7/30, place 12/148, carrycontact 21/148, e2e
   nested 8/30); the pick MDE is 0.101, not 0.078.
5. **S2** The place symmetry control (polE-dDP) exists only for the superseded 63-demo machine arm; carrycontact — the
   tightest null — has no symmetry control at all and runs on human-policy entries.
6. **S2** Matched-N matches tapes, never rows (contact 989 v 580 rows), and can only ever cut the machine arm, so the
   headline pick comparison is unmatchable by the rule.
7. **S2** ~60 human-vs-machine tests, no multiplicity control, three p < 0.05 — exactly the null expectation.
8. **S2** The robomimic falsifier's 95 % interval is [0.098, 0.517]: Δ ≠ 0 is established, the registered Δ ≥ 0.15 is not.
9. **S2** DP_PRUNED_GAP's adopted stratification moves the DP effect from +0.053 to +0.080 (p unchanged at 0.123); RG
   adopts the recommendation but keeps the old number.
10. **S3** Amendment (e) was committed 17 s *after* its jobs were submitted and wave 2 93 s after; (d)'s self-stated
    registration time postdates its own submission. Nothing looks back-written, but the docs' clocks are not the record.
11. **S3** The 4 v 4 → 8 v 8 trigger has two readings; the permissive one was used and was met by exactly 0 episodes.
12. **Survived:** the permutation code is correct and every published p, Δ and MDE reproduces to 3 dp; `restore_failed`
    is counted as failure and the same five entries fail for both arms in all 34 cells; `polE_contact` is 80/80
    balanced; `matched_n.py` is a faithful seeded subsample; the 8 v 8 end-to-end pool is budget-clean; ROBOMIMIC A2
    was registered before its controls; amendments (g)/(h) do not yet exist.
