# AUDIT — PREREG §11 gates 2 (results) and 3 (claims), 2026-08-28

Auditor: Fable, in the same session that produced the work (NOT fresh-context). Mitigation: every
numeric, mechanism and provenance check below was run by a fresh-context agent that had no access
to this session's history, working only from raw artifact lines
(`scratchpad/harvest_2026-08-28.md`, regenerated from the cluster this morning), the cluster source
tree, and the notes under audit. Verdicts I could re-derive from the raw lines myself I marked
"(re-verified)". A strictly independent audit still requires a fresh session started from
`AUDIT_INDEX.md`; this document is the input to that.

Scope: N7, N10, N11, N13, N14, N15 (+N12/N12a), the E2 mechanism claim, the E3 exposure
arithmetic, provenance of the N15 arms, the disk/preemption diagnosis, the batched-Genesis
decision, and prose consistency across `AUDIT_INDEX`, `UPDATE_2026-08-25`, `NOTES_IN_PLAIN_ENGLISH`.

---

## 0. Headline verdicts

| item | verdict |
|---|---|
| N15 point estimates (0.767/0.533 hold, 0.642/0.478 rnd; RLPD 0.55→0.15) | **REPRODUCE** (re-verified: 46/60, 24/45, 77/120, 43/90) |
| N15 CIs | **DISCREPANCY** — wrong critical values in both rows (hold used pooled df; rnd used df=4). Correct Welch: hold [-0.259, +0.726], rnd [-0.228, +0.556]. Perm p 0.286 reproduces |
| N15 narrative "sel hid a 5× larger effect" | **FALSE** — on N15's own seed sets `sel` already shows +0.211 vs hold +0.233 (re-verified). The change from N12 is seed composition: fails s82 (sel 0.13) landed; fails s83 (sel 0.80) was overwritten in place by the 08-27 relaunch and is unrecoverable |
| N15 exposure caveat ("demos ~1–3 % of the ring, 1/17th of RLPD") | **FALSE** — `demo_reinject_every 150000` keeps demos at ~27 % of the ring in the fails arm (~19 % fail rows) and ~10 % in the control, for the whole run. The "all demo frames evicted by 450k" log line is a stale prefill-only computation |
| E2 mechanism sentence (as rewritten in N15) | **STILL FALSE** — r2dreamer has a second critic loss (`dreamer.py:539-563`, weight 0.3) that backs up a λ-return along the recorded replay trajectory, demo tapes included. "No recorded transition enters a critic target" is wrong. Citations `models.py:412, :384-393` are to the wrong codebase |
| N15 arm design | **CONFOUNDED** at a magnitude not previously recognised: adding the 8 fail tapes raises the arm's demo share 2.7× (9.8→26.7 %) and tape length 12× (301 vs ≤25 rows), not just fail content |
| N15 provenance ("same tapes, same world") | **CONFIRMED** — 8 fail tapes byte-identical across RLPD and WM arms; dR2D ⊂ dR2DDPfails; fingerprints consistent per arm across all seeds; sim_variant base everywhere (RLPD's is by construction, pre-plumbing) |
| N13 | **NO PROTOCOL READOUT YET** — only `sel` landed (dH 0.689 n=3, dDP 0.833 n=4, ceiling 0.933); re-scores submitted today |
| N7 | **PREDICTIONS (1)(2) FAIL; (3) not evaluable (3/6 seeds); falsifier condition nominally met** with an effect indistinguishable from zero (+0.031 rnd, p 0.49). `NOTES_IN_PLAIN_ENGLISH` inverted the pre-registered reading |
| N14 | **NOTHING TO CONCLUDE** — 2/12 landed (dH_A only); the other 10 died in the disk incident and had not been resubmitted until today |
| N10 / UPDATE §3b at n=10 | "vanished spread" **HOLDS** (dH ≤ dDP in all four corrected cells); the n=4 dense gap (0.22 vs 0.08) was noise (0.16 vs 0.17 at n=10); "per-seed maxima 0.33" is false (0.87, 0.67, 0.47 exist) |
| N11 / BC source claim | **STALE** — at n=10 corrected-world DP rnd dH 0.547 vs dDP 0.487, Welch CI [+0.006, +0.114], perm p 0.041 (single unadjusted comparison, top-up not pre-registered as this test) |
| Disk-incident diagnosis | **CONFIRMED** — 20 DP resubmits of 08-27 02:13: 8 completed, 12 FAILED on the full volume; the "CUDA transient" attribution for that block was wrong |
| Batched-Genesis (probe 2970418) | **NOT VIABLE AS-IS** — per-env reset perturbs the other envs (1.2e-3 max diff, phase repeat not bit-exact); batched render fine. Parked; no result depends on it |

---

## 1. N15 — what is actually supportable

Raw (RESCORE-RESULT, fresh process per episode, protocol readouts):

| arm | seed | hold | rnd | sel (R2D-RESULT) |
|---|---|---|---|---|
| dR2D | 80 | 15/15 | 19/30 | 0.933 |
| dR2D | 81 | 7/15 | 13/30 | 0.600 |
| dR2D | 82 | 10/15 | 19/30 | 0.600 |
| dR2D | 83 | 14/15 | 26/30 | 0.933 |
| +fails | 80 | 11/15 | 21/30 | 0.800 |
| +fails | 81 | 9/15 | 12/30 | 0.733 |
| +fails | 82 | 4/15 | 10/30 | 0.133 |

- hold: 0.767 vs 0.533, diff +0.233, Welch df 4.52, **95 % CI [-0.259, +0.726]**, exact perm p 0.286.
- rnd: 0.642 vs 0.478, diff +0.164, Welch df 4.17, **95 % CI [-0.228, +0.556]**, perm p 0.286.
- relative loss: hold 30.4 %, **rnd 25.5 %** (the note quotes only "~30 %"). RLPD same tapes: 0.550→0.150 = 72.7 % (hold), 71 % (rnd).
- **Minimum attainable two-sided permutation p at 4v3 is 2/35 = 0.057** — the design cannot reach 0.05 under any outcome. Episode-pooled Fisher tests (p≈0.02) are invalid (seed is the unit) and must not be quoted.
- `sel` on the same seeds: 0.767 vs 0.556 = +0.211. The N12 figures 0.822/0.778 were on seed sets {80,82,83}/{80,81,83}. **The gap did not appear because the readout changed; it appeared because the seed sets changed.** The "most consequential methodological error" framing in N15 must be re-attributed: the readout error was real (E1) but numerically minor here; the material problem is seed bookkeeping (a completed seed silently dropped and then destroyed).

Mechanism (from source, `/cluster/.../r2dreamer/dreamer.py`):
- Imagination roots = every replayed posterior state incl. demo rows (`dreamer.py:479-484`); actor loss `:508-512`; imagined critic `:514-522` (on-policy imagined λ-return, supervised reward head `:470`, no Q(s,a), no max) — CONFIRMED.
- **Replay critic loss `:539-563` (weight 0.3)**: λ-return along the recorded trajectory, bootstrapped with the imagined return — a direct value backup through fail-tape states toward "keep failing". This channel is absent from every mechanism sentence written so far.
- Exposure: sim-step budget 3e6; `demo_reinject_every 150000` × `demo_duplicate 4` → 19 re-injections/run; ring share at steady state **dR2D 9.8 %, fails arm 26.7 % (fail rows 18.8 %)**; P(update touches a demo row) ≈ 84 % / 99 %. RLPD: 50 % demo per batch, fail rows 35.8 %. Exposure ratio ≈ 2×, not 17×.
- Fail tapes are 301 rows each (cap-truncated), 3× the training `time_limit` of 100 agent steps and 12× the median success tape; they are 70 % of the fails arm's demo rows.
- BEST_selected.pt is a byte copy of the named ckpt in all four runs checked; RESCORE and selection use the same file — CONFIRMED. Config diff between arms: only demo_dir/logdir — CONFIRMED.

**Supportable claim today:** "Adding 8 DP fail tapes to the best demo set lowered the world model's protocol readouts by ~25–30 % relative (n=4 vs 3; CIs span zero; min attainable p 0.057) versus ~73 % for RLPD on the same tapes. The arms differ in total demo share and tape length as well as fail content, and both learners have a critic loss that regresses through the recorded fail states, so the size difference cannot yet be attributed to a mechanism."

**Not supportable:** "the WM is unmoved" (dead), "hurts the WM half as much" as a quantitative statement, "1/17th the exposure", "no recorded transition enters the WM critic", "sel hid a 5× effect".

## 2. Run-identity rule (new, required before any pooling)

The 08-27 relaunch reused seed numbers of completed runs, in place: dR2DDPfails s83 (done, sel 0.80) is overwritten; dH s102 and dDP s101 (done on `sel`, BEST missing) will be. Rule: **a seed id names one run.** A rerun replaces the earlier run for all purposes; the earlier readouts are dropped from every pooled statistic and listed as lost. `results_table.py` now dedups on (arm, seed) and `sbatch_*` relaunches must take fresh seed ids in future. Registry warnings (REGISTRY-WARN semantic match) exist on all five reruns but the registry rows carry no `warn` field — add one.

## 3. N7 against its pre-registration

| arm | n | hold | rnd |
|---|---|---|---|
| dH corrected (20–29) | 10 | 0.887 | 0.547 |
| dHallpruned_1e3 | 3 | 0.911 | 0.578 |
| dHallpruned_1e2 | 3 | 0.933 | 0.578 |
| dDP corrected | 10 | 0.873 | 0.487 |
| dDPallpruned_1e3 | 2 | 0.933 | 0.517 |
| dDPallpruned_1e2 | 1 | 0.933 | 0.433 |

(1) pruned hold < dH by ≥0.05: FAILS (+0.024 / +0.047, and hold is ceilinged). (2) dose-response: FAILS (rnd identical at both eps). (3) dDP null control: NOT EVALUABLE (3/6 seeds; 3 resubmitted today). Falsifier "dHallpruned ≥ dH": nominally met, effect +0.031 rnd, CI [-0.020, +0.083]. The registered reading of that outcome was "density revived as the explanation"; `NOTES_IN_PLAIN_ENGLISH` wrote the opposite. Honest statement: no effect resolvable at n=3 vs 10; density is neither confirmed nor excluded as mediator. The "22 %" figure (eps 1e-2 pruned fraction) is not in any artifact — source or delete.

## 4. N10 / N11 at n=10 (corrected world, seeds 20–29)

RLPD hold: sparse dH 0.187 vs dDP 0.227 (CI [-0.225, +0.145]); dense 0.160 vs 0.173. Old world sparse hold dH 0.417 vs dDP 0.250, CI [-0.236, +0.570] — driven by one 0.73 seed; not a resolved contrast either. So N10 is a comparison of two unresolved numbers; keep it as an open question but delete "cleanest source contrast in the project" and "per-seed maxima 0.33".

DP rnd: dH 0.547 vs dDP 0.487 at n=10, CI [+0.006, +0.114], p 0.041 (unadjusted, one comparison). Report it with that caveat; "not resolvable at n=5" is stale. Hold is flat (+0.013).

## 5. Prose consistency (file:line)

Stale or wrong, still presented as live:
- `AUDIT_INDEX.md:51,58-60,62-63,80-81` — N12 "not quotable until re-score lands", E1 "running", E2 "N12 not rewritten", §5 hypothetical — all superseded; no N15 row; N7 listed as running.
- `PAPER_NOTES.md:14` N1 banner points to N12/N12a → N15. `:323-340` N12a has no SUPERSEDED banner and still says "no harm to the world model"; its corrected critical value is still wrong (t(0.975, 2.16)=4.012, not 4.303). `:318-321` N13 prediction rests on a dead premise. `:114-115` N5 fail share "74 %/28 %" — data 71.6/25.8 (UPDATE's 72/26 is right).
- `NOTES_IN_PLAIN_ENGLISH.md:108-109` "we can explain why" — overclaims; N7 paragraph inverts the falsifier; N13 lacks the dR2D density-confound caveat.
- `UPDATE_2026-08-25.md:14,27-28,79-80` — "running now", "learner-specific" (presence/absence form is dead), dense n=4 gap. Needs a frozen-as-of banner.
- Exposure figures "~3 %", "~1–3 %", "0.90 %/3.04 %" (PAPER_NOTES:294,391; AUDIT_INDEX:82; CRITIQUE E3 table) — all wrong (see §1).
- Mirror: `r2d/dR2DDPfails` was absent off-cluster (fixed today). Provenance chain to state once: RLPD manifest sha `b222df06…` = `repeat.json.src_sha` → native sha `3a6df556…` → registry fp `c5ae239b…`.

## 6. Cluster actions taken during this audit (all logged in SESSION_LOG)

- 20 re-score jobs (hold+rnd) for the 10 completed r2d runs that had only `sel`: dR2D s86 s87; fails s84 s85 s86; dH s101 s103; dDP s100 s102 s103.
- 13 N7/N14 DP jobs resubmitted (disk casualties). My guard used the wrong HEADLINE path, 24 were submitted, 11 duplicates cancelled before start by submission order (2979703-10, 2979712, 2979715-16).
- Mirror of `matched_v2/r2d/dR2DDPfails` pulled.
- `analysis/results_table.py`: dedup by (arm, seed); wave-aware world; parses RESCORE (headline) and R2D-RESULT (flagged sel-only).

## 7. Ranked change list (gate 3 output)

1. **Rewrite N15** per §1: correct CIs, add rnd relative loss, min-p 0.057, seed-composition attribution, s83 loss, corrected mechanism paragraph (two critic losses, `dreamer.py` citations), corrected exposure (27 %/10 % ring; 2× not 17×), and the demo-share/tape-length confound. Downgrade the plain-English one-paragraph version accordingly.
2. **Run-identity rule** (§2) into PREREG as amendment A9; fresh seed ids for reruns; `warn` field in the registry.
3. **Design fix for the N15 question**: the clean contrast needs a share-matched control (same demo row count and tape-length distribution, success content) — the registered share-matched arm (N5) and the DUPLICATE-based exposure arm. Until one runs, N15 is a two-arm observation with three simultaneous differences.
4. **N7**: record predictions (1)(2) failed, (3) pending, falsifier met at zero effect; fix the plain-English inversion; source or delete "22 %".
5. **N11/UPDATE §3a**: n=10 numbers with the p=0.041 caveat. **N10/UPDATE §3b**: n=10 numbers; delete the dense gap and "maxima 0.33"; CI on the old-world contrast.
6. **AUDIT_INDEX**: N15 row; N12 OVERTURNED, N12a SUPERSEDED (banner + fix 4.303→4.012 or delete); close E1/E2; N7 → reported; §5 rewritten; date.
7. **N13**: add "no protocol readout yet; sel means not reportable; s99 unplanned; dR2D arm density-confounded (E4)".
8. **N14**: "2/12; nothing yet"; label world.
9. Fix `PAPER_NOTES:114-115` fail share; retire every "1–3 %" exposure figure; correct `models.py` citations.
10. Stamp `UPDATE_2026-08-25.md` frozen-as-of; move "running now" content out.

## 8. What would change the paper's primary result

If the pooled N15 (n≈8–12/arm after re-scores) keeps a ~25–30 % WM drop with a CI excluding zero, the direction claim survives; the **size-vs-RLPD** claim does not survive without the share-matched control (item 3). If the share-matched control shows the same drop from success-only tapes at matched share, the "fail content hurts the WM" reading dies and the result becomes "demo volume/length, not failure, moves the WM".
