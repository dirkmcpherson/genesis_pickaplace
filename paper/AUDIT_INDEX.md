# AUDIT INDEX — entry point for adversarial review

Maintained for reviewers arriving cold (human or agent). 46 documents accumulated over five days;
several are superseded and a few contradict each other. This file says what is live, what is dead,
what is known-broken, and where to attack. Last updated 2026-08-26.

**Ground rule adopted 08-25 (N9):** results are quoted from `analysis/results_table.py` output only,
never from recollection. If a number in prose is not in `paper/RESULTS_TABLE_*.md` or a job's
`.out`/`HEADLINE.txt`/`RESCORE-RESULT` line, treat it as unverified.

---

## 1. Read in this order

| # | file | what it is |
|---|---|---|
| 1 | `PREREG_final_round_robin_2026-08-23.md` | the frozen spec + amendments A1–A8. §5 eval protocol and §11 audit gates are the rules the project is meant to obey |
| 1b | `NOTES_IN_PLAIN_ENGLISH.md` | the same findings without jargon — start here if you are not already deep in the project |
| 2 | `PAPER_NOTES.md` | **N1–N14**: every claim, at the strength its evidence supports. Three carry SUPERSEDED banners. This is the primary target for review |
| 3 | `RESULTS_TABLE_2026-08-25.md` | the canonical numbers, generated from artifacts (`analysis/results_table.py`) |
| 4 | `UPDATE_2026-08-25.md` | narrative status: what changed and why, with caveats attached |
| 5 | `SESSION_LOG_2026-08-23_cluster.md` | **every cluster command**, timestamped, with the repo git sha at run time and its exit code. 105+ rows |
| 6 | the four critiques (below) | prior adversarial passes and what was done about them |

## 2. Independent reviews already run (and whether they were acted on)

| file | scope | outcome |
|---|---|---|
| `AUDIT_sources_2026-08-23.md` | our learners vs published RLPD / DP / DreamerV3 / shaping | found the r2d shaping-γ mismatch (fixed 08-24), the missing terminal guard, per-learner budget disclosures |
| `AUDIT_design_2026-08-22.md`, `AUDIT_impl_2026-08-22.md` | experimental design + implementation | source of the terminal-guard fix, the eval-protocol unification, the fails-arm design |
| `CRITIQUE_design_final_rr_2026-08-23.md` | the final round-robin design | accepted; produced the one-recorder / one-clock / one-harness rebuild |
| `CRITIQUE_launch_plan_2026-08-25.md` | the launch plan | accepted; reordered waves, funded seed top-ups, deferred N8, caught the placeholder-dR2D provenance bug |
| `AUDIT_prelaunch_2026-08-23.md` | PREREG gate 1 | passed with amendments A4–A8 |
| **`CRITIQUE_decisions_2026-08-26.md`** | **the assistant's own judgment over 24 h** | **six errors; see §4. Partially acted on — open items listed there** |

## 3. Claim ledger — status of every note

| note | claim | status |
|---|---|---|
| N1 | fail tapes broke dDP_RLPD via unanchored bootstrapping | **mechanism LIVE**, framing SUPERSEDED (it was an encoding bug, not a model-demo property) |
| N2 | dense shaping collapses the r2d ignition lottery | **SUPERSEDED** — measured under a shaping γ that was patched out |
| N3 | dR2D_DP is the best BC cell; model demos beat human | **SUPERSEDED** — the gap is a translation artifact (N6/N9/N11) |
| N4 | dv3 shows transients, no confirmed ignition | live |
| N5 | fail-tape mechanism; pinned on the WM cells | live; its registered share-matched arm **has not run** |
| N6 | corrected-world block results | live, **conditionally stated** (see N9); one max-vs-mean qualifier added |
| N7 | naive density pruning should hurt dH_DP | **pre-registered, running** (12 jobs) |
| N8 | does the WM need repeat 4? | **deferred** — arms unbuildable (no stride-1 pixel demos), launcher ignores its knobs |
| N9 | "source parity" is a BC result, not an RLPD result | live — **correction of the assistant's own overstatement** |
| N10 | the RLPD source spread vanished in the corrected world; unexplained | **open question**, three untested candidates |
| N11 | the BC claim rests on a ceilinged readout; move to `rnd` | live — correction; `sel` max is 14/15 (uid 234, 0/430) |
| N12 | the WM is unmoved by the tapes that broke RLPD | **NOT QUOTABLE until the re-score lands** — its two rows are on different IC sets (E1) |
| N12a | power on that null | live, **corrected twice**; interval does not exclude an RLPD-sized effect at n=3 |
| N13 | human-vs-model WM comparison launched | running; **only the dH-vs-dDP pair is clean** (the dR2D arm is 7× density-confounded, E4) |
| N14 | split-half demo-draw robustness | **pre-registered, running** (12 jobs) |

## 4. Known-broken / open, as of this writing

1. **E1 — N12's headline is not like-for-like.** RLPD reported `hold`; r2dreamer reported `sel`, and
   selected on `sel` too. Re-score running (`cluster/r2d_rescore.sh`, 14 jobs). Until it lands N12
   must not be quoted.
2. **E2 — the stated WM mechanism was false.** Demo states ARE imagination roots and DO receive
   actor and critic gradient (`models.py:412`, `:384-393`). The corrected distinction is in
   `CRITIQUE_decisions_2026-08-26.md` E2; PAPER_NOTES N12 has not yet been rewritten to match.
3. **E3 — the exposure-matched arm was flawed and was cancelled.** It put the arms at 10.1% vs
   34.2% demo share, and its "matches RLPD's 36%" compared a demo share to a fail share. The clean
   version (`DUPLICATE` at fixed `BUFFER_MAX`) and the registered share-matched fails arm are
   **both unlaunched**.
4. **E5 — the adopted world change is broader than described.** `grasp_timeconst` also stiffens
   can↔goal-can and can↔shelf contacts 1.6×, which are the metric-bearing contacts for `contact`
   and `nested`. The gripper report's downstream numbers are therefore partly direct effects.
5. **Assistant errors this week, for calibration:** N6 overstated for two days (user caught it);
   N12a used the wrong critical value twice; a re-score wrapper reported success over 540 crashed
   processes (no exit-code check, missing `GENESIS_PICKAPLACE_ROOT`); `scancel --name=... --state=PENDING`
   was over-broad and killed two valid waves. All are logged in `SESSION_LOG_2026-08-23_cluster.md`.
6. **Unmeasured input:** the real shelf height above the table, which decides `shelf6` vs `shelf10`.
   Inert for pick scope; blocks place/nested claims.

## 5. Where I would attack, if I were reviewing

- **N12 after the re-score.** If `hold`/`rnd` moves the WM numbers materially, the primary result
  changes and every downstream sentence with it.
- **The exposure confound.** A world model at ~1–3% demo exposure being unharmed is weaker than it
  sounds. Until the share-matched arm runs, the null and the dilution explanation are not separated.
- **n.** Most cells are 3–5 seeds with per-seed spreads of 0.1–0.9. Ask for the interval, not the mean.
- **World-vs-version entanglement (N10).** Demos are recordings made in a world, so the two cannot
  be separated by re-recording. N14 (split-half) addresses the draw, not the world.
- **Whether the SUPERSEDED notes are cited anywhere as live** in prose (`UPDATE`, `POSTMAINT`).

## 6. Reproducing anything

Every training/eval job registers in `cluster/RUN_REGISTRY.jsonl` (script, arm, seed, git sha,
semantic knobs, demo sha, node) at job START. Demo sets carry `manifest.json` with a content sha,
the recorder version, the sim variant and per-tape provenance. `paper/final_rr_sets_2026-08-23/`
holds the committed set manifests and censuses. Artifacts (9.3 GB: sets, checkpoints, all `.out`
files, registry) are mirrored off-cluster at `~/workspace/final_rr_artifacts_2026-08-24/`.
