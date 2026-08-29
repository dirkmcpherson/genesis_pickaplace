# AUDIT INDEX — entry point for adversarial review

Maintained for reviewers arriving cold (human or agent). 46 documents accumulated over five days;
several are superseded and a few contradict each other. This file says what is live, what is dead,
what is known-broken, and where to attack. Last updated 2026-08-28 (post gate-2/3 audit: `AUDIT_results_2026-08-28.md`).

**Ground rule adopted 08-25 (N9):** results are quoted from `analysis/results_table.py` output only,
never from recollection. If a number in prose is not in `paper/RESULTS_TABLE_*.md` or a job's
`.out`/`HEADLINE.txt`/`RESCORE-RESULT` line, treat it as unverified.

---

## 1. Read in this order

| # | file | what it is |
|---|---|---|
| 0 | **`AUDIT_results_2026-08-28.md`** | **the PREREG §11 gate-2/3 audit: verdict table, corrected N15 numbers, the two-critic-loss mechanism, the 27 %/10 % exposure correction, the run-identity rule, ranked change list. Read before trusting any N-note** |
| 0b | `DV3_DIAGNOSIS_2026-08-28.md` | why dv3 fails where r2dreamer works (critic runaway, update budget, horizon, late demo terminal); the flag-gated fix and the 6 diagnostic jobs |
| 1 | `PREREG_final_round_robin_2026-08-23.md` | the frozen spec + amendments A1–A8. §5 eval protocol and §11 audit gates are the rules the project is meant to obey |
| 1b | `NOTES_IN_PLAIN_ENGLISH.md` | the same findings without jargon — start here if you are not already deep in the project |
| 2 | `PAPER_NOTES.md` | **N1–N15**: every claim, at the strength its evidence supports. N2, N3, N12, N12a are SUPERSEDED; N15 carries an 08-28 audit addendum that its body does not yet reflect. This is the primary target for review |
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
| N7 | naive density pruning should hurt dH_DP | **REPORTED, predictions (1)(2) FAILED, (3) 3/6 seeds**; falsifier met at zero effect (+0.03 rnd, p 0.49). Plain-English note inverted the registered reading — audit §3 |
| N8 | does the WM need repeat 4? | **deferred** — arms unbuildable (no stride-1 pixel demos), launcher ignores its knobs |
| N9 | "source parity" is a BC result, not an RLPD result | live — **correction of the assistant's own overstatement** |
| N10 | the RLPD source spread vanished in the corrected world; unexplained | **open question**, three untested candidates |
| N11 | the BC claim rests on a ceilinged readout; move to `rnd` | live — correction; `sel` max is 14/15 (uid 234, 0/430) |
| N12 | the WM is unmoved by the tapes that broke RLPD | **OVERTURNED → N15** |
| N12a | power on that null | **SUPERSEDED** (its seed sets no longer exist; its critical value is still wrong — 4.012 not 4.303) |
| N13 | human-vs-model WM comparison launched | **no protocol readout yet** (sel only: dH 0.69 n=3, dDP 0.83 n=4, ceiling 0.93 — not reportable); re-scores submitted 08-28; only the dH-vs-dDP pair is clean (dR2D arm density-confounded, E4) |
| N14 | split-half demo-draw robustness | **2/12 landed** (10 died in the disk incident; resubmitted 08-28) — nothing to conclude |
| N15 | fail tapes hurt the WM ~25–30 % vs RLPD ~73 % | **live, DIRECTION ONLY.** Audit 08-28: CIs corrected ([-0.26,+0.73] hold, [-0.23,+0.56] rnd), min attainable p 0.057 at 4v3, the 'sel hid a 5× effect' story is false (seed composition; fails s83 lost), exposure is ~27 %/10 % of the ring not 1–3 %, and the arms differ in demo share (2.7×) and tape length (12×) as well as fail content. Size-vs-RLPD claim needs the share-matched control |

## 4. Known-broken / open, as of this writing

1. **E1 — CLOSED 08-27/28.** Re-score landed → N15. Audit finding: the readout change moved the gap
   ~10 %; the rest was seed composition. Re-scores for 10 more completed seeds submitted 08-28.
2. **E2 — REOPENED 08-28.** The rewritten mechanism sentence in N15 is still false: r2dreamer has a
   REPLAY critic loss (`dreamer.py:539-563`, weight 0.3) that backs up a λ-return along the recorded
   trajectory, demo/fail tapes included. Citations `models.py:412,:384-393` are to dreamerv3-torch,
   not r2dreamer. Correct wording in `AUDIT_results_2026-08-28.md` §1.
2b. **E6 (new) — exposure arithmetic wrong everywhere.** `demo_reinject_every 150000` keeps demos at
   ~27 % (fails arm) / ~10 % (control) of the 450k ring all run; the "evicted by 450k" log line is a
   stale prefill-only computation. Every "~1–3 %" / "1/17th" figure is retired.
2e. **E9 (new, 08-28) — RLPD's critic diverges in 69/74 runs.** Every RLPD number in the tables is the
   best checkpoint before a critic blow-up (Q watchdog trips by 10-30k steps in nearly every run; critic
   loss reaches 10^2-10^4; training success peaks at 0.85-0.97 in runs whose final checkpoint scores 0).
   The RLPD row of the matrix is therefore provisional until N18's fix factorial (gamma 0.99 / UTD 5)
   reads out; if it holds, the RLPD matrix is re-run and every RLPD number so far is superseded.
2d. **E8 (new, 08-28) — the WM arm trains with a return-target clamp.** r2dreamer uses `return_clamp: 100`
   plus a replay critic loss; RLPD and dv3 have neither. The 08-23 "no TD-target clamp in the matrix"
   decision covered RLPD only. Disclose in every cross-learner mechanism sentence. See
   `DV3_DIAGNOSIS_2026-08-28.md` (why dv3 never ignites: unclamped critic runs to 4-5x the max return).
2c. **E7 (new) — run-identity.** The 08-27 relaunch reused seed ids of completed runs in place
   (fails s83 destroyed; dH s102, dDP s101 pending). Rule: a seed id names one run; reruns replace.
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

- **N15's three-way confound.** The fails arm differs from the control in fail content, total demo
  share (2.7×) and tape length (12×). Until the share-matched arm runs, "fail content hurts the WM"
  and "demo volume moves the WM" are not separated.
- **N15's power.** 4v3 cannot reach p<0.05; the pooled n≈8–12 (re-scores in flight) is the first
  readout worth an interval.
- **n.** Most cells are 3–5 seeds with per-seed spreads of 0.1–0.9. Ask for the interval, not the mean.
- **World-vs-version entanglement (N10).** Demos are recordings made in a world, so the two cannot
  be separated by re-recording. N14 (split-half) addresses the draw, not the world.
- **Whether the SUPERSEDED notes are cited anywhere as live** in prose (`UPDATE`, `POSTMAINT`).

## 6. Reproducing anything

Every training/eval job registers in `cluster/RUN_REGISTRY.jsonl` (script, arm, seed, git sha,
semantic knobs, demo sha, node) at job START. Demo sets carry `manifest.json` with a content sha,
the recorder version, the sim variant and per-tape provenance. `paper/final_rr_sets_2026-08-23/`
holds the committed set manifests and censuses. Artifacts (sets incl. `r2d/dR2DDPfails` since 08-28, checkpoints, all `.out`
files, re-score logs, registry) are mirrored off-cluster at `~/workspace/final_rr_artifacts_2026-08-24/`.

- **E8 update 08-29:** the replay critic loss (`repval 0.3`) matches the official DreamerV3 v2 recipe (beta_repval 0.3, danijar/dreamerv3); repval is a published component. Only the return clamp (100) is a deviation. Standard r2dreamer arm = `env.return_clamp=0` with repval kept (runs NOCLAMP s122/123). See paper/WM_CANDIDATES_2026-08-29.md.
