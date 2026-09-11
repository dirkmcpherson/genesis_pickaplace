# What stands — one page (2026-09-08)

Written because the evidence base got hard to hold in your head. Everything below is either **final**, **final with a
caveat**, or **not usable**. Nothing is deleted anywhere: superseded numbers stay in `results.md` (marked) and in git.

---

## 1. Rely on these

| result | number | reading |
|---|---|---|
| **Place, human v machine (matched 39)** | 0.715 v 0.652, Δ +0.063, p 0.112 | null survives on the corrected bank with pinned entries |
| **Place, sampled** | 0.708 v 0.652, Δ +0.056, p 0.206 | same |
| **Place symmetry control** (machine-generated bank) | 0.695 v 0.623, Δ +0.072, p 0.129 | the bank of origin does **not** carry the null: \|Δ_polE − Δ_polEdDP\| = **0.009** |
| **Slide (contact-after-release), matched 11** | 0.580 v 0.598, Δ −0.018, p 0.690 | null survives (record 0.593 v 0.602) |
| **End-to-end, every stage** | see §8.4 of `../paper/EVAL_FIXES_2026-09-07.md` | every registered prediction met; **no stage outside ±0.10** |
| **The phase pipeline reproduces exactly** | 3488 episodes, 0 differences | same checkpoints re-scored on other nodes are bit-identical |

## 2. Rely on, but only with the caveat attached

- **Most end-to-end "nulls" are underpowered.** Five of seven stages have an **MDE larger than the ±0.10 margin**
  (picked 0.210, contact 0.243, contact_push 0.169, placed_v2 0.154, nested_proxy 0.141). Say *"no source effect
  detectable at this sample size"*, never *"the arms are equivalent"*. Only `nested_honest` (0.088) and `slide_success`
  (0.053) are powered below the margin.
- **The one difference worth watching:** `nested_honest` — machine **0.104** v human **0.046** (p 0.087). The machine arm
  completes the task honestly **2.3×** as often. The published proxy hid most of this (1.4×).
- **End-to-end cells are reproducible only on the same CPU class and only as a whole 30-episode sequence.** All 64 are
  now pinned to one class. Cross-class movement reached **0.100** on `contact` — the width of the margin.
- **The end-to-end machine demonstrations were chosen by outcome; the human ones were not.** The machine set keeps the
  **best of up to three attempts** at each start (`--one-per-ic-best`, best ≠ first on 24 of 72 starts); the human set
  keeps **every** attempt including failures. Selection lifts the machine set's demonstrated reward **+57 %** and
  **doubles** its completed demonstrations. So no end-to-end number — the `nested_honest` gap and the ignition effect
  included — currently separates *demonstration source* from *how we filtered tapes*. It is being corrected, not merely
  disclosed: PHASE_PLAN **(v)** retrains all three learners on a first-attempt-per-start machine set (§4).
- **Learning curves are world-model only** (`curves/`). RLPD persisted nothing training-time; DP is offline and never can.
  The e2e ignition finding (human ~156k steps earlier to pick, p 0.019) is **exploratory, one learner**, and flat on
  pick/place/slide — that contrast is the interesting part.

## 3. Do not use

| do not use | why |
|---|---|
| published place §2.y **0.703 / 0.647** | superseded — raw-grip bank, entries drawn with replacement |
| any `polE` number predating 2026-09-08 | same bank defect (30 of 148 entries restored with the fingers wrong) |
| `nested` as end-to-end completion | it is the **training proxy**; honest is 0.046 / 0.104, not 0.138 / 0.192 |
| `placed` in the full scope | stale base-world band, 0/240 — unearnable, not informative |
| **`slide_success` as a statistic of record** | it implements amendment **(l)**, which **(p) withdrew**: `grip < 0.3` passes **2 of 74 demonstrations**. The column is real and computed, but it scores a predicate the demonstrations themselves fail. Treat it as a diagnostic until (p) clause 5 is calibrated. |
| `slide_success` as a **reward** | same reason, and the code refuses it (`full_env.py:286`) |
| DP `pick_spots60_asrecorded` / `prune_dp_spots60` before commit `7e79bae` | 15 "seeds" were 10 policies double-counted |
| flag-based end-to-end curves | truncated episodes logged all-zero; rebuilt from `episode/score` |
| any end-to-end cell as evidence about **demonstration source** | the machine arm's tapes were outcome-selected and the human arm's were not (§2). The numbers are sound as *what these two sets produce*; they do not isolate source until **(v)** lands. |

## 4. Still running (do not write these up yet)

- **PHASE_PLAN (v) — the de-confounded end-to-end arm.** 12 runs, 4 seeds × {world model, RLPD, Diffusion Policy}, on a
  machine set rebuilt as **first attempt per start** (Σ reward 131 v 206 selected v 118 human; picked 63 v 70 v 64;
  completions 8 v 16 v 3 — de-selection puts the machine arm *below* the human arm on picked and contact and above it
  only on completions). Jobs `v1st_wm_s0-3` / `v1st_rlpd_s0-3` / `v1st_dp_s0-3`. Reads out three ways, all registered in
  advance: whether the source null survives de-selection, whether selection helped the learner at all, and — the
  decisive one — whether the human **ignition advantage** survives or was substantially our filtering.
- **carrycontact** re-score — a *control*, not a result; it exists only to show how much plain `contact` credit came
  from carrying the can without releasing (~69 %).
- **DONE 2026-09-08:** the pinned **end-to-end** re-score, all 64 cells, and the slide cells.

---

**If you are auditing prose rather than reading results:** start at `AUDIT_GUIDE_2026-09-08.md`, which gives the
verification chain, the registration check, the ten traps that have actually fired here, and which phrasings each class of
result can support.

**If you read only one thing:** place and slide nulls survive the corrections; end-to-end passes every registered
prediction but is underpowered at five of seven stages; the single result pointing anywhere is `nested_honest`, favouring
the **machine** arm at p 0.087.

**2026-09-11 addendum.** The two end-to-end demonstration sets are matched by phase under the
unified predicates (slide 13/74 human v 14/72 machine; every stage within 0.06) — provenance,
method and table in `HRI_results/DEMO_SETS_2026-09-11.md`. The machine set's teacher was trained
on the pruned human set; state that as a property of the machine set. All e2e learner cells
produced before 2026-09-11 were trained under ladders now known to be defective (RLPD: the
proxy; r2dreamer: an unpayable top rung); the unified-ladder pilot and its sparse-versus-staged
question are in `paper/LADDER_UNIFY_BRIEF_2026-09-10.md`.
