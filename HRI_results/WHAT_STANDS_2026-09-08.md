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

## 4. Still running (do not write these up yet)

- **carrycontact** re-score (~20/64) — a *control*, not a result; it exists only to show how much plain `contact` credit
  came from carrying the can without releasing (~69 %).
- last **end-to-end** cells (~42/64) and the final 2 slide cells.

---

**If you read only one thing:** place and slide nulls survive the corrections; end-to-end passes every registered
prediction but is underpowered at five of seven stages; the single result pointing anywhere is `nested_honest`, favouring
the **machine** arm at p 0.087.
