# Live-work handoff (written 2026-09-07 18:30) — what is running, who owns it, what is pending

*Insurance against context loss. Everything of scientific record is already in `REVIEW_GUIDE_2026-09-07.md` (§8 = corrections after the adversarial reviews), `PHASE_PLAN_2026-09-04.md` (registrations (a)–(k)), `PHASE_RESULTS_2026-09-05.md`, `ROBOMIMIC_{PLAN,LOG}`, and `WM_FIX_LOG_2026-09-03.md`. This file holds only the volatile part: the agent roster and the open decisions.*

## Subagents of session `genesis-pickaplace-04` (lost if a NEW session is started; survive compaction)
| id | mandate | brief | deliverable |
|---|---|---|---|
| a3fe26076ec2458bc | robomimic leg: G2b no-demo gate (jobs 3350586–93) + A4 action-statistics control (approved, building pairs) | `~/wm_fix_2026-09-03/agent_brief_robomimic.md` | `ROBOMIMIC_LOG_2026-09-06.md`, plan A2/A4 |
| add435b72620f67b7 | DP + RLPD place phase (32 runs 3351067–98) + RLPD sampled-action and DP spots60 pick re-evals | `agent_brief_place_dp_rlpd.md` | PHASE_PLAN (h), `place_readout.sh` |
| adf426e066fa8c3bd | evaluator fixes (pinned entries, physical-grip banks, honest nested, placed_v2) + CPU re-score | `agent_brief_eval_fixes.md` | PHASE_PLAN (j), `EVAL_FIXES_2026-09-07.md` |
| ab8c55d8c1a1d8331 | `contact_push` predicate (tool point) + re-score (3350666–74) | `agent_brief_contact_push.md` | `CONTACT_PUSH_2026-09-07.md` |
| a0fc122adee6cb378 | dv3 port: G1 passed; G2/G3 pick evals due ~18:00–19:00 (runs 3337054–57) | `agent_brief_dv3.md` | `DV3_DEBUG_2026-09-05.md` |

## Peer sessions (independent; message via ListAgents names)
- `WM_FIX_PLAN_2026_09_03 verification ⑂` — **machine-first arm** (dRL): SAC-without-demos teachers 3349688–91, r2dreamer no-prefill teachers 3349910–13, gate rnd30 ≥ 0.45, then harvest → learners. Registration `MACHINE_FIRST_PLAN_2026-09-07.md` on branch `worktree-machine-first-arm`.
- `genesis-pickaplace-ec` — **slide definition + the 74-trial end-to-end renders** (handed off 2026-09-07 18:20; render running locally, outputs `can_pos_recovery/videos_census/<uid>_full_gc_kp4_riser3_shelf6.mp4`).

## Monitors armed in this session
`bir2exruy` robomimic matrix (queue + per-arm eval cells, 30 min).

## Cluster work in flight (all self-contained; no VPN needed once submitted)
robomimic matrix (r2d finishing, dp running) · RLPD controls MGall/MG718s/MG200s@300k 3349120–43 · G2b 3350586–93 · A4 pairs building · place DP+RLPD 3351067–98 (queued) · amendment (i) symmetry evals (48 CPU) · amendment (k) spots60 WM evals (32 CPU) · contact_push re-score · dv3 pick evals.

## Open decisions for the user
1. Nothing blocking. (Test sets settled: `spots60` = in-training-distribution, `rnd30`/`rnd300` = out-of-distribution reported stratified, all-74 = full demo-start set. A4 approved. Slide definition owned by the peer session.)
2. Standing: whether the paper's framing survives the robomimic result (decide after A4 + the world-model arm read out).
3. Standing: whether to raise end-to-end to 16 v 16 (current MDE ≈ 0.21).

## Code state
Tag `results-of-record-2026-09-07` pins every Genesis number of record. Rule since 16:30: commit each registration BEFORE submitting its jobs.
