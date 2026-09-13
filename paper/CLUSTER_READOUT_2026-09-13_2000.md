# Cluster readout 2026-09-13 ~20:00 (VPN restored by the user at 19:40) — the state-based `nested_sparse10` arm is dead for both learners; only the pixel world model solves the task

Every number below is from a file on the cluster read at 19:44–19:55 EDT (`$W/ln14_milestone_table.py`, the runs'
`console.log`, `cluster/ln_cell_census.py` over `$LAB/gp_ac/baselines/rl/checkpoints/e2e_ac/`). Absent cells are
listed as absent.

## 1. Amendment (ac): `nested_sparse10` (+10 on `home`), STATE observation, 2 v 2 per learner

### {r2dreamer} (the port's own contrastive representation loss), 4M budget, packed, at 3.2–3.5M online

| seed | arm | counter | episodes | training `home` (ever) | last-60 picked / home / tipped | cells |
|---|---|---|---|---|---|---|
| s957 | human | 3 543 648 | 3469 | **0** | 0.00 / 0.00 / 0.25 | 0.5M/1M cells FAILED `unknown ladder 'nested_sparse10'` (scored against the pinned `gp_ladderN` tree before the eval sbatch learned to read the run's `genesis_pickaplace_root`; not yet retried — the sweep's 6 slots are full of rev-4 cells) |
| s958 | human | 3 542 644 | 3389 | **0** | 0.00 / 0.00 / 0.02 | same |
| s977 | machine | 3 226 380 | 3174 | **0** | 0.00 / 0.00 / 0.02 | 0.5M/1M/2M: picked 0–1/30, `home` 0 (all nine cells) |
| s978 | machine | 3 247 500 | 3059 | **0** | 0.00 / 0.00 / 0.07 | 0.5M/1M/2M: picked 0–1/30, `home` 0 (all nine cells) |

**P-ac-1 ("`nested_sparse10` ≥ `nested_sparse` on `home`; picked > 0 at 1M where `nested_sparse` was 0") — its
disconfirm branch fires:** with +10 the state-based r2dreamer learner does not pick at 1M, 2M or 3.4M on either arm,
so "the sparse deficit is a normalisation-scale effect" is wrong for this learner. (The +1 `nested_sparse` human
seeds ignited 3/4 at 4M — `AUDIT_R2D_NESTED_SPARSE_HvM` — which +10 has not matched by 3.5M on 0/4.)

### {RLPD}, 500k decisions, final checkpoint, in-job preview cells

| seed | arm | hold15 MODE picked / placed / `home` | hold15 sampled `home` | rnd30 MODE picked / `home` | spots60 MODE picked / `home` |
|---|---|---|---|---|---|
| s957 | human | 15/15 / 14/15 / **2/15** | 1/15 | 16/30 / 0 | 55/60 / 0 (sampled 1/60) |
| s958 | human | 10/15 / 0/15 / 0 | 0 | 7/30 / 0 | 50/60 / 0 |
| s977 | machine | 0/15 / 0 / 0 | 0 | 0/30 (placed_v2 3 = reset grants) / 0 | 2/60 / 0 |
| s978 | machine | 0/15 / 0 / 0 | 0 | 0/30 / 0 | 0/60 / 0 |

**P-ac-2 (RLPD within noise of `nested_sparse`)**: the RLPD `nested_sparse` (+1) 250k finals are not in this table
(not re-read tonight); on `nested_sparse10` the human seeds pick (s957 places 14/15 and reaches `home` 2/15 on demo
starts; s958 picks without placing) and the machine seeds **never pick** (0/15, 0/30, 0–2/60 at 500k) — the same
direction as the +1 r2dreamer contrast and the ramp-ladder RLPD machine seed s970 (which picked 14/15 under the ramp).

## 2. Amendment (aa) REVISION 5: 8 new `nested_sparse` (+1) seeds, PACKED, all four packs on **pax011**

s1955–1958 (human) and s1975–1978 (machine) at counter 383k–417k (~0.27M online), 261–296 episodes, 0 `home`
anywhere yet, last-60 picked 0.00–0.22, tipped 0.28–0.67 — the early phase. Both arms on one node, so the pax049
confound of the original 4 v 4 (`AUDIT_TRAIL_R2D_NESTED_SPARSE_HvM` B1) does not recur here.

## 3. Other running: (aa) rev 4 ramp packs 17 h in (s954/959/960/961 human, s974/979/980/981 machine, four nodes);
milestone sweep saturated (6 `lnms` running: rev-4 ramp 2M cells, ctl 0.5M/1M/final cells). Disk 195 GB free.

## 4. What this settles, with the local (ae) runs beside it

| learner / observation / reward | human seeds igniting | machine seeds igniting | best `home` cell |
|---|---|---|---|
| {r2dreamer} state, `nested_sparse` +1, 4M | 3/4 | 0/4 (s976 at 2M) | 20/30 |
| {r2dreamer} state, `nested_sparse10` +10, ~3.4M | 0/2 (training record) | 0/2 | 0 |
| {RLPD} state, `nested_sparse10` +10, 500k | 1/2 (hold15 2/15) | 0/2 (never picks) | 2/15 |
| {dv3 local} **pixels**+proprio, shift4, `nested_sparse10`, 1M | 3/3 | 2/2 | 20/30 (machine s1), 18/30 (human s0) |

- The +10 terminal did not rescue the state-based world model; the pixel observation (with reconstruction loss and
  augmentation) is the change that made the world model solve the task, on both arms. The attribution control
  that would seal it — pixels + `nested_ramp`, or state + `nested_sparse10` + reconstruction loss from scratch — is
  still unrun, but "+10 alone" is now excluded for the r2dreamer state learner by these four seeds.
- Every state-based sparse result, on either learner, lives in a regime where the learner usually fails; the
  human-vs-machine direction there (human ignites sometimes, machine not) is consistent across r2dreamer +1, RLPD +10,
  and is absent under pixels. That is the pattern to explain, not a source effect to report: candidate mechanism =
  the idle content of the human tapes (0.365 v 0.007) stabilising a state-based actor early; test = the idle-collapsed
  human control (amendment (n)) for the world model, never run.

## 5. Actions
- The (ac) human r2dreamer cells will be retried by the sweep when slots free (the eval sbatch now reads the run's
  provenance root); they will read 0 — the training records already say so.
- Nothing submitted from this box. Cluster ops remain the other workstation's (handoff §3f).
