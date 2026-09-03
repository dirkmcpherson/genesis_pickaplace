# Demo-set census — 2026-08-22 16:24
Constants: pick_z 0.1505, delta cap 0.025 rad/step, leash 0.125 rad, tip rule tilt>60.0 deg & grip cmd<0.3. NN samples 6000. Sets: dDP=/home/travel/workspace/genesis_pickaplace-4dof-cartesian/baselines/m1all_harvest, dDPsucc=/tmp/claude-1000/-home-travel-workspace-genesis-pickaplace/65bc5977-e458-4033-a00d-271aecab941d/scratchpad/m1all_harvest_succ, dDPtiptrunc=/tmp/claude-1000/-home-travel-workspace-genesis-pickaplace/65bc5977-e458-4033-a00d-271aecab941d/scratchpad/m1all_harvest_tiptrunc, dHrerec_all=/home/travel/workspace/genesis_pickaplace-4dof-cartesian/baselines/episodes_delta_rerecord_pick_all, dHrerec_succ=/home/travel/workspace/genesis_pickaplace-4dof-cartesian/baselines/episodes_delta_rerecord_pick.
Skipped (layout mismatch / unreadable): cart_abs6: /home/travel/workspace/genesis_pickaplace-4dof-cartesian/baselines/episodes_cart_abs6_pruned: 232.npz: states (1059, 18) actions (1059, 7) (expected (T,17),(T,7))

### Composition
| metric | dDP | dDPsucc | dDPtiptrunc | dHrerec_all | dHrerec_succ |
|---|---|---|---|---|---|
| tapes | 93 | 63 | 90 | 72 | 54 |
| by stage | {'no-pick': 30, 'picked': 63} | {'picked': 63} | {'no-pick': 27, 'picked': 63} | {'no-pick': 18, 'picked': 54} | {'picked': 54} |
| by label | {'fail': 30, 'success': 63} | {'success': 63} | {'fail': 27, 'success': 63} | {'fail': 18, 'success': 54} | {'success': 54} |
| success (lifted) | 63 | 63 | 63 | 58 | 54 |
| fail (never lifted) | 30 | 0 | 27 | 14 | 0 |
| transitions | 70028 | 34028 | 63467 | 91670 | 71136 |
| fail share of buffer | 0.514 | 0.000 | 0.464 | 0.172 | 0.000 |
| tape len p50 | 680 | 522 | 632 | 961 | 786 |
| tape len mean | 753 | 540 | 705 | 1273 | 1317 |
| tape len max | 1200 | 1136 | 1200 | 9932 | 9932 |
| success len p50 | 522 | 522 | 522 | 828 | 786 |
| fail len p50 | 1200 | — | 1200 | 1200 | — |
| tapes at max len (timed out) | 30 | 1 | 23 | 1 | 1 |
| unique ICs (can xy @t0) | 58 | 44 | 56 | 68 | 52 |
| IC grid cells occupied /16 | 10 | 8 | 10 | 11 | 9 |
| goal xy | [(0.672, -0.221)] | [(0.672, -0.221)] | [(0.672, -0.221)] | [(0.672, -0.221)] | [(0.672, -0.221)] |
| tapes with images | 93 | 63 | 90 | 0 | 0 |

### Reward (pick scope)
| metric | dDP | dDPsucc | dDPtiptrunc | dHrerec_all | dHrerec_succ |
|---|---|---|---|---|---|
| rewarded transitions | 63 | 63 | 63 | 58 | 54 |
| reward density | 0.00090 | 0.00185 | 0.00099 | 0.00063 | 0.00076 |
| expected rewards per 128-demo batch | 0.115 | 0.237 | 0.127 | 0.081 | 0.097 |
| first-lift idx p50 | 512 | 512 | 512 | 826 | 784 |
| first-lift idx mean | 530 | 530 | 530 | 1295 | 1315 |
| frames kept after lift p50 | 10 | 10 | 10 | 2 | 2 |

### Fidelity under delta_joint encoding (cap/leash)
| metric | dDP | dDPsucc | dDPtiptrunc | dHrerec_all | dHrerec_succ |
|---|---|---|---|---|---|
| frac frames over cap | 0.078 | 0.077 | 0.076 | 0.034 | 0.028 |
|   successes | 0.077 | 0.077 | 0.077 | 0.031 | 0.028 |
|   fails | 0.079 | — | 0.074 | 0.051 | — |
| frac frames one-step label err >1e-3 | 0.073 | 0.072 | 0.070 | 0.028 | 0.022 |
| label err p99 (rad, median tape) | 0.0335 | 0.0336 | 0.0335 | 0.0081 | 0.0059 |
| label err max (rad) | 0.299 | 0.238 | 0.299 | 0.031 | 0.031 |
| |delta cmd| p99 (median tape) | 0.0585 | 0.0586 | 0.0585 | 0.0331 | 0.0309 |
| frac zero-delta frames | 0.000 | 0.000 | 0.000 | 0.173 | 0.164 |
| frac frames |cmd-qmeas|>leash | 0.052 | 0.046 | 0.056 | 0.000 | 0.000 |
| lead p99 (median tape) | 0.135 | 0.135 | 0.135 | 0.125 | 0.125 |

### Env-consistency (pick-scope termination) & can state
| metric | dDP | dDPsucc | dDPtiptrunc | dHrerec_all | dHrerec_succ |
|---|---|---|---|---|---|
| tapes where can tilts >tip_deg | 8 | 0 | 5 | 4 | 2 |
|   of which fails | 8 | 0 | 5 | 2 | 0 |
| POST-TERMINATION frames / buffer | 0.060 | 0.000 | 0.000 | 0.016 | 0.012 |
|   / fail frames | 0.117 | 0.000 | 0.000 | 0.036 | 0.000 |
| longest post-term chain (frames) | 1200 | 0 | 1 | 612 | 612 |
| tapes tipped at t0 (lying-can IC) | 3 | 0 | 0 | 2 | 2 |
| tilt @t0 p50 (deg) | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| tilt @t0 max | 90.0 | 0.0 | 1.1 | 90.0 | 90.0 |
| can max displacement, fails p50 (m) | 0.103 | — | 0.110 | 0.070 | — |
| can max displacement, max (m) | 0.247 | 0.172 | 0.227 | 0.343 | 0.227 |
| can z @t0 p50 | 0.112 | 0.112 | 0.112 | 0.112 | 0.112 |
| can z min | 0.029 | 0.097 | 0.031 | 0.076 | 0.082 |

### Behaviour
| metric | dDP | dDPsucc | dDPtiptrunc | dHrerec_all | dHrerec_succ |
|---|---|---|---|---|---|
| grip cmd closed (<0.5) frac | 0.630 | 0.642 | 0.626 | 0.782 | 0.765 |
| grip effort mean | 4.335 | 5.498 | 4.410 | 3.037 | 3.714 |
| joint path length, successes p50 (rad) | 6.71 | 6.71 | 6.71 | 7.86 | 7.64 |
| mean |dq| per frame (rad) | 0.00236 | 0.00256 | 0.00244 | 0.00199 | 0.00197 |

### OOD-ness within set (fail frames vs own success frames; std units of the success frames)
| metric | dDP | dDPsucc | dDPtiptrunc | dHrerec_all | dHrerec_succ |
|---|---|---|---|---|---|
| NN dist fail->succ p50 | 2.71 | — | 2.08 | 0.90 | — |
|   p90 | 42.06 | — | 7.91 | 4.77 | — |
|   p99 | 43.88 | — | 42.61 | 15.09 | — |
| succ->succ p50 | 0.070 | — | 0.069 | 0.019 | — |
| succ->succ p99 | 1.06 | — | 0.94 | 1.08 | — |
| frac fail frames beyond succ p99 | 0.81 | — | 0.80 | 0.43 | — |
| NN dist, tipped fail frames p50 | 42.08 | — | 43.29 | 14.89 | — |
| NN dist, non-tipped fail frames p50 | 2.03 | — | 2.05 | 0.82 | — |

### Cross-set OOD matrix: NN distance (p50 / p90, pooled-success std units) of ROW set frames to COLUMN set SUCCESS frames
| row (set:part) | dDP | dDPsucc | dDPtiptrunc | dHrerec_all | dHrerec_succ |
|---|---|---|---|---|---|
| dDP:all | 0.49 / 12.50 | 0.50 / 12.41 | 0.48 / 12.47 | 0.89 / 5.24 | 0.89 / 5.26 |
| dDP:fail | 2.25 / 16.37 | 2.25 / 16.32 | 2.23 / 16.28 | 1.99 / 7.20 | 1.93 / 7.06 |
| dDP:success | 0.07 / 0.34 | 0.06 / 0.33 | 0.06 / 0.31 | 0.56 / 1.28 | 0.55 / 1.24 |
| dDPsucc:all | 0.06 / 0.31 | 0.06 / 0.31 | 0.06 / 0.30 | 0.56 / 1.25 | 0.55 / 1.23 |
| dDPsucc:success | 0.06 / 0.32 | 0.06 / 0.31 | 0.06 / 0.31 | 0.56 / 1.32 | 0.55 / 1.28 |
| dDPtiptrunc:all | 0.29 / 4.44 | 0.29 / 4.44 | 0.29 / 4.45 | 0.79 / 3.42 | 0.79 / 3.49 |
| dDPtiptrunc:fail | 1.82 / 5.79 | 1.82 / 5.79 | 1.83 / 5.81 | 1.55 / 5.14 | 1.50 / 5.19 |
| dDPtiptrunc:success | 0.06 / 0.31 | 0.06 / 0.30 | 0.06 / 0.30 | 0.56 / 1.24 | 0.56 / 1.23 |
| dHrerec_all:all | 1.10 / 4.48 | 1.11 / 4.46 | 1.11 / 4.45 | 0.07 / 1.06 | 0.08 / 1.10 |
| dHrerec_all:fail | 1.29 / 5.34 | 1.27 / 5.33 | 1.28 / 5.34 | 0.99 / 5.13 | 0.98 / 5.13 |
| dHrerec_all:success | 1.05 / 4.31 | 1.07 / 4.28 | 1.06 / 4.26 | 0.02 / 0.39 | 0.04 / 0.51 |
| dHrerec_succ:all | 1.09 / 3.83 | 1.10 / 3.83 | 1.09 / 3.83 | 0.03 / 0.39 | 0.03 / 0.34 |
| dHrerec_succ:success | 1.13 / 3.59 | 1.15 / 3.62 | 1.14 / 3.62 | 0.02 / 0.38 | 0.02 / 0.35 |

### How to read
- fail share / post-termination share / longest chain: what the RL critic bootstraps through that the online MDP never produces (ROUND_ROBIN_RESULTS_2026-08-22 "Why dDP_RLPD < dH_RLPD").
- over cap / label err: frames the delta_joint encoder cannot represent (AUDIT_normalization_2026-08-17 C1); leash: PD lead beyond the env leash.
- NN distances: large = states nothing grounded covers; tipped-can frames land ~40 std away because success frames never vary in can orientation.
- reward density is informative but was shown NOT to move RLPD ignition (hold-reward arm, 25x density, FABLE_HANDOFF §20).
