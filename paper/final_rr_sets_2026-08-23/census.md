# Demo-set census — 2026-08-23 13:54
Constants: pick_z 0.1505, delta cap 0.025 rad/step, leash 0.125 rad, tip rule tilt>60.0 deg & grip cmd<0.3. NN samples 6000. Sets: dH=baselines/matched_v1/dH, dDP=baselines/matched_v1/dDP, dR2D=baselines/matched_v1/dR2D, dDPfails=baselines/matched_v1/dDPfails, dR2DDPfails=baselines/matched_v1/dR2DDPfails.

### Composition
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| tapes | 51 | 51 | 51 | 65 | 65 |
| by stage | {'picked': 51} | {'picked': 51} | {'picked': 51} | {'none': 14, 'picked': 51} | {'none': 14, 'picked': 51} |
| by label | {'success': 51} | {'success': 51} | {'success': 51} | {'fail': 14, 'success': 51} | {'fail': 14, 'success': 51} |
| success (lifted) | 51 | 51 | 51 | 51 | 51 |
| fail (never lifted) | 0 | 0 | 0 | 14 | 14 |
| transitions | 6491 | 6294 | 861 | 8700 | 3267 |
| fail share of buffer | 0.000 | 0.000 | 0.000 | 0.277 | 0.736 |
| tape len p50 | 113 | 113 | 17 | 116 | 17 |
| tape len mean | 127 | 123 | 17 | 134 | 50 |
| tape len max | 285 | 233 | 24 | 300 | 300 |
| success len p50 | 113 | 113 | 17 | 113 | 17 |
| fail len p50 | — | — | — | 300 | 300 |
| tapes at max len (timed out) | 1 | 1 | 1 | 8 | 8 |
| unique ICs (can xy @t0) | 50 | 50 | 50 | 55 | 55 |
| IC grid cells occupied /16 | 13 | 13 | 13 | 13 | 13 |
| goal xy | [(0.672, -0.221)] | [(0.672, -0.221)] | [(0.672, -0.221)] | [(0.672, -0.221)] | [(0.672, -0.221)] |
| tapes with images | 51 | 51 | 51 | 65 | 65 |

### Reward (pick scope)
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| rewarded transitions | 51 | 51 | 51 | 51 | 51 |
| reward density | 0.00786 | 0.00810 | 0.05923 | 0.00586 | 0.01561 |
| expected rewards per 128-demo batch | 1.006 | 1.037 | 7.582 | 0.750 | 1.998 |
| first-lift idx p50 | 112 | 112 | 16 | 112 | 16 |
| first-lift idx mean | 126 | 122 | 16 | 122 | 16 |
| frames kept after lift p50 | 1 | 1 | 1 | 1 | 1 |

### Fidelity under delta_joint encoding (cap/leash)
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| frac frames over cap | 0.326 | 0.348 | 0.975 | 0.295 | 0.372 |
|   successes | 0.326 | 0.348 | 0.975 | 0.348 | 0.975 |
|   fails | — | — | — | 0.156 | 0.156 |
| frac frames one-step label err >1e-3 | 0.318 | 0.337 | 0.975 | 0.286 | 0.368 |
| label err p99 (rad, median tape) | 0.0750 | 0.0631 | 0.0699 | — | — |
| label err max (rad) | 0.075 | 0.075 | 0.075 | 0.075 | 0.075 |
| |delta cmd| p99 (median tape) | 0.1000 | 0.0881 | 0.0949 | — | — |
| frac zero-delta frames | 0.064 | 0.000 | 0.000 | 0.000 | 0.000 |
| frac frames |cmd-qmeas|>leash | 0.151 | 0.078 | 0.502 | 0.079 | 0.192 |
| lead p99 (median tape) | 0.164 | 0.150 | 0.179 | 0.142 | 0.177 |

### Contract v1 (recorded-as-executed at the decision clock; PREREG 2026-08-23 §4)
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| contract-v1 tapes | 51 | 51 | 51 | 65 | 65 |
| action_repeat stamp | [4] | [4] | [4] | [4] | [4] |
| frac decisions at the cap (|a_arm|>=0.999 any dim) | 0.055 | 0.010 | 0.048 | 0.010 | 0.020 |
|   successes | 0.055 | 0.010 | 0.048 | 0.010 | 0.048 |
|   fails | — | — | — | 0.010 | 0.010 |
| mean |a_arm| (cap units) | 0.109 | 0.113 | 0.370 | 0.103 | 0.154 |
| tapes ending terminated | 51 | 51 | 51 | 57 | 57 |
|   of which tipped | 0 | 0 | 0 | 6 | 6 |
| tapes ending truncated (cap) | 0 | 0 | 0 | 8 | 8 |
| tapes with terminal BEFORE last row (contract violation) | 0 | 0 | 0 | 0 | 0 |
| tapes with eef_pos | 51 | 51 | 51 | 65 | 65 |
| min eef-can distance p50 (m) | 0.048 | 0.056 | 0.011 | 0.060 | 0.012 |
| human re-record: tapes with dilation stat | 0 | 0 | 0 | 0 | 0 |
|   time dilation p50 | — | — | — | — | — |
|   max | — | — | — | — | — |

### Time-base: stride-4 (action_repeat=4) re-encoding fidelity of the stride-1 tape
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| decision windows | — | — | — | — | — |
| frac windows EXACTLY representable (dev<1e-4 rad) | — | — | — | — | — |
|   successes | — | — | — | — | — |
|   fails | — | — | — | — | — |
| command-vs-ramp dev p50 (rad, median tape) | — | — | — | — | — |
|   p99 (median tape) | — | — | — | — | — |
|   max | — | — | — | — | — |
|   p50 in cap units | — | — | — | — | — |
| frac windows over N*cap (clipped) | — | — | — | — | — |
|   successes | — | — | — | — | — |
|   fails | — | — | — | — | — |
| frac windows with intra-window reversal | — | — | — | — | — |
|   successes | — | — | — | — | — |
|   fails | — | — | — | — | — |
| frac windows with a grip toggle inside | — | — | — | — | — |

### Env-consistency (pick-scope termination) & can state
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| tapes where can tilts >tip_deg | 0 | 0 | 2 | 6 | 8 |
|   of which fails | 0 | 0 | 0 | 6 | 6 |
| POST-TERMINATION frames / buffer | 0.000 | 0.000 | 0.000 | 0.001 | 0.002 |
|   / fail frames | 0.000 | 0.000 | 0.000 | 0.002 | 0.002 |
| longest post-term chain (frames) | 0 | 0 | 0 | 1 | 1 |
| tapes tipped at t0 (lying-can IC) | 0 | 0 | 0 | 6 | 6 |
| tilt @t0 p50 (deg) | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| tilt @t0 max | 0.0 | 0.0 | 0.0 | 90.0 | 90.0 |
| can max displacement, fails p50 (m) | — | — | — | 0.063 | 0.063 |
| can max displacement, max (m) | 0.126 | 0.110 | 0.265 | 0.118 | 0.265 |
| can z @t0 p50 | 0.112 | 0.112 | 0.112 | 0.112 | 0.112 |
| can z min | 0.100 | 0.100 | 0.100 | 0.084 | 0.084 |

### Behaviour
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| grip cmd closed (<0.5) frac | 0.679 | 0.602 | 0.533 | 0.506 | 0.327 |
| grip effort mean | 4.702 | 5.823 | 3.198 | 5.003 | 2.943 |
| joint path length, successes p50 (rad) | 6.42 | 5.79 | 2.62 | 5.79 | 2.62 |
| mean |dq| per frame (rad) | 0.01034 | 0.00924 | 0.02909 | 0.00772 | 0.02329 |

### OOD-ness within set (fail frames vs own success frames; std units of the success frames)
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| NN dist fail->succ p50 | — | — | — | 1.73 | 5.05 |
|   p90 | — | — | — | 2.54 | 12.46 |
|   p99 | — | — | — | 3.26 | 16.50 |
| succ->succ p50 | — | — | — | 0.000 | 0.000 |
| succ->succ p99 | — | — | — | 0.23 | 0.00 |
| frac fail frames beyond succ p99 | — | — | — | 0.96 | 1.00 |
| NN dist, tipped fail frames p50 | — | — | — | 40.66 | 13.44 |
| NN dist, non-tipped fail frames p50 | — | — | — | 1.73 | 5.04 |

### Cross-set OOD matrix: NN distance (p50 / p90, pooled-success std units) of ROW set frames to COLUMN set SUCCESS frames
| row (set:part) | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| dH:all | 0.00 / 0.00 | 0.59 / 1.92 | 2.21 / 4.33 | 0.59 / 1.92 | 2.21 / 4.33 |
| dH:success | 0.00 / 0.00 | 0.58 / 1.94 | 2.18 / 4.31 | 0.58 / 1.93 | 2.18 / 4.31 |
| dDP:all | 0.52 / 0.93 | 0.00 / 0.00 | 2.30 / 4.77 | 0.00 / 0.00 | 2.30 / 4.77 |
| dDP:success | 0.53 / 0.95 | 0.00 / 0.00 | 2.33 / 4.83 | 0.00 / 0.00 | 2.33 / 4.83 |
| dR2D:all | 1.28 / 4.86 | 1.37 / 5.50 | 0.00 / 0.00 | 1.38 / 5.45 | 0.00 / 0.00 |
| dR2D:success | 1.28 / 4.86 | 1.37 / 5.50 | 0.00 / 0.00 | 1.38 / 5.45 | 0.00 / 0.00 |
| dDPfails:all | 0.59 / 1.77 | 0.00 / 1.99 | 2.40 / 4.59 | 0.00 / 1.99 | 2.40 / 4.59 |
| dDPfails:fail | 1.32 / 2.18 | 1.75 / 2.58 | 2.50 / 3.66 | 1.75 / 2.58 | 2.50 / 3.66 |
| dDPfails:success | 0.52 / 0.94 | 0.00 / 0.00 | 2.29 / 4.81 | 0.00 / 0.00 | 2.29 / 4.81 |
| dR2DDPfails:all | 1.31 / 2.25 | 1.60 / 2.68 | 2.18 / 3.49 | 1.60 / 2.68 | 2.18 / 3.49 |
| dR2DDPfails:fail | 1.32 / 2.18 | 1.75 / 2.58 | 2.50 / 3.66 | 1.75 / 2.58 | 2.50 / 3.66 |
| dR2DDPfails:success | 1.28 / 4.86 | 1.37 / 5.50 | 0.00 / 0.00 | 1.38 / 5.45 | 0.00 / 0.00 |

### How to read
- fail share / post-termination share / longest chain: what the RL critic bootstraps through that the online MDP never produces (ROUND_ROBIN_RESULTS_2026-08-22 "Why dDP_RLPD < dH_RLPD").
- over cap / label err: frames the delta_joint encoder cannot represent (AUDIT_normalization_2026-08-17 C1); leash: PD lead beyond the env leash.
- contract v1: tapes from baselines/record_demos.py are one row per decision in the learners' own action space; the stride table is left blank for them (exact by construction); at-cap = decisions where the teacher's intent was clipped by the env cap.
- time-base: a repeat-4 learner (r2dreamer/dv3; RLPD if run with --action-repeat 4) consumes stride-1 tapes through the window encoder; dev = how far the recorded command path departs from the linear ramp the learner would execute for the re-encoded action (0 = native repeat-N tape). Reversal = the teacher changed direction inside the window (averaged away by the encoder).
- NN distances: large = states nothing grounded covers; tipped-can frames land ~40 std away because success frames never vary in can orientation.
- reward density is informative but was shown NOT to move RLPD ignition (hold-reward arm, 25x density, FABLE_HANDOFF §20).
