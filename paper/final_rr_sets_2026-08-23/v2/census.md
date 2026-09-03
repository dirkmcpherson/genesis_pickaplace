# Demo-set census — 2026-08-23 14:17
Constants: pick_z 0.1505, delta cap 0.025 rad/step, leash 0.125 rad, tip rule tilt>60.0 deg & grip cmd<0.3. NN samples 6000. Sets: dH=baselines/matched_v2/dH, dDP=baselines/matched_v2/dDP, dR2D=baselines/matched_v2/dR2D, dDPfails=baselines/matched_v2/dDPfails, dR2DDPfails=baselines/matched_v2/dR2DDPfails.

### Composition
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| tapes | 56 | 56 | 56 | 64 | 64 |
| by stage | {'picked': 56} | {'picked': 56} | {'picked': 56} | {'none': 8, 'picked': 56} | {'none': 8, 'picked': 56} |
| by label | {'success': 56} | {'success': 56} | {'success': 56} | {'fail': 8, 'success': 56} | {'fail': 8, 'success': 56} |
| success (lifted) | 56 | 56 | 56 | 56 | 56 |
| fail (never lifted) | 0 | 0 | 0 | 8 | 8 |
| transitions | 7002 | 6909 | 954 | 9309 | 3354 |
| fail share of buffer | 0.000 | 0.000 | 0.000 | 0.258 | 0.716 |
| tape len p50 | 115 | 114 | 17 | 122 | 17 |
| tape len mean | 125 | 123 | 17 | 145 | 52 |
| tape len max | 284 | 233 | 24 | 300 | 300 |
| success len p50 | 115 | 114 | 17 | 114 | 17 |
| fail len p50 | — | — | — | 300 | 300 |
| tapes at max len (timed out) | 1 | 1 | 1 | 8 | 8 |
| unique ICs (can xy @t0) | 55 | 55 | 55 | 56 | 56 |
| IC grid cells occupied /16 | 13 | 13 | 13 | 13 | 13 |
| goal xy | [(0.672, -0.221)] | [(0.672, -0.221)] | [(0.672, -0.221)] | [(0.672, -0.221)] | [(0.672, -0.221)] |
| tapes with images | 56 | 56 | 56 | 64 | 64 |

### Reward (pick scope)
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| rewarded transitions | 56 | 56 | 56 | 56 | 56 |
| reward density | 0.00800 | 0.00811 | 0.05870 | 0.00602 | 0.01670 |
| expected rewards per 128-demo batch | 1.024 | 1.037 | 7.514 | 0.770 | 2.137 |
| first-lift idx p50 | 114 | 114 | 16 | 114 | 16 |
| first-lift idx mean | 124 | 122 | 16 | 122 | 16 |
| frames kept after lift p50 | 1 | 1 | 1 | 1 | 1 |

### Fidelity under delta_joint encoding (cap/leash)
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| frac frames over cap | 0.346 | 0.355 | 0.978 | 0.304 | 0.390 |
|   successes | 0.346 | 0.355 | 0.978 | 0.355 | 0.978 |
|   fails | — | — | — | 0.157 | 0.157 |
| frac frames one-step label err >1e-3 | 0.340 | 0.345 | 0.978 | 0.295 | 0.386 |
| label err p99 (rad, median tape) | 0.0750 | 0.0632 | 0.0699 | 0.0631 | 0.0695 |
| label err max (rad) | 0.075 | 0.075 | 0.075 | 0.075 | 0.075 |
| |delta cmd| p99 (median tape) | 0.1000 | 0.0882 | 0.0949 | 0.0881 | 0.0945 |
| frac zero-delta frames | 0.013 | 0.000 | 0.000 | 0.000 | 0.000 |
| frac frames |cmd-qmeas|>leash | 0.189 | 0.079 | 0.500 | 0.079 | 0.200 |
| lead p99 (median tape) | 0.166 | 0.150 | 0.178 | 0.147 | 0.177 |

### Contract v1 (recorded-as-executed at the decision clock; PREREG 2026-08-23 §4)
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| contract-v1 tapes | 56 | 56 | 56 | 64 | 64 |
| action_repeat stamp | [4] | [4] | [4] | [4] | [4] |
| frac decisions at the cap (|a_arm|>=0.999 any dim) | 0.069 | 0.010 | 0.047 | 0.010 | 0.019 |
|   successes | 0.069 | 0.010 | 0.047 | 0.010 | 0.047 |
|   fails | — | — | — | 0.007 | 0.007 |
| mean |a_arm| (cap units) | 0.119 | 0.115 | 0.370 | 0.105 | 0.159 |
| tapes ending terminated | 56 | 56 | 56 | 56 | 56 |
|   of which tipped | 0 | 0 | 0 | 0 | 0 |
| tapes ending truncated (cap) | 0 | 0 | 0 | 8 | 8 |
| tapes with terminal BEFORE last row (contract violation) | 0 | 0 | 0 | 0 | 0 |
| tapes with eef_pos | 56 | 56 | 56 | 64 | 64 |
| min eef-can distance p50 (m) | 0.053 | 0.051 | 0.011 | 0.054 | 0.012 |
| human re-record: tapes with dilation stat | 56 | 0 | 0 | 0 | 0 |
|   time dilation p50 | 1.03 | — | — | — | — |
|   max | 1.65 | — | — | — | — |

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
| tapes where can tilts >tip_deg | 0 | 0 | 2 | 0 | 2 |
|   of which fails | 0 | 0 | 0 | 0 | 0 |
| POST-TERMINATION frames / buffer | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 |
|   / fail frames | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 |
| longest post-term chain (frames) | 0 | 0 | 0 | 0 | 0 |
| tapes tipped at t0 (lying-can IC) | 0 | 0 | 0 | 0 | 0 |
| tilt @t0 p50 (deg) | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| tilt @t0 max | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| can max displacement, fails p50 (m) | — | — | — | 0.090 | 0.090 |
| can max displacement, max (m) | 0.126 | 0.110 | 0.265 | 0.118 | 0.265 |
| can z @t0 p50 | 0.112 | 0.112 | 0.112 | 0.112 | 0.112 |
| can z min | 0.100 | 0.100 | 0.100 | 0.100 | 0.100 |

### Behaviour
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| grip cmd closed (<0.5) frac | 0.664 | 0.607 | 0.540 | 0.516 | 0.333 |
| grip effort mean | 4.950 | 5.725 | 3.173 | 5.451 | 3.217 |
| joint path length, successes p50 (rad) | 6.46 | 5.85 | 2.62 | 5.85 | 2.62 |
| mean |dq| per frame (rad) | 0.01076 | 0.00944 | 0.02921 | 0.00873 | 0.02603 |

### OOD-ness within set (fail frames vs own success frames; std units of the success frames)
| metric | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| NN dist fail->succ p50 | — | — | — | 1.73 | 4.95 |
|   p90 | — | — | — | 2.54 | 12.16 |
|   p99 | — | — | — | 3.19 | 16.09 |
| succ->succ p50 | — | — | — | 0.000 | 0.000 |
| succ->succ p99 | — | — | — | 0.57 | 0.00 |
| frac fail frames beyond succ p99 | — | — | — | 0.85 | 1.00 |
| NN dist, tipped fail frames p50 | — | — | — | — | — |
| NN dist, non-tipped fail frames p50 | — | — | — | 1.73 | 4.95 |

### Cross-set OOD matrix: NN distance (p50 / p90, pooled-success std units) of ROW set frames to COLUMN set SUCCESS frames
| row (set:part) | dH | dDP | dR2D | dDPfails | dR2DDPfails |
|---|---|---|---|---|---|
| dH:all | 0.00 / 0.07 | 0.60 / 2.00 | 2.24 / 4.42 | 0.61 / 2.00 | 2.24 / 4.42 |
| dH:success | 0.00 / 0.09 | 0.59 / 2.00 | 2.30 / 4.42 | 0.59 / 2.00 | 2.30 / 4.42 |
| dDP:all | 0.53 / 0.99 | 0.00 / 0.09 | 2.38 / 4.78 | 0.00 / 0.07 | 2.38 / 4.78 |
| dDP:success | 0.52 / 0.99 | 0.00 / 0.07 | 2.33 / 4.76 | 0.00 / 0.05 | 2.33 / 4.76 |
| dR2D:all | 1.31 / 4.88 | 1.38 / 5.42 | 0.00 / 0.00 | 1.38 / 5.42 | 0.00 / 0.00 |
| dR2D:success | 1.31 / 4.88 | 1.38 / 5.42 | 0.00 / 0.00 | 1.38 / 5.42 | 0.00 / 0.00 |
| dDPfails:all | 0.61 / 1.78 | 0.00 / 1.93 | 2.37 / 4.48 | 0.00 / 1.93 | 2.37 / 4.48 |
| dDPfails:fail | 1.35 / 2.15 | 1.74 / 2.71 | 2.40 / 3.56 | 1.74 / 2.57 | 2.40 / 3.56 |
| dDPfails:success | 0.52 / 0.96 | 0.00 / 0.09 | 2.26 / 4.67 | 0.00 / 0.07 | 2.26 / 4.67 |
| dR2DDPfails:all | 1.34 / 2.23 | 1.58 / 2.80 | 2.02 / 3.42 | 1.57 / 2.67 | 2.02 / 3.42 |
| dR2DDPfails:fail | 1.35 / 2.15 | 1.74 / 2.71 | 2.40 / 3.56 | 1.74 / 2.57 | 2.40 / 3.56 |
| dR2DDPfails:success | 1.31 / 4.88 | 1.38 / 5.42 | 0.00 / 0.00 | 1.38 / 5.42 | 0.00 / 0.00 |

### How to read
- fail share / post-termination share / longest chain: what the RL critic bootstraps through that the online MDP never produces (ROUND_ROBIN_RESULTS_2026-08-22 "Why dDP_RLPD < dH_RLPD").
- over cap / label err: frames the delta_joint encoder cannot represent (AUDIT_normalization_2026-08-17 C1); leash: PD lead beyond the env leash.
- contract v1: tapes from baselines/record_demos.py are one row per decision in the learners' own action space; the stride table is left blank for them (exact by construction); at-cap = decisions where the teacher's intent was clipped by the env cap.
- time-base: a repeat-4 learner (r2dreamer/dv3; RLPD if run with --action-repeat 4) consumes stride-1 tapes through the window encoder; dev = how far the recorded command path departs from the linear ramp the learner would execute for the re-encoded action (0 = native repeat-N tape). Reversal = the teacher changed direction inside the window (averaged away by the encoder).
- NN distances: large = states nothing grounded covers; tipped-can frames land ~40 std away because success frames never vary in can orientation.
- reward density is informative but was shown NOT to move RLPD ignition (hold-reward arm, 25x density, FABLE_HANDOFF §20).
