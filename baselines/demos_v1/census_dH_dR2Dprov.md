# Demo-set census — 2026-08-23 06:53
Constants: pick_z 0.1505, delta cap 0.025 rad/step, leash 0.125 rad, tip rule tilt>60.0 deg & grip cmd<0.3. NN samples 3000. Sets: dH=baselines/demos_v1/dH, dR2Dprov=baselines/demos_v1/dR2Dprov.

### Composition
| metric | dH | dR2Dprov |
|---|---|---|
| tapes | 51 | 64 |
| by stage | {'picked': 51} | {'picked': 64} |
| by label | {'success': 51} | {'success': 64} |
| success (lifted) | 51 | 64 |
| fail (never lifted) | 0 | 0 |
| transitions | 6491 | 1100 |
| fail share of buffer | 0.000 | 0.000 |
| tape len p50 | 113 | 17 |
| tape len mean | 127 | 17 |
| tape len max | 285 | 24 |
| success len p50 | 113 | 17 |
| fail len p50 | — | — |
| tapes at max len (timed out) | 1 | 1 |
| unique ICs (can xy @t0) | 50 | 62 |
| IC grid cells occupied /16 | 13 | 14 |
| goal xy | [(0.672, -0.221)] | [(0.672, -0.221)] |
| tapes with images | 51 | 64 |

### Reward (pick scope)
| metric | dH | dR2Dprov |
|---|---|---|
| rewarded transitions | 51 | 64 |
| reward density | 0.00786 | 0.05818 |
| expected rewards per 128-demo batch | 1.006 | 7.447 |
| first-lift idx p50 | 112 | 16 |
| first-lift idx mean | 126 | 16 |
| frames kept after lift p50 | 1 | 1 |

### Fidelity under delta_joint encoding (cap/leash)
| metric | dH | dR2Dprov |
|---|---|---|
| frac frames over cap | 0.326 | 0.978 |
|   successes | 0.326 | 0.978 |
|   fails | — | — |
| frac frames one-step label err >1e-3 | 0.318 | 0.978 |
| label err p99 (rad, median tape) | 0.0750 | 0.0701 |
| label err max (rad) | 0.075 | 0.075 |
| |delta cmd| p99 (median tape) | 0.1000 | 0.0951 |
| frac zero-delta frames | 0.064 | 0.000 |
| frac frames |cmd-qmeas|>leash | 0.151 | 0.503 |
| lead p99 (median tape) | 0.164 | 0.178 |

### Contract v1 (recorded-as-executed at the decision clock; PREREG 2026-08-23 §4)
| metric | dH | dR2Dprov |
|---|---|---|
| contract-v1 tapes | 51 | 64 |
| action_repeat stamp | [4] | [4] |
| frac decisions at the cap (|a_arm|>=0.999 any dim) | 0.055 | 0.047 |
|   successes | 0.055 | 0.047 |
|   fails | — | — |
| mean |a_arm| (cap units) | 0.109 | 0.371 |
| tapes ending terminated | 51 | 64 |
|   of which tipped | 0 | 0 |
| tapes ending truncated (cap) | 0 | 0 |
| tapes with terminal BEFORE last row (contract violation) | 0 | 0 |
| tapes with eef_pos | 51 | 64 |
| min eef-can distance p50 (m) | 0.048 | 0.012 |
| human re-record: tapes with dilation stat | 0 | 0 |
|   time dilation p50 | — | — |
|   max | — | — |

### Time-base: stride-4 (action_repeat=4) re-encoding fidelity of the stride-1 tape
| metric | dH | dR2Dprov |
|---|---|---|
| decision windows | — | — |
| frac windows EXACTLY representable (dev<1e-4 rad) | — | — |
|   successes | — | — |
|   fails | — | — |
| command-vs-ramp dev p50 (rad, median tape) | — | — |
|   p99 (median tape) | — | — |
|   max | — | — |
|   p50 in cap units | — | — |
| frac windows over N*cap (clipped) | — | — |
|   successes | — | — |
|   fails | — | — |
| frac windows with intra-window reversal | — | — |
|   successes | — | — |
|   fails | — | — |
| frac windows with a grip toggle inside | — | — |

### Env-consistency (pick-scope termination) & can state
| metric | dH | dR2Dprov |
|---|---|---|
| tapes where can tilts >tip_deg | 0 | 2 |
|   of which fails | 0 | 0 |
| POST-TERMINATION frames / buffer | 0.000 | 0.000 |
|   / fail frames | 0.000 | 0.000 |
| longest post-term chain (frames) | 0 | 0 |
| tapes tipped at t0 (lying-can IC) | 0 | 0 |
| tilt @t0 p50 (deg) | 0.0 | 0.0 |
| tilt @t0 max | 0.0 | 0.0 |
| can max displacement, fails p50 (m) | — | — |
| can max displacement, max (m) | 0.126 | 0.265 |
| can z @t0 p50 | 0.112 | 0.112 |
| can z min | 0.100 | 0.100 |

### Behaviour
| metric | dH | dR2Dprov |
|---|---|---|
| grip cmd closed (<0.5) frac | 0.679 | 0.544 |
| grip effort mean | 4.702 | 3.158 |
| joint path length, successes p50 (rad) | 6.42 | 2.64 |
| mean |dq| per frame (rad) | 0.01034 | 0.02945 |

### OOD-ness within set (fail frames vs own success frames; std units of the success frames)
| metric | dH | dR2Dprov |
|---|---|---|
| NN dist fail->succ p50 | — | — |
|   p90 | — | — |
|   p99 | — | — |
| succ->succ p50 | — | — |
| succ->succ p99 | — | — |
| frac fail frames beyond succ p99 | — | — |
| NN dist, tipped fail frames p50 | — | — |
| NN dist, non-tipped fail frames p50 | — | — |

### Cross-set OOD matrix: NN distance (p50 / p90, pooled-success std units) of ROW set frames to COLUMN set SUCCESS frames
| row (set:part) | dH | dR2Dprov |
|---|---|---|
| dH:all | 0.01 / 0.43 | 2.18 / 4.40 |
| dH:success | 0.00 / 0.44 | 2.25 / 4.44 |
| dR2Dprov:all | 1.43 / 4.57 | 0.00 / 0.00 |
| dR2Dprov:success | 1.43 / 4.57 | 0.00 / 0.00 |

### How to read
- fail share / post-termination share / longest chain: what the RL critic bootstraps through that the online MDP never produces (ROUND_ROBIN_RESULTS_2026-08-22 "Why dDP_RLPD < dH_RLPD").
- over cap / label err: frames the delta_joint encoder cannot represent (AUDIT_normalization_2026-08-17 C1); leash: PD lead beyond the env leash.
- contract v1: tapes from baselines/record_demos.py are one row per decision in the learners' own action space; the stride table is left blank for them (exact by construction); at-cap = decisions where the teacher's intent was clipped by the env cap.
- time-base: a repeat-4 learner (r2dreamer/dv3; RLPD if run with --action-repeat 4) consumes stride-1 tapes through the window encoder; dev = how far the recorded command path departs from the linear ramp the learner would execute for the re-encoded action (0 = native repeat-N tape). Reversal = the teacher changed direction inside the window (averaged away by the encoder).
- NN distances: large = states nothing grounded covers; tipped-can frames land ~40 std away because success frames never vary in can orientation.
- reward density is informative but was shown NOT to move RLPD ignition (hold-reward arm, 25x density, FABLE_HANDOFF §20).
