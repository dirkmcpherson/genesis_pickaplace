# Demo characterization: machine (dDP) vs raw human (dHv2raw) in the same world — 2026-09-04

PHASE_PLAN_2026-09-04 §6 item 3. Read-only on the tapes; nothing re-simulated; no learner run.
Scripts (committed): `paper/characterize_dDP_vs_dHv2raw_2026-09-04.py` (descriptors, tables, stats, figures) and
`paper/gallery_dDP_vs_dHv2raw_2026-09-04.py` (side-by-side videos). Outputs (data, rsync-only, NOT committed):
`~/wm_fix_2026-09-03/characterization/` on the pop-os box — `tape_descriptors.csv` (one row per tape, every descriptor),
`per_set_table.md`, `paired_table.md`, `stats.json`, `fig_descriptors_dDP_vs_dHv2raw.png`, `fig_xy_paths_gallery.png`,
`videos/uid<u>_human_L_machine_R.mp4` ×12 (+ a `_grasp_frame.png` still per video), `sets/` (the rsynced tapes).

## 1. Sets

| set | tapes | source (from the npz stamps) | world | recorded |
|---|---|---|---|---|
| dHv2raw (human) | 66 | the operator's real joystick stream re-executed by `record_demos.py v1` (`teacher=human`, `act_mode=sample`, no pruning) | `gc_kp4_riser3_shelf6` | git 172a811 (2026-09-01) |
| dDP (machine) | 58 | DP teacher `baselines/outputs/dp_pilotw2/dH_DP_s0/checkpoints/100000` (trained on the PRUNED human set dH), `act_mode=sample`, `verify=pass` | `gc_kp4_riser3_shelf6` | git 2061f08 (2026-08-24) |

Cluster paths `$LAB/genesis_pickaplace/baselines/matched_w3/{dHv2raw,dDP}`; local copy `~/wm_fix_2026-09-03/characterization/sets/`.
Contract v1, one row per decision at 7.5 Hz (`action_repeat` 4 on the 30 Hz sim, dt = 0.1333 s), pick scope: every tape
ends at its first `picked` row (`end_reason=terminated`, 124/124), `picked` is a single trailing True. Images present in all
124 tapes (`(n+1,64,64,6)` = top view | wrist view). Both sets stamp the same `sim_variant`; the recordings are eight days
and two commits apart, so physics identity beyond the stamp is not verifiable from the tapes (neither carries the ts5
gripper fix — CONFOUNDS row on the frozen w3 sets).

**Pairing by `ic_uid`: 55 ICs in both sets.** Human-only 11: 236 239 244 245 246 255 286 293 295 300 331
(these are the ICs where the DP teacher produced no verified success; they are the LONGEST human tapes — median 468
decisions, idle fraction 0.51, vs median 131 for the 55 paired human tapes). Machine-only 3: 278 301 319.
Per-set tables use all 66 / 58 tapes; the paired comparison uses the 55 shared ICs.

## 2. Descriptor definitions (fields → formula)

All from the npz fields; `a` = `actions` (n,7), `d` = `actions_delta` (n,7), `s` = `states` (n,17), `e` = `eef_pos` (n+1,3),
`fs` = `final_state`, dt = 1/7.5 s. Grip cmd = `a[:,6]` ∈ [0,1], 0 = open, plateau ≈ 0.8 = closed. Measured finger
position = `s[:,6]`. Can xy/z = `s[:,8:10]`, `s[:,10]`. Arm step per decision = `d[:, :6] · 4 · delta_cap` = 0.1 rad · d
(verified: `a[t]−a[t−1]` = 0.1·d[t] to 1e-4 on every row except the terminal row, whose sim window is cut at the
sub-step where `picked` fires — `len(sim_states) = 4n − {0..3}` in both sets).

| descriptor | computed as |
|---|---|
| tape length / duration | n = `len(actions)`; n·dt |
| idle fraction | share of decisions t = 1..n−1 with max_j \|a[t,j] − a[t−1,j]\| < 1e-3 rad over the 6 arm joints (the brief's definition). Also reported with the fig12 convention: share of all decisions with max_j \|d[t,j]\| < 1e-3 (cap units = 1e-4 rad) |
| leading idle | decisions before the first non-idle decision |
| active decisions / active speed / active \|delta\| | non-idle decisions; tool path covered during them ÷ their time; mean \|d[:, :6]\| over them |
| tool path length / net displacement / tortuosity | Σ_t ‖e[t+1] − e[t]‖; ‖e[n] − e[0]‖; ratio |
| mean / peak / p95 tool speed | path ÷ (n·dt); max_t ‖e[t+1] − e[t]‖/dt; 95th percentile of the same |
| jerk RMS | sqrt(mean_t ‖Δ³e[t]‖²) / dt³ with Δ³ the third finite difference of `eef_pos` |
| time-to-grasp (closure) | t_g = first t with grip cmd > 0.5; grip onset t_on = first t with cmd > 0.05 |
| grip-close duration | cmd: t_g − t_on; measured: (first t with `s[t,6]` ≥ 0.9·`fs[6]`) − t_on |
| closure → pick | (n−1) − t_g |
| arrival at the grasp pose | t_a = first decision from which the tool xy stays within 1.5 cm of its xy at t_g (the tool has stopped ~2 s before the cmd crosses 0.5, so a window ending at t_g measures nothing — checked: 1–2 mm of xy travel there in both sets) |
| dwell | t_g − t_a (parked at the grasp pose before cmd > 0.5); t_on − t_a (before the grip starts closing) |
| approach height | mean e[:,2] over the 8 decisions (1.07 s) before t_a; also e[t_a,2], e[t_g,2] and e[t_g,2] − can z(t_g) |
| approach heading vs can | angle between the tool's xy motion over those 8 decisions and the vector from the tool (at the window start) to the can xy at t_g; 0° = heading straight at the can |
| approach dip angle | atan2(descent, xy travel) over the same window; approach bearing = world-frame direction from the can to the tool at the window start (circular mean per set, circular difference per pair) |
| tool–can xy offset at closure | ‖e[t_g, :2] − can xy(t_g)‖ |
| can displacement before the grasp | ‖can xy(t) − can xy(0)‖ at t_g and its max over t ≤ t_g; "nudged" = max > 5 mm |
| pick-frame tool height / can z | e[n, 2] (the position after the terminating decision); `fs[10]` (pick_z threshold 0.1505) |
| largest can-z drop after closure | max_t≥t_g (running max of can z − can z) |
| saturation | share of decisions with any \|d[t, 0:6]\| ≥ 1 − 1e-6 (arm only — the grip column `2·grip−1` is −1 whenever the gripper is open, so an any-of-7 count is meaningless: it reads 0.114 vs 0.056); also the per joint-decision share and the share of decisions with any \|d\| > 0.5 |

`eef_pos` is the recorder's tool frame: it sits ≈ 8 cm BELOW the can centre at closure in both sets, i.e. the frame is
not the fingertip. Absolute heights are frame-dependent; between-set differences are not.

Statistics (paired, 55 ICs, difference = human − machine): mean difference with a 10 000-draw bootstrap 95% CI, median
difference, Cohen d_z = mean diff / sd of the differences, fraction of pairs with human > machine, Wilcoxon signed-rank
p (scipy `method='exact'`, zero differences dropped; `<1e-15` = below double precision) and a sign-flip permutation p
on the mean difference (Monte-Carlo, 200 000 flips, seed 0; the floor is 5e-6). No multiplicity correction — 47 rows;
treat p ≈ 0.01–0.05 as suggestive.

## 3. Per-set descriptors (all tapes; mean / median [IQR])

| descriptor | unit | human dHv2raw (n=66) mean / median [IQR] | machine dDP (n=58) mean / median [IQR] |
|---|---|---|---|
| tape length | decisions | 217.0 / 146.5 [96.2, 217.5] | 125.6 / 119.5 [90.5, 144.8] |
| duration | s | 28.94 / 19.53 [12.83, 29.00] | 16.75 / 15.93 [12.07, 19.30] |
| idle fraction (\|dq_cmd\| < 1e-3 rad) |  | 0.458 / 0.458 [0.364, 0.561] | 0.041 / 0.030 [0.009, 0.058] |
| idle fraction (\|delta\| < 1e-3 cap) |  | 0.271 / 0.238 [0.166, 0.392] | 0.000 / 0.000 [0.000, 0.000] |
| leading idle | decisions | 16.3 / 7.5 [2.0, 21.0] | 0.0 / 0.0 [0.0, 0.0] |
| tool path length | m | 0.608 / 0.492 [0.378, 0.726] | 0.545 / 0.516 [0.413, 0.616] |
| net tool displacement | m | 0.144 / 0.150 [0.116, 0.170] | 0.140 / 0.143 [0.116, 0.163] |
| tortuosity (path/net) |  | 4.584 / 3.604 [2.672, 5.406] | 4.426 / 3.689 [2.777, 4.785] |
| mean tool speed | m/s | 0.027 / 0.025 [0.019, 0.034] | 0.035 / 0.034 [0.028, 0.040] |
| peak tool speed | m/s | 0.158 / 0.145 [0.129, 0.167] | 0.138 / 0.137 [0.118, 0.154] |
| p95 tool speed | m/s | 0.103 / 0.102 [0.088, 0.114] | 0.104 / 0.104 [0.091, 0.119] |
| jerk RMS | m/s^3 | 2.254 / 2.233 [1.563, 2.725] | 1.966 / 1.993 [1.555, 2.277] |
| time-to-grasp (cmd>0.5) | decisions | 158.3 / 104.5 [62.0, 164.0] | 79.8 / 75.0 [55.2, 95.8] |
| time-to-grasp | s | 21.11 / 13.93 [8.267, 21.87] | 10.64 / 10.00 [7.367, 12.77] |
| grip onset (cmd>0.05) | decisions | 130.7 / 88.5 [49.8, 132.2] | 58.8 / 56.0 [37.2, 70.8] |
| grip-close duration (cmd onset->0.5) | decisions | 27.6 / 14.0 [8.0, 20.0] | 21.0 / 13.0 [8.0, 21.8] |
| grip-close duration (onset->90% closed, measured) | decisions | 35.8 / 21.5 [13.2, 31.0] | 30.4 / 21.0 [14.2, 38.5] |
| closure -> pick | decisions | 57.7 / 33.0 [24.0, 51.0] | 44.8 / 34.5 [27.0, 52.0] |
| active (non-idle) decisions | decisions | 102 / 74.00 [53.00, 119] | 119 / 116 [86.25, 141] |
| tool speed over active decisions | m/s | 0.049 / 0.047 [0.039, 0.057] | 0.036 / 0.034 [0.030, 0.042] |
| mean \|delta\| over active decisions | cap units | 0.169 / 0.166 [0.135, 0.197] | 0.126 / 0.121 [0.096, 0.154] |
| arrival at grasp pose | decisions | 126 / 77.50 [46.25, 134] | 53.43 / 46.00 [30.25, 69.75] |
| dwell: arrival -> cmd>0.5 | decisions | 32.18 / 26.00 [15.00, 35.50] | 26.38 / 23.50 [12.25, 33.00] |
| dwell: arrival -> grip onset | decisions | 4.576 / 9.000 [2.250, 16.75] | 5.414 / 7.000 [3.000, 18.50] |
| tool-can xy distance 8 dec before arrival | m | 0.048 / 0.038 [0.026, 0.062] | 0.045 / 0.037 [0.022, 0.060] |
| z settle arrival -> closure | m | 0.013 / -0.000 [-0.001, 0.007] | 0.017 / 0.000 [-0.001, 0.018] |
| approach height (mean eef z, 8 dec before arrival) | m | 0.039 / 0.029 [0.017, 0.042] | 0.041 / 0.027 [0.016, 0.042] |
| eef z at arrival | m | 0.036 / 0.024 [0.015, 0.039] | 0.038 / 0.026 [0.016, 0.037] |
| eef z at closure | m | 0.023 / 0.020 [0.014, 0.029] | 0.022 / 0.019 [0.014, 0.028] |
| eef z above can centre at closure | m | -0.078 / -0.082 [-0.087, -0.072] | -0.080 / -0.083 [-0.088, -0.073] |
| xy travel in the 8 dec before arrival | m | 0.035 / 0.032 [0.015, 0.047] | 0.035 / 0.032 [0.016, 0.055] |
| descent in the 8 dec before arrival | m | 0.007 / 0.000 [-0.000, 0.001] | 0.006 / 0.000 [-0.001, 0.007] |
| approach dip angle | deg | 9.477 / 0.556 [-0.051, 6.578] | 8.367 / 0.643 [-0.653, 17.29] |
| approach heading vs can | deg | 17.40 / 8.148 [3.394, 16.15] | 19.17 / 8.116 [3.783, 23.42] |
| approach bearing (world) | deg | circ-mean -174 | circ-mean -162 |
| tool-can xy offset at closure | m | 0.018 / 0.011 [0.006, 0.015] | 0.015 / 0.010 [0.005, 0.014] |
| can xy displacement at closure | m | 0.021 / 0.018 [0.005, 0.030] | 0.012 / 0.007 [0.003, 0.017] |
| max can xy displacement before closure | m | 0.022 / 0.019 [0.005, 0.030] | 0.013 / 0.007 [0.003, 0.019] |
| nudged (>5 mm) | frac | 0.727 / 1.000 [0.000, 1.000] | 0.621 / 1.000 [0.000, 1.000] |
| pick-frame tool height | m | 0.096 / 0.098 [0.089, 0.103] | 0.091 / 0.093 [0.078, 0.099] |
| pick-frame can z | m | 0.177 / 0.178 [0.174, 0.181] | 0.171 / 0.172 [0.166, 0.179] |
| largest can-z drop after closure | m | 0.001 / 0.000 [0.000, 0.001] | 0.003 / 0.001 [0.000, 0.004] |
| decisions with \|delta\|==1 (any arm joint) |  | 0.039 / 0.030 [0.014, 0.060] | 0.024 / 0.013 [0.000, 0.041] |
| joint-decisions at \|delta\|==1 |  | 0.008 / 0.006 [0.003, 0.011] | 0.005 / 0.002 [0.000, 0.008] |
| decisions with \|delta\|>0.5 (any joint) |  | 0.171 / 0.158 [0.096, 0.213] | 0.194 / 0.183 [0.127, 0.262] |
| mean \|delta\| (arm) | cap units | 0.093 / 0.090 [0.061, 0.122] | 0.122 / 0.121 [0.093, 0.150] |
| grip cmd at pick |  | 0.798 / 0.816 [0.723, 0.880] | 0.783 / 0.817 [0.707, 0.857] |

## 4. Paired comparison on the 55 shared ICs (human − machine)

| descriptor | unit | human mean | machine mean | mean diff H-M [95% CI] | median diff | d_z | frac H>M | Wilcoxon p | perm p |
|---|---|---|---|---|---|---|---|---|---|
| tape length | decisions | 167.5 | 125.2 | 42.3 [22.0, 65.5] | 16.0 | +0.51 | 0.69 | 0.00018 | 0.00013 |
| duration | s | 22.33 | 16.69 | 5.646 [2.865, 8.715] | 2.133 | +0.51 | 0.69 | 0.00018 | 8e-05 |
| idle fraction (\|dq_cmd\| < 1e-3 rad) |  | 0.447 | 0.041 | 0.406 [0.374, 0.437] | 0.391 | +3.40 | 1.00 | <1e-15 | 5e-06 |
| idle fraction (\|delta\| < 1e-3 cap) |  | 0.264 | 0.000 | 0.264 [0.226, 0.302] | 0.225 | +1.81 | 0.98 | <1e-15 | 5e-06 |
| leading idle | decisions | 16.4 | 0.0 | 16.4 [11.0, 23.0] | 8.0 | +0.71 | 0.87 | 7.1e-15 | 5e-06 |
| tool path length | m | 0.537 | 0.543 | -0.007 [-0.046, 0.040] | -0.004 | -0.04 | 0.36 | 0.22 | 0.78 |
| net tool displacement | m | 0.144 | 0.140 | 0.004 [-0.002, 0.010] | 0.001 | +0.17 | 0.55 | 0.42 | 0.21 |
| tortuosity (path/net) |  | 4.155 | 4.436 | -0.281 [-0.756, 0.120] | -0.068 | -0.17 | 0.40 | 0.18 | 0.23 |
| mean tool speed | m/s | 0.028 | 0.035 | -0.006 [-0.009, -0.004] | -0.006 | -0.67 | 0.24 | 3.3e-06 | 5e-06 |
| peak tool speed | m/s | 0.156 | 0.139 | 0.017 [0.002, 0.036] | 0.002 | +0.27 | 0.60 | 0.12 | 0.041 |
| p95 tool speed | m/s | 0.104 | 0.104 | -0.000 [-0.006, 0.005] | 0.001 | -0.02 | 0.58 | 0.63 | 0.91 |
| jerk RMS | m/s^3 | 2.291 | 1.975 | 0.316 [0.091, 0.569] | 0.232 | +0.35 | 0.67 | 0.0062 | 0.0076 |
| time-to-grasp (cmd>0.5) | decisions | 126.8 | 79.7 | 47.2 [26.8, 70.1] | 21.0 | +0.57 | 0.82 | 4.7e-07 | 1e-05 |
| time-to-grasp | s | 16.91 | 10.62 | 6.291 [3.598, 9.375] | 2.800 | +0.57 | 0.82 | 4.7e-07 | 1e-05 |
| grip onset (cmd>0.05) | decisions | 108.0 | 59.1 | 48.9 [29.4, 71.4] | 21.0 | +0.61 | 0.84 | 1.3e-08 | 5e-06 |
| grip-close duration (cmd onset->0.5) | decisions | 18.8 | 20.5 | -1.7 [-4.9, 1.3] | 0.0 | -0.14 | 0.27 | 0.33 | 0.32 |
| grip-close duration (onset->90% closed, measured) | decisions | 27.0 | 30.0 | -3.0 [-6.4, 0.1] | 0.0 | -0.24 | 0.27 | 0.3 | 0.086 |
| closure -> pick | decisions | 39.7 | 44.5 | -4.8 [-9.4, -0.7] | -2.0 | -0.29 | 0.27 | 0.01 | 0.032 |
| active (non-idle) decisions | decisions | 85.24 | 118 | -32.80 [-43.84, -21.47] | -33.00 | -0.77 | 0.18 | 1.6e-08 | 5e-06 |
| tool speed over active decisions | m/s | 0.050 | 0.036 | 0.014 [0.011, 0.017] | 0.013 | +1.16 | 0.89 | 9.8e-12 | 5e-06 |
| mean \|delta\| over active decisions | cap units | 0.171 | 0.127 | 0.044 [0.033, 0.056] | 0.040 | +0.98 | 0.89 | 7.9e-10 | 5e-06 |
| arrival at grasp pose | decisions | 96.36 | 53.16 | 43.20 [25.05, 64.84] | 21.00 | +0.56 | 0.80 | 7.9e-08 | 5e-06 |
| dwell: arrival -> cmd>0.5 | decisions | 30.47 | 26.49 | 3.982 [-2.109, 11.33] | 0.000 | +0.16 | 0.49 | 0.28 | 0.28 |
| dwell: arrival -> grip onset | decisions | 11.67 | 5.982 | 5.691 [-0.873, 13.29] | 1.000 | +0.21 | 0.55 | 0.2 | 0.13 |
| tool-can xy distance 8 dec before arrival | m | 0.044 | 0.046 | -0.002 [-0.008, 0.003] | -0.001 | -0.10 | 0.38 | 0.27 | 0.46 |
| z settle arrival -> closure | m | 0.014 | 0.017 | -0.003 [-0.013, 0.004] | -0.000 | -0.10 | 0.40 | 0.22 | 0.48 |
| approach height (mean eef z, 8 dec before arrival) | m | 0.040 | 0.042 | -0.002 [-0.011, 0.006] | -0.000 | -0.06 | 0.45 | 0.34 | 0.68 |
| eef z at arrival | m | 0.036 | 0.039 | -0.003 [-0.011, 0.005] | -0.000 | -0.08 | 0.45 | 0.85 | 0.57 |
| eef z at closure | m | 0.023 | 0.022 | 0.001 [-0.001, 0.003] | 0.000 | +0.10 | 0.53 | 0.66 | 0.48 |
| eef z above can centre at closure | m | -0.079 | -0.080 | 0.001 [-0.001, 0.003] | 0.000 | +0.12 | 0.67 | 0.11 | 0.4 |
| xy travel in the 8 dec before arrival | m | 0.033 | 0.037 | -0.004 [-0.010, 0.002] | -0.000 | -0.17 | 0.42 | 0.41 | 0.22 |
| descent in the 8 dec before arrival | m | 0.007 | 0.006 | 0.000 [-0.004, 0.005] | -0.000 | +0.03 | 0.44 | 0.51 | 0.85 |
| approach dip angle | deg | 10.16 | 8.121 | 2.042 [-3.584, 8.409] | -0.120 | +0.09 | 0.45 | 0.63 | 0.52 |
| approach heading vs can | deg | 18.91 | 17.85 | -1.806 [-9.671, 5.259] | 0.897 | -0.07 | 0.58 | 0.85 | 0.66 |
| approach bearing (world) | deg | -170 | -166 | 15.09 [1.808, 29.26] | 0.342 | +0.29 | 0.56 | 0.14 | 0.034 |
| tool-can xy offset at closure | m | 0.016 | 0.016 | 0.000 [-0.001, 0.002] | 0.000 | +0.09 | 0.62 | 0.12 | 0.49 |
| can xy displacement at closure | m | 0.019 | 0.012 | 0.006 [0.002, 0.011] | 0.000 | +0.37 | 0.55 | 0.047 | 0.0063 |
| max can xy displacement before closure | m | 0.019 | 0.013 | 0.007 [0.002, 0.011] | 0.000 | +0.39 | 0.56 | 0.032 | 0.0041 |
| nudged (>5 mm) | frac | 0.709 | 0.618 | 0.091 [-0.018, 0.200] | 0.000 | +0.21 | 0.15 | 0.21 | 0.23 |
| pick-frame tool height | m | 0.097 | 0.091 | 0.006 [0.003, 0.009] | 0.005 | +0.48 | 0.76 | 0.00024 | 0.00075 |
| pick-frame can z | m | 0.177 | 0.171 | 0.006 [0.004, 0.009] | 0.005 | +0.72 | 0.75 | 1.7e-06 | 5e-06 |
| largest can-z drop after closure | m | 0.001 | 0.003 | -0.002 [-0.003, -0.001] | -0.000 | -0.34 | 0.27 | 0.00038 | 0.00073 |
| decisions with \|delta\|==1 (any arm joint) |  | 0.038 | 0.025 | 0.014 [0.007, 0.021] | 0.008 | +0.51 | 0.67 | 0.00048 | 0.00025 |
| joint-decisions at \|delta\|==1 |  | 0.008 | 0.005 | 0.003 [0.001, 0.004] | 0.002 | +0.52 | 0.69 | 6e-05 | 0.00016 |
| decisions with \|delta\|>0.5 (any joint) |  | 0.172 | 0.197 | -0.025 [-0.050, -0.001] | -0.009 | -0.27 | 0.38 | 0.075 | 0.053 |
| mean \|delta\| (arm) | cap units | 0.095 | 0.123 | -0.028 [-0.038, -0.018] | -0.026 | -0.71 | 0.20 | 1.6e-06 | 5e-06 |
| grip cmd at pick |  | 0.795 | 0.779 | 0.016 [-0.006, 0.037] | 0.018 | +0.19 | 0.71 | 0.0036 | 0.16 |

Cross-set consistency per IC (Spearman of the human value against the machine value over the 55 uids): tape length
ρ = 0.75, time-to-grasp 0.68, max can displacement 0.54, approach bearing 0.37 — the IC sets much of the per-uid
structure and the DP inherits it. Human/machine length ratio: median 1.15, IQR [0.98, 1.60], human longer in 69% of pairs.

## 5. Gallery (12 matched ICs) and figures

`~/wm_fix_2026-09-03/characterization/videos/uid<u>_human_L_machine_R.mp4` — human (dHv2raw) left, machine (dDP)
right; top view above the wrist view, the stored 64×64 rig frames upscaled 4× nearest-neighbour, 7.5 fps (real time);
header = uid, set, `decision k/n`, `grasp @t_g`; a red border + "GRASP" for four frames from the closure decision
(first grip cmd > 0.5); the shorter tape freezes on its final (picked) frame with a green border + "PICKED"; 12 frames
held at the end. Selection rule: the 55 paired ICs sorted by human/machine length ratio, 12 at even quantiles (so the
gallery spans "machine longer" to "human much longer"; it is not a cherry-pick of either). uid: human n / machine n
(closure decision) — 247: 194/104 (166/72); 250: 480/135 (435/82); 252: 206/227 (101/60); 257: 96/81 (62/46);
262: 97/48 (81/30); 273: 125/86 (94/55); 276: 182/137 (158/111); 279: 212/199 (18/24); 280: 56/104 (39/75);
297: 88/89 (60/59); 298: 84/118 (58/95); 329: 91/80 (64/56).

`fig_descriptors_dDP_vs_dHv2raw.png` — violins + every tape as a point for 21 descriptors, human orange / machine
purple (paper source colours, colstyle.py), titles carry the paired Wilcoxon p and d_z. `fig_xy_paths_gallery.png` —
top-down time-coloured tool xy paths for the 12 gallery ICs, both sets on one axis per uid, with the can path (dashed),
the closure decision (×), the can at t0 (circle) and the goal (star): the machine retraces the human's route almost
exactly on every IC except the long human wanders (250: the operator tours the workspace for 60 s before the pick).

The file-sending tool was disabled in the session that produced this note; the videos and PNGs sit at the paths
above (2.2 MB of mp4 in total) for the coordinating session to forward.

## 6. Summary

1. **Idle time is the dominant difference (d_z = 3.4).** 45% of raw-human decisions are zero-motion (< 1e-3 rad
   commanded step) vs 4% for the machine; 15.2 s of stillness per human tape vs 0.8 s. Only ~7% of human decisions are
   the leading block (16 of 217) — most pauses are mid-tape, after the approach and around the closure — so leading-idle
   pruning (dHpruned) removes a minority of the idle mass.
2. **When the human moves it moves faster and in bigger steps (d_z ≈ +1).** Active-decision tool speed 0.050 vs
   0.036 m/s (+40%), active mean |delta| 0.171 vs 0.127 cap units, and the human needs fewer moving decisions to reach
   the pick (85 vs 118, d_z −0.77). Whole-tape path length, net displacement and tortuosity are identical (0.54 m,
   0.14 m, ≈4.3; p ≥ 0.18): the two sets drive the same route with different time structure.
3. **Time-to-grasp: +6.3 s [3.6, 9.4] for the human (d_z 0.57; median +2.8 s)**, entirely from the wait before the grip
   starts closing (grip onset +49 decisions); closing itself is the same (19 vs 21 decisions cmd, 27 vs 30 measured,
   p ≥ 0.3), the dwell at the grasp pose is the same (30 vs 26, p 0.28).
4. **Approach geometry is the same** — height, heading (median 8° off the tool→can line in both), dip (near-horizontal
   arrival), world bearing (from the −x/base side in both, paired circular difference 15° [2, 29], p 0.14) and tool–can
   offset at closure (16 mm both). Any student difference is not in where or how the tool reaches the can.
5. The human nudges the can more before grasping (max pre-grasp displacement 19 vs 13 mm, d_z 0.39; > 2 cm in 31/66
   human vs 14/58 machine tapes) and lifts it 6 mm higher by the terminating row (can z 0.177 vs 0.171, d_z 0.72) —
   the faster lift overshoots the 0.1505 threshold further. The machine's lift is slightly slower after closure (+4.8
   decisions, p 0.01) with larger can-z sags (3 vs 1 mm, d_z −0.34; > 5 mm in 8/58 vs 5/66), i.e. a less secure first lift.
6. Jerk is higher for the human (2.29 vs 1.98 m/s³, d_z 0.35, p 0.006) — a consequence of the stop-go structure, not
   of higher peak speeds (peak 0.156 vs 0.139, p 0.12; p95 equal).
7. **Saturation is rare in both sets**: 3.8% of human vs 2.5% of machine decisions clip an arm joint at |delta| = 1
   (0.8% vs 0.5% of joint-decisions, d_z 0.5); 18/58 machine tapes never clip vs 9/66 human. The cap distorts neither set
   materially; it bites the human's fast moves slightly more.
8. For a learner: a raw-human tape presents an ambiguous target — from visually in-motion states the label is "stop"
   45% of the time — consistent with DP degrading on raw human tapes while RLPD/WM tolerate them (RESULTS, A31); the
   machine tapes are steadier (smaller, continuous steps, no pauses, 4 pp fewer clipped decisions) and hence
   lower-variance regression targets with the same route coverage.
9. The 11 human-only ICs are the hardest ones (median 468 decisions, idle 0.51): they are in every human-arm training
   set but have no machine counterpart, so full-set human-vs-machine learner contrasts carry 11 long, pause-heavy
   tapes that the paired analysis above excludes.
10. Counts: 66 human / 58 machine tapes, 55 paired ICs; 12 videos; all descriptors and p-values in `paired_table.md` /
    `stats.json`; nothing in this note required a simulator call.
