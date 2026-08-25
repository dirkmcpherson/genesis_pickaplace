# Canonical results table (regenerate: analysis/results_table.py)

Smoke runs (seed 0) excluded. World from the seed block: 10-14 old (matched_v2), 20-29 corrected (matched_w3).

| learner | world | reward | arm | n | hold | rnd | per-seed hold |
|---|---|---|---|---|---|---|---|
| DP | corrected | - | dDP | 5 | 0.85 | 0.49 | 0.87 0.93 0.80 0.93 0.73 |
| DP | corrected | - | dH | 5 | 0.91 | 0.55 | 0.87 0.87 0.93 0.93 0.93 |
| RLPD | corrected | dense | dDP | 4 | 0.08 | 0.07 | 0.20 0.00 0.00 0.13 |
| RLPD | corrected | dense | dH | 4 | 0.22 | 0.15 | 0.00 0.67 0.00 0.20 |
| RLPD | corrected | sparse | dDP | 4 | 0.20 | 0.13 | 0.27 0.13 0.33 0.07 |
| RLPD | corrected | sparse | dH | 4 | 0.18 | 0.10 | 0.13 0.13 0.27 0.20 |
| RLPD | old | dense | dDP | 4 | 0.17 | 0.14 | 0.00 0.00 0.27 0.40 |
| RLPD | old | dense | dH | 4 | 0.15 | 0.07 | 0.13 0.00 0.13 0.33 |
| RLPD | old | dense | dR2D | 4 | 0.18 | 0.09 | 0.27 0.33 0.07 0.07 |
| RLPD | old | sparse | dDP | 4 | 0.25 | 0.14 | 0.20 0.07 0.40 0.33 |
| RLPD | old | sparse | dDPfails | 4 | 0.25 | 0.17 | 0.13 0.27 0.33 0.27 |
| RLPD | old | sparse | dH | 4 | 0.42 | 0.31 | 0.27 0.73 0.53 0.13 |
| RLPD | old | sparse | dR2D | 4 | 0.55 | 0.26 | 0.53 0.93 0.27 0.47 |
| RLPD | old | sparse | dR2DDPfails | 4 | 0.15 | 0.07 | 0.20 0.07 0.33 0.00 |
