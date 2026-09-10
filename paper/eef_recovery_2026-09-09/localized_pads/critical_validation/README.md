# Expanded frozen validation — complete; candidate rejected

All nineteen declared jobs completed: nine paired full-source comparisons and
one independent saved-action verification. The verification of 233 reproduces
every saved array exactly. Neither source commands nor scoring thresholds changed.

| Model | Supplied metric | Strict sequence |
|---|---:|---:|
| original_fixed | 3/9 | 2/9 |
| matching_rigid | 2/9 | 2/9 |
| soft_candidate | 1/9 | 1/9 |

The soft setting provides no new complete recoveries and regresses 237 and 224
relative to the original metric. It is rejected as a general replacement.

| Demo | Final distance original / rigid / soft, mm | Final goal shift original / rigid / soft, mm |
|---|---|---|
| 176 | 65.84 / 65.93 / 65.85 | 7.6 / 5.5 / 3.8 |
| 185 | 108.27 / 161.00 / 113.31 | 7.8 / 60.5 / 14.1 |
| 237 | 66.55 / 65.73 / 81.26 | 4.3 / 46.4 / 57.0 |
| 156 | 127.86 / 391.85 / 181.87 | 0.0 / 1.7 / 1.2 |
| 181 | 144.40 / 150.08 / 150.49 | 0.0 / 0.1 / 1.3 |
| 224 | 65.87 / 111.83 / 515.85 | 86.8 / 59.4 / 477.2 |
| 198 | 181.86 / 140.16 / 125.67 | 0.0 / 0.4 / 0.8 |
| 243 | 189.06 / 258.65 / 226.89 | 0.0 / 0.7 / 1.7 |
| 246 | 845.35 / 218.09 / 195.89 | 0.0 / 1.3 / 0.8 |

Goal motion matters: soft 224 moves the goal about 477 mm; soft 237 about
57 mm. Matching rigid 237 also moves it about 46 mm despite passing the strict
sequence. A score pass alone does not establish faithful goal interaction.

Selection was frozen before these candidate outcomes: old reserved 176/185/237
plus six source-closure-stratified cases 156/181/224/198/243/246. Historical
baseline outcomes were available, so this is not pristine unseen validation.
See selection.json for seed, strata, exclusions and source hashes. These nine
selected recordings do not support population recovery-rate or causal day claims.

The calibration review ../233_critical_soft_pad_real_sim.mp4 illustrates improved
loaded seating in 233. It makes contact during the supported push but settles
with a 1.405 mm gap and zero final solver goal contacts, within the strict 2 mm
ending tolerance. That illustrative success did not transfer to this panel.

The mass and return-dynamics bench is a model consistency check, not a hardware
calibration. Contact-point multiplication and impedance already have negative
full-task evidence (confound 49); do not assume either solves the current hand.
No torsional-friction treatment was tested in this batch.
