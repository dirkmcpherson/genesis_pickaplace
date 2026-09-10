# Completed expanded soft-pad comparison

All 144 frozen full replays completed and passed the source, action, geometry and normal-damping checks. The sample contains 36 additional recordings, 12 per day, beyond the earlier 12-demo experiment. Both early days retain their mount yaw corrections (−9.7° and −19.2°). No configuration is adopted.

| Configuration | Supplied slide metric | Complete sequence | Median final center distance |
|---|---:|---:|---:|
| original_fixed | 6/36 | 3/36 | 181.7 mm |
| rigid_g1 | 2/36 | 1/36 | 165.6 mm |
| soft_g1 | 6/36 | 1/36 | 133.8 mm |
| rigid_g2 | 4/36 | 1/36 | 145.9 mm |
| soft_g2 | 7/36 | 1/36 | 142.7 mm |

Soft1 adds four supplied-metric passes and loses four versus the original. Soft2 adds five and loses four. Both gain strict completion on262 but lose196,225 and232. Every new condition, including both rigid controls, completes262; there is no softness-specific strict gain. Softness does improve the supplied metric over the same-hand rigid controls (soft1 six versus two; soft2 seven versus four), which is retained as partial progress rather than an e2e fidelity claim.

The final additional metric gain,191 soft2, ends68.89mm from the goal, but moves the goal178.10mm and never establishes the supported-release sequence. Its original reference also fails (228.89mm final distance). This is not evidence that the demonstrated final slide was faithfully reconstructed. Thresholds stay unchanged.

| Day | Original metric / sequence | Soft1 metric / sequence | Soft2 metric / sequence |
|---|---:|---:|---:|
| 12-16 | 1/12 · 0/12 | 1/12 · 0/12 | 1/12 · 0/12 |
| 12-17 | 2/12 · 2/12 | 0/12 · 0/12 | 1/12 · 0/12 |
| 12-18 | 3/12 · 1/12 | 5/12 · 1/12 | 5/12 · 1/12 |

The improved models establish supported release more often, but the final supported push remains the main failure after release: soft2 has23 no-supported-push failures,8 no-supported-release failures,3 not-picked cases,1 lost final retention and1 completion. Original counts are11,19,3,0 and3 respectively. Better intermediate behavior does not by itself satisfy the full task.

Five inherited can/shelf overlaps remain included and flagged. Selection used fixed SHA256 ordering without reading outcomes; these are historical development source pools, not pristine unseen data or independent participants. The final distances and medians use all36 recordings, including physical failures. Neither lower median distance nor a proximity pass establishes hardware fidelity.

The existing262 real/sim video and independent action verification remain available in `262_review/`; the196 regression has clock-matched real images in `196_review/`. Numerical solver/precision sensitivity is a separate unresolved issue. See `summary.json` for every UID, paired comparison, unchanged scorer output and provenance hash.
