# Limited soft compression: completed calibration

All seven declared replays completed. The same critical-return hand and 3 mm
material region were retained; only the travel over which the softer stiffness
acts was limited. Beyond that travel the original incremental stiffness returns.
This is not a hard stop or measured rubber strain.

| Demo | Soft travel | Final center distance, mm | Strict sequence |
|---|---:|---:|---|
| 233 | 0 mm | 65.84 | Complete |
| 233 | 0.5 mm | 65.92 | Complete |
| 233 | 1 mm | 65.85 | Complete |
| 113 | 0.5 mm | 313.16 | Not picked |
| 113 | 1 mm | 223.28 | Not picked |
| 184 | 0.5 mm | 193.93 | Not picked |
| 184 | 1 mm | 171.47 | Not picked |

Only 233 passes the unchanged supplied metric. Its zero-travel control reproduces
every saved array of `critical_return/233_tc0.02` exactly and reports zero softened
contacts. All seven runs preserve source action arrays and satisfy the callback
count checks. `readout.py` regenerates `summary.json` from terminal records.

Neither nonzero setting recovers either early calibration pickup; no candidate
was advanced to validation or adopted. Final proximity in the already successful
233 does not establish improvement over the original recovery. No real-image
comparison has been performed for these new limited-travel traces.
