# December 18 timestamp reconstruction: completed collection

All 74 declared replays finished, with no worker failures. The unchanged conservative
sequence scorer reports **10 complete**, compared with **8** in the matched fixed-clock
controls. New completions **237 and 317** passed independent saved-EEF-action replay
bit-for-bit and selected-phase visual review. They are separately packaged timestamp
reconstructions. No og4, added slide motion or terminal hold is used.

| Outcome | Matched fixed clock | Timestamp reconstruction |
|---|---:|---:|
| Complete | 8 | 10 |
| No pickup | 3 | 4 |
| No upright supported release | 27 | 27 |
| No qualifying supported push/contact | 35 | 33 |
| Final arrangement not retained | 1 | 0 |

The matched control column uses upright-corrected initial poses for 234 and 318,
just like the timestamp run. The original archived-IC census instead had five
pickup failures and 25 release failures. Those IC corrections do not explain either
new complete sequence. Archived horizontal positions are retained throughout.

All eight previously complete sequences remain numerically complete. Four other
trials (308, 309, 333, 335) gain upright release but still fail the slide. Four
(244, 259, 297, 298) lose upright release and 246 loses pickup. Thus the extra two
completions do not imply uniformly improved grasp or placement behavior.

Timestamp reconstruction also interpolates measurements onto a 30 ms grid. It
therefore changes timing and sampled path together. Round-trip interpolation errors,
window timestamp limitations and source corrections are documented in README.md;
this is not a calibrated real-world fidelity claim. The pool is development data,
not a held-out estimate, and it is distinct from the early December 16/17 corpus.

Independent verification of every numerical pass is tracked in each verification
folder; final_collection_validation.json records the currently completed checks.
Only the two new sequences have been newly visually reviewed in this collection.
The broader recovery objective remains open: 64 of these 74 still fail the full rule.
The early-day pool and stratified compliance diagnostic continue separately.

All ten numerically complete traces have now passed exact independent saved-action
replay (trajectory, actions and observations). Only 237 and 317 have new visual
review and separate timestamp-bank admission in this pass.
