# Conditional image-derived placement: better seating, no full recovery

All three full trial 113 replays are complete. Neither supplied-metric nor strict
end-to-end success occurs. The same-engine rigid hand gains upright supported
shelf release, but fails the subsequent supported push. The soft hand still
loses the can during release. No configuration or initial pose is adopted.

The initial XY is the nominal unheld-can rim fit, declared before any of these
outcomes: [0.433524, -0.154337] m, or [17.24, -5.41] mm relative to the archived
placement. Camera extrinsics use rigid cap landmarks only; the rim fit assumes
the current table plane and object dimensions. Declared camera, annotation and
table-height sensitivity spans x=0.42456–0.44288 m and y=-0.15915–-0.14909 m.
These are sensitivity scenarios, not confidence intervals or measured ground
truth. The original placement was outcome-fitted and is not ground truth either.

The copied source NPZ is byte-identical to the archive. Only its sidecar initial
XY differs; reset z=0.113 m, arm/grip commands, timestamps, early-day yaw, shelf,
goal and scoring remain fixed. All runs contain all 4,187 source decisions,
without extensions or holds. `readout.py` verifies these invariants, parameter
hashes and fresh feedback. `plan.json` and `executed_sources/` preserve provenance.

| Condition | Strict failure | Final center distance | Final goal displacement |
| --- | --- | ---: | ---: |
| Original fixed coupling, pinned engine | No supported release | 0.1066 m | 64.49 mm |
| Adaptive rigid pads, Genesis 1.4 elliptic/ratio 10 | No supported push to contact | 1.2677 m | 1,092.68 mm |
| Same adaptive hand, soft pads | No supported release | 26.0095 m | 60.80 mm |

Large final distances are retained failed trajectories, not discarded numerical
outliers. The original fixed hand is a cross-engine reference; only the rigid/
soft pair isolates normal pad response within the same engine and placement.
The unchanged supplied predicate labels release before pickup in all three;
its final arrival condition fails. Strict sequence evidence remains separate.

Image projections show a substantial seating improvement over archived soft
113: cap-relative rim error at 8/10/12/14/18/20.04 s changes from
24.3/23.3/22.4/20.7/29.4/21.9 px to 6.1/5.7/6.5/8.8/15.6/7.0 px with rigid pads,
and 4.5/7.5/9.0/11.5/15.4/6.7 px with soft pads. Both overlays were inspected.
The camera was fitted to caps rather than these rim checkpoints; 18 s has a
partly occluded cap and larger calibration error. These projections compare
relative seating at the recorded wrist, removing arm tracking error. Softness
does not consistently beat rigid on this image measure.

Rigid 113 settles upright near [0.561, -0.189, 0.220] m by 21 s. Soft 113 instead
escapes backward by 20.34 s. The archived-placement force audit identified a
shelf-edge impulse, but that attribution cannot simply be transferred to this
changed-placement pair. `observe_release.py` completes a separate exact-reference
force observation: soft opening adds downward impulse, followed by a large
backward shelf impulse. Rigid retains more upward support while opening and
settles after impact. See `release_wrenches/README.md` for decomposition and
force-balance limitations.

`113_conditional_rigid_release_real_sim.mp4` shows 18.00–33.84 s beside two real
cameras. The video explicitly labels the conditional start, rigid pads and
full-task failure. All 89 frames decode, the video/trace/URDF hashes are recorded,
and release/settling/end stills were visually inspected. Maximum camera timing
errors are 17.93/20.77 ms. Rendering replays saved poses without stepping physics;
viewpoints differ. The full125.61s source continues beyond this clip.
