# Upright initial-pose correction probe

Real frame-zero inspection confirms upright cans for both 234 and 318 (see
real_start_frames.jpg, with 233 as reference). Their archived poses instead specify
90-degree tilt at z=0.085 m. This development probe sets upright quaternion and
standard upright center z=0.113 m, retaining archived xy and all source commands.
The z value follows existing simulator geometry, not a new video height measurement.
Frozen placements remain unchanged. No og4, added push, or terminal hold is used.

Both corrected starts permit pickup; both archived-pose EEF controls failed pickup.
Neither corrected episode achieves supported upright release or the complete slide.
Full traces and summary.json retain both failures. This fixes a demonstrated initial
orientation error but does not establish complete recovery, accurate xy, or calibrated
finger/contact physics. Source arrays were checked identical to census inputs.
No episode is admitted to the complete bank; no independent replay or sim-video
review has yet been performed for these failed probes.
