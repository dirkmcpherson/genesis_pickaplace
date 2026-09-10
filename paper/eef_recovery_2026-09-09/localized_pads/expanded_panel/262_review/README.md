# Newly recovered262: real/sim review, attribution still limited

All four new configurations complete the strict sequence and supplied slide
metric; original fixed reconstruction fails the supported push-to-contact sequence
and ends0.3666m from the goal. Soft1 ends0.065995m from the goal with0.398mm goal
movement. Because matching rigid controls also succeed, this is not evidence that
extra normal softness caused the gain.

`262_soft1_real_sim.mp4` covers all29.73s,166 decoded frames, with two real
cameras and saved simulation poses. Every frame decoded and trace/video hashes
match. Maximum camera timestamp mismatch15.22ms /20.21ms. Selected grasp,
release, push and endpoint frames were visually reviewed. The upright supported
slide and withdrawal are visible, with differences in hand seating/viewpoint.
No fitted camera overlay or numerical real-world trajectory-error claim.

Independent saved-EEF-action replay is complete: all14 saved arrays match exactly
over991 decisions. Numerical/contact checks pass and the strict sequence completes
again (`action_verification/verification.json`). Broader adoption remains unproven.
