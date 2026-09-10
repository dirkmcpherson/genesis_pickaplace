# Trial196: retained regression and real-image review

All four new configurations fail the supported push-to-contact sequence; the
original strict sequence completes. Soft damping2 ends with45.18mm surface gap,
versus original approximately-0.13mm. Original moves the goal26.50mm, versus
10.85–12.44mm across new conditions. Initial shelf overlap is inherited and
retained, so this is not a clean material calibration.

`inspect_real.py` extracted seven selected times from both real cameras. Maximum
timestamp mismatch16.89ms. At53.58s the real can is upright on the shelf with the
hand withdrawn; at60.63s the cans visibly meet during the final hand motion.
Both images and the full seven-time montage were visually inspected. This
supports retaining the lost contact sequence as a regression; it does not
establish numerical real-world pose error or that the original goal displacement
is physically correct. No registered camera overlay, source edit or pose fit.

`real_release_slide_montage.jpg` and individual full-resolution images retain
simulated can height, surface gap and contact counts for original/soft damping2.
Hashes and exact camera-frame matches are in`real_frame_matches.json`.
