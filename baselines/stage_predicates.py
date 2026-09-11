#!/usr/bin/env python3
"""Stage predicates for the unified end-to-end ladder (LADDER_UNIFY_BRIEF_2026-09-10, D3/D4).

PURE NUMPY. No Genesis import, no simulator, no settle. Everything here is a function of a short
history of poses, contact booleans and the two env-owned stage flags (`picked`, `placed_v2`), so
the same code runs inside the training env, inside both evaluators, inside the demo relabel, and
offline over a recorded episode. That is the point: one definition, four call sites.

Why this module exists
----------------------
Three predicates that shared a name did not share a meaning (E2E_AUDIT_BRIEF_2026-09-10 §4):

* `nested_proxy` = contact AND grip cmd < 0.3 AND both cans upright, read on a single frame.
  Measured against the settled predicate over 540 {RLPD} episodes its precision is 0.114 (human
  arm) / 0.029 (machine arm) and it REVERSES the human-vs-machine ordering. It is not merely
  noisy; its error is arm-dependent.
* `nested_honest` = the same clauses after a 100-step post-episode settle. Correct, but it
  simulates, so it can only ever be a post-episode column -- it can never gate a reward.
* `slide_success` in the env is still amendment (l) (`grip_cmd < 0.3`), which amendment (p)
  withdrew after measuring that it passes 2 of 74 human demonstrations: people release fully,
  re-close the fingers to ~0.4 and push the can home with a fist.

`nested_v2` here is the settled predicate's clauses made readable WITHOUT simulating, by
replacing "let the world settle and see if it stays" with two state facts that a trajectory
already carries: the can is not in the hand, and the can is not moving. Neither needs a gripper
term, which is the clause that broke every earlier definition.

The predicates
--------------
`in_hand`      |tool_xy - can_xy| < HELD_LEVER_M.  NO gripper term. A held can sits one grasp
               lever from the tool point (SLIDE_ANATOMY_2026-09-07: median 1.5 cm at set-down);
               a fist pushing the can contacts its SURFACE, so the tool point cannot be closer
               than the can radius, 3.3 cm. The default 2.5 cm sits between the two populations.
               Lane-1 calibration: see paper/NESTED_V2_PREDICATE_2026-09-10.md.
`at_rest`      the can's xy position varied by <= AT_REST_MM across the last AT_REST_FRAMES env
               frames (12 frames = 3 decisions at action_repeat 4). Requires a full window:
               before AT_REST_FRAMES updates exist it is False, never vacuously True.
`released`     `placed_v2` has been granted at some point (the env owns that flag). Sticky.
`pushed`       goalward progress of >= PUSH_GAIN_M accumulated over the frames AFTER the first
               `placed_v2` grant on which the can is NOT in hand. See `goalward_gain_m` below
               for the exact accumulation rule.
`contact_push` `placed_v2` granted AND can<->goal contact AND the tool is on the FAR side of the
               can along the can->goal line (xy dot < 0) AND no gripper<->goal contact. Sticky.
               Differs from the env's legacy `contact_push` in one clause only: the precondition
               is `placed_v2`-granted (release first), not `picked` -- the change that stops the
               rung paying for pressing a HELD can against the goal.
`nested_v2`    picked AND placed_v2-granted AND dist_xy(can, goal) <= NESTED_TOUCH_DIST AND
               tilt(can) < TILT_MAX_DEG AND tilt(goal) < TILT_MAX_DEG AND the can's z is in the
               shelf resting band AND not in_hand AND at_rest.
`slide_success` released AND pushed AND nested_v2.  Sticky (it is the terminal rung).

Stickiness, exactly
-------------------
STICKY (latch True and never clear): `released`, `pushed`, `contact_push`, `slide_success`.
INSTANTANEOUS (recomputed every frame): `in_hand`, `at_rest`, `nested_v2`.

`nested_v2` is deliberately NOT sticky: "the can is resting nested" is a statement about the
state now, and a can that is nested and then picked back up is not nested. The sticky
"it happened at some point" reading is available as `nested_v2_ever` for episode-level columns.
`slide_success` IS sticky because D2 makes it the terminal rung: it is paid once, at the frame
it first becomes true.

`grip_cmd` is accepted by `update()` and echoed as a diagnostic. It is used by NO predicate here.
That is the whole lesson of amendments (l)/(p)/(x): release is a fact about where the can's
weight is, not about the hand.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

__all__ = [
    'NESTED_TOUCH_DIST', 'AT_REST_MM', 'AT_REST_FRAMES', 'PUSH_GAIN_MM', 'HELD_LEVER_M',
    'TILT_MAX_DEG', 'BAND_LO_M', 'BAND_HI_M', 'FLAG_KEYS', 'STICKY_KEYS',
    'tilt_deg', 'StageTracker',
]

# ---- constants of record (all overridable per instance; see StageTracker.__init__) ------------
NESTED_TOUCH_DIST = 0.081   # m. can diameter 0.066 + 15 mm noise floor. replay_harness value.
AT_REST_MM = 2.0            # mm. can xy spread that still counts as stationary.
AT_REST_FRAMES = 12         # env frames (3 decisions at action_repeat 4) the spread is read over.
PUSH_GAIN_MM = 10.0         # mm of goalward progress, after release, that counts as a push.
HELD_LEVER_M = 0.025        # m. tool-to-can-centre distance below which the can is in hand.
TILT_MAX_DEG = 20.0         # deg. "upright", both cans. Same value the settled predicate uses.
BAND_LO_M = 0.01            # m above shelf_top_z: bottom of the resting band (full_env's band).
BAND_HI_M = 0.07            # m above shelf_top_z: top of the resting band.

FLAG_KEYS = ('in_hand', 'at_rest', 'released', 'pushed', 'contact_push', 'nested_v2',
             'slide_success')
STICKY_KEYS = ('released', 'pushed', 'contact_push', 'slide_success')


def tilt_deg(quat):
    """Angle in degrees between the body z axis and world z. COPIED from
    can_pos_recovery/replay_harness.tilt_deg (same arithmetic, same 1e-9 guard, same clip) so
    that this module needs no Genesis-side import. Quaternion order is (w, x, y, z), which is
    what Genesis `get_quat()` returns and what every caller in this repo passes."""
    w_, x, y, z = [float(v) for v in quat]
    zz = 1 - 2 * (x * x + y * y)
    zx = 2 * (x * z + w_ * y)
    zy = 2 * (y * z - w_ * x)
    n = (zx * zx + zy * zy + zz * zz) ** 0.5 + 1e-9
    c = max(-1.0, min(1.0, zz / n))
    return float(math.degrees(math.acos(c)))


def _xy(p):
    a = np.asarray(p, dtype=np.float64).reshape(-1)
    return a[:2]


class StageTracker:
    """One instance per episode. Call `update()` once per ENV FRAME, after the sim step.

    `goal_xy` is the nominal (static) goal used only when a frame supplies no `goal_pos`; when
    `goal_pos` is given it wins, because the goal can is a free body and does get bumped.
    `shelf_top_z` is the WORLD's shelf top (full_env.shelf_top_z), not the base-world constant --
    the resting band is [shelf_top_z + BAND_LO_M, shelf_top_z + BAND_HI_M], the same band
    full_env's `placed_v2` uses.
    """

    def __init__(self, goal_xy, shelf_top_z, *,
                 nested_touch_dist=NESTED_TOUCH_DIST,
                 at_rest_mm=AT_REST_MM,
                 at_rest_frames=AT_REST_FRAMES,
                 push_gain_mm=PUSH_GAIN_MM,
                 held_lever_m=HELD_LEVER_M,
                 tilt_max_deg=TILT_MAX_DEG,
                 band_lo_m=BAND_LO_M,
                 band_hi_m=BAND_HI_M):
        self.goal_xy = _xy(goal_xy).copy()
        self.shelf_top_z = float(shelf_top_z)
        self.nested_touch_dist = float(nested_touch_dist)
        self.at_rest_m = float(at_rest_mm) / 1000.0
        self.at_rest_frames = int(at_rest_frames)
        self.push_gain_m = float(push_gain_mm) / 1000.0
        self.held_lever_m = float(held_lever_m)
        self.tilt_max_deg = float(tilt_max_deg)
        self.band_lo_m = float(band_lo_m)
        self.band_hi_m = float(band_hi_m)
        assert self.at_rest_frames >= 1, self.at_rest_frames
        self.reset()

    # -- lifecycle ------------------------------------------------------------------------
    def reset(self):
        self.t = 0
        self._hist = deque(maxlen=self.at_rest_frames)   # can xy, one entry per env frame
        # sticky flags
        self.released = False
        self.pushed = False
        self.contact_push = False
        self.slide_success = False
        self.nested_v2_ever = False
        # instantaneous flags (last computed values; False before the first update)
        self.in_hand = False
        self.at_rest = False
        self.nested_v2 = False
        # frame stamps (env-frame index at which a sticky flag first fired; None = never)
        self.released_frame = None
        self.contact_push_frame = None
        self.slide_frame = None
        self.pushed_frame = None
        self.nested_v2_frame = None
        # push accumulator, see _update_push()
        self._banked_gain = 0.0
        self._run_ref = None     # dist at the start of the current not-in_hand run
        self._run_min = None     # smallest dist seen inside the current not-in_hand run
        self.goalward_gain_m = 0.0
        self.lever_m = float('nan')
        self.dist_xy_m = float('nan')

    # -- the push accumulator -------------------------------------------------------------
    def _update_push(self, dist, in_hand):
        """`pushed` = cumulative goalward progress after the first `placed_v2` grant, counted
        only on frames where the can is NOT in hand.

        Accumulation rule, stated precisely because "cumulative" admits two readings:

          The frames after the grant split into maximal RUNS of consecutive not-in_hand frames.
          Each run contributes max(0, dist at the run's first frame - the smallest dist inside
          the run).  Runs are summed.

        A single uninterrupted free push therefore gives exactly amendment (x)'s
        `dist[release] - min(dist[release:])`.  Picking the can back up ends a run and banks it,
        so a carry never counts AGAINST earned progress (that is what "only over frames where
        not in_hand" buys), and a second push after a re-place adds its own progress.

        The rejected reading is a per-frame ratchet (sum of every frame-to-frame decrease).
        It is not noise-safe: an episode is up to 1200 env frames, and summing only the negative
        half of a symmetric jitter of even 0.05 mm/frame manufactures 30 mm of "push" from a can
        that never moved. The run-wise net is bounded by the can's actual displacement.
        """
        if not self.released:
            return
        if in_hand:
            if self._run_ref is not None:
                self._banked_gain += max(0.0, self._run_ref - self._run_min)
                self._run_ref = self._run_min = None
            live = 0.0
        else:
            if self._run_ref is None:
                self._run_ref = self._run_min = dist
            else:
                self._run_min = min(self._run_min, dist)
            live = max(0.0, self._run_ref - self._run_min)
        self.goalward_gain_m = self._banked_gain + live
        if not self.pushed and self.goalward_gain_m >= self.push_gain_m:
            self.pushed = True
            self.pushed_frame = self.t

    # -- the one entry point ---------------------------------------------------------------
    def update(self, *, can_pos, can_quat, goal_pos=None, goal_quat=(1.0, 0.0, 0.0, 0.0),
               tool_xy, grip_cmd=None, picked=False, can_goal_contact=False,
               gripper_goal_contact=False, placed_v2=False):
        """Advance one env frame. All arguments keyword-only (the brief's interface).

        can_pos  (3,) can centre, world m.      can_quat  (4,) w,x,y,z.
        goal_pos (3,) or (2,) goal can centre; None -> the constructor's static goal_xy.
        goal_quat (4,) w,x,y,z; default upright (a fixed goal that cannot tip).
        tool_xy  (2,) the TOOL point (genv.tool_pos()[:2]), NOT the wrist link -- the wrist sits
                 ~0.145 m behind the tool and a wrist-based lever test would call every held can
                 "not in hand".
        grip_cmd commanded grip 0..1. Diagnostic only; no predicate reads it.
        picked / placed_v2  the env's own sticky stage flags.
        can_goal_contact / gripper_goal_contact  solver contact booleans for THIS frame.

        Returns a dict with FLAG_KEYS plus diagnostics.
        """
        can = np.asarray(can_pos, dtype=np.float64).reshape(-1)
        can_xy = can[:2]
        gxy = self.goal_xy if goal_pos is None else _xy(goal_pos)

        dist = float(np.hypot(can_xy[0] - gxy[0], can_xy[1] - gxy[1]))
        lever = float(np.hypot(_xy(tool_xy)[0] - can_xy[0], _xy(tool_xy)[1] - can_xy[1]))
        in_hand = bool(lever < self.held_lever_m)

        # at_rest: full window only. `maxlen` deque, so the window is the last at_rest_frames
        # samples INCLUDING this one; spread is measured against the current position.
        self._hist.append(can_xy.copy())
        if len(self._hist) < self.at_rest_frames:
            at_rest = False
        else:
            h = np.asarray(self._hist, dtype=np.float64)
            at_rest = bool(np.max(np.linalg.norm(h - can_xy, axis=1)) <= self.at_rest_m)

        if placed_v2 and not self.released:
            self.released = True
            self.released_frame = self.t
        self._update_push(dist, in_hand)

        # contact_push: release FIRST (this is the change from the legacy predicate), then the
        # can touching the goal with the tool behind it and the gripper clear of the goal.
        if self.released and can_goal_contact and not gripper_goal_contact:
            dot = float((_xy(tool_xy)[0] - can_xy[0]) * (gxy[0] - can_xy[0])
                        + (_xy(tool_xy)[1] - can_xy[1]) * (gxy[1] - can_xy[1]))
            if dot < 0.0 and not self.contact_push:
                self.contact_push = True
                self.contact_push_frame = self.t

        can_tilt = tilt_deg(can_quat)
        goal_tilt = tilt_deg(goal_quat)
        in_band = bool(self.shelf_top_z + self.band_lo_m < can[2] < self.shelf_top_z + self.band_hi_m)
        nested_v2 = bool(picked and self.released
                         and dist <= self.nested_touch_dist
                         and can_tilt < self.tilt_max_deg
                         and goal_tilt < self.tilt_max_deg
                         and in_band and (not in_hand) and at_rest)
        if nested_v2 and not self.nested_v2_ever:
            self.nested_v2_ever = True
            self.nested_v2_frame = self.t

        if self.released and self.pushed and nested_v2 and not self.slide_success:
            self.slide_success = True
            self.slide_frame = self.t

        self.in_hand, self.at_rest, self.nested_v2 = in_hand, at_rest, nested_v2
        self.lever_m, self.dist_xy_m = lever, dist
        self.t += 1
        return dict(in_hand=in_hand, at_rest=at_rest, released=self.released,
                    pushed=self.pushed, contact_push=self.contact_push,
                    nested_v2=nested_v2, slide_success=self.slide_success,
                    goalward_gain_m=self.goalward_gain_m, lever_m=lever,
                    # extra diagnostics; not part of the D3 flag set
                    nested_v2_ever=self.nested_v2_ever, dist_xy_m=dist,
                    can_tilt_deg=can_tilt, goal_tilt_deg=goal_tilt, in_band=in_band,
                    grip_cmd=(None if grip_cmd is None else float(grip_cmd)), frame=self.t - 1)

    # -- episode-level view -----------------------------------------------------------------
    def episode(self):
        """The sticky, episode-level record. `nested_v2` here is `nested_v2_ever`; `nested_v2_now`
        is the value on the last frame seen."""
        return dict(released=self.released, pushed=self.pushed, contact_push=self.contact_push,
                    nested_v2=self.nested_v2_ever, nested_v2_now=self.nested_v2,
                    slide_success=self.slide_success, goalward_gain_m=self.goalward_gain_m,
                    frames=self.t, released_frame=self.released_frame,
                    pushed_frame=self.pushed_frame, contact_push_frame=self.contact_push_frame,
                    nested_v2_frame=self.nested_v2_frame, slide_frame=self.slide_frame)

    def constants(self):
        """The exact constants this instance ran with -- goes into the provenance stamp (D6)."""
        return dict(nested_touch_dist=self.nested_touch_dist, at_rest_mm=self.at_rest_m * 1000.0,
                    at_rest_frames=self.at_rest_frames, push_gain_mm=self.push_gain_m * 1000.0,
                    held_lever_m=self.held_lever_m, tilt_max_deg=self.tilt_max_deg,
                    band_lo_m=self.band_lo_m, band_hi_m=self.band_hi_m,
                    shelf_top_z=self.shelf_top_z, goal_xy=[float(v) for v in self.goal_xy])


def replay(frames, goal_xy, shelf_top_z, **kw):
    """Offline convenience: run a whole recorded episode through a fresh tracker.

    `frames` is an iterable of dicts using `update()`'s keyword names. Returns
    (per_frame_list, episode_dict). Used by the validation harness and the unit tests; the
    training env calls `update()` directly.
    """
    tr = StageTracker(goal_xy, shelf_top_z, **kw)
    out = [tr.update(**f) for f in frames]
    return out, tr.episode()
