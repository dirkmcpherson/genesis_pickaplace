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
Ladder N (2026-09-11)
---------------------
The user's redesign ("nest such that max reward comes from release, moving the gripper to the
opposite side of the can, and sliding it towards the goal can") needs three more predicates.
They are ADDITIVE: `farside`, `slide_gain_m` and `home` are new names; no existing flag's
definition changes except `released`, which now also requires `picked` (below).

`farside`      released AND not in_hand AND the tool is on the FAR side of the can along the
               can->goal line (xy dot < 0) AND |tool_xy - can_xy| <= FARSIDE_REACH_M. Sticky.
               "The gripper is on the opposite side of the can, close enough to push."
               NO solver-contact term, which is the one clause that separates it from the (g)
               `contact_push`: 10 of the 13 human sim-slides never make a can<->goal solver
               contact at all, so a contact term would score the world rather than the human.
`slide_gain_m` metres of goalward progress made WHILE the farside condition holds this frame
               and the can is not in hand, counted only on NEW MINIMA of dist_xy(can, goal)
               since the release. See `_update_slide_gain` for why the minimum tracks every
               frame while the credit does not.
`home`         nested_v2 AND farside-granted AND slide_gain_m >= SLIDE_GAIN_MIN_M. Sticky.
               "Settled arrival by a push from the far side" -- the outcome Ladder N pays.
`release_far`  whether the release that granted `released` happened at
               dist_xy(can, goal) >= FAR_RELEASE_DIST_M. Recorded ALWAYS; it only GATES
               anything when the tracker is built with `far_release=True`, in which case a
               release inside 0.10 m earns no `farside`, no slide gain and no `home`
               (a drop-and-nudge must not pay for a slide).

`released` requires `picked` (2026-09-11)
-----------------------------------------
`placed_v2` is a state predicate -- grip open, can in the shelf footprint, in the z band,
upright, 10 frames. 4 of the 30 `rnd30` starts satisfy it AT RESET with the arm at home and
the can never touched (CONFOUNDS row 82), so `released` used to latch on frame 0 of those
episodes and every downstream rung inherited the artefact. Requiring `picked` closes it.
This is a REAL behaviour change, not a no-op: it is disabled by `released_requires_picked=False`
so the previous semantics stay reproducible, and the two settings are measured against each
other on the demonstration sets (paper/LADDER_N_DEMO_CHECK_2026-09-11.md). On any trajectory
where the pick precedes the release -- every real demonstration -- the flags are identical.

`in_hand`      |tool_xy - can_xy| < HELD_LEVER_M.  NO gripper term. A held can sits one grasp
               lever from the tool point; a fist pushing the can contacts its SURFACE, so the
               tool point cannot be closer than the can radius, 3.3 cm. MEASURED on the 74 human
               tapes (paper/NESTED_V2_PREDICATE_2026-09-10.md §3): held 1.24-2.64 cm (p1-p99,
               n=5575 airborne frames, median 1.54 -- reproducing SLIDE_ANATOMY's 1.5 cm), fist
               contact 3.53-11.17 cm (p1-p99, n=478), with an EMPTY GAP from 2.64 to 3.53 cm.
               The registered default 0.025 sits inside the held tail and misreads 219/5575 held
               frames as free; 0.030 misreads 8 and is the Lane-1 recommendation. It changes no
               episode-level or tape-level count measured anywhere, so the default is left at the
               registered value and the change is the coordinator's to make.
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
STICKY (latch True and never clear): `released`, `pushed`, `contact_push`, `slide_success`,
`farside`, `home`, `release_far`.
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
    'TILT_MAX_DEG', 'BAND_LO_M', 'BAND_HI_M', 'FARSIDE_REACH_M', 'SLIDE_GAIN_MIN_M',
    'FAR_RELEASE_DIST_M', 'FLAG_KEYS', 'STICKY_KEYS', 'DIAG_KEYS',
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
# --- Ladder N (2026-09-11) ---------------------------------------------------------------
FARSIDE_REACH_M = 0.08      # m. tool-to-can-centre distance inside which the tool can push.
                            # 8 cm = the can radius 3.3 cm plus ~4.7 cm of finger/fist; wider
                            # than that the tool is not in a position to push anything.
SLIDE_GAIN_MIN_M = 0.01     # m of farside slide gain `home` requires (10 mm, = PUSH_GAIN_MM).
FAR_RELEASE_DIST_M = 0.10   # m. with far_release=True the release must be at least this far
                            # from the goal to count. Human set-down remaining distance is
                            # 10.7-12.3 cm (SLIDE_ANATOMY), so the clause is calibrated to
                            # pass a human set-down and fail a drop-and-nudge at the goal.

FLAG_KEYS = ('in_hand', 'at_rest', 'released', 'pushed', 'contact_push', 'nested_v2',
             'slide_success', 'farside', 'home', 'release_far')
STICKY_KEYS = ('released', 'pushed', 'contact_push', 'slide_success', 'farside', 'home',
               'release_far')
# the diagnostics full_env copies into info on every frame (the rest are extras)
DIAG_KEYS = ('goalward_gain_m', 'lever_m', 'slide_gain_m')


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


tilt_from_quat = tilt_deg   # the name the Lane-2 interface test uses; same function


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
                 band_hi_m=BAND_HI_M,
                 farside_reach_m=FARSIDE_REACH_M,
                 slide_gain_min_m=SLIDE_GAIN_MIN_M,
                 far_release=False,
                 far_release_dist_m=FAR_RELEASE_DIST_M,
                 released_requires_picked=True):
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
        self.farside_reach_m = float(farside_reach_m)
        self.slide_gain_min_m = float(slide_gain_min_m)
        self.far_release = bool(far_release)
        self.far_release_dist_m = float(far_release_dist_m)
        self.released_requires_picked = bool(released_requires_picked)
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
        self.farside = False
        self.home = False
        self.release_far = False
        # instantaneous flags (last computed values; False before the first update)
        self.in_hand = False
        self.at_rest = False
        self.nested_v2 = False
        self.farside_now = False
        # frame stamps (env-frame index at which a sticky flag first fired; None = never)
        self.released_frame = None
        self.contact_push_frame = None
        self.slide_frame = None
        self.pushed_frame = None
        self.nested_v2_frame = None
        self.farside_frame = None
        self.home_frame = None
        # push accumulator, see _update_push()
        self._banked_gain = 0.0
        self._run_ref = None     # dist at the start of the current not-in_hand run
        self._run_min = None     # smallest dist seen inside the current not-in_hand run
        self.goalward_gain_m = 0.0
        # slide accumulator (Ladder N), see _update_slide_gain()
        self._slide_min = None   # running minimum of dist since the release, ALL frames
        self.slide_gain_m = 0.0
        self.lever_m = float('nan')
        self.dist_xy_m = float('nan')
        self.release_dist_m = float('nan')

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

    # -- the slide accumulator (Ladder N) ---------------------------------------------------
    def _update_slide_gain(self, dist, farside_now):
        """`slide_gain_m` = metres of NEW goalward progress made while the gripper is in a
        position to push (the farside condition THIS frame, can not in hand).

        Two rules, and the pair is the whole design:

          * the running minimum `_slide_min` tracks EVERY frame after the release, so progress
            the robot makes by CARRYING the can lowers the bar without paying anything;
          * credit is added only when a new minimum is reached ON a farside frame, so the
            gain is bounded by the can's actual net displacement and oscillating the can back
            and forth cannot farm it (pulling it away raises dist above the minimum, and
            pushing it back only returns to a minimum that was already paid for).

        `slide_gain_m` is therefore monotone non-decreasing, which is what lets the Ladder N
        ramp be paid incrementally without ever clawing reward back."""
        if not self.released:
            return
        if self._slide_min is None:
            self._slide_min = dist
            return
        if dist < self._slide_min:
            if farside_now:
                self.slide_gain_m += self._slide_min - dist
            self._slide_min = dist

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

        if placed_v2 and (picked or not self.released_requires_picked) and not self.released:
            self.released = True
            self.released_frame = self.t
            self.release_dist_m = dist
            # recorded on every tracker; only GATES anything when far_release=True
            self.release_far = bool(dist >= self.far_release_dist_m)
        self._update_push(dist, in_hand)

        # dot(tool - can, goal - can) in xy: negative means the tool is on the FAR side of the
        # can from the goal -- behind it, in a position to push it home. One computation, read
        # by both `contact_push` (which adds a solver-contact term) and `farside` (which does
        # not, deliberately: 10 of the 13 human sim-slides never make that contact).
        dot = float((_xy(tool_xy)[0] - can_xy[0]) * (gxy[0] - can_xy[0])
                    + (_xy(tool_xy)[1] - can_xy[1]) * (gxy[1] - can_xy[1]))

        # farside (Ladder N): released, hand off the can, tool behind it and within reach.
        far_gate = (not self.far_release) or self.release_far
        farside_now = bool(self.released and far_gate and (not in_hand)
                           and dot < 0.0 and lever <= self.farside_reach_m)
        self._update_slide_gain(dist, farside_now)
        if farside_now and not self.farside:
            self.farside = True
            self.farside_frame = self.t

        # contact_push: release FIRST (this is the change from the legacy predicate), then the
        # can touching the goal with the tool behind it and the gripper clear of the goal.
        if self.released and can_goal_contact and not gripper_goal_contact:
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

        # home (Ladder N): settled arrival reached BY a push from the far side. Sticky, and
        # the terminal rung of both nested ladders.
        if (nested_v2 and self.farside and self.slide_gain_m >= self.slide_gain_min_m
                and not self.home):
            self.home = True
            self.home_frame = self.t

        self.in_hand, self.at_rest, self.nested_v2 = in_hand, at_rest, nested_v2
        self.farside_now = farside_now
        self.lever_m, self.dist_xy_m = lever, dist
        self.t += 1
        return dict(in_hand=in_hand, at_rest=at_rest, released=self.released,
                    pushed=self.pushed, contact_push=self.contact_push,
                    nested_v2=nested_v2, slide_success=self.slide_success,
                    farside=self.farside, home=self.home, release_far=self.release_far,
                    goalward_gain_m=self.goalward_gain_m, lever_m=lever,
                    slide_gain_m=self.slide_gain_m,
                    # extra diagnostics; not part of the D3 flag set
                    nested_v2_ever=self.nested_v2_ever, dist_xy_m=dist,
                    farside_now=farside_now, dot_tool_goal=dot,
                    can_tilt_deg=can_tilt, goal_tilt_deg=goal_tilt, in_band=in_band,
                    grip_cmd=(None if grip_cmd is None else float(grip_cmd)), frame=self.t - 1)

    # -- episode-level view -----------------------------------------------------------------
    def episode(self):
        """The sticky, episode-level record. `nested_v2` here is `nested_v2_ever`; `nested_v2_now`
        is the value on the last frame seen."""
        return dict(released=self.released, pushed=self.pushed, contact_push=self.contact_push,
                    nested_v2=self.nested_v2_ever, nested_v2_now=self.nested_v2,
                    slide_success=self.slide_success, goalward_gain_m=self.goalward_gain_m,
                    farside=self.farside, home=self.home, release_far=self.release_far,
                    slide_gain_m=self.slide_gain_m, release_dist_m=self.release_dist_m,
                    frames=self.t, released_frame=self.released_frame,
                    pushed_frame=self.pushed_frame, contact_push_frame=self.contact_push_frame,
                    nested_v2_frame=self.nested_v2_frame, slide_frame=self.slide_frame,
                    farside_frame=self.farside_frame, home_frame=self.home_frame)

    def constants(self):
        """The exact constants this instance ran with -- goes into the provenance stamp (D6)."""
        return dict(nested_touch_dist=self.nested_touch_dist, at_rest_mm=self.at_rest_m * 1000.0,
                    at_rest_frames=self.at_rest_frames, push_gain_mm=self.push_gain_m * 1000.0,
                    held_lever_m=self.held_lever_m, tilt_max_deg=self.tilt_max_deg,
                    band_lo_m=self.band_lo_m, band_hi_m=self.band_hi_m,
                    farside_reach_m=self.farside_reach_m,
                    slide_gain_min_m=self.slide_gain_min_m,
                    far_release=self.far_release, far_release_dist_m=self.far_release_dist_m,
                    released_requires_picked=self.released_requires_picked,
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
