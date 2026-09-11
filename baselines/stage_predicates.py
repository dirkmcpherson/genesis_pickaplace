"""Shared stage predicates for the unified end-to-end ladder (LADDER_UNIFY_BRIEF D3).

=======================================================================================
  *** STUB -- LANE 2 PLACEHOLDER.  LANE 1 OWNS THIS FILE AND REPLACES IT WHOLESALE. ***
=======================================================================================

Why a stub exists at all: `baselines/rl/full_env.py` (Lane 2) must import and drive this
module, and its unit tests must run, before Lane 1's validated module lands. This file
therefore implements the brief's D3 text LITERALLY and NOTHING ELSE. It is NOT the
predicate of record:

  * the constants below are the brief's STARTING values, not calibrated ones -- in
    particular HELD_LEVER_M is the brief's "start at 2.5 cm", NOT a measured separation;
  * nothing here has been validated against `nested_honest`, the (l) predicate, the 74
    human census tapes or any policy rollout;
  * no confusion matrix exists for it. `paper/NESTED_V2_PREDICATE_2026-09-10.md` (Lane 1)
    is where the calibrated constants and the validation live.

Do not quote a number produced with this file in any table.

--------------------------------------------------------------------------------------
INTERFACE (the contract Lane 2 codes against; Lane 1's module must keep it)

    tracker = StageTracker(goal_xy, shelf_top_z)
    tracker.reset()
    flags = tracker.update(can_pos=..., can_quat=..., goal_pos=..., goal_quat=...,
                           tool_xy=..., grip_cmd=..., picked=..., can_goal_contact=...,
                           gripper_goal_contact=..., placed_v2=...)

`update` is called ONCE PER ENV FRAME, AFTER the sim step (one env frame = the 3 scene
steps `GenesisCanEnv.step` takes; a decision is `action_repeat` env frames). It returns

    {'in_hand', 'at_rest',                       instantaneous, this frame
     'released', 'pushed', 'contact_push',
     'nested_v2', 'slide_success',               STICKY: once True they stay True
     'goalward_gain_m', 'lever_m'}               diagnostics (floats)

Pure functions over poses / contacts / a short history. NO Genesis import, no torch, no
world handle -- so it is unit-testable on synthetic histories and callable offline.

--------------------------------------------------------------------------------------
THE DEFINITIONS THIS STUB IMPLEMENTS (brief D2/D3), verbatim:

  in_hand        |tool_xy - can_xy| < HELD_LEVER_M            (no gripper term: a fist
                 push contacts the can surface at >= the can radius 3.3 cm, so a lever
                 test separates carry from push without outlawing a closed-finger push)
  at_rest        can xy displacement <= AT_REST_MM over the last AT_REST_FRAMES frames
  released       placed_v2 has been granted (full_env owns placed_v2) AND not in_hand
  pushed         after the FIRST placed_v2 grant, the can's xy distance to the goal has
                 decreased by >= PUSH_GAIN_MM cumulatively while not in_hand
  contact_push   placed_v2 granted AND can<->goal solver contact AND the tool on the far
                 side of the can along the can->goal line AND no gripper<->goal contact
  nested_v2      picked AND placed_v2 granted AND dist_xy(can, goal) <= NESTED_TOUCH_DIST
                 AND tilt(can) < NESTED_TILT_DEG AND tilt(goal) < NESTED_TILT_DEG AND can
                 z inside the shelf resting band AND not in_hand AND at_rest
  slide_success  placed_v2 granted AND pushed AND nested_v2        <- the paid, terminal
                 top rung; the ONE definition the env, the relabel and both evaluators
                 use (D4)

`grip_cmd` is accepted because the interface names it, and is deliberately UNUSED by
every predicate above: PHASE_PLAN (p) withdrew (l)'s `grip < 0.3` clause after it passed
2 of 74 human demonstrations -- the human releases and then pushes the can home with the
fingers re-closed to ~0.4.
"""
import math

# --- constants (brief D3 starting values; Lane 1 replaces them with calibrated ones) ---
NESTED_TOUCH_DIST = 0.081   # m, centre-to-centre xy; replay_harness.NESTED_TOUCH_DIST
AT_REST_MM = 2.0            # mm of xy travel allowed over the at-rest window
AT_REST_FRAMES = 12         # env frames = 3 decisions at action_repeat 4
PUSH_GAIN_MM = 10.0         # mm of cumulative goalward gain, not in hand, post-release
HELD_LEVER_M = 0.025        # m; UNCALIBRATED (brief: "start at 2.5 cm")
NESTED_TILT_DEG = 20.0      # both cans near-upright (same threshold as placed_v2/_nested)
SHELF_BAND_LO = 0.01        # can centre z in (shelf_top + LO, shelf_top + HI)
SHELF_BAND_HI = 0.07

FLAG_KEYS = ('in_hand', 'at_rest', 'released', 'pushed', 'contact_push',
             'nested_v2', 'slide_success')
DIAG_KEYS = ('goalward_gain_m', 'lever_m')


def tilt_from_quat(quat):
    """Angle (deg) between the body z axis and world z, for a wxyz quaternion.

    Same arithmetic as can_pos_recovery.replay_harness.tilt_deg, re-stated here ONLY
    because this module must import nothing that pulls Genesis in. Kept byte-comparable:
    a unit test asserts the two agree."""
    w_, x, y, z = [float(v) for v in quat]
    zz = 1 - 2 * (x * x + y * y)
    zx = 2 * (x * z + w_ * y)
    zy = 2 * (y * z - w_ * x)
    n = math.sqrt(zx * zx + zy * zy + zz * zz) + 1e-9
    c = max(-1.0, min(1.0, zz / n))
    return math.degrees(math.acos(c))


def _dist_xy(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


class StageTracker:
    """Sticky stage bookkeeping over one episode. One instance per env, reset per episode."""

    def __init__(self, goal_xy, shelf_top_z):
        # goal_xy is the reset-time goal (the tracker also reads the live goal pose each
        # frame; the constructor value is kept so a caller can build the tracker before
        # the first step and so a moved goal is visible as a diagnostic).
        self.goal_xy0 = (float(goal_xy[0]), float(goal_xy[1]))
        self.shelf_top_z = float(shelf_top_z)
        self.reset()

    def reset(self):
        self._hist = []              # recent can xy, newest last, len <= AT_REST_FRAMES
        self._released = False
        self._pushed = False
        self._contact_push = False
        self._nested_v2 = False
        self._slide = False
        self._gain_m = 0.0           # cumulative goalward gain since the placed_v2 grant
        self._last_free_dist = None  # last not-in-hand distance used for the gain sum
        self._lever_m = float('nan')
        self.frames = 0

    # ------------------------------------------------------------------ the one update
    def update(self, *, can_pos, can_quat, goal_pos, goal_quat, tool_xy, grip_cmd,
               picked, can_goal_contact, gripper_goal_contact, placed_v2):
        """Call once per env frame AFTER the sim step. Returns the flag dict (see module
        docstring). `grip_cmd` is accepted and deliberately unused ((p) withdrew the grip
        clause); `placed_v2` is the env's own sustained release predicate (sticky by the
        time it reaches here -- full_env passes its granted state, not the raw frame)."""
        self.frames += 1
        can_xy = (float(can_pos[0]), float(can_pos[1]))
        goal_xy = (float(goal_pos[0]), float(goal_pos[1]))
        can_z = float(can_pos[2])

        lever = _dist_xy(tool_xy, can_xy)
        self._lever_m = lever
        in_hand = lever < HELD_LEVER_M

        self._hist.append(can_xy)
        if len(self._hist) > AT_REST_FRAMES:
            self._hist.pop(0)
        if len(self._hist) < AT_REST_FRAMES:
            at_rest = False          # not enough history yet: an absent value is not a zero
        else:
            x0, y0 = self._hist[0]
            at_rest = all(math.hypot(x - x0, y - y0) <= AT_REST_MM / 1000.0
                          for x, y in self._hist)

        placed = bool(placed_v2)
        d_goal = _dist_xy(can_xy, goal_xy)

        # released: the can is on the shelf (placed_v2 granted) and the tool has left it
        if placed and not in_hand:
            self._released = True

        # pushed: cumulative goalward gain accrued only on not-in-hand frames after the
        # placed_v2 grant. Carrying the can toward the goal must NOT count, which is why
        # the reference distance is dropped while in hand and re-seeded on release.
        if placed:
            if in_hand:
                self._last_free_dist = None
            else:
                if self._last_free_dist is None:
                    self._last_free_dist = d_goal
                elif d_goal < self._last_free_dist:
                    self._gain_m += (self._last_free_dist - d_goal)
                    self._last_free_dist = d_goal
                else:
                    self._last_free_dist = d_goal
            if self._gain_m >= PUSH_GAIN_MM / 1000.0:
                self._pushed = True

        # contact_push: a push, not a carry -- requires the release to have been granted
        if placed and can_goal_contact and not gripper_goal_contact:
            far = ((float(tool_xy[0]) - can_xy[0]) * (goal_xy[0] - can_xy[0])
                   + (float(tool_xy[1]) - can_xy[1]) * (goal_xy[1] - can_xy[1])) < 0.0
            if far:
                self._contact_push = True

        # nested_v2: state only, no settle simulation
        nested_now = bool(
            picked and placed
            and d_goal <= NESTED_TOUCH_DIST
            and tilt_from_quat(can_quat) < NESTED_TILT_DEG
            and tilt_from_quat(goal_quat) < NESTED_TILT_DEG
            and (self.shelf_top_z + SHELF_BAND_LO) < can_z < (self.shelf_top_z + SHELF_BAND_HI)
            and not in_hand
            and at_rest)
        if nested_now:
            self._nested_v2 = True

        # slide_success: the paid top rung. Sticky, in-episode, one definition (D4).
        if placed and self._pushed and self._nested_v2:
            self._slide = True

        return dict(in_hand=bool(in_hand), at_rest=bool(at_rest),
                    released=bool(self._released), pushed=bool(self._pushed),
                    contact_push=bool(self._contact_push), nested_v2=bool(self._nested_v2),
                    slide_success=bool(self._slide),
                    goalward_gain_m=float(self._gain_m), lever_m=float(self._lever_m))


# Marker other code can assert on, so a run cannot silently be scored with the stub.
IS_STUB = True
STUB_NOTE = ('baselines/stage_predicates.py is the LANE-2 STUB: brief-literal, '
             'UNCALIBRATED (HELD_LEVER_M is the starting 2.5 cm), never validated '
             'against nested_honest. Lane 1 replaces this file.')
