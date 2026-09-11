"""Unit tests for baselines/stage_predicates.py (LADDER_UNIFY_BRIEF_2026-09-10 D3/D4, Lane 1).

Synthetic histories only: no Genesis, no torch, no simulator. Every scenario is a hand-built
sequence of env frames, so each assertion names exactly one clause of the predicate.

Run:
  PYTHONPATH=<pytest target> ~/workspace/genesis_sim2real/venv/bin/python \
      -m pytest baselines/tests/test_stage_predicates.py -q
or, with no pytest on the box:
  ~/workspace/genesis_sim2real/venv/bin/python baselines/tests/test_stage_predicates.py
"""
import pathlib as pl
import re
import sys

import numpy as np

REPO = pl.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'baselines'))

from stage_predicates import (  # noqa: E402
    AT_REST_FRAMES, FAR_RELEASE_DIST_M, FARSIDE_REACH_M, HELD_LEVER_M, NESTED_TOUCH_DIST,
    PUSH_GAIN_MM, SLIDE_GAIN_MIN_M, StageTracker, replay, tilt_deg,
)

# ---- world geometry for the synthetic scenes -------------------------------------------------
GOAL = np.array([0.700, -0.200, 0.147])
SHELF_TOP = 0.110          # band is 0.120 .. 0.180
REST_Z = 0.148             # a can resting on the shelf
UPRIGHT = (1.0, 0.0, 0.0, 0.0)
TIPPED = (0.7071068, 0.7071068, 0.0, 0.0)      # 90 deg about x
CAN_RADIUS = 0.033


def _frame(can_xy, tool_xy, *, z=REST_Z, can_quat=UPRIGHT, goal_quat=UPRIGHT,
           picked=True, placed_v2=True, can_goal_contact=False, gripper_goal_contact=False,
           grip_cmd=0.4):
    return dict(can_pos=np.array([can_xy[0], can_xy[1], z]), can_quat=can_quat,
                goal_pos=GOAL, goal_quat=goal_quat, tool_xy=np.asarray(tool_xy, float),
                grip_cmd=grip_cmd, picked=picked, placed_v2=placed_v2,
                can_goal_contact=can_goal_contact, gripper_goal_contact=gripper_goal_contact)


def _tool_behind(can_xy, gap):
    """Tool point `gap` metres from the can centre, on the far side from the goal (so that
    dot(tool - can, goal - can) < 0 -- the contact_push geometry)."""
    v = np.asarray(can_xy, float) - GOAL[:2]
    v = v / (np.linalg.norm(v) + 1e-12)
    return np.asarray(can_xy, float) + v * gap


def _near_goal(d):
    """A can xy at distance `d` from the goal, approaching from +x (the shelf side)."""
    return np.array([GOAL[0] + d, GOAL[1]])


# =============================================================================================
# tilt_deg: the copied arithmetic must equal replay_harness's, byte-for-byte in behaviour
# =============================================================================================
def test_tilt_deg_matches_replay_harness_source():
    """replay_harness imports genesis, so the function body is extracted and exec'd in a bare
    namespace instead of imported. This is the check that the COPY did not drift."""
    src = (REPO / 'can_pos_recovery' / 'replay_harness.py').read_text()
    m = re.search(r'^def tilt_deg\(quat\):\n(?:    .*\n|\n)+', src, re.M)
    assert m, 'tilt_deg not found in replay_harness.py'
    ns = {'np': np}
    exec(m.group(0), ns)
    ref = ns['tilt_deg']
    rng = np.random.default_rng(0)
    q = rng.normal(size=(500, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    for row in q:
        assert abs(tilt_deg(row) - ref(row)) < 1e-9


def test_tilt_deg_known_values():
    # NOTE the floor: replay_harness's `+ 1e-9` normalisation guard makes an exactly upright
    # quaternion read 0.00256 deg, not 0. Copied deliberately (see the test above), and 2.6e-3 deg
    # against a 20 deg threshold changes nothing -- but a test asserting exact 0 would be wrong.
    assert 0.0 < tilt_deg(UPRIGHT) < 0.01
    assert abs(tilt_deg(TIPPED) - 90.0) < 1e-3
    assert abs(tilt_deg((0.9848078, 0.1736482, 0.0, 0.0)) - 20.0) < 1e-3


# =============================================================================================
# 1. carry the can in and HOLD it at the goal -> never nested_v2
# =============================================================================================
def test_carry_in_and_hold_never_nests():
    """The failure mode the whole redesign exists to kill: the policy picks the can, presses it
    against the goal and runs to the horizon. Tool one grasp lever (1.5 cm) from the can."""
    frames = []
    for i in range(60):
        d = max(0.02, 0.20 - 0.004 * i)             # carried in from 20 cm to 2 cm
        c = _near_goal(d)
        frames.append(_frame(c, _tool_behind(c, 0.015), placed_v2=False,
                             can_goal_contact=(d < 0.07), grip_cmd=0.9))
    for _ in range(60):                              # then held motionless AT the goal
        c = _near_goal(0.02)
        frames.append(_frame(c, _tool_behind(c, 0.015), placed_v2=False,
                             can_goal_contact=True, grip_cmd=0.9))
    per, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert all(f['in_hand'] for f in per), 'a 1.5 cm lever must read in_hand'
    assert any(f['at_rest'] for f in per), 'the held can IS motionless -- at_rest must fire'
    assert not ep['nested_v2'], 'held at the goal is not nested'
    assert not ep['released'] and not ep['pushed'] and not ep['slide_success']
    assert not ep['contact_push'], 'contact_push must require release first'


def test_carry_in_and_hold_even_if_placed_v2_were_granted():
    """Isolates the in_hand clause: grant placed_v2 anyway and keep the can in the hand.
    Everything else about the state is a perfect nest."""
    c = _near_goal(0.02)
    frames = [_frame(c, _tool_behind(c, 0.015), placed_v2=True, can_goal_contact=True)
              for _ in range(60)]
    per, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert ep['released'] and all(f['in_hand'] for f in per)
    assert all(f['at_rest'] for f in per[AT_REST_FRAMES - 1:])
    assert not ep['nested_v2'], 'in_hand alone must veto nested_v2'
    assert not ep['slide_success']


# =============================================================================================
# 2. set down near the goal, tool retreats, can at rest -> nested_v2, NO slide (no push)
# =============================================================================================
def test_set_down_at_goal_then_retreat_is_nested_but_not_a_slide():
    c = _near_goal(0.06)                              # inside NESTED_TOUCH_DIST (0.081)
    frames = []
    for i in range(40):                               # tool retreats 1.5 cm -> 16.5 cm
        frames.append(_frame(c, _tool_behind(c, 0.015 + 0.00375 * i), can_goal_contact=True))
    per, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert ep['released']
    assert not per[0]['at_rest'], 'window not full on frame 0'
    assert per[-1]['nested_v2'], 'set down, at rest, in band, upright, within touch -> nested'
    assert ep['nested_v2']
    assert not ep['pushed'], f"no goalward motion: gain {ep['goalward_gain_m'] * 1000:.3f} mm"
    assert not ep['slide_success'], 'a drop-in nest is NOT a slide'
    # the tool leaves the lever band exactly once, at the frame the gap crosses HELD_LEVER
    flips = [i for i in range(1, len(per)) if per[i]['in_hand'] != per[i - 1]['in_hand']]
    assert len(flips) == 1 and per[flips[0]]['lever_m'] >= HELD_LEVER_M


def test_out_of_band_can_is_not_nested():
    """Same scene, can 4 cm above the band (still held up in the air by nothing) -> not nested."""
    c = _near_goal(0.06)
    frames = [_frame(c, _tool_behind(c, 0.15), z=SHELF_TOP + 0.09) for _ in range(40)]
    _, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert not ep['nested_v2']


def test_too_far_from_goal_is_not_nested():
    c = _near_goal(NESTED_TOUCH_DIST + 0.005)
    frames = [_frame(c, _tool_behind(c, 0.15)) for _ in range(40)]
    _, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert not ep['nested_v2']


# =============================================================================================
# 3. set down, then FIST PUSH >= 10 mm goalward with the tool at >= 3.3 cm -> slide_success
# =============================================================================================
def _fist_push_episode(gap=0.035, start_d=0.105, end_d=0.060, n_push=30, n_rest=40):
    """Release at `start_d` from the goal, then push the can to `end_d` with the tool `gap`
    from the can CENTRE (>= the can radius 3.3 cm: a fist touches the surface, never the axis),
    then retreat and let it rest."""
    frames = []
    for _ in range(AT_REST_FRAMES + 4):               # set down and settle
        c = _near_goal(start_d)
        frames.append(_frame(c, _tool_behind(c, 0.14)))
    for i in range(n_push):                           # the push
        d = start_d + (end_d - start_d) * (i + 1) / n_push
        c = _near_goal(d)
        frames.append(_frame(c, _tool_behind(c, gap), can_goal_contact=(d <= 0.070)))
    for i in range(n_rest):                           # tool retreats, can rests
        c = _near_goal(end_d)
        frames.append(_frame(c, _tool_behind(c, gap + 0.006 * (i + 1)), can_goal_contact=True))
    return frames


def test_fist_push_is_a_slide():
    frames = _fist_push_episode()
    per, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert ep['released']
    assert ep['pushed'], f"gain {ep['goalward_gain_m'] * 1000:.1f} mm"
    assert ep['goalward_gain_m'] * 1000 >= PUSH_GAIN_MM
    assert abs(ep['goalward_gain_m'] - 0.045) < 1e-9, 'gain must equal the actual 45 mm travelled'
    assert ep['contact_push'], 'tool behind the can, can touching the goal, gripper clear'
    assert ep['nested_v2'] and ep['slide_success']
    assert not any(f['in_hand'] for f in per), 'a 3.3 cm fist gap must never read in_hand'
    assert ep['slide_frame'] > ep['pushed_frame'] >= ep['released_frame']


def test_fist_push_at_the_can_radius_is_not_in_hand():
    """The separation the HELD_LEVER default rests on: 1.5 cm grasp lever vs 3.3 cm can radius."""
    c = _near_goal(0.08)
    held = replay([_frame(c, _tool_behind(c, 0.015))], GOAL[:2], SHELF_TOP)[0][0]
    fist = replay([_frame(c, _tool_behind(c, CAN_RADIUS))], GOAL[:2], SHELF_TOP)[0][0]
    assert held['in_hand'] and not fist['in_hand']
    assert held['lever_m'] < HELD_LEVER_M < fist['lever_m']


def test_push_short_of_the_threshold_is_not_pushed():
    frames = _fist_push_episode(start_d=0.070, end_d=0.062)      # 8 mm < 10 mm
    _, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert not ep['pushed'] and abs(ep['goalward_gain_m'] - 0.008) < 1e-9
    assert ep['nested_v2'] and not ep['slide_success']


def test_gripper_touching_the_goal_vetoes_contact_push():
    frames = _fist_push_episode()
    for f in frames:
        f['gripper_goal_contact'] = True
    _, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert not ep['contact_push']
    assert ep['pushed'] and ep['slide_success'], 'contact_push is a separate rung, not a clause'


def test_tool_on_the_near_side_is_not_contact_push():
    """Pulling the can toward the tool from the goal side: dot > 0, so no contact_push, even
    though the can does approach the goal."""
    frames = []
    for i in range(30):
        d = 0.105 - 0.0015 * i
        c = _near_goal(d)
        near = GOAL[:2] + (c - GOAL[:2]) * (1.0 - 0.035 / max(d, 1e-9))   # between can and goal
        frames.append(_frame(c, near, can_goal_contact=True))
    _, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert ep['pushed'] and not ep['contact_push']


# =============================================================================================
# 4. goal tipped -> not nested
# =============================================================================================
def test_tipped_goal_blocks_nested_and_slide():
    frames = _fist_push_episode()
    for f in frames:
        f['goal_quat'] = TIPPED
    _, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert ep['released'] and ep['pushed'] and ep['contact_push']
    assert not ep['nested_v2'] and not ep['slide_success']


def test_tipped_can_blocks_nested_and_slide():
    frames = _fist_push_episode()
    for f in frames:
        f['can_quat'] = TIPPED
    _, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert ep['pushed'] and not ep['nested_v2'] and not ep['slide_success']


def test_not_picked_blocks_nested():
    frames = _fist_push_episode()
    for f in frames:
        f['picked'] = False
    _, ep = replay(frames, GOAL[:2], SHELF_TOP)
    assert not ep['nested_v2'], 'a can shoved in without ever being picked is not a nest'


# =============================================================================================
# 5. release then REGRASP
# =============================================================================================
def test_regrasp_clears_nested_and_freezes_the_push_accumulator():
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    c = _near_goal(0.060)
    for _ in range(AT_REST_FRAMES + 6):                      # released, resting, nested
        r = tr.update(**_frame(c, _tool_behind(c, 0.15), can_goal_contact=True))
    assert r['nested_v2'] and tr.nested_v2_ever
    gain_before = r['goalward_gain_m']

    for i in range(20):                                      # regrasp: tool comes back in
        gap = 0.15 - 0.0075 * (i + 1)
        r = tr.update(**_frame(c, _tool_behind(c, max(gap, 0.012)), can_goal_contact=True))
    assert r['in_hand'] and not r['nested_v2'], 'back in the hand is not nested'

    for i in range(30):                                      # carried 50 mm CLOSER while held
        d = 0.060 - 0.0016 * (i + 1)
        c2 = _near_goal(max(d, 0.012))
        r = tr.update(**_frame(c2, _tool_behind(c2, 0.012), can_goal_contact=True))
    assert r['in_hand']
    assert abs(r['goalward_gain_m'] - gain_before) < 1e-12, \
        'progress made while the can is IN HAND must not count as a push'
    assert not tr.pushed
    assert tr.nested_v2_ever, 'the sticky "it happened" record survives the regrasp'
    assert not tr.nested_v2, 'the instantaneous flag does not'


def test_carry_away_then_push_again_still_accumulates():
    """The reason `pushed` counts runs rather than a net from the release frame: a carry AWAY
    must not cancel earned progress."""
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    for d in np.linspace(0.100, 0.088, 12):                  # free push, 12 mm -> pushed
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.05)))
    banked = tr.goalward_gain_m
    assert tr.pushed and abs(banked - 0.012) < 1e-9
    for d in np.linspace(0.088, 0.300, 20):                  # carried far away, IN HAND
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.012)))
    assert abs(tr.goalward_gain_m - banked) < 1e-12
    for d in np.linspace(0.300, 0.280, 10):                  # a second free push, 20 mm
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.05)))
    assert abs(tr.goalward_gain_m - (banked + 0.020)) < 1e-9


def test_push_needs_release_first():
    """Before `placed_v2` the accumulator does not run at all: carrying the can 20 cm toward the
    goal is not a push."""
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    for d in np.linspace(0.300, 0.100, 40):
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.05), placed_v2=False))
    assert not tr.released and not tr.pushed and tr.goalward_gain_m == 0.0


def test_jitter_does_not_manufacture_a_push():
    """The noise-safety claim behind the run-wise accumulation rule (see _update_push): 1200
    frames of +-0.05 mm jitter on a stationary can must not reach the 10 mm threshold. A
    per-frame ratchet would bank roughly 30 mm here; the assertion below is what rules it out."""
    rng = np.random.default_rng(7)
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    base = _near_goal(0.060)
    per_frame_ratchet, prev = 0.0, None
    for _ in range(1200):
        c = base + rng.normal(scale=5e-5, size=2)
        r = tr.update(**_frame(c, _tool_behind(base, 0.15)))
        if prev is not None:
            per_frame_ratchet += max(0.0, prev - r['dist_xy_m'])
        prev = r['dist_xy_m']
    assert not tr.pushed, f'jitter banked {tr.goalward_gain_m * 1000:.2f} mm'
    assert tr.goalward_gain_m * 1000 < PUSH_GAIN_MM
    assert per_frame_ratchet * 1000 > PUSH_GAIN_MM, \
        'the rejected per-frame-ratchet reading DOES fire on pure noise -- that is why it is rejected'


# =============================================================================================
# 6. at_rest window semantics at the boundary
# =============================================================================================
def test_at_rest_requires_a_full_window():
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    c = _near_goal(0.060)
    vals = [tr.update(**_frame(c, _tool_behind(c, 0.15)))['at_rest']
            for _ in range(AT_REST_FRAMES + 3)]
    assert vals[:AT_REST_FRAMES - 1] == [False] * (AT_REST_FRAMES - 1), \
        'an unfilled window is False, never vacuously True'
    assert all(vals[AT_REST_FRAMES - 1:]), 'first True on the AT_REST_FRAMES-th update'


def test_at_rest_threshold_is_exactly_2mm_of_spread():
    """1.9 mm of travel inside the window is at rest; 2.1 mm is not. The window is anchored on
    the CURRENT frame, so the comparison is max |x_i - x_now|."""
    for travel_mm, want in ((1.9, True), (2.1, False)):
        tr = StageTracker(GOAL[:2], SHELF_TOP)
        r = None
        for i in range(AT_REST_FRAMES):
            c = _near_goal(0.060) + np.array([travel_mm / 1000.0 * i / (AT_REST_FRAMES - 1), 0.0])
            r = tr.update(**_frame(c, _tool_behind(c, 0.15)))
        assert r['at_rest'] is want, (travel_mm, r['at_rest'])


def test_motion_clears_at_rest_for_exactly_one_window():
    """A single 3 mm jump must suppress at_rest until it has left the window, and not longer."""
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    c0 = _near_goal(0.060)
    for _ in range(AT_REST_FRAMES):
        tr.update(**_frame(c0, _tool_behind(c0, 0.15)))
    assert tr.at_rest
    c1 = c0 + np.array([0.003, 0.0])
    seq = [tr.update(**_frame(c1, _tool_behind(c1, 0.15)))['at_rest']
           for _ in range(2 * AT_REST_FRAMES)]
    assert seq[:AT_REST_FRAMES - 1] == [False] * (AT_REST_FRAMES - 1), \
        'the jumped-from samples are still inside the window'
    assert all(seq[AT_REST_FRAMES - 1:]), 'and at_rest returns as soon as they age out'


# =============================================================================================
# 7. LADDER N: farside / slide_gain_m / home / release_far   (LADDER_UNIFY_BRIEF "Ladder N")
# =============================================================================================
def test_farside_needs_release_reach_and_the_far_side():
    """The four clauses, one at a time. Baseline: the fist-push episode fires `farside`."""
    _, ep = replay(_fist_push_episode(), GOAL[:2], SHELF_TOP)
    assert ep['farside'], 'tool behind the can, within reach, released -> farside'

    # (a) never released
    frames = [dict(f, placed_v2=False) for f in _fist_push_episode()]
    assert not replay(frames, GOAL[:2], SHELF_TOP)[1]['farside']
    # (b) out of reach: the tool is behind the can but 20 cm from it
    assert not replay(_fist_push_episode(gap=0.20), GOAL[:2], SHELF_TOP)[1]['farside']
    # (c) in hand: a 1.5 cm lever is the grasp, not a push. (Read on the PUSH frames only --
    # `_fist_push_episode` then retreats the tool, which legitimately leaves the hand band.)
    per, ep = replay(_fist_push_episode(gap=0.015), GOAL[:2], SHELF_TOP)
    push = per[AT_REST_FRAMES + 4:AT_REST_FRAMES + 4 + 30]
    assert all(f['in_hand'] and not f['farside_now'] for f in push)
    assert ep['slide_gain_m'] == 0.0, 'a 45 mm CARRY earns no slide gain'
    assert not ep['home']
    # (d) near side: the tool between can and goal, pulling rather than pushing
    frames = []
    for i in range(30):
        d = 0.105 - 0.0015 * i
        c = _near_goal(d)
        near = GOAL[:2] + (c - GOAL[:2]) * (1.0 - 0.035 / max(d, 1e-9))
        frames.append(_frame(c, near, can_goal_contact=True))
    ep = replay(frames, GOAL[:2], SHELF_TOP)[1]
    assert ep['pushed'] and not ep['farside'], 'a pull is goalward progress but not a push'


def test_farside_reach_boundary():
    c = _near_goal(0.090)
    inside = replay([_frame(c, _tool_behind(c, FARSIDE_REACH_M - 0.001))], GOAL[:2], SHELF_TOP)[1]
    outside = replay([_frame(c, _tool_behind(c, FARSIDE_REACH_M + 0.001))], GOAL[:2], SHELF_TOP)[1]
    assert inside['farside'] and not outside['farside']


def test_slide_gain_counts_only_the_frames_the_tool_could_push_on():
    """`goalward_gain_m` (the (x) push) counts any free goalward motion; `slide_gain_m` counts
    only motion made while the gripper is behind the can and within reach. A can that slides
    home on its own, with the tool parked 20 cm away, earns the first and not the second."""
    near, ep_near = replay(_fist_push_episode(gap=0.035), GOAL[:2], SHELF_TOP)
    far, ep_far = replay(_fist_push_episode(gap=0.20), GOAL[:2], SHELF_TOP)
    assert abs(ep_near['slide_gain_m'] - 0.045) < 1e-9, ep_near['slide_gain_m']
    assert abs(ep_near['goalward_gain_m'] - 0.045) < 1e-9
    assert abs(ep_far['goalward_gain_m'] - 0.045) < 1e-9, 'the can still travelled 45 mm'
    assert ep_far['slide_gain_m'] == 0.0, 'but no frame of it had the tool in a pushing pose'
    assert ep_near['home'] and not ep_far['home']


def test_slide_gain_ignores_progress_made_in_hand():
    """Carrying the can 5 cm closer must not create slide gain, and must not leave the gain
    available to be claimed later: the running minimum tracks the carry."""
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    for d in np.linspace(0.150, 0.100, 30):          # carried in, in hand
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.012)))
    assert tr.slide_gain_m == 0.0 and tr.released
    for d in np.linspace(0.100, 0.098, 5):           # 2 mm of real push
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.035)))
    assert abs(tr.slide_gain_m - 0.002) < 1e-9, \
        'only the 2 mm made from the far side counts, not the 50 mm carried'


def test_oscillation_cannot_farm_slide_gain():
    """Push in 30 mm, let it come back 30 mm, push in 30 mm again: the gain is the NET 30 mm,
    because only new minima pay."""
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    for d in np.linspace(0.120, 0.090, 31):          # push 30 mm
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.035)))
    assert abs(tr.slide_gain_m - 0.030) < 1e-9
    for d in np.linspace(0.090, 0.120, 31):          # back out 30 mm
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.035)))
    for d in np.linspace(0.120, 0.090, 31):          # and in again
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.035)))
    assert abs(tr.slide_gain_m - 0.030) < 1e-9, f'oscillation banked {tr.slide_gain_m * 1000:.1f} mm'
    for d in np.linspace(0.090, 0.080, 11):          # 10 mm of genuinely new progress
        c = _near_goal(d)
        tr.update(**_frame(c, _tool_behind(c, 0.035)))
    assert abs(tr.slide_gain_m - 0.040) < 1e-9


def test_home_needs_a_nest_a_farside_and_a_centimetre():
    # a drop-in nest: released at the goal, tool retreats, no push at all
    c = _near_goal(0.060)
    drop = [_frame(c, _tool_behind(c, 0.015 + 0.00375 * i), can_goal_contact=True)
            for i in range(40)]
    ep = replay(drop, GOAL[:2], SHELF_TOP)[1]
    assert ep['nested_v2'] and not ep['home'], 'a drop is a nest but not a home'
    # a push of 8 mm: farside, nested, but under the 10 mm the rung asks for
    ep = replay(_fist_push_episode(start_d=0.070, end_d=0.062), GOAL[:2], SHELF_TOP)[1]
    assert ep['farside'] and ep['nested_v2']
    assert ep['slide_gain_m'] < SLIDE_GAIN_MIN_M and not ep['home']
    # and 12 mm clears it
    ep = replay(_fist_push_episode(start_d=0.074, end_d=0.062), GOAL[:2], SHELF_TOP)[1]
    assert ep['slide_gain_m'] >= SLIDE_GAIN_MIN_M and ep['home']
    assert ep['home_frame'] >= ep['farside_frame']


def test_home_is_sticky_and_needs_the_can_at_rest():
    frames = _fist_push_episode(n_rest=4)   # tool stops, but the window still holds the push
    ep = replay(frames, GOAL[:2], SHELF_TOP)[1]
    assert ep['farside'] and ep['slide_gain_m'] >= SLIDE_GAIN_MIN_M
    assert not ep['nested_v2'] and not ep['home'], 'still moving -> not home'
    per, ep = replay(_fist_push_episode(), GOAL[:2], SHELF_TOP)
    assert ep['home'] and per[-1]['home'] and all(f['home'] for f in per[ep['home_frame']:])


def test_release_far_is_recorded_always_and_gates_only_when_asked():
    """`far_release` is the switch; `release_far` is the measurement. A release 6 cm from the
    goal is a drop-and-nudge: with the switch on it earns no farside, no gain and no home."""
    close = _fist_push_episode(start_d=0.060, end_d=0.020)     # released INSIDE 10 cm
    far = _fist_push_episode(start_d=0.105, end_d=0.060)       # released outside it
    for frames, want_far in ((close, False), (far, True)):
        ep_off = replay(frames, GOAL[:2], SHELF_TOP)[1]
        ep_on = replay(frames, GOAL[:2], SHELF_TOP, far_release=True)[1]
        assert ep_off['release_far'] is want_far, 'release_far is measured either way'
        assert ep_on['release_far'] is want_far
        assert ep_off['release_dist_m'] >= FAR_RELEASE_DIST_M if want_far else True
        assert ep_off['farside'] and ep_off['home'], 'the switch OFF: both episodes pay'
        assert ep_on['farside'] is want_far
        assert ep_on['home'] is want_far
        if not want_far:
            assert ep_on['slide_gain_m'] == 0.0, 'a gated release earns no gain either'
    # the gate does not touch the (x) predicates: pushed/nested_v2/slide_success are unchanged
    a = replay(close, GOAL[:2], SHELF_TOP)[1]
    b = replay(close, GOAL[:2], SHELF_TOP, far_release=True)[1]
    for k in ('released', 'pushed', 'nested_v2', 'slide_success', 'contact_push',
              'goalward_gain_m'):
        assert a[k] == b[k], (k, a[k], b[k])


# =============================================================================================
# 8. `released` requires `picked` (the rnd30 reset artefact, CONFOUNDS row 82)
# =============================================================================================
def test_released_requires_picked():
    """A can that satisfies placed_v2 with the arm at home and no pick must not count as
    released. 4 of the 30 rnd30 starts are exactly that state at reset."""
    c = _near_goal(0.060)
    frames = [_frame(c, _tool_behind(c, 0.30), picked=False, placed_v2=True) for _ in range(40)]
    ep = replay(frames, GOAL[:2], SHELF_TOP)[1]
    assert not ep['released'] and not ep['farside'] and not ep['nested_v2']
    old = replay(frames, GOAL[:2], SHELF_TOP, released_requires_picked=False)[1]
    assert old['released'], 'the previous semantics, kept reachable so old rows reproduce'


def test_released_requires_picked_changes_nothing_when_the_pick_comes_first():
    """The invariant that makes this safe for every stored row: on any history where `picked`
    is true on every frame that `placed_v2` is -- every real demonstration -- the two settings
    produce identical flags, frame for frame."""
    batteries = [_fist_push_episode(), _fist_push_episode(gap=0.015),
                 _fist_push_episode(start_d=0.070, end_d=0.062),
                 [_frame(_near_goal(0.06), _tool_behind(_near_goal(0.06), 0.15))
                  for _ in range(40)]]
    for frames in batteries:
        assert all(f['picked'] for f in frames if f['placed_v2']), 'battery precondition'
        new = replay(frames, GOAL[:2], SHELF_TOP)
        old = replay(frames, GOAL[:2], SHELF_TOP, released_requires_picked=False)
        assert new[1] == old[1]
        for a, b in zip(new[0], old[0]):
            assert a == b, (a, b)


# =============================================================================================
# housekeeping
# =============================================================================================
def test_reset_clears_every_flag():
    frames = _fist_push_episode()
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    for f in frames:
        tr.update(**f)
    assert tr.slide_success
    tr.reset()
    ep = tr.episode()
    assert ep['frames'] == 0
    assert not any(ep[k] for k in ('released', 'pushed', 'contact_push', 'nested_v2',
                                   'slide_success'))
    assert ep['goalward_gain_m'] == 0.0


def test_update_returns_every_contract_key():
    tr = StageTracker(GOAL[:2], SHELF_TOP)
    c = _near_goal(0.06)
    r = tr.update(**_frame(c, _tool_behind(c, 0.15)))
    for k in ('in_hand', 'at_rest', 'released', 'pushed', 'contact_push', 'nested_v2',
              'slide_success', 'goalward_gain_m', 'lever_m'):
        assert k in r, k


def test_constants_are_overridable():
    tr = StageTracker(GOAL[:2], SHELF_TOP, held_lever_m=0.040)
    c = _near_goal(0.06)
    r = tr.update(**_frame(c, _tool_behind(c, CAN_RADIUS)))
    assert r['in_hand'], 'a 4 cm lever must capture the 3.3 cm fist gap'
    assert tr.constants()['held_lever_m'] == 0.040


def test_no_genesis_import():
    """The module must be usable wherever numpy is -- the relabel, the tests, offline analysis.
    Checked on the AST (imports only), not on the text: the docstrings DO name replay_harness and
    full_env, which is the point -- it says where the arithmetic was copied from."""
    import ast

    import stage_predicates
    tree = ast.parse(pl.Path(stage_predicates.__file__).read_text())
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            mods.add(node.module.split('.')[0])
    assert mods <= {'numpy', 'math', 'collections', '__future__'}, mods
    for bad in ('genesis', 'torch', 'taichi', 'replay_harness', 'full_env', 'genesis_can_env'):
        assert bad not in mods, f'stage_predicates.py must not import {bad!r}'


if __name__ == '__main__':
    fns = [(n, f) for n, f in sorted(globals().items()) if n.startswith('test_') and callable(f)]
    bad = 0
    for n, f in fns:
        try:
            f()
            print(f'  PASS {n}')
        except Exception as e:                                    # noqa: BLE001
            bad += 1
            print(f'  FAIL {n}: {type(e).__name__}: {e}')
    print(f'\n{len(fns) - bad}/{len(fns)} passed')
    sys.exit(1 if bad else 0)
