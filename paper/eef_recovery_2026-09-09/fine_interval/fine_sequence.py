"""Conservative development scorer for saved EEF recovery traces.

Uses physical support/release/push order, never the gripper feedback threshold.
Does not replace frozen training/evaluation stage definitions.
"""
import numpy as np


def sustained_starts(mask, length=9):
    mask = np.asarray(mask, dtype=bool)
    if len(mask) < length:
        return np.array([], dtype=int)
    return np.flatnonzero(np.convolve(mask.astype(int), np.ones(length, dtype=int), 'valid') == length)


def score_sequence(data, shelf_rest_z=.2205, radius=.033):
    rows = np.asarray(data['trajectory'])
    contacts = np.asarray(data['contact_counts'])
    goals = np.asarray(data['goal_pose'])
    picked = np.asarray(data['stages'])[:, 0].astype(bool)
    can = rows[:, 13:16]
    upright = rows[:, 20] < 20
    q = goals[:, 3:7]
    goal_upright = 1 - 2*(q[:, 1]**2 + q[:, 2]**2) > np.cos(np.deg2rad(20))
    at_height = np.abs(can[:, 2] - shelf_rest_z) <= .004
    support = (contacts[:, 0] > 0) & at_height & upright
    released = picked & support & (contacts[:, 1] == 0)
    release_starts = sustained_starts(released)
    goal_touch = (contacts[:, 2] > 0) & support & goal_upright
    goal_touch &= np.abs(goals[:, 2] - shelf_rest_z) <= .004
    touch_starts = sustained_starts(goal_touch)
    final_gap = float(np.linalg.norm(can[-1, :2] - goals[-1, :2]) - 2*radius)
    result = dict(complete=False, reason='not_picked', release_start=None,
                  push_start=None, contact_start=None, final_surface_gap_m=final_gap)
    if not picked.any():
        return result
    result['reason'] = 'no_supported_release'
    if not len(release_starts):
        return result
    result['release_start'] = int(release_starts[0])
    result['reason'] = 'no_supported_push_to_contact'
    for touch in touch_starts:
        earlier_release = release_starts[release_starts + 9 <= touch]
        if not len(earlier_release):
            continue
        # At least one fully separated shelf-supported interval must precede
        # the push. Search all subsequent contacts; don't depend on a feedback
        # threshold or count contact while the can is still carried.
        release = int(earlier_release[0])
        candidates = np.flatnonzero(support & (contacts[:, 1] > 0))
        candidates = candidates[(candidates >= release + 9) & (candidates < touch)]
        for push in candidates:
            toward = goals[push, :2] - can[push, :2]
            norm = np.linalg.norm(toward)
            if norm < 1e-8:
                continue
            movement = float(np.dot(can[touch, :2] - can[push, :2], toward/norm))
            interval = slice(push, touch+9)
            support_fraction = float(support[interval].mean())
            # Allow brief contact-solver flicker, but not a lift-and-carry.
            stays_low = bool(np.all(np.abs(can[interval, 2] - shelf_rest_z) <= .008))
            stays_upright = bool(np.all(upright[interval]))
            if movement < .01 or support_fraction < .8 or not stays_low or not stays_upright:
                continue
            result.update(release_start=release, push_start=int(push), contact_start=int(touch),
                          push_toward_goal_m=movement, support_fraction=support_fraction)
            retained = bool(support[-1] and goal_upright[-1] and abs(final_gap) <= .002)
            result.update(complete=retained, reason='complete' if retained else 'final_not_retained')
            return result
    return result
