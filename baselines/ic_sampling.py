"""Single source of truth for the randomized initial-condition (IC) distribution.

Extracted verbatim from eval_policy.py:80-87 so that DP eval, BC eval, SAC eval, and
the AI-demo harvester all draw ICs from the SAME support -- can xy uniform over the
demo-support box (with a 1 cm margin), can upright, goal at the standard shelf spot.
Making this a shared function turns "every student is evaluated on the same
distribution" into a code guarantee rather than a copy-paste hope.
"""
import numpy as np


def support_box(env):
    """(lo, hi) can-xy bounds = demo can positions' bounding box + 1 cm margin."""
    cxy = np.array([env.placements[u]['can_pos'][:2] for u in env.solved_uids])
    return cxy.min(0) - 0.01, cxy.max(0) + 0.01


def sample_support_ics(env, n, seed=0):
    """List of n reset-kwarg dicts: can xy uniform over the demo-support box, upright
    can at the correct rest height, goal at the standard shelf spot. uid=None so the
    env spawns from the explicit pose rather than a demo replay.

    Mirrors eval_policy.py's --random path exactly (same can_z / goal_z formulas).
    """
    rng = np.random.default_rng(seed)
    lo, hi = support_box(env)
    can_z = 0.05 + env.world_cfg['can_height'] / 2 + 0.0125
    goal = (0.6, -0.2, 0.11 + env.world_cfg['can_height'] / 2 + 0.0425)
    return [dict(can_pos=(float(rng.uniform(lo[0], hi[0])),
                          float(rng.uniform(lo[1], hi[1])), can_z),
                 goal_pos=goal, uid=None) for _ in range(n)]


def demo_ics(env, uids=None, reps=3):
    """List of reset-kwarg dicts for demo-IC eval: each success uid repeated `reps`
    times (mirrors eval_policy.py's default episodes construction)."""
    if uids is None:
        uids = [u for u in env.solved_uids
                if env.placements[u].get('label') == 'success']
    return [dict(uid=uid) for uid in uids for _ in range(reps)]
