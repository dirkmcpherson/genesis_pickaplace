"""Single shared closed-loop eval loop + metrics.

Extracted from eval_policy.py:91-113 so DP, BC, and SAC students are all scored by the
EXACT same rollout/aggregation/funnel code -- the honesty protocol requires that "we
evaluate every policy identically" be literally true. Any policy that exposes
    policy_action(obs) -> physical 7-vector action
plugs in; DP additionally passes a policy_reset (its action-chunking queue must be
cleared per episode), BC/SAC pass None.

Optional per-episode video capture (record=True) uses the env's camera if it was built
with a render_size -- reuses the same cv2 mp4 writer convention as render_collected.py.
"""
import pathlib as pl

import numpy as np

STAGES = ('picked', 'placed', 'contact', 'nested')


def run_eval(env, policy_action, episodes, policy_reset=None, verbose=True,
             record_dir=None, tag='eval'):
    """Roll `policy_action` through each IC in `episodes` (list of env.reset kwargs).

    Returns dict(picked, placed, contact, nested, n) of summed successes. Prints the
    same per-episode lines + summary + failure funnel as eval_policy.py when verbose.
    """
    import cv2  # local import: BC/SAC paths may run without cv2 loaded

    agg = dict(picked=0, placed=0, contact=0, nested=0, n=0)
    record = record_dir is not None and env.render_size is not None \
        and env.w.get('cam') is not None
    if record:
        pl.Path(record_dir).mkdir(parents=True, exist_ok=True)
    for i, ep in enumerate(episodes):
        obs = env.reset(**ep)
        if policy_reset is not None:
            policy_reset()
        done, info, frames = False, {}, []
        while not done:
            action = policy_action(obs)
            obs, done, info = env.step(action)
            if record:
                frames.append(np.asarray(env.w['cam'].render()[0])[:, :, ::-1])
        for k in STAGES:
            agg[k] += bool(info.get(k))
        agg['n'] += 1
        if verbose:
            print(f"{ep.get('uid')} ep{i}: picked={info.get('picked')} "
                  f"placed={info.get('placed')} contact={info.get('contact')} "
                  f"nested={info.get('nested')}", flush=True)
        if record and frames:
            oc = ('S-nested' if info.get('nested') else 'S' if info.get('contact')
                  else 'placed' if info.get('placed') else 'picked'
                  if info.get('picked') else 'fail')
            out = pl.Path(record_dir) / f'{tag}_{i}_{oc}.mp4'
            vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'mp4v'), 20,
                                 (frames[0].shape[1], frames[0].shape[0]))
            for fr in frames:
                vw.write(fr.astype(np.uint8))
            vw.release()
    print_summary(agg, tag=tag)
    return agg


def print_summary(agg, tag='eval'):
    n = max(agg['n'], 1)
    print(f"\n[{tag}] {n} rollouts: picked {agg['picked']/n:.2f} "
          f"placed {agg['placed']/n:.2f} contact {agg['contact']/n:.2f} "
          f"nested {agg['nested']/n:.2f}", flush=True)
    p, pl_, c = max(agg['picked'], 1), max(agg['placed'], 1), max(agg['contact'], 1)
    print(f"[{tag}] funnel: P(pick)={agg['picked']/n:.2f} -> "
          f"P(place|pick)={agg['placed']/p:.2f} -> "
          f"P(slide|place)={agg['contact']/pl_:.2f} -> "
          f"P(nested|slide)={agg['nested']/c:.2f}", flush=True)
