#!/usr/bin/env python3
"""Re-execute ONE human demonstration tape through `FullTaskEnv` and log the per-ENV-FRAME
poses, contacts and stage flags that `baselines/stage_predicates.StageTracker` consumes.

Lane 1, LADDER_UNIFY_BRIEF_2026-09-10. The recorder tapes
(`$W/demos_state_full/src_dHfull_all/*.npz`) carry `states (n,17)` and `eef_pos (n+1,3)` at
DECISION resolution only, and carry no solver contacts and no `placed_v2`. Re-execution recovers
all of them at env-frame resolution, plus the honest settle from `end_of_episode()`. The replay
is `record_demos.HumanFollower.verify`'s: a fresh reset of the same IC, then the tape's
`actions_delta` (the normalised [-1,1]^7 decisions the env actually executed) fed back in order.

Output is the SAME `frames/ep<k>.npz` schema `baselines/eval_e2e_stagerec.py` writes, so
`nested_v2_validate.py` and `proxy_firing_frame.py` read it unchanged.

One world per process (Genesis), so this does one tape per invocation.

usage:
  GENESIS_SIM_VARIANT is set from --sim-variant automatically.
  python3 baselines/diagnostics/replay_tape_stagerec.py --tape <file.npz> --out <dir> \
      [--sim-variant gc_kp4_riser3_shelf6] [--max-steps 2400] [--threads 2]
"""
import argparse
import os
import pathlib as pl
import socket
import sys

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--tape', required=True)
ap.add_argument('--out', required=True)
ap.add_argument('--sim-variant', default='gc_kp4_riser3_shelf6')
ap.add_argument('--max-steps', type=int, default=2400,
                help='SIM steps; the recorder cap was 2400 (max_sim_steps in the tapes)')
ap.add_argument('--threads', type=int, default=None)
args = ap.parse_args()

if args.threads is not None:
    for _v in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
               'NUMEXPR_NUM_THREADS', 'TI_NUM_THREADS'):
        os.environ[_v] = str(args.threads)

import numpy as np  # noqa: E402
import torch        # noqa: E402

if args.threads is not None:
    torch.set_num_threads(int(args.threads))


def np_(x):
    return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)


T = np.load(args.tape, allow_pickle=True)
ic_uid = int(T['ic_uid'])
A = T['actions_delta'].astype(np.float32)
repeat = int(T['action_repeat'])
assert str(T['env_class']) == 'FullTaskEnv' and str(T['scope']) == 'full', (T['env_class'], T['scope'])
assert str(T['sim_variant']) == args.sim_variant, (T['sim_variant'], args.sim_variant)
print(f'[replay] {pl.Path(args.tape).name} ic_uid {ic_uid} n {len(A)} repeat {repeat} '
      f'recorded stage {str(T["stage"])!r} end {str(T["end_reason"])!r} node {socket.gethostname()}',
      flush=True)

os.environ['GENESIS_SIM_VARIANT'] = args.sim_variant
from sim_variant_hook import apply_pre, apply_post   # noqa: E402
apply_pre(args.sim_variant)
from full_env import FullTaskEnv                     # noqa: E402

env = FullTaskEnv(backend='cpu', max_steps=args.max_steps, scope='full',
                  action_mode='delta_joint', delta_cap=float(T['delta_cap']),
                  delta_leash_mult=float(T['delta_leash']) / float(T['delta_cap']),
                  action_repeat=repeat, delta_ref=str(T['delta_ref']), render_size=None)
apply_post(env, args.sim_variant)

FR = []
_orig = env._step_once


def _hooked(action):
    out = _orig(action)
    _obs, _r, _term, _trunc, _info = out
    w = env.genv.w
    bp = np_(w['bottle'].get_pos()); bq = np_(w['bottle'].get_quat())
    gp = np_(w['goal'].get_pos()); gq = np_(w['goal'].get_quat())
    tool = np_(env.genv.tool_pos())
    c = np_(w['bottle'].get_contacts(w['goal'])['position'])
    gg = np_(w['goal'].get_contacts(w['kinova'])['position'])
    a = np.asarray(action, dtype=np.float64)
    FR.append(dict(
        can_pos=np.asarray(bp, np.float64)[:3], can_quat=np.asarray(bq, np.float64)[:4],
        goal_pos=np.asarray(gp, np.float64)[:3], goal_quat=np.asarray(gq, np.float64)[:4],
        tool=np.asarray(tool, np.float64)[:3],
        wrist=np.asarray(np_(w['eef'].get_pos()), np.float64)[:3],
        grip_cmd=float((np.clip(a[6], -1.0, 1.0) + 1.0) / 2.0),
        picked=bool(_info.get('picked')),
        placed_v2=bool('placed_v2' in env._granted),
        nested_proxy=bool('nested' in env._granted or _info.get('nested')),
        contact_legacy=bool(_info.get('contact')),
        contact_push_legacy=bool(_info.get('contact_push')),
        slide_l_sustained=bool(_info.get('slide_success')),
        can_goal_contact=bool(c.size and c.shape[0]),
        gripper_goal_contact=bool(gg.size and gg.shape[0]),
        term=bool(_term), trunc=bool(_trunc)))
    return out


env._step_once = _hooked

obs, _ = env.reset(options={'uid': ic_uid})
info, t, ep_r = {}, 0, 0.0
for a in A:
    obs, r, term, trunc, info = env.step(a)
    ep_r += float(r); t += 1
    if term or trunc:
        break
end = env.genv.end_of_episode()

OUT = pl.Path(args.out); (OUT / 'frames').mkdir(parents=True, exist_ok=True)
cols = {k: np.asarray([f[k] for f in FR]) for k in (FR[0].keys() if FR else [])}
np.savez_compressed(
    OUT / 'frames' / f'ep{ic_uid}.npz', n_frames=len(FR), ep=ic_uid, uid=ic_uid,
    shelf_top_z=float(env.shelf_top_z), action_repeat=repeat,
    ic_can=np.full(3, np.nan),
    ref_nested_honest=bool(end['nested']), ref_slide_l=bool(end['slide_success']),
    ref_slide_route=str(end.get('slide_route')),
    ref_nested_proxy=bool('nested' in env._granted or info.get('nested')),
    ref_picked=bool('picked' in env._granted or info.get('picked')),
    ref_placed_v2=bool('placed_v2' in env._granted or info.get('placed_v2')),
    ref_contact=bool('contact' in env._granted or info.get('contact')),
    ref_contact_push=bool('contact_push' in env._granted or info.get('contact_push')),
    ref_tipped=bool(info.get('tipped')), ref_reward=float(ep_r), decisions=int(t),
    # provenance + the tape's OWN recorded verdicts, so the replay can be checked against them
    tape=str(pl.Path(args.tape).name), tape_n=int(T['n']), tape_stage=str(T['stage']),
    tape_end_reason=str(T['end_reason']), tape_label=str(T['label']),
    tape_picked=bool(T['picked'][-1]) if len(T['picked']) else False,
    tape_nested=bool(T['nested'].any()) if 'nested' in T.files else False,
    tape_contact=bool(T['contact'].any()) if 'contact' in T.files else False,
    tape_tipped=bool(T['tipped'].any()) if 'tipped' in T.files else False,
    tape_reward=float(np.sum(T['rewards'])),
    **cols)
print(f'[replay] uid {ic_uid}: {t}/{len(A)} decisions, {len(FR)} env frames, r {ep_r:.1f} '
      f'(tape {float(np.sum(T["rewards"])):.1f}) | picked {int(bool(info.get("picked")))} '
      f'placed_v2 {int("placed_v2" in env._granted)} contact {int(bool(info.get("contact")))} '
      f'cp {int(bool(info.get("contact_push")))} proxy {int("nested" in env._granted)} '
      f'nestedH {int(end["nested"])} slideL {int(end["slide_success"])} ({end.get("slide_route")})',
      flush=True)
