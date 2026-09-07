#!/usr/bin/env python3
"""contact_push bank geometry (2026-09-07, ADVERSARIAL_REVIEW_eval_env S2-5 / S2-4): for every entry of the phase banks,
put the arm at the banked qpos (pure forward kinematics, no physics) and count the directional tests against the
recorded can/goal:  ee_x < can_x (`contact`'s clause, wrist link)  |  dot < 0 with the WRIST (withdrawn contact_push)  |
dot < 0 with the TOOL point (contact_push of record)  |  tool_x < can_x.  Also the S2-4 grip-scale audit: machine-generated
banks (polE_*) store the raw [-1,1] action as grip_cmd while the restore reads physical 0..1 (clipped), human banks
(holdE_*) store physical. Writes <out>.json with the affected/unaffected pseudo-uid lists per bank.
usage: contact_push_bank_geometry.py --banks <dir with polE_place.json polE_contact.json holdE_place.json holdE_contact.json> [--out F.json]"""
import argparse, json, os, pathlib as pl, sys
import numpy as np

HERE = pl.Path(__file__).resolve().parent
REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', HERE.parents[1]))
for p in (REPO / 'baselines', REPO / 'baselines' / 'rl', REPO / 'can_pos_recovery'):
    sys.path.insert(0, str(p))
ap = argparse.ArgumentParser()
ap.add_argument('--banks', required=True)
ap.add_argument('--variant', default='gc_kp4_riser3_shelf6')
ap.add_argument('--out', default=None)
ap.add_argument('--release', type=float, default=0.45, help='FullTaskEnv.PLACE_RELEASE: physical grip below this = released')
ap.add_argument('--tol', type=float, default=0.05, help='|restored - intended| grip tolerance for the strict subset')
args = ap.parse_args()

import torch
import sim_variant_hook as svh
svh.apply_pre(args.variant)
from genesis_can_env import GenesisCanEnv, np_, _quat_to_R
from replay_harness import HARDCODED_START
env = GenesisCanEnv(backend='cpu', max_steps=10**9)
svh.apply_post(env, args.variant)
w = env.w
env.reset(can_pos=[0.5, 0.0, 0.113], goal_pos=(0.672, -0.221, w['goal_start_z']))   # calibrates the tool offset at HARDCODED_START
q0 = np_(w['kinova'].get_dofs_position(dofs_idx_local=w['kdofs'])).copy()

def fk(qpos6):
    q = q0.copy(); q[:6] = qpos6
    lp, lq = w['kinova'].forward_kinematics(qpos=torch.as_tensor(q, dtype=torch.float32))
    i = w['eef'].idx_local
    wp = np_(lp[i]).astype(np.float64); wq = np_(lq[i]).astype(np.float64)
    return wp, wp + _quat_to_R(wq) @ env._tool_offset

out = {}
print(f"| bank (n) | source | `ee_x < can_x` (wrist) | `dot<0` WRIST | `dot<0` TOOL (contact_push) | `tool_x < can_x` | grip_cmd range | S2-4 affected (review criterion) | strict subset (restored≈intended ±{args.tol}) |")
print("|---|---|---|---|---|---|---|---|---|")
for name in ('polE_place', 'polE_contact', 'holdE_place', 'holdE_contact'):
    f = pl.Path(args.banks) / f'{name}.json'
    if not f.exists(): print(f'| {name} | MISSING | | | | | | | |'); continue
    raw = json.loads(f.read_text())
    entries = [dict(e, uid=int(u)) for u, e in raw.items()] if isinstance(raw, dict) else [dict(e, uid=int(e['uid'])) for e in raw]
    machine = any('source' in e for e in entries)   # --dump-entries banks carry source/scope/mode; human banks (make_phase_banks) do not
    rows = []
    for e in entries:
        wp, tp = fk(np.asarray(e['qpos'], np.float64))
        bp = np.asarray(e['can_pos'], np.float64); gxy = np.asarray(e['goal_xy'], np.float64)
        dw = float((wp[0]-bp[0])*(gxy[0]-bp[0]) + (wp[1]-bp[1])*(gxy[1]-bp[1]))
        dt = float((tp[0]-bp[0])*(gxy[0]-bp[0]) + (tp[1]-bp[1])*(gxy[1]-bp[1]))
        g = float(e['grip_cmd']); go = float(e['grip_obs'])
        restored = float(np.clip(g, 0.0, 1.0))                 # gripper_targets clips g_pos/100 to [0,1]
        intended = (g + 1.0) / 2.0 if machine else g            # raw [-1,1] action (S2-4) vs physical
        rows.append(dict(uid=e['uid'], ee_x_lt=bool(wp[0] < bp[0]), dot_wrist=dw, dot_tool=dt, tool_x_lt=bool(tp[0] < bp[0]),
                         grip_cmd=g, grip_obs=go, restored=restored, intended=intended,
                         opens_vs_obs=bool(g < go),                              # review's polE_place criterion (35/148)
                         strict_ok=bool(abs(restored - intended) <= args.tol),
                         released_intended=bool(intended < args.release), released_restored=bool(restored < args.release)))
    n = len(rows)
    aff = [r['uid'] for r in rows if (r['opens_vs_obs'] if name.endswith('place') else (abs(r['restored'] - r['intended']) > args.tol))]
    if not machine: aff = []   # human banks store physical grips: nothing is on the wrong scale
    strict = [r['uid'] for r in rows if r['strict_ok']]
    qual = [r['uid'] for r in rows if (r['released_intended'] == r['released_restored']) and not (name.endswith('place') and r['opens_vs_obs'])]
    out[name] = dict(n=n, machine_generated=machine, ee_x_lt=sum(r['ee_x_lt'] for r in rows), dot_wrist_neg=sum(r['dot_wrist'] < 0 for r in rows),
                     dot_tool_neg=sum(r['dot_tool'] < 0 for r in rows), tool_x_lt=sum(r['tool_x_lt'] for r in rows),
                     grip_cmd_min=min(r['grip_cmd'] for r in rows), grip_cmd_max=max(r['grip_cmd'] for r in rows),
                     grip_cmd_mean=float(np.mean([r['grip_cmd'] for r in rows])), intended_mean=float(np.mean([r['intended'] for r in rows])),
                     affected_uids=sorted(aff), strict_ok_uids=sorted(strict), qualitative_ok_uids=sorted(qual), rows=rows)
    o = out[name]
    print(f"| {name} ({n}) | {'machine (--dump-entries, raw action)' if machine else 'human (physical)'} | {o['ee_x_lt']}/{n} | {o['dot_wrist_neg']}/{n} | **{o['dot_tool_neg']}/{n}** | {o['tool_x_lt']}/{n} | {o['grip_cmd_min']:.3f} … {o['grip_cmd_max']:.3f} (mean {o['grip_cmd_mean']:.3f}; intended physical mean {o['intended_mean']:.3f}) | {len(aff)}/{n} | {len(strict)}/{n} (qualitative-release-preserving {len(qual)}/{n}) |")
for name, o in out.items():
    if o['affected_uids']: print(f"\n- {name} S2-4 affected pseudo-uids ({len(o['affected_uids'])}): {o['affected_uids']}")
if args.out:
    json.dump(out, open(args.out, 'w'), indent=1); print(f"\nwrote {args.out}")
