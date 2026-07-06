"""Minimal 30 Hz env wrapper around the pick-a-place world for policy evaluation.

Interface (gym-like, no gym dependency):
    env = GenesisCanEnv(render_size=(96, 96))          # builds world once (one per process)
    obs = env.reset(uid=243)        # solved-trial initial conditions from trial_placements
    obs, done, info = env.step(action)   # action: [6 joint targets (rad), gripper 0..1]
    info carries picked/placed/contact/nested checkpoint flags.

Observation dict:
    state: float32[16] = 6 joint pos, gripper motor/100, can xyz, can quat, goal xy
    image: uint8[H,W,3] scene camera (roughly matches the real 96x96 cam obs)

The action convention matches the demos: absolute joint-position targets applied for
3 physics steps (dt=0.01, substeps from the placement table's world config).
"""
import json, sys, pathlib as pl
import numpy as np

REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO / 'can_pos_recovery'))
import torch
from replay_harness import (build_world, gripper_targets, tilt_deg, in_shelf_footprint,
                            HARDCODED_START, BOX_TOP_Z, GP_CLOSE)

def np_(x): return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)


class GenesisCanEnv:
    def __init__(self, backend='cpu', render_size=None, max_steps=1200):
        table = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())
        self.world_cfg = table['world']
        self.placements = {int(u): r for u, r in table['trials'].items()
                           if r['status'] in ('ok', 'ok_batch')}
        self.max_steps = max_steps
        self.render_size = render_size
        w = build_world(backend=backend,
                        finger_force=self.world_cfg['finger_force'],
                        finger_kp=self.world_cfg['finger_kp'],
                        can_height=self.world_cfg['can_height'],
                        can_rho=self.world_cfg['can_rho'],
                        substeps=self.world_cfg.get('substeps', 1),
                        table=self.world_cfg.get('table', False),
                        can_radius=self.world_cfg.get('can_radius', 0.035),
                        camera=render_size is not None)
        self.w = w
        self._t = 0
        self._uid = None

    @property
    def solved_uids(self):
        return sorted(self.placements)

    def reset(self, uid=None, can_pos=None, can_quat=None, goal_pos=None):
        w = self.w
        if uid is not None:
            r = self.placements[uid]
            can_pos = r['can_pos']; goal_pos = r['goal_pos']
            can_quat = r.get('can_quat') or [1, 0, 0, 0]
        self._uid = uid
        kin = w['kinova']
        kin.set_dofs_position(np.array(HARDCODED_START), w['kdofs'])
        kin.zero_all_dofs_velocity()
        w['bottle'].set_pos(can_pos); w['bottle'].set_quat(list(can_quat or [1, 0, 0, 0]))
        w['goal'].set_pos(goal_pos); w['goal'].set_quat([1, 0, 0, 0])
        for ent in (w['bottle'], w['goal']):
            try: ent.zero_all_dofs_velocity()
            except Exception: pass
        w['scene'].step()
        self._t = 0
        self._picked = self._placed = self._contact = False
        return self._obs()

    def step(self, action):
        w = self.w
        action = np.asarray(action, dtype=np.float64)
        arm, grip = action[:6], float(np.clip(action[6], 0.0, 1.0))
        w['kinova'].control_dofs_position(arm, dofs_idx_local=w['kdofs'][:6])
        w['kinova'].control_dofs_position(np.array(gripper_targets(grip * 100.0)),
                                          dofs_idx_local=np.array(w['kdofs'][-4:]))
        for _ in range(3):
            w['scene'].step()
        self._t += 1
        bp = np_(w['bottle'].get_pos())
        if bp[2] > w['pick_z'] and grip * 100.0 > GP_CLOSE: self._picked = True
        if self._picked and in_shelf_footprint(bp) and \
           BOX_TOP_Z + 0.01 < bp[2] < BOX_TOP_Z + 0.07: self._placed = True
        c = np_(w['bottle'].get_contacts(w['goal'])['position'])
        if (c.size and c.shape[0]) and float(np_(w['eef'].get_pos())[0]) < float(bp[0]):
            self._contact = True
        done = self._t >= self.max_steps
        info = dict(picked=self._picked, placed=self._placed, contact=self._contact,
                    t=self._t, uid=self._uid)
        if done:
            info['nested'] = self._nested()
        return self._obs(), done, info

    def _nested(self):
        w = self.w
        for _ in range(100):
            w['scene'].step()
        c = np_(w['bottle'].get_contacts(w['goal'])['position'])
        ncon = 0 if c.size == 0 else c.shape[0]
        return bool(ncon and tilt_deg(np_(w['bottle'].get_quat())) < 20
                    and tilt_deg(np_(w['goal'].get_quat())) < 20)

    def _obs(self):
        w = self.w
        q = np_(w['kinova'].get_dofs_position(dofs_idx_local=w['kdofs']))
        # gripper motor 0..1 from driver joint angle (invert of gripper_targets)
        theta = float(q[7])
        motor = 1.0 - (theta - (-0.09)) / (0.96 - (-0.09))
        bp = np_(w['bottle'].get_pos()); bq = np_(w['bottle'].get_quat())
        gp_ = np_(w['goal'].get_pos())
        state = np.concatenate([q[:6], [np.clip(motor, 0, 1)], bp, bq, gp_[:2]]).astype(np.float32)
        obs = dict(state=state)
        if self.render_size is not None and w.get('cam') is not None:
            rgb = np.asarray(w['cam'].render()[0]).astype(np.uint8)
            if rgb.shape[:2] != tuple(self.render_size):
                import cv2
                rgb = cv2.resize(rgb, (self.render_size[1], self.render_size[0]))
            obs['image'] = rgb
        return obs
