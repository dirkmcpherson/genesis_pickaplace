"""RLPD on robosuite Can (paper/ROBOMIMIC_PLAN_2026-09-05.md §4) -- thin trainer.

Same algorithm object as the Genesis arm (baselines/rl/rlpd_sac.py make_rlpd: SAC, UTD 10, E10/Z2 LayerNorm
critic ensemble, 50/50 online/demo batches, gamma 0.99 = the recipe of record A17, target entropy -dim/2,
auto alpha, backup_entropy off), on RobosuiteCanEnv (obs 23, act 7, +1 terminal at the first success, 400-cap)
with the demo half loaded from convert_arms.py's rlpd/transitions.npz (no re-encoding; the tuples DemoData
already consumes). Budget in DECISIONS (env steps). K=5 archived checkpoints at 20..100% + rlpd_final.zip, each
with a sidecar json (learner, arm, demo sha, gamma, horizon, steps) so the evaluator never guesses.

  $LAB/robo_venv/bin/python baselines/robomimic/train_rlpd_robosuite.py --demo $LAB/robomimic_data/arms/PH200/rlpd/transitions.npz \
      --steps 100000 --seed 0 --out $LAB/robomimic_runs/rlpd/PH200_s0 --device cuda
"""
import argparse
import json
import os
import pathlib as pl
import sys
import time

import numpy as np

HERE = pl.Path(__file__).resolve().parent
REPO = pl.Path(os.environ.get("GENESIS_PICKAPLACE_ROOT", HERE.parents[1]))
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(REPO / "baselines" / "rl"))


def robomimic_demo_transitions(npz_path):
    """rlpd/transitions.npz -> the (obs, a, r, next_obs, done) tuples DemoData stacks (plan §3)."""
    z = np.load(npz_path)
    obs, act, rew, nobs, done = z["obs"], z["act"], z["rew"], z["next_obs"], z["done"]
    assert obs.shape[1] == 23 and act.shape[1] == 7 and len(rew) == len(obs) == len(nobs) == len(done), (obs.shape, act.shape)
    return [(obs[i], act[i], float(rew[i]), nobs[i], bool(done[i] > 0)) for i in range(len(rew))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", required=True, help="arms/<ARM>/rlpd/transitions.npz (or 'none' for the no-demo control)")
    ap.add_argument("--arm", default=None)
    ap.add_argument("--hdf5", default=None, help="env_meta source; default robo_common.HDF5['ph']")
    ap.add_argument("--steps", type=int, default=100_000, help="budget in decisions (env steps)")
    ap.add_argument("--horizon", type=int, default=400)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--out", required=True)
    ap.add_argument("--gamma", type=float, default=0.99)
    ap.add_argument("--utd", type=int, default=10)
    ap.add_argument("--ensemble-size", type=int, default=10)
    ap.add_argument("--subset-size", type=int, default=2)
    ap.add_argument("--demo-batch", type=int, default=128)
    ap.add_argument("--ent-coef", default="auto")
    ap.add_argument("--ckpt-fracs", default="0.2,0.4,0.6,0.8,1.0")
    ap.add_argument("--wandb-project", default=None)
    ap.add_argument("--run-name", default=None)
    args = ap.parse_args()
    t0 = time.time()
    import torch as th
    from robosuite_can_env import RobosuiteCanEnv
    from robo_common import HDF5, sha256_file
    from rlpd_sac import make_rlpd, DemoData
    hdf5 = args.hdf5 or str(HDF5["ph"])
    env = RobosuiteCanEnv(hdf5=hdf5, horizon=args.horizon)
    env.reset(seed=args.seed)
    print(f"[env] RobosuiteCanEnv built in {time.time() - t0:.1f}s | {env.env_meta['env_name']} v{env.env_meta.get('env_version')} "
          f"horizon {args.horizon} obs {env.observation_space.shape} act {env.action_space.shape}", flush=True)
    model = make_rlpd(env, args.seed, args.device, gamma=args.gamma, utd=args.utd, ent_coef=args.ent_coef,
                      ensemble_size=args.ensemble_size, subset_size=args.subset_size, demo_batch=args.demo_batch,
                      backup_entropy=False, per_member_ln=False)
    print(f"[cfg] RLPD | E={args.ensemble_size} Z={args.subset_size} UTD={args.utd} gamma={args.gamma} ent_coef={args.ent_coef} "
          f"target_entropy={model.target_entropy} demo_batch={args.demo_batch}/256 steps={args.steps} (decisions)", flush=True)
    if args.demo == "none":
        # G2b negative control (plan §5; adversarial review S1, 2026-09-07): NO demonstrations. The demo half is EMPTY --
        # demo_batch is forced to 0 so every 256-row batch is online data (RLPDSAC.train: online_bs = 256 - 0; the demo
        # sampler draws 0 rows). A single zero row exists only to satisfy set_demo_data's non-empty assertion; it is
        # never sampled (asserted below). Everything else (E10/Z2 LN critics, UTD 10, gamma, alpha) is the recipe of record.
        if args.demo_batch != 0:
            print(f"[demos] NONE: forcing --demo-batch {args.demo_batch} -> 0 (empty demo half)", flush=True)
            args.demo_batch = 0
            model.demo_batch = 0
        transitions = [(np.zeros(23, np.float32), np.zeros(7, np.float32), 0.0, np.zeros(23, np.float32), False)]
        demo_sha = None; print("[demos] NONE (G2b no-demo control; demo_batch 0 -> pure online SAC with the RLPD critic recipe)", flush=True)
    else:
        transitions = robomimic_demo_transitions(args.demo); demo_sha = sha256_file(args.demo)
        man = json.loads((pl.Path(args.demo).parent / "manifest.json").read_text())
        assert man["n_transitions"] == len(transitions), (man["n_transitions"], len(transitions))
        print(f"[demos] {args.demo}: {len(transitions)} transitions, {sum(t[2] > 0 for t in transitions)} rewarded, "
              f"{sum(t[4] for t in transitions)} terminal, sha256 {demo_sha[:16]}... arm {man['arm']}", flush=True)
    demo = DemoData(transitions, None, th.device(args.device), seed=args.seed)
    model.set_demo_data(demo)
    if args.demo == "none":
        assert model.demo_batch == 0 and demo.sample(0).rewards.shape[0] == 0, (model.demo_batch, "demo half must be empty")
    out = pl.Path(args.out); out.mkdir(parents=True, exist_ok=True)
    sidecar = dict(learner="rlpd", task="robosuite_can", hdf5=hdf5, arm=args.arm, demo=args.demo, demo_sha256=demo_sha,
                   gamma=args.gamma, utd=args.utd, ensemble_size=args.ensemble_size, subset_size=args.subset_size,
                   demo_batch=args.demo_batch, ent_coef=args.ent_coef, horizon=args.horizon, steps=args.steps,
                   steps_unit="decisions", seed=args.seed, state_dim=23, act_dim=7, action_space="osc_pose_delta[-1,1]")
    from stable_baselines3.common.callbacks import BaseCallback, CallbackList

    class ArchiveCheckpointCallback(BaseCallback):
        def __init__(self, fracs, total):
            super().__init__(); self._thr = sorted(set(max(1, int(round(f * total))) for f in fracs)); self._done = set()

        def _on_step(self):
            for thr in self._thr:
                if thr in self._done or self.num_timesteps < thr:
                    continue
                pct = int(round(100.0 * thr / max(1, args.steps))); d = out / f"ckpt_{pct:03d}"; d.mkdir(exist_ok=True)
                self.model.save(str(d / "rlpd_ckpt"))
                (d / "rlpd_ckpt.sidecar.json").write_text(json.dumps(dict(sidecar, ckpt_step=int(self.num_timesteps))))
                self._done.add(thr); print(f"[ckpt] {d}/rlpd_ckpt.zip @ {self.num_timesteps}", flush=True)
            return True

    class EpisodeLog(BaseCallback):
        def __init__(self):
            super().__init__(); self.succ = []; self.t_last = time.time()

        def _on_step(self):
            for info, done in zip(self.locals.get("infos", []), self.locals.get("dones", [])):
                if done:
                    self.succ.append(float(info.get("success", False)))
            if self.num_timesteps % 5000 == 0:
                recent = self.succ[-20:]
                print(f"[train] step {self.num_timesteps} eps {len(self.succ)} succ(last20) {np.mean(recent) if recent else 0:.2f} "
                      f"{5000 / (time.time() - self.t_last):.1f} steps/s {(time.time() - t0) / 3600:.2f}h", flush=True)
                self.t_last = time.time()
            return True

    cbs = [ArchiveCheckpointCallback([float(f) for f in args.ckpt_fracs.split(",")], args.steps), EpisodeLog()]
    if args.wandb_project:
        try:
            import wandb
            wandb.init(project=args.wandb_project, name=args.run_name or out.name, config=sidecar)
        except Exception as e:
            print(f"[wandb] disabled: {e}")
    model.learn(total_timesteps=args.steps, log_interval=10, callback=CallbackList(cbs))
    model.save(str(out / "rlpd_final"))
    (out / "rlpd_final.sidecar.json").write_text(json.dumps(dict(sidecar, ckpt_step=int(model.num_timesteps), hours=(time.time() - t0) / 3600)))
    print(f"[rlpd] done in {(time.time() - t0) / 3600:.2f}h -> {out}/rlpd_final.zip", flush=True)


if __name__ == "__main__":
    main()
