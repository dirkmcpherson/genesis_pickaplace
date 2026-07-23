"""Light wandb wiring for the SB3 SACfD trainers.

Design constraints (user):
  - light: scalars only, no model artifacts, no gradient logging
  - video evals: periodic random-IC eval WITH mp4s, run as a CPU subprocess so the
    GPU trainer never blocks; the training process is the ONLY wandb writer
    (concurrent writers to one run corrupt it).

Usage in a trainer:
    run = init_wandb(args, name='sacfd_all_v3')          # None if --no-wandb / no key
    cb  = [WandbScalarCallback(run), VideoEvalCallback(run, out_dir, eval_freq=25_000)]
    model.learn(..., callback=CallbackList(cb))
"""
import os
import json, pathlib as pl, subprocess, sys, tempfile

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
PY = REPO / '.venv-eval/bin/python'


def init_wandb(args, name=None, project='genesis_pickaplace', tags=()):
    """wandb.init unless disabled; falls back to offline if not logged in."""
    if getattr(args, 'no_wandb', False):
        return None
    import wandb
    mode = 'online' if (wandb.api.api_key or None) else 'offline'
    if mode == 'offline':
        print('[wandb] no API key -- logging OFFLINE (run `wandb login` + wandb sync)',
              flush=True)
    run = wandb.init(project=project, name=name, mode=mode, job_type='train',
                     tags=list(tags), config={k: v for k, v in vars(args).items()})
    # eval rows arrive AFTER later train-step rows (async subprocess) -- backdated
    # wandb.log(step=) rows are SILENTLY DROPPED. Log eval at the current step and
    # plot against eval/train_step instead.
    run.define_metric('eval/train_step')
    run.define_metric('eval/*', step_metric='eval/train_step')
    return run


class WandbScalarCallback(BaseCallback):
    """Log SB3's own scalar stream (ep_rew_mean, actor/critic losses) every log_freq."""

    def __init__(self, run, log_freq=1000):
        super().__init__()
        self.run, self.log_freq = run, log_freq

    def _on_step(self):
        if self.run is None or self.n_calls % self.log_freq:
            return True
        log = {k: v for k, v in self.model.logger.name_to_value.items()
               if isinstance(v, (int, float, np.floating))}
        buf = self.model.ep_info_buffer
        if buf:
            log['rollout/ep_rew_mean'] = float(np.mean([e['r'] for e in buf]))
            log['rollout/ep_len_mean'] = float(np.mean([e['l'] for e in buf]))
        if log:
            self.run.log(log, step=self.num_timesteps)
        return True


class VideoEvalCallback(BaseCallback):
    """Every eval_freq steps: snapshot the policy to a tmp .zip, spawn wandb_eval.py
    (CPU, --no-wandb) on it, and on a later step -- once the subprocess exits -- log
    its metrics + up to max_videos mp4s at the step the snapshot was taken.

    The tmp checkpoint is overwritten each round (nothing accumulates on disk) and is
    NOT uploaded anywhere -- only scalars and small 480p mp4s go to wandb.
    """

    def __init__(self, run, out_dir, eval_freq=25_000, n_random=10, max_steps=400,
                 max_videos=3, seed=0):
        super().__init__()
        self.run = run
        self.dir = pl.Path(out_dir) / 'wandb_eval'
        self.dir.mkdir(parents=True, exist_ok=True)
        self.eval_freq, self.n_random = eval_freq, n_random
        self.max_steps, self.max_videos, self.seed = max_steps, max_videos, seed
        self.proc, self.proc_step = None, None

    def _poll(self):
        if self.proc is None or self.proc.poll() is None:
            return
        step, rc = self.proc_step, self.proc.returncode
        self.proc = None
        mfile = self.dir / 'metrics.json'
        if rc != 0 or not mfile.exists():
            print(f'[wandb_eval] subprocess failed (rc={rc}) -- see '
                  f'{self.dir}/eval.log', flush=True)
            return
        res = json.loads(mfile.read_text())
        log = dict(res['metrics'])
        if self.run is not None:
            import wandb
            if res.get('tiled'):
                log['eval/rollouts_tiled'] = wandb.Video(res['tiled'], format='mp4',
                                                         caption='all episodes tiled')
            for i, v in enumerate(res['videos'][:self.max_videos]):
                log[f'eval/video_{i}'] = wandb.Video(v, format='mp4',
                                                     caption=pl.Path(v).stem)
            # NO step= here: backdating below the scalar callback's step gets the
            # whole row dropped. eval/train_step (in metrics) is the real x-axis.
            self.run.log(log)
        print(f"[wandb_eval] step {step}: " +
              ' '.join(f"{k.split('/')[1]}={v:.2f}" for k, v in res['metrics'].items()
                       if k != 'eval/n'), flush=True)

    def _on_step(self):
        if self.proc is not None and self.proc.poll() is not None:
            self._poll()
        if self.n_calls % self.eval_freq:
            return True
        if self.proc is not None:
            print('[wandb_eval] previous eval still running -- skipping this round',
                  flush=True)
            return True
        ck = self.dir / 'snapshot.zip'
        self.model.save(str(ck))
        rec = self.dir / 'videos'
        for old in rec.glob('*.mp4') if rec.exists() else []:
            old.unlink()
        cmd = [str(PY), str(REPO / 'baselines/wandb_eval.py'), '--kind', 'sac',
               '--checkpoint', str(ck), '--random', str(self.n_random),
               '--seed', str(self.seed), '--max-steps', str(self.max_steps),
               '--record-dir', str(rec), '--json', str(self.dir / 'metrics.json'),
               '--no-wandb', '--step', str(self.num_timesteps)]
        self.proc_step = self.num_timesteps
        self.proc = subprocess.Popen(cmd, stdout=open(self.dir / 'eval.log', 'w'),
                                     stderr=subprocess.STDOUT)
        print(f'[wandb_eval] step {self.num_timesteps}: eval subprocess launched',
              flush=True)
        return True

    def _on_training_end(self):
        if self.proc is not None:
            print('[wandb_eval] waiting for final eval subprocess...', flush=True)
            self.proc.wait()
            self._poll()
