"""Backfill + follow the SACfD eval curve into a visible wandb run.

The live sacfd_all_v3 trainer logs eval rows backdated -> wandb drops them (files
uploaded, no history). This tails the train log's '[wandb_eval] step N: ...' lines
into a FRESH run (monotonic steps => every row visible), until the trainer exits.
"""
import re, time, pathlib as pl
import wandb

LOG = pl.Path('/home/j/workspace/genesis_pickaplace/baselines/rl/train_sacfd_v3.log')
PAT = re.compile(r'\[wandb_eval\] step (\d+): picked=([\d.]+) placed=([\d.]+) '
                 r'contact=([\d.]+) nested=([\d.]+)')

run = wandb.init(project='genesis_pickaplace', name='sacfd_all_v3-evalcurve',
                 job_type='eval', config=dict(source_run='hskw217f',
                                              note='backfill; hskw217f history rows were '
                                                   'dropped (backdated steps)'))
seen = set()


def scan():
    new = 0
    for line in LOG.read_text().splitlines():
        m = PAT.search(line)
        if not m:
            continue
        step = int(m.group(1))
        if step in seen:
            continue
        seen.add(step)
        run.log({'eval/picked': float(m.group(2)), 'eval/placed': float(m.group(3)),
                 'eval/contact': float(m.group(4)), 'eval/nested': float(m.group(5)),
                 'eval/train_step': step}, step=step)
        new += 1
    return new


n = scan()
print(f'backfilled {n} eval rounds', flush=True)
import subprocess
while subprocess.run(['pgrep', '-f', '[t]rain_sacfd_full.py'],
                     capture_output=True).returncode == 0:
    time.sleep(120)
    k = scan()
    if k:
        print(f'+{k} new rounds', flush=True)
scan()
run.finish()
print('trainer exited; curve run closed', flush=True)
