import json, glob, subprocess, collections
import re

L = '/cluster/tufts/shortlab/jstale02'
def sh(c):
    try: return subprocess.run(c, shell=True, capture_output=True, text=True, timeout=90).stdout
    except Exception: return ''

# --- queue state per learner ---
q = collections.Counter()
for line in sh('squeue -u $USER -h -o "%j %T"').splitlines():
    p = line.split()
    if len(p) != 2 or not p[0].startswith('e2eL_'): continue
    lea = 'R2' if p[0].startswith('e2eL_r2') else ('RLPD' if p[0].startswith('e2eL_rl') else None)
    if lea: q[(lea, p[1])] += 1

# --- terminal states over the batch's lifetime ---
# NOTE 2026-09-10: this block previously used %%-escaped format strings, which reach sacct
# literally -> "Invalid time specification" -> ZERO lines -> every counter read 0. It reported
# failed=0 while a job had in fact died. Single % is correct here. MIN_JOB scopes to the
# current batch so historical failures do not latch the alarm on forever.
MIN_JOB = 3484586
term = collections.Counter()
raw = sh('sacct -S $(date -d "-20 hours" +%Y-%m-%dT%H:%M) -u $USER '
         '--format=JobID%16,JobName%28,State%16,ExitCode%8 -X -n')
if not raw.strip():
    term['SACCT-EMPTY'] += 1            # never let a silent zero look like health
for l in raw.splitlines():
    p_ = l.split()
    if len(p_) < 3 or 'e2eL_' not in l: continue
    try:
        if int(p_[0].split('.')[0].split('_')[0]) < MIN_JOB: continue
    except Exception: continue
    name, st, ec = p_[1], p_[2], p_[-1]
    for k in ('COMPLETED', 'FAILED', 'TIMEOUT', 'PREEMPTED', 'NODE_FAIL', 'OUT_OF_ME', 'CANCELLED'):
        if st.startswith(k): term[k] += 1
    # Benign, known signature: an ORIGINAL r2dreamer seed (job suffix _s0.._s7) that trained to
    # budget and then tripped the pre-fix post-train log check -> FAILED with ExitCode 3:0.
    # Taken from sacct, the authoritative source, rather than inferred from run directories.
    if st.startswith('FAILED') and ec.startswith('3:') and name.startswith('e2eL_r2_') \
       and re.match(r'.*_s[0-7]$', name):
        term['BENIGN_POSTTRAIN'] += 1

# --- disk: this project has lost every running job to a full filesystem before ---
gb = -1
for l in sh('df -BG /cluster/tufts/shortlab').splitlines():
    p = l.split()
    if len(p) >= 4 and p[3].endswith('G'):
        try: gb = int(p[3][:-1])
        except Exception: pass
dstate = 'CRIT' if 0 <= gb < 60 else ('LOW' if 0 <= gb < 150 else 'OK')

# --- {RLPD} episode-record health: structure AND per-episode value agreement ---
rl_eps = rl_ep_pick = rl_term_pick = rl_disagree = 0
rl_runs = rl_nokeys = 0
RL = [f'{L}/gp_e2e/baselines/rl/checkpoints/e2e/e2e_rlpd_dH_s90*/episode_rollouts.jsonl',
      f'{L}/gp_e2e/baselines/rl/checkpoints/e2e/e2e_rlpd_dDPfirst_s92*/episode_rollouts.jsonl']
for f in sorted(set(sum([glob.glob(g) for g in RL], []))):
    if '_FAILED_' in f: continue
    rows = []
    try:
        for l in open(f):
            try: rows.append(json.loads(l))
            except Exception: pass
    except Exception: continue
    if not rows: continue
    rl_runs += 1
    if not any(k.startswith('episode/train_ep_') for k in rows[-1]): rl_nokeys += 1
    for r in rows:
        rl_eps += 1
        e = bool(r.get('episode/train_ep_picked')); t = bool(r.get('episode/train_picked'))
        rl_ep_pick += e; rl_term_pick += t
        if e != t: rl_disagree += 1

# --- {r2dreamer} sticky keys present ---
r2_runs = r2_nokeys = 0
R2 = [f'{L}/wm_fix_2026-09-03/runs/full_r2d_state_dHfull_all_rx_s9*/metrics.jsonl',
      f'{L}/wm_fix_2026-09-03/runs/full_r2d_state_dDPfull_first_rx_s9*/metrics.jsonl']
r2_young = 0; r2_trained = 0; r2_trained_noeval = 0
for d in sorted(set(sum([glob.glob(g) for g in R2], []))):
    if '_FAILED_' in d: continue        # preserved crash dirs are not live runs
    # A run that reached its 4M budget has COMPLETE training artifacts even if its job then
    # exited nonzero. 2026-09-10: the launcher post-train check greps its own log by an
    # assumed job NAME, which any -J rename breaks, so runs die AFTER train rc=0 with
    # latest.pt + metrics.jsonl intact. Those deaths are benign; count them apart.
    try:
        stp = 0
        for l in open(d):
            try: stp = max(stp, json.loads(l).get('step', 0))
            except Exception: pass
        if stp >= 3995000:
            r2_trained += 1
            # Only the ORIGINAL seeds (900-907 / 920-927) spooled the pre-fix launcher and are
            # therefore expected to exit nonzero after a complete training run. Seeds >=908/928
            # were resubmitted against the fixed launcher and must NOT be excused.
            base = os.path.basename(os.path.dirname(d))
            try: sd = int(base.rsplit('_s', 1)[1])
            except Exception: sd = -1
            if 900 <= sd <= 907 or 920 <= sd <= 927:
                r2_trained_noeval += 1
    except Exception: pass
    try:
        ks = set(); nep = 0
        for i, l in enumerate(open(d)):
            try: o = json.loads(l)
            except Exception: continue
            ks |= set(o)
            if any(k.startswith('episode/') for k in o): nep += 1
            if i > 300: break
    except Exception: continue
    if not ks: continue
    r2_runs += 1
    # A run that has not finished an episode yet CANNOT have episode keys -- that is startup,
    # not a defect. Only judge runs that have actually logged episodes. (False alarm 2026-09-10:
    # the freshly resubmitted s906 tripped BAD-NOKEYS at 1 row / 0 episodes.)
    if nep < 3:
        r2_young += 1
    elif not any(k.startswith('episode/train_ep_') for k in ks):
        r2_nokeys += 1

DEATH_BASELINE = 30   # deaths already accounted for at 2026-09-10 02:00 (pre-fix submissions + the bus-error seed, resubmitted as 3486259)
alive = q[('R2','RUNNING')] + q[('R2','PENDING')] + q[('RLPD','RUNNING')] + q[('RLPD','PENDING')]
v = []
if dstate != 'OK': v.append('DISK-' + dstate)
if rl_nokeys or r2_nokeys: v.append('BAD-NOKEYS')
# Equality of sticky vs terminal is EXPECTED (measured 2026-09-09: info['picked'] is already
# sticky at the done step, 660 eps, 0 per-episode disagreements). Only sticky losing
# information is a fault.
if rl_ep_pick < rl_term_pick: v.append('BAD-STICKY-LOSES-INFO')
deaths = term['FAILED'] + term['TIMEOUT'] + term['NODE_FAIL'] + term['OUT_OF_ME']
unexplained = deaths - DEATH_BASELINE - term['BENIGN_POSTTRAIN']
if unexplained > 0: v.append(f'JOB-DEATHS({deaths},unexplained={unexplained})')
if term['SACCT-EMPTY']: v.append('BAD-SACCT-RETURNED-NOTHING')
if alive == 0 and term['COMPLETED']: v.append('ALL-ENDED')
verdict = ' '.join(v) if v else 'OK'

print(f"R2 run={q[('R2','RUNNING')]} pend={q[('R2','PENDING')]} nokeys={r2_nokeys}/{r2_runs} | "
      f"RLPD run={q[('RLPD','RUNNING')]} pend={q[('RLPD','PENDING')]} nokeys={rl_nokeys}/{rl_runs} "
      f"eps={rl_eps} picked={rl_ep_pick} disagree={rl_disagree} young={r2_young} | "
      f"done={term['COMPLETED']} deaths={deaths} r2trained={r2_trained}/32 benign_posttrain={term['BENIGN_POSTTRAIN']} "
      f"preempted={term['PREEMPTED']} | disk={gb}G/{dstate} | {verdict}")
