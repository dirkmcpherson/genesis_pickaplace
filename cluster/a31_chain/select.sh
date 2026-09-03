#!/bin/bash
# Stage 2a: §3.1 teacher selection for one world. Usage: sbatch ... select.sh <old|w3>
# Rule (V2_BUILD addendum 3, decision of record): among seeds clearing the pilot bar (sel >= 0.5 at the
# selected ckpt), rank by hold@selected ASC (2 seeds -> the LOWER, "median not best"), tie -> sel@selected
# ASC, tie -> lower seed id. No seed clears the bar -> FATAL (chain halts; nothing harvests from a broken teacher).
source /cluster/tufts/shortlab/jstale02/genesis_pickaplace/cluster/a31_chain/common.sh; STAGE=select; world_cfg "${1:?world}"; cd "$GPR" || exit 1
clog "start teacher selection: $TDIR (seeds 0 1)"
python3 - "$TDIR" "$RAWBASE" "$PRUNEDBASE" <<'PY' || exit 1
import json, os, re, sys
tdir, rawbase, prunedbase = sys.argv[1:4]
cands = []
for k in (0, 1):
    p = f'{tdir}/dHv2_DP_s{k}/sweep/HEADLINE.txt'
    if not os.path.exists(p): sys.exit(f'FATAL: {p} missing (teacher seed {k} produced no DP-HEADLINE)')
    line = open(p).read().strip().splitlines()[-1]
    m = re.search(r'seed=(\d+) .*selected=(\d+) sel=(\d+)/(\d+) hold=(\d+)/(\d+) rnd=(\d+)/(\d+)', line)
    if not m: sys.exit(f'FATAL: unparsable headline: {line}')
    seed, ck, a, b, c, d, e, f = m.groups()
    ck_path = f'{tdir}/dHv2_DP_s{seed}/checkpoints/{ck}/pretrained_model'
    if not os.path.isdir(ck_path): sys.exit(f'FATAL: selected ckpt dir missing: {ck_path}')
    cands.append(dict(seed=int(seed), ckpt=ck, sel=int(a)/int(b), hold=int(c)/int(d), rnd=int(e)/int(f),
                      sel_s=f'{a}/{b}', hold_s=f'{c}/{d}', rnd_s=f'{e}/{f}', path=ck_path, line=line))
ok = [c for c in cands if c['sel'] >= 0.5]
for c in cands: print('CAND', c['line'])
if not ok: sys.exit('FATAL: TEACHER-BAR-FAILED: no seed reaches sel >= 0.5 at its selected ckpt; chain halted')
ok.sort(key=lambda c: (c['hold'], c['sel'], c['seed']))
w = ok[0]
why = 'hold-rank' if len(ok) == 1 or ok[0]['hold'] != ok[1]['hold'] else ('sel-tiebreak' if ok[0]['sel'] != ok[1]['sel'] else 'seed-id-tiebreak')
out = (f"seed={w['seed']}\nckpt={w['ckpt']}\nckpt_path={w['path']}\nrule=hold@selected ASC (lower of 2 = median-not-best), tie sel ASC, tie seed ASC; bar sel>=0.5\n"
       f"decided_by={why}\nsel={w['sel_s']} hold={w['hold_s']} rnd={w['rnd_s']}\n" + ''.join('candidate: ' + c['line'] + '\n' for c in cands))
open(f'{tdir}/TEACHER_SELECTED.txt', 'w').write(out); print(out)
# harvest IC pool = union of the raw base ICs (A31: ALL dHv2raw ICs) and the pruned base ICs (so dDPv2p can be per-IC exact)
uids = set()
for b in (rawbase, prunedbase):
    uids |= set(int(u) for u in json.load(open(f'{b}/manifest.json'))['ic_uid_histogram'])
open(f'{tdir}/HARVEST_UIDS.txt', 'w').write(' '.join(str(u) for u in sorted(uids)) + '\n')
print('HARVEST_UIDS', len(uids), sorted(uids))
PY
clog "TEACHER_SELECTED: $(grep -E '^(seed|ckpt|decided_by|sel)=' "$TDIR/TEACHER_SELECTED.txt" | tr '\n' ' ')"
clog "HARVEST_UIDS n=$(wc -w < "$TDIR/HARVEST_UIDS.txt") -> $TDIR/HARVEST_UIDS.txt"
