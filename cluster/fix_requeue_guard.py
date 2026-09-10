import shutil
p = '/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/wmfix_full.sbatch'
s = open(p).read()
old = '[ -e "$LOGDIR" ] && { echo "FATAL: $LOGDIR exists"; exit 2; }'
new = ('# A REQUEUED job (preemption) legitimately finds its own partial logdir, and the handler\n'
       '# below is written to clear it -- but this guard used to exit 2 first, so that handler was\n'
       '# unreachable and every preempted run died instantly with elapsed 00:00:00.\n'
       '# 2026-09-10: cost 5 seeds (dH s905/s906/s913, dM s929/s932), all preempted on pax111.\n'
       '[ -e "$LOGDIR" ] && [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ] && { echo "FATAL: $LOGDIR exists"; exit 2; }')
assert s.count(old) == 1, ('anchor', s.count(old))
shutil.copy(p, p + '.bak_requeue_0910')
open(p, 'w').write(s.replace(old, new))
print('guard now yields to the requeue handler')
