"""Exact two-sided permutation test on per-seed picked counts (MACHINE_FIRST_PLAN §3 statistic):
enumerates every split of the pooled seeds into two groups of the observed sizes (C(16,8) = 12870 for 8 v 8),
p = fraction of splits whose |mean difference| >= the observed |difference| (ties count). Rates are counts / N.
usage: python mf_perm.py <N per seed> <a1,a2,...> <b1,b2,...> [labelA labelB]"""
import itertools
import sys

import numpy as np


def exact_perm(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    pool = np.concatenate([a, b]); n = len(a); obs = a.mean() - b.mean()
    idx = np.arange(len(pool)); cnt = tot = 0
    for comb in itertools.combinations(idx, n):
        m = np.zeros(len(pool), bool); m[list(comb)] = True
        d = pool[m].mean() - pool[~m].mean()
        tot += 1; cnt += abs(d) >= abs(obs) - 1e-12
    return obs, cnt / tot, tot


if __name__ == '__main__':
    N = float(sys.argv[1]); a = [int(x) for x in sys.argv[2].split(',')]; b = [int(x) for x in sys.argv[3].split(',')]
    la, lb = (sys.argv[4], sys.argv[5]) if len(sys.argv) > 5 else ('A', 'B')
    obs, p, tot = exact_perm(a, b)
    print(f'{la} {a} = {sum(a)}/{len(a) * int(N)} = {sum(a) / (len(a) * N):.3f} | {lb} {b} = {sum(b)}/{len(b) * int(N)} = {sum(b) / (len(b) * N):.3f} | '
          f'delta({la}-{lb}) = {obs / N:+.3f} | exact two-sided p = {p:.4f} ({tot} splits)')
