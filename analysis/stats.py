#!/usr/bin/env python3
"""Seed-level statistics for two-arm comparisons. THE unit is the seed (never the episode).

    python analysis/stats.py --a 15/15 7/15 10/15 14/15 --b 11/15 9/15 4/15
    python analysis/stats.py --a 0.93 0.60 0.60 0.93 --b 0.80 0.73 0.13
    python analysis/stats.py --selftest

Reports: means, sds, difference, Welch t with Welch-Satterthwaite df and the CORRECT critical value for
that df (the 08-27 N15 note used pooled-df and df=4 values by mistake), the exact two-sided permutation
p over all label assignments, the minimum attainable permutation p for the design, and relative loss.
Pure python + math; no scipy (t quantile by bisection on the regularized incomplete beta).
"""
import argparse, itertools, math, sys


def _betacf(a, b, x, maxit=300, eps=3e-14):
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c, d = 1.0, 1.0 - qab * x / qap
    d = 1.0 / (d if abs(d) > 1e-300 else 1e-300); h = d
    for m in range(1, maxit + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d; d = 1.0 / (d if abs(d) > 1e-300 else 1e-300)
        c = 1.0 + aa / (c if abs(c) > 1e-300 else 1e-300); h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d; d = 1.0 / (d if abs(d) > 1e-300 else 1e-300)
        c = 1.0 + aa / (c if abs(c) > 1e-300 else 1e-300); de = d * c; h *= de
        if abs(de - 1.0) < eps: break
    return h


def betainc(a, b, x):
    if x <= 0: return 0.0
    if x >= 1: return 1.0
    bt = math.exp(math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1 - x))
    if x < (a + 1) / (a + b + 2): return bt * _betacf(a, b, x) / a
    return 1.0 - bt * _betacf(b, a, 1 - x) / b


def t_cdf(t, df):
    x = df / (df + t * t)
    p = 0.5 * betainc(df / 2.0, 0.5, x)
    return 1 - p if t > 0 else p


def t_ppf(q, df, lo=0.0, hi=200.0):
    for _ in range(200):
        mid = (lo + hi) / 2
        if t_cdf(mid, df) < q: lo = mid
        else: hi = mid
    return (lo + hi) / 2


def parse(vals):
    out = []
    for v in vals:
        if '/' in v:
            k, n = v.split('/'); out.append(int(k) / int(n))
        else: out.append(float(v))
    return out


def welch(a, b, alpha=0.05):
    na, nb = len(a), len(b)
    ma, mb = sum(a) / na, sum(b) / nb
    va = sum((x - ma) ** 2 for x in a) / (na - 1) if na > 1 else float('nan')
    vb = sum((x - mb) ** 2 for x in b) / (nb - 1) if nb > 1 else float('nan')
    se = math.sqrt(va / na + vb / nb)
    df = (va / na + vb / nb) ** 2 / ((va / na) ** 2 / (na - 1) + (vb / nb) ** 2 / (nb - 1))
    tcrit = t_ppf(1 - alpha / 2, df)
    d = ma - mb; t = d / se if se > 0 else float('inf')
    p = 2 * (1 - t_cdf(abs(t), df)) if se > 0 else 0.0
    return dict(mean_a=ma, mean_b=mb, sd_a=math.sqrt(va), sd_b=math.sqrt(vb), diff=d, se=se, df=df,
                t=t, t_crit=tcrit, ci=(d - tcrit * se, d + tcrit * se), p_welch=p)


def perm_exact(a, b):
    """Exact permutation test on the difference of means over all label assignments.
    Returns (p_abs, p_2x1s, p_min_2x1s, n_splits):
      p_abs   = share of splits with |diff| >= |observed|   (two-sided by absolute value)
      p_2x1s  = 2 x one-sided tail (diff >= observed, for observed > 0), capped at 1 -- the doubled
                one-sided convention used in AUDIT_results_2026-08-28; its minimum attainable value is
                2/n_splits regardless of n_a == n_b, which is the design-power number to quote."""
    pool = a + b; na = len(a); nb = len(b)
    obs = sum(a) / na - sum(b) / nb
    idx = range(len(pool)); total = 0; ge_abs = 0; ge_one = 0
    for comb in itertools.combinations(idx, na):
        s = set(comb); xa = [pool[i] for i in comb]; xb = [pool[i] for i in idx if i not in s]
        d = sum(xa) / na - sum(xb) / nb; total += 1
        if abs(d) >= abs(obs) - 1e-12: ge_abs += 1
        if (d >= obs - 1e-12) if obs >= 0 else (d <= obs + 1e-12): ge_one += 1
    return ge_abs / total, min(1.0, 2 * ge_one / total), 2 / total, total


def report(a, b, label_a='A', label_b='B'):
    w = welch(a, b); p, p2, pmin, n = perm_exact(a, b)
    rel = (w['mean_a'] - w['mean_b']) / w['mean_a'] if w['mean_a'] else float('nan')
    print(f'{label_a}: n={len(a)} mean {w["mean_a"]:.3f} sd {w["sd_a"]:.3f}   {label_b}: n={len(b)} mean {w["mean_b"]:.3f} sd {w["sd_b"]:.3f}')
    print(f'diff {w["diff"]:+.3f}  se {w["se"]:.3f}  Welch df {w["df"]:.2f}  t_crit {w["t_crit"]:.3f}  '
          f'95% CI [{w["ci"][0]:+.3f}, {w["ci"][1]:+.3f}]  Welch p {w["p_welch"]:.3f}')
    print(f'exact permutation p: |diff| {p:.3f}, 2x one-sided {p2:.3f} ({n} splits; minimum attainable 2x1s {pmin:.3f})  '
          f'relative loss {100*rel:.1f}%')
    return w, p2, pmin


def selftest():
    # N15 hold (RESCORE-RESULT, 2026-08-27): dR2D 15,7,10,14 /15 vs +fails 11,9,4 /15
    a = parse('15/15 7/15 10/15 14/15'.split()); b = parse('11/15 9/15 4/15'.split())
    w, p, pmin = report(a, b, 'dR2D', 'dR2D+DPfails')
    assert abs(w['diff'] - 0.2333) < 1e-3 and abs(w['df'] - 4.52) < 0.05, w
    assert abs(w['ci'][0] + 0.259) < 0.003 and abs(w['ci'][1] - 0.726) < 0.003, w['ci']
    assert abs(p - 10 / 35) < 1e-9 and abs(pmin - 2 / 35) < 1e-9, (p, pmin)
    # rnd
    a = parse('19/30 13/30 19/30 26/30'.split()); b = parse('21/30 12/30 10/30'.split())
    w, p, pmin = report(a, b, 'dR2D', 'dR2D+DPfails')
    assert abs(w['ci'][0] + 0.228) < 0.003 and abs(w['ci'][1] - 0.556) < 0.003, w['ci']
    # t quantile sanity vs known table values
    assert abs(t_ppf(0.975, 4) - 2.776) < 0.002 and abs(t_ppf(0.975, 2.16) - 4.012) < 0.01
    print('selftest OK (reproduces AUDIT_results_2026-08-28 §1)')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--a', nargs='*'); ap.add_argument('--b', nargs='*')
    ap.add_argument('--label-a', default='A'); ap.add_argument('--label-b', default='B'); ap.add_argument('--selftest', action='store_true')
    x = ap.parse_args()
    if x.selftest: return selftest()
    if not x.a or not x.b: sys.exit('need --a and --b (per-seed values, as k/n or floats)')
    report(parse(x.a), parse(x.b), x.label_a, x.label_b)


if __name__ == '__main__':
    main()
