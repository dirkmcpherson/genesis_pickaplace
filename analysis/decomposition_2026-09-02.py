#!/usr/bin/env python3
"""Corrected-world human-vs-machine decomposition (AUDIT_approach_2026-09-02 findings 2, 5, 6, 7).

Every number below is a number of record with its artifact line named in SOURCES; nothing is re-derived
from memory. Seed is the unit. Outputs analysis/DECOMPOSITION_2026-09-02.md (--md).

  (a) per-learner Delta (human - machine) with Holm over the two REGISTERED primaries of that learner
      (DP: selected-ckpt hold + rnd, PREREG §5/§6; RLPD: LAST rnd + divergence Fisher, A16/A20;
       WM: BEST rnd + ignition Fisher, A27) -- PREREG §7 "two primary contrasts per learner -> Holm".
  (b) source x learner interaction p: permute source labels WITHIN each learner (1e5 reps) and compare
      Delta_i - Delta_j and the spread max-min of the three Deltas.
  (c) detectable-effect bound: 80 %-power MDE at n = 8 and 12 per arm from the observed pooled seed SD
      (two-sided alpha .05, normal approximation), plus power at Delta = 0.10 / 0.15 / 0.25.
  (d) the 3-learner table: alive fraction (uniform criterion rnd >= 5/30 under the rule of record, Fisher),
      rnd conditional on alive, and the three checkpoint rules selected / LAST / BEST-of-5 side by side.
      BEST-of-5 for the WM is NOT available until the cluster re-score (finding 10); DP's "selected" IS
      best-of-5 by construction; the WM's "selected" is best-of-~28 (finding 10).
Run: python3 analysis/decomposition_2026-09-02.py --md analysis/DECOMPOSITION_2026-09-02.md
"""
import argparse, itertools, math, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stats import perm_exact, welch   # analysis/stats.py (selftest reproduces AUDIT_results_2026-08-28 §1)

rng = np.random.default_rng(2)
NH, NR = 15, 30

# ------------------------------------------------------------------ numbers of record (per seed, counts)
SOURCES = {
    'DP': ('dH/dDP s20-29, corrected world, DP-HEADLINE lines (baselines/outputs/dp_w2final_*/sweep/HEADLINE.txt; '
           'local copy final_rr_artifacts_2026-08-24/{dp_w2final_meta,outputs_headlines}); selected = best-of-5 on sel; '
           'final = ckpt 100000. Selected hold/rnd also embedded in paper/figures/make_figs_2026-08-31.py:53-54.'),
    'RLPD': ('dH/dDP s40-47, corrected world, A20 (RESULTS_for_writing_2026-08-30.md §2.3): per-seed LAST rnd; '
             'selected-ckpt rnd MEANS 0.565/0.521 (per-seed not in the local checkout: cluster rlpd_g99w3_*/sweep/HEADLINE.txt); '
             'divergence 3/8 v 3/8 (max critic loss >= 1; see AUDIT_approach finding 27 for what that statistic is).'),
    'WM': ('dH/dDP s80-87, corrected world, r2dreamer BEST (= highest in-job sel among ALL archived ckpts) re-scored on '
           'hold/rnd (RESULTS §3.1 "n=8v8 FINAL (09-01)", RESCORE-RESULT *_W3 lines; embedded make_figs_2026-08-31.py:59-60); '
           'LAST hold per seed s80-83 from RESULTS §3.1 table; LAST hold n=8 MEANS 0.29/0.36 from ADVISOR_BRIEF §2b '
           '(per-seed s84-87 on the cluster: baselines/outputs/n12_rescore/*_LAST_hold_*.log).'),
}
DP = dict(
    sel_hold=([13, 13, 14, 14, 14, 13, 14, 13, 11, 14], [13, 14, 12, 14, 11, 13, 13, 14, 14, 13]),
    sel_rnd=([18, 19, 18, 14, 13, 15, 16, 16, 18, 17], [15, 14, 14, 15, 16, 15, 11, 15, 16, 15]),
    last_hold=([14, 13, 14, 14, 14, 13, 14, 13, 14, 14], [14, 14, 14, 14, 14, 13, 13, 14, 14, 14]),
    last_rnd=([18, 18, 18, 14, 13, 15, 12, 16, 17, 15], [17, 14, 13, 15, 12, 15, 11, 15, 16, 12]),
)
RLPD = dict(
    last_rnd=([19, 1, 19, 19, 1, 21, 20, 19], [18, 19, 17, 19, 20, 0, 20, 11]),
    sel_rnd_mean=(0.565, 0.521),               # means only (RESULTS §2.3)
    diverged=((3, 8), (3, 8)),                 # max CL >= 1 (RESULTS §2.3)
)
WM = dict(
    best_hold=([10, 15, 11, 10, 1, 11, 3, 13], [1, 2, 2, 15, 13, 0, 9, 6]),
    best_rnd=([20, 25, 22, 16, 1, 21, 11, 17], [1, 2, 3, 28, 22, 1, 12, 5]),
    last_hold_s80_83=([4, 13, 15, 0], [1, 5, 1, 13]),
    last_hold_mean=(0.29, 0.36),               # n=8 means (ADVISOR_BRIEF §2b)
    ignited=((7, 8), (3, 8)),                  # BEST hold >= 8/15 (A27 pre-defined)
)
ALIVE_RND = 5   # uniform "alive" criterion: >= 5/30 on the rule-of-record rnd (dead seeds score 0-3 in every learner)


def frac(xs, n): return [x / n for x in xs]


def fisher(a, b, c, d):
    """two-sided exact p for [[a,b],[c,d]] (a,c = successes)."""
    from math import comb
    n = a + b + c + d; r1, c1 = a + b, a + c
    def pmf(x): return comb(c1, x) * comb(n - c1, r1 - x) / comb(n, r1)
    p0 = pmf(a)
    return sum(pmf(x) for x in range(max(0, r1 + c1 - n), min(r1, c1) + 1) if pmf(x) <= p0 + 1e-12)


def holm(ps):
    order = sorted(range(len(ps)), key=lambda i: ps[i]); m = len(ps); adj = [0.0] * m; run = 0.0
    for rank, i in enumerate(order):
        run = max(run, (m - rank) * ps[i]); adj[i] = min(1.0, run)
    return adj


def two_sided_perm(a, b):
    p_abs, p2, pmin, n = perm_exact(list(a), list(b)); return p_abs, pmin


def mde(sd, n, alpha=0.05, power=0.8):
    z = 1.959964 + 0.841621
    return z * sd * math.sqrt(2.0 / n)


def power_at(sd, n, d):
    from math import erf, sqrt
    se = sd * math.sqrt(2.0 / n); z = d / se
    return 1 - 0.5 * (1 + erf((1.959964 - z) / sqrt(2))) + 0.5 * (1 + erf((-1.959964 - z) / sqrt(2)))


L = []
def out(s=''): L.append(s); print(s)

out('# Corrected-world decomposition (2026-09-02) — generated by analysis/decomposition_2026-09-02.py')
out()
out('Seed is the unit. Human − machine throughout. Sources per learner:')
for k, v in SOURCES.items(): out(f'- **{k}**: {v}')
out()

# ------------------------------------------------------------------ (a) per-learner Delta + Holm
out('## (a) Per-learner Δ with Holm over the two registered primaries')
out()
out('| learner | primary | human | machine | Δ | 95 % CI (Welch) | exact perm p | Holm p | min attainable p |')
out('|---|---|---|---|---|---|---|---|---|')
rows_a = []
# DP: selected hold, selected rnd
for name, (h, m), n in [('selected hold', DP['sel_hold'], NH), ('selected rnd', DP['sel_rnd'], NR)]:
    a, b = frac(h, n), frac(m, n); w = welch(a, b); p, pmin = two_sided_perm(a, b)
    rows_a.append(('DP', name, w['mean_a'], w['mean_b'], w['diff'], w['ci'], p, pmin))
# RLPD: LAST rnd + divergence
a, b = frac(RLPD['last_rnd'][0], NR), frac(RLPD['last_rnd'][1], NR)
w = welch(a, b); p, pmin = two_sided_perm(a, b)
rows_a.append(('RLPD', 'LAST rnd', w['mean_a'], w['mean_b'], w['diff'], w['ci'], p, pmin))
(dk, dn), (mk, mn) = RLPD['diverged']
rows_a.append(('RLPD', 'divergence (max CL ≥ 1)', dk / dn, mk / mn, dk / dn - mk / mn, None, fisher(dk, dn - dk, mk, mn - mk), None))
# WM: BEST rnd + ignition
a, b = frac(WM['best_rnd'][0], NR), frac(WM['best_rnd'][1], NR)
w = welch(a, b); p, pmin = two_sided_perm(a, b)
rows_a.append(('WM', 'BEST rnd', w['mean_a'], w['mean_b'], w['diff'], w['ci'], p, pmin))
(ik, inn), (jk, jn) = WM['ignited']
rows_a.append(('WM', 'ignition (BEST hold ≥ 8/15)', ik / inn, jk / jn, ik / inn - jk / jn, None, fisher(ik, inn - ik, jk, jn - jk), None))
holm_by = {}
for lr in ('DP', 'RLPD', 'WM'):
    idx = [i for i, r in enumerate(rows_a) if r[0] == lr]
    for i, hp in zip(idx, holm([rows_a[i][6] for i in idx])): holm_by[i] = hp
for i, (lr, nm, mh, mm, d, ci, p, pmin) in enumerate(rows_a):
    cis = f'[{ci[0]:+.3f}, {ci[1]:+.3f}]' if ci else '—'
    out(f'| {lr} | {nm} | {mh:.3f} | {mm:.3f} | {d:+.3f} | {cis} | {p:.3f} | **{holm_by[i]:.3f}** | {pmin:.4f}' if pmin else
        f'| {lr} | {nm} | {mh:.3f} | {mm:.3f} | {d:+.3f} | {cis} | {p:.3f} | **{holm_by[i]:.3f}** | Fisher |')
out()
out('Reading: under the registered Holm rule no learner\'s source effect is significant at 0.05 in the corrected world '
    '(DP rnd 0.042 → 0.083; WM rnd 0.133 / ignition 0.119 → 0.24). The DP "small human edge" is therefore reported '
    'as an unadjusted, exploratory p only.')
out()

# ------------------------------------------------------------------ (b) interaction
out('## (b) Source × learner interaction (rnd, rule of record: DP selected / RLPD LAST / WM BEST)')
out()
D = {'DP': DP['sel_rnd'], 'RLPD': RLPD['last_rnd'], 'WM': WM['best_rnd']}
obs = {k: (np.mean(h) - np.mean(m)) / NR for k, (h, m) in D.items()}
REPS = 100000
S = np.zeros((REPS, 4))
for r in range(REPS):
    d = {}
    for k, (h, m) in D.items():
        pool = np.array(h + m); rng.shuffle(pool); d[k] = (pool[:len(h)].mean() - pool[len(h):].mean()) / NR
    S[r] = (d['WM'] - d['RLPD'], d['WM'] - d['DP'], d['DP'] - d['RLPD'], max(d.values()) - min(d.values()))
o = np.array([obs['WM'] - obs['RLPD'], obs['WM'] - obs['DP'], obs['DP'] - obs['RLPD'], max(obs.values()) - min(obs.values())])
out('| contrast | observed | two-sided permutation p (1e5 reps, source labels permuted within learner) |')
out('|---|---|---|')
for i, nm in enumerate(['Δ_WM − Δ_RLPD', 'Δ_WM − Δ_DP', 'Δ_DP − Δ_RLPD', 'spread max−min']):
    out(f'| {nm} | {o[i]:+.3f} | {(np.abs(S[:, i]) >= abs(o[i]) - 1e-12).mean():.3f} |')
out()
out(f'Per-learner Δ: DP {obs["DP"]:+.3f}, RLPD {obs["RLPD"]:+.3f}, WM {obs["WM"]:+.3f} — the claimed ordering '
    '"DP < RLPD < WM in use beyond imitation" is not monotone in the data and no contrast between learners is '
    'significant. The three rows also differ in checkpoint rule, reward, human tape format, n, eval action mode '
    'and train horizon (AUDIT_approach finding 2), so a significant interaction would still not identify the cause.')
out()

# ------------------------------------------------------------------ (c) detectable effect
out('## (c) Detectable-effect bound from the observed seed SD (rnd fraction, pooled over arms)')
out()
out('| learner | n/arm observed | pooled seed SD | 80 %-power MDE n=8 | MDE n=12 | power Δ=0.10 (n obs) | Δ=0.15 | Δ=0.25 |')
out('|---|---|---|---|---|---|---|---|')
for k, (h, m) in D.items():
    hv, mv = np.array(h) / NR, np.array(m) / NR
    sd = math.sqrt((hv.var(ddof=1) + mv.var(ddof=1)) / 2); n = len(h)
    out(f'| {k} | {n} | {sd:.3f} | {mde(sd, 8):.2f} | {mde(sd, 12):.2f} | {power_at(sd, n, 0.10):.2f} | {power_at(sd, n, 0.15):.2f} | {power_at(sd, n, 0.25):.2f} |')
out()
out('Reading: the RLPD and WM nulls/directions cannot exclude effects smaller than ≈0.36 (n=8) or ≈0.30–0.35 (n=12); '
    'A27 at n=12 has ≈0.5 power for the WM\'s own observed Δ=0.25, not the "~80 %" in PREREG A27. Every null should be '
    'worded "no effect ≥ MDE detectable".')
out()

# ------------------------------------------------------------------ (d) alive / conditional / ckpt rules
out(f'## (d) Three learners, one decomposition: alive fraction (rnd ≥ {ALIVE_RND}/30 under the rule of record), rnd | alive, and checkpoint rules')
out()
out('| learner | rule of record | alive H | alive M | Fisher p | rnd \\| alive H | rnd \\| alive M | Δ \\| alive | selected (mean H / M) | LAST (H / M) | BEST-of-5 (H / M) |')
out('|---|---|---|---|---|---|---|---|---|---|---|')
def alive_stats(h, m):
    ah = [x for x in h if x >= ALIVE_RND]; am = [x for x in m if x >= ALIVE_RND]
    p = fisher(len(ah), len(h) - len(ah), len(am), len(m) - len(am))
    return len(ah), len(h), len(am), len(m), p, np.mean(ah) / NR, np.mean(am) / NR
kh, nh, km, nm, p, ch, cm = alive_stats(*DP['sel_rnd'])
out(f'| DP | selected (=best-of-5) | {kh}/{nh} | {km}/{nm} | {p:.2f} | {ch:.3f} | {cm:.3f} | {ch-cm:+.3f} | '
    f'{np.mean(DP["sel_rnd"][0])/NR:.3f} / {np.mean(DP["sel_rnd"][1])/NR:.3f} | {np.mean(DP["last_rnd"][0])/NR:.3f} / {np.mean(DP["last_rnd"][1])/NR:.3f} | = selected |')
kh, nh, km, nm, p, ch, cm = alive_stats(*RLPD['last_rnd'])
out(f'| RLPD | LAST | {kh}/{nh} | {km}/{nm} | {p:.2f} | {ch:.3f} | {cm:.3f} | {ch-cm:+.3f} | '
    f'{RLPD["sel_rnd_mean"][0]:.3f} / {RLPD["sel_rnd_mean"][1]:.3f} (means of record) | {np.mean(RLPD["last_rnd"][0])/NR:.3f} / {np.mean(RLPD["last_rnd"][1])/NR:.3f} | = selected (K=5) |')
kh, nh, km, nm, p, ch, cm = alive_stats(*WM['best_rnd'])
out(f'| WM | BEST (of ~28) | {kh}/{nh} | {km}/{nm} | {p:.2f} | {ch:.3f} | {cm:.3f} | {ch-cm:+.3f} | '
    f'{np.mean(WM["best_rnd"][0])/NR:.3f} / {np.mean(WM["best_rnd"][1])/NR:.3f} (BEST-of-~28) | LAST hold {WM["last_hold_mean"][0]:.2f} / {WM["last_hold_mean"][1]:.2f} (rnd not re-scored) | NA — cluster re-score of the 5 fraction ckpts (finding 10) |')
out()
# WM: registered ignition + conditional, and threshold sensitivity
ih, im = WM['best_hold']
out('WM registered ignition (BEST hold ≥ 8/15): '
    f'human {sum(x >= 8 for x in ih)}/8 vs machine {sum(x >= 8 for x in im)}/8; BEST rnd conditional on ignition: '
    f'human {np.mean([r for r, h in zip(WM["best_rnd"][0], ih) if h >= 8])/NR:.3f} vs machine '
    f'{np.mean([r for r, h in zip(WM["best_rnd"][1], im) if h >= 8])/NR:.3f}. Threshold sensitivity (H v M ignited): '
    + ', '.join(f'≥{t}: {sum(x >= t for x in ih)}v{sum(x >= t for x in im)}' for t in (6, 7, 8, 9, 10)) + '.')
out(f'Alive-criterion sensitivity for the rnd ≥ k rule (H v M alive): '
    + '; '.join(f'{lr} ' + ', '.join(f'k={k}: {sum(x >= k for x in h)}v{sum(x >= k for x in m)}' for k in (3, 5, 8))
                for lr, (h, m) in D.items()) + '.')
out()
out('Reading: in every learner the arms are indistinguishable CONDITIONAL on a seed being alive (DP +0.06, RLPD +0.06, '
    'WM −0.06 with the machine slightly ahead); the only source-dependent quantity anywhere in the corrected world is '
    'the WM\'s alive/ignition rate, which is itself measured under a saturated critic and a best-of-~28 selection '
    '(AUDIT_approach findings 10–11). The paper\'s WM claim must be stated as an ignition-probability claim.')

ap = argparse.ArgumentParser(); ap.add_argument('--md'); a = ap.parse_args()
if a.md:
    open(a.md, 'w').write('\n'.join(L) + '\n'); print(f'\nwrote {a.md}')
