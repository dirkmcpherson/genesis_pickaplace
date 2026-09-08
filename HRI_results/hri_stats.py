#!/usr/bin/env python3
"""Statistics for HRI_results: the frequentist trio (exact permutation p, MDE, CI) and a
hierarchical Bayesian equivalence posterior.  Imported by make_tables.py; runnable as a
self-test (`python3 hri_stats.py`).

WHY THESE STATISTICS
--------------------
Nearly every comparison in this project is a NULL and the registered prediction is an
EQUIVALENCE claim, |Δ| < 0.10.  A p-value cannot support equivalence: p = 0.875 says only that
we failed to detect a difference.  So each cell gets

  * the exact two-sided permutation test on per-seed counts  -- unchanged, it is the project's
    registered statistic and this file must not alter it;
  * the minimum detectable effect at 80 % power, which is what makes a null interpretable
    (a null with an MDE of 0.35 has excluded almost nothing);
  * a 95 % CI on Δ;
  * a posterior P(Δ ∈ ROPE) with ROPE = the registered ±0.10 margin, which is the quantity the
    registration actually asks about, plus P(Δ > 0) and an interval Bayes factor BF01.

THE MODEL, AND WHY IT IS NOT A POOLED BETA-BINOMIAL
---------------------------------------------------
Seed-to-seed spread dominates the variance here: within a single arm the picked count ranges
8-19 of 30, and several arms carry a dead seed (0-2 of 30).  Pooling the seeds into one
beta-binomial would treat 240 episodes as 240 independent Bernoulli draws, understate the
standard error several-fold and manufacture false confidence -- exactly the error the
equivalence claim must not make.  So the model puts a random effect on the seed:

    k_aj ~ Binomial(n_aj, p_aj)                       seed j of arm a
    logit(p_aj) = mu_a + sigma * eps_aj,   eps_aj ~ N(0, 1)

`mu_a` is the arm's latent mean on the logit scale and `sigma` is the seed-level SD, SHARED
across the two arms of a comparison (primary).  Sharing sigma is the stable choice at n = 8:
the contrast of interest is between the means, and giving each arm its own dispersion parameter
at 8 seeds makes both poorly identified.  `--sep-sigma` fits one sigma per arm as a sensitivity
check.

ASSUMPTIONS, STATED PLAINLY
  * Seeds are exchangeable within an arm and the seed effect is Gaussian on the logit scale.
    A dead seed is therefore modelled as a draw from the same distribution's lower tail, not as
    a separate failure mode.  Where dead seeds drive a result this is the model's weakest point
    and the cell is flagged.
  * Episodes within a seed are conditionally independent given that seed's rate.  Evaluation
    starts are shared across arms (a fixed bank), so this ignores start-level pairing -- it
    makes the intervals CONSERVATIVE for the paired contrast, not anti-conservative.
  * The estimand is the difference of POPULATION-AVERAGED success rates,
    r_a = E_eps[logistic(mu_a + sigma*eps)], integrated over the seed distribution, because the
    registered +/-0.10 margin is a statement about rates, not about logits.

The posterior is computed by deterministic grid quadrature over (mu_H, mu_M, sigma) -- no MCMC,
so results are reproducible bit-for-bit and there is no convergence to diagnose.  The two arms
are conditionally independent given sigma, which is what makes the grid cheap.
"""
import itertools
import math

import numpy as np
from scipy.special import gammaln, logsumexp
from scipy.stats import t as student_t

# ------------------------------------------------------------------ frequentist

MAX_EXACT_PERMS = 400_000


def perm_test(a, b, n_a=None, n_b=None, seed=20260908, max_exact=MAX_EXACT_PERMS):
    """Exact two-sided permutation test on per-seed counts -- the project's statistic of record.

    `a`, `b` are per-seed success counts.  Denominators must be constant within a cell (they are,
    for every cell of record); the statistic is the difference of arm mean RATES.  Enumerates all
    C(n_a+n_b, n_a) splits when that is <= max_exact, otherwise Monte-Carlo samples that many.
    Returns (delta_rate, p, n_perms, exact).
    """
    a = np.asarray(a, float); b = np.asarray(b, float)
    na, nb = len(a), len(b)
    if na < 2 or nb < 2:
        return (float('nan'), float('nan'), 0, False)
    d_a = float(n_a) if n_a else 1.0
    d_b = float(n_b) if n_b else 1.0
    if d_a != d_b:
        raise ValueError("permutation test of record assumes one denominator per cell")
    obs = a.mean() / d_a - b.mean() / d_b
    pool = np.concatenate([a, b])
    total = math.comb(na + nb, na)
    if total <= max_exact:
        cnt = 0
        idx_all = range(na + nb)
        for idx in itertools.combinations(idx_all, na):
            m = np.zeros(na + nb, bool); m[list(idx)] = True
            d = pool[m].mean() / d_a - pool[~m].mean() / d_b
            cnt += abs(d) >= abs(obs) - 1e-12
        return (obs, cnt / total, total, True)
    rng = np.random.default_rng(seed)
    cnt = 0
    for _ in range(max_exact):
        p = rng.permutation(pool)
        d = p[:na].mean() / d_a - p[na:].mean() / d_b
        cnt += abs(d) >= abs(obs) - 1e-12
    return (obs, (cnt + 1) / (max_exact + 1), max_exact, False)


def mde_ci(a, b, n_a, n_b, alpha=0.05, power=0.80):
    """Minimum detectable effect at `power` and a (1-alpha) CI on Delta, on the RATE scale.

    Per-seed rates are the unit of analysis.  Uses the pooled-SD two-sample convention
        MDE = (t_{1-alpha/2,df} + t_{power,df}) * s_pooled * sqrt(1/na + 1/nb)
        CI  = Delta +/- t_{1-alpha/2,df} * s_pooled * sqrt(1/na + 1/nb)
    which is the convention the project's own review numbers were computed with (it reproduces
    the recorded RLPD pick pair: MDE 0.345, CI +/-0.245).
    """
    ra = np.asarray(a, float) / float(n_a)
    rb = np.asarray(b, float) / float(n_b)
    na, nb = len(ra), len(rb)
    if na < 2 or nb < 2:
        return dict(delta=float('nan'), mde=float('nan'), ci_lo=float('nan'), ci_hi=float('nan'),
                    se=float('nan'), sd_a=float('nan'), sd_b=float('nan'))
    df = na + nb - 2
    sp2 = (((na - 1) * ra.var(ddof=1)) + ((nb - 1) * rb.var(ddof=1))) / df
    se = math.sqrt(sp2 * (1.0 / na + 1.0 / nb))
    if se == 0.0:
        # every seed inside each arm gave the identical count. The sample SD is then exactly 0
        # and the CI collapses to a point -- an artefact of tiny n, not precision. Report the
        # effect and refuse the interval rather than print a zero-width one.
        return dict(delta=ra.mean() - rb.mean(), mde=float('nan'), ci_lo=float('nan'),
                    ci_hi=float('nan'), se=0.0, sd_a=0.0, sd_b=0.0, degenerate=True)
    t_a = student_t.ppf(1 - alpha / 2, df)
    t_p = student_t.ppf(power, df)
    d = ra.mean() - rb.mean()
    return dict(delta=d, mde=(t_a + t_p) * se, ci_lo=d - t_a * se, ci_hi=d + t_a * se,
                se=se, sd_a=ra.std(ddof=1), sd_b=rb.std(ddof=1))


# ------------------------------------------------------------------ Bayesian

PRIORS = {
    # name: (sd of mu on the logit scale, scale of the half-normal on sigma)
    'primary': (1.5, 1.0),
    'wide':    (3.0, 2.0),
    'tight':   (1.0, 0.5),
}


def _log_sigmoid(x):
    return -np.logaddexp(0.0, -x)


def _binom_loglik_grid(k, n, theta):
    """log Binom(k | n, logistic(theta)) for an array of theta, dropping no constants."""
    c = gammaln(n + 1) - gammaln(k + 1) - gammaln(n - k + 1)
    return c + k * _log_sigmoid(theta) + (n - k) * _log_sigmoid(-theta)


_LOGLIK_CACHE = {}
_PRIOR_ROPE_CACHE = {}


def bayes_equivalence(a_k, a_n, b_k, b_n, rope=0.10, prior='primary',
                      n_mu=201, n_sigma=71, n_gh=32, sep_sigma=False):
    """Hierarchical (seed random-effects) posterior for Delta = r_human - r_machine.

    Returns P(Delta in ROPE), P(Delta > 0), an interval Bayes factor BF01 for
    H0: |Delta| < rope against H1: |Delta| >= rope, the posterior mean and a 95 % credible
    interval.  Deterministic grid quadrature; `prior` selects a row of PRIORS.
    """
    mu_sd, sig_scale = PRIORS[prior] if isinstance(prior, str) else prior
    a_k = np.asarray(a_k, float); b_k = np.asarray(b_k, float)
    if len(a_k) < 2 or len(b_k) < 2:
        return None

    mu = np.linspace(-8.0, 8.0, n_mu)
    sig = np.linspace(1e-3, max(4.0, 3 * sig_scale), n_sigma)
    gh_x, gh_w = np.polynomial.hermite_e.hermegauss(n_gh)     # weight exp(-x^2/2)
    log_gh_w = np.log(gh_w) - 0.5 * math.log(2 * math.pi)     # normalises to a standard normal

    # theta[m, s, g] = mu_m + sigma_s * x_g
    theta = mu[:, None, None] + sig[None, :, None] * gh_x[None, None, :]

    def arm_loglik(ks, n):
        # The marginal log-likelihood surface depends only on the DATA and the (mu, sigma, GH)
        # grid -- never on the prior. It was being recomputed for every prior variant and for the
        # separate-sigma refit: four identical passes per arm, which was the whole cost of a
        # rebuild. Cache it. Identical inputs, identical outputs, no numerical change.
        key = (tuple(sorted(ks)), float(n), n_mu, n_sigma, n_gh, float(sig[-1]))
        hit = _LOGLIK_CACHE.get(key)
        if hit is not None:
            return hit
        out = np.zeros((n_mu, n_sigma))
        for k in ks:
            out += logsumexp(log_gh_w[None, None, :] + _binom_loglik_grid(k, float(n), theta),
                             axis=2)
        _LOGLIK_CACHE[key] = out
        return out

    # population-averaged rate r(mu, sigma) = E_eps[logistic(mu + sigma eps)]
    rate = np.exp(logsumexp(log_gh_w[None, None, :] + _log_sigmoid(theta), axis=2))

    log_prior_mu = -0.5 * (mu / mu_sd) ** 2
    log_prior_sig = -0.5 * (sig / sig_scale) ** 2            # half-normal (sigma > 0 by grid)

    LA = arm_loglik(a_k, a_n)
    LB = arm_loglik(b_k, b_n)

    def accumulate(use_lik):
        """Weighted posterior (or prior) mass -> flat arrays of (delta, weight).

        Shared sigma: the two arms are conditionally independent GIVEN sigma, so for each sigma
        the joint over (mu_H, mu_M) is an outer sum and delta is an outer difference of rates.
        Separate sigma: the model factorises completely, so each arm is marginalised over its own
        (mu, sigma) and the two rate distributions are combined on a common rate grid.
        """
        la_lik = LA if use_lik else np.zeros_like(LA)
        lb_lik = LB if use_lik else np.zeros_like(LB)
        if not sep_sigma:
            deltas, weights = [], []
            for si in range(n_sigma):
                la = la_lik[:, si] + log_prior_mu
                lb = lb_lik[:, si] + log_prior_mu
                m = la[:, None] + lb[None, :] + log_prior_sig[si]
                d = rate[:, si][:, None] - rate[:, si][None, :]
                deltas.append(d.ravel()); weights.append(m.ravel())
            d = np.concatenate(deltas); w = np.concatenate(weights)
            w = np.exp(w - w.max())
            return d, w / w.sum()
        # separate sigma: marginalise each arm independently, then difference on a rate grid
        def arm_hist(lik):
            m = lik + log_prior_mu[:, None] + log_prior_sig[None, :]
            w = np.exp(m - m.max()).ravel()
            w /= w.sum()
            idx = np.clip((rate.ravel() * (NBIN - 1)).astype(int), 0, NBIN - 1)
            h = np.zeros(NBIN)
            np.add.at(h, idx, w)
            return h
        NBIN = 401
        ha, hb = arm_hist(la_lik), arm_hist(lb_lik)
        grid = np.linspace(0.0, 1.0, NBIN)
        d = (grid[:, None] - grid[None, :]).ravel()
        w = (ha[:, None] * hb[None, :]).ravel()
        return d, w / w.sum()

    d_post, w_post = accumulate(True)

    # The PRIOR mass in the ROPE is data-independent: it depends only on the prior and the grid,
    # so it is identical for every comparison and was being recomputed ~45 times per prior. Only
    # the scalar is needed (for BF01), so cache that.
    pkey = (mu_sd, sig_scale, rope, n_mu, n_sigma, n_gh, bool(sep_sigma))
    p_rope_prior = _PRIOR_ROPE_CACHE.get(pkey)
    if p_rope_prior is None:
        d_pri, w_pri = accumulate(False)
        p_rope_prior = float(w_pri[np.abs(d_pri) < rope].sum())
        _PRIOR_ROPE_CACHE[pkey] = p_rope_prior

    p_rope = float(w_post[np.abs(d_post) < rope].sum())
    p_gt0 = float(w_post[d_post > 0].sum())

    def odds(p):
        p = min(max(p, 1e-12), 1 - 1e-12)
        return p / (1 - p)
    bf01 = odds(p_rope) / odds(p_rope_prior)

    order = np.argsort(d_post)
    ds, ws = d_post[order], w_post[order]
    cw = np.cumsum(ws)
    q = lambda t: float(np.interp(t, cw, ds))
    return dict(p_rope=p_rope, p_gt0=p_gt0, bf01=bf01, prior=prior,
                mean=float((d_post * w_post).sum()), lo=q(0.025), hi=q(0.975),
                p_rope_prior=p_rope_prior)


# ------------------------------------------------------------------ self-test
if __name__ == '__main__':
    # 1. permutation reproduces the recorded r2dreamer pick cell (p = 0.875)
    h = [15, 19, 19, 19, 21, 17, 20, 18]; m = [20, 18, 18, 18, 16, 19, 19, 18]
    d, p, n, ex = perm_test(h, m, 30, 30)
    print(f"perm r2d pick: delta={d:+.4f} p={p:.3f} perms={n} exact={ex}   [doc: +0.008, 0.875]")
    assert abs(p - 0.875) < 1e-3 and abs(d - 0.00833) < 1e-4

    # 2. MDE/CI reproduce the recorded RLPD pick pair (MDE 0.345, CI +/-0.245)
    rh = [23, 20, 18, 20, 18, 2, 24, 19]; rm = [18, 19, 17, 19, 20, 0, 20, 11]
    s = mde_ci(rh, rm, 30, 30)
    print(f"RLPD pick: delta={s['delta']:+.3f} MDE={s['mde']:.3f} CI=+/-{(s['ci_hi']-s['ci_lo'])/2:.3f}"
          f"   [doc: MDE 0.345, CI +/-0.245]")
    assert abs(s['mde'] - 0.345) < 0.005 and abs((s['ci_hi'] - s['ci_lo']) / 2 - 0.245) < 0.005

    # 3. Bayesian: a tight null should sit almost entirely in the ROPE; a large gap should not.
    b = bayes_equivalence(h, 30, m, 30)
    print(f"bayes r2d pick: P(rope)={b['p_rope']:.3f} P(>0)={b['p_gt0']:.3f} BF01={b['bf01']:.1f} "
          f"mean={b['mean']:+.3f} CI[{b['lo']:+.3f},{b['hi']:+.3f}]")
    assert b['p_rope'] > 0.85
    big_h = [41, 44, 39, 46, 45, 42, 43, 45]; big_m = [6, 1, 8, 2, 4, 6, 6, 5]
    bb = bayes_equivalence(big_h, 50, big_m, 50)
    print(f"bayes DP robomimic: P(rope)={bb['p_rope']:.3f} BF01={bb['bf01']:.3g} mean={bb['mean']:+.3f}")
    assert bb['p_rope'] < 0.01 and bb['mean'] > 0.5

    # 4. the pooled-vs-hierarchical point: pooling must not be used, show it understates SE
    import numpy as _np
    pooled_se = math.sqrt(0.5 * 0.5 * (1 / 240 + 1 / 240))
    print(f"pooled binomial SE={pooled_se:.4f} vs seed-level SE={s['se']:.4f} "
          f"({s['se']/pooled_se:.1f}x larger) -- why a pooled beta-binomial is not used")
    print("SELF-TEST PASS")
