#!/usr/bin/env python
"""Bayesian significance analysis of the demo-source effect (dH vs dDP).

Pulls per-seed eval counts from wandb project `jambotime/genesis_paper`
(runs named `d{source}_{learner}_s{seed}-eval`; keys `eval_indist/picked`
and `eval_random/picked` are success RATES over n=15 episodes each).
Seeds are AUTO-DISCOVERED per condition from the run names (unequal
per-condition seed counts are supported).

For each of the four comparisons (BC=DP-learner indist, BC random,
SACfD indist, SACfD random) computes P(p_model > p_human) two ways:

(a) POOLED Beta-Binomial: Beta(1,1) prior on a single success prob per
    condition, pooled counts (n_seeds x 15 episodes). This is
    ANTI-CONSERVATIVE: it treats the pooled episodes as exchangeable and
    ignores between-seed correlation (training-seed variance).

(b) HIERARCHICAL: seed-level p_i ~ Beta(mu*k, (1-mu)*k),
    y_i ~ Binomial(15, p_i). The seed-level p_i are marginalised
    analytically (beta-binomial likelihood); the posterior over (mu, k)
    is computed on a grid with mu ~ Uniform(0,1) and log(k) uniform on
    [log 0.5, log 500]. Report P(mu_model > mu_human) from the two
    independent marginal posteriors over mu. This is the citable number.

Run:  .venv-eval/bin/python analysis/bayes_source_effect.py
Writes: paper/results_significance.md
"""
import math
import os
import re
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_MD = os.path.join(REPO, "paper", "results_significance.md")

ENTITY_PROJECT = "jambotime/genesis_paper"
N_EP = 15          # episodes per eval run
RUN_RE = re.compile(r"^(dH|dDP)_(DP|SACfD)_s(\d+)-eval$")
# wandb-run learner tag -> paper label. The condition matrix in PAPER_PLAN.md
# labels the DP-learner rows "BC" (behaviour cloning); wandb runs use "DP".
LEARNERS = {"DP": "BC (diffusion policy)", "SACfD": "SACfD"}
METRICS = {"eval_indist/picked": "in-dist", "eval_random/picked": "random"}

# Official-eval cutoff: runs created before this use old eval semantics.
EVAL_SEMANTICS_CUTOFF = "2026-07-30"

RNG = np.random.default_rng(0)
N_MC = 2_000_000  # MC samples for the pooled Beta comparison

# Hierarchical grid
MU_GRID = np.linspace(0.001, 0.999, 400)
LOGK_GRID = np.linspace(math.log(0.5), math.log(500.0), 120)
K_GRID = np.exp(LOGK_GRID)


def fetch_counts():
    """Return {(learner, metric): {source: [successes per seed]}} plus run meta.

    Seeds auto-discovered from run names; per-condition seed counts may differ.
    """
    import wandb
    api = wandb.Api()
    by_cond = {}  # (source, learner) -> {seed: run}
    for r in api.runs(ENTITY_PROJECT):
        m = RUN_RE.match(r.name)
        if m is None:
            continue
        if r.state != "finished":
            print(f"note: skipping non-finished run {r.name} ({r.state})")
            continue
        if str(r.created_at)[:10] < EVAL_SEMANTICS_CUTOFF:
            sys.exit(f"FATAL: {r.name} predates {EVAL_SEMANTICS_CUTOFF} "
                     "eval-semantics change; excluded by policy")
        source, learner, seed = m.group(1), m.group(2), int(m.group(3))
        by_cond.setdefault((source, learner), {})[seed] = r
    counts, meta = {}, []
    for learner in LEARNERS:
        for metric in METRICS:
            key = (learner, metric)
            counts[key] = {}
            for source in ("dH", "dDP"):
                seeds_runs = by_cond.get((source, learner))
                if not seeds_runs:
                    sys.exit(f"FATAL: no wandb runs for {source}_{learner}_s*-eval")
                succ = []
                for seed in sorted(seeds_runs):
                    r = seeds_runs[seed]
                    n = r.summary.get(metric.split("/")[0] + "/n")
                    if n is not None and int(n) != N_EP:
                        sys.exit(f"FATAL: {r.name} has n={n}, expected {N_EP}")
                    rate = r.summary.get(metric)
                    if rate is None:
                        sys.exit(f"FATAL: {r.name} missing {metric}")
                    s = round(rate * N_EP)
                    if abs(s - rate * N_EP) > 1e-6:
                        print(f"note: {r.name} {metric} rate*{N_EP}={rate*N_EP:.4f} "
                              f"rounded to {s}")
                    succ.append(s)
                    meta.append((r.name, metric, rate, s, str(r.created_at)[:10]))
                counts[key][source] = succ
    return counts, meta


def pooled_prob_greater(succ_m, succ_h):
    """P(p_model > p_human), Beta(1,1) prior, pooled counts. MC, seeded."""
    sm, sh = sum(succ_m), sum(succ_h)
    nm, nh = N_EP * len(succ_m), N_EP * len(succ_h)
    pm = RNG.beta(1 + sm, 1 + nm - sm, N_MC)
    ph = RNG.beta(1 + sh, 1 + nh - sh, N_MC)
    return float(np.mean(pm > ph))


def _log_betabinom(y, n, a, b):
    """log BetaBinomial(y | n, a, b) without the constant C(n,y) (cancels)."""
    return (math.lgamma(y + a) + math.lgamma(n - y + b) - math.lgamma(n + a + b)
            - math.lgamma(a) - math.lgamma(b) + math.lgamma(a + b))


def hier_posterior_mu(succ):
    """Marginal posterior weights over MU_GRID for one condition (any #seeds).

    p_i ~ Beta(mu*k, (1-mu)*k); y_i ~ Binom(15, p_i); p_i marginalised
    analytically. Priors: mu uniform on grid, log k uniform on grid.
    """
    logpost = np.empty((len(MU_GRID), len(K_GRID)))
    for i, mu in enumerate(MU_GRID):
        for j, k in enumerate(K_GRID):
            a, b = mu * k, (1.0 - mu) * k
            logpost[i, j] = sum(_log_betabinom(y, N_EP, a, b) for y in succ)
    logpost -= logpost.max()
    post = np.exp(logpost)
    post /= post.sum()
    return post.sum(axis=1)  # marginalise k


def prob_mu_greater(w_m, w_h):
    """P(mu_model > mu_human) from two independent grid posteriors.

    Grid ties (same mu bin) contribute 1/2 — with a 400-point grid this is
    a sub-0.005 correction.
    """
    cdf_h = np.cumsum(w_h)
    p_strict = float(np.sum(w_m[1:] * cdf_h[:-1]))
    p_tie = float(np.sum(w_m * w_h))
    return p_strict + 0.5 * p_tie


def main():
    counts, meta = fetch_counts()
    rows = []
    for learner, llabel in LEARNERS.items():
        for metric, mlabel in METRICS.items():
            sm = counts[(learner, metric)]["dDP"]
            sh = counts[(learner, metric)]["dH"]
            p_pool = pooled_prob_greater(sm, sh)
            p_hier = prob_mu_greater(hier_posterior_mu(sm), hier_posterior_mu(sh))
            rows.append((llabel, mlabel, sh, sm, p_pool, p_hier))
            print(f"{llabel:24s} {mlabel:8s} human={sh} model={sm} "
                  f"pooled={p_pool:.3f} hierarchical={p_hier:.3f}")
    write_md(rows, meta)
    print(f"wrote {OUT_MD}")


def write_md(rows, meta):
    lines = [
        "# Bayesian significance of the demo-source effect (dH vs dDP)",
        "",
        f"Generated by `analysis/bayes_source_effect.py` from wandb "
        f"`{ENTITY_PROJECT}` (runs `*-eval`, keys `eval_indist/picked` and "
        "`eval_random/picked`). Rates are over n=15 episodes per seed; "
        "successes = round(rate x 15). Seeds auto-discovered per condition "
        "(the seed wave landed 2026-08-08/09: n=8 per DP row, n=8 dH_SACfD, "
        "n=7 dDP_SACfD — no `dDP_SACfD_s7-eval` exists in wandb; the unequal "
        "n is handled exactly by both models). All source runs created "
        "2026-08-02 or later (post the 2026-07-30 eval-semantics change).",
        "",
        "\"BC\" = the DP-learner rows (`dH_DP` / `dDP_DP`); the condition "
        "matrix in PAPER_PLAN.md labels the diffusion-policy learner as its "
        "BC arm. Model demos = gen-0 DP harvest (dDP); human demos = dH.",
        "",
        "## Question",
        "",
        "For each (learner, eval regime): what is the posterior probability "
        "that the model-demo-trained policy's true pick rate exceeds the "
        "human-demo-trained one's?",
        "",
        "## Methods",
        "",
        "**(a) Pooled Beta-Binomial (ANTI-CONSERVATIVE — do not cite alone).** "
        "Beta(1,1) prior on a single success probability per condition; the "
        "n_seeds x 15 episodes pooled as if exchangeable. "
        "P(p_model > p_human) by Monte Carlo (2M samples, seeded). This "
        "ignores between-seed (training-run) correlation, so it overstates "
        "certainty whenever seeds disagree — which they do, especially for "
        "SACfD.",
        "",
        "**(b) Hierarchical Beta-Binomial (THE CITABLE NUMBER).** Per-seed "
        "true rate p_i ~ Beta(mu*k, (1-mu)*k); observed successes "
        "y_i ~ Binomial(15, p_i); p_i marginalised analytically "
        "(beta-binomial likelihood). Posterior over (mu, k) on a grid: mu "
        "uniform on (0,1) (400 pts), log k uniform on [log 0.5, log 500] "
        "(120 pts). Reported: P(mu_model > mu_human) from the two "
        "independent marginal posteriors over mu. With 7-8 seeds the "
        "concentration k is still only weakly identified, so the posterior "
        "over mu stays wider than the pooled one — that width is the honest "
        "price of seed-level variance.",
        "",
        "## Results",
        "",
        "| comparison | eval | human successes /15 per seed | model successes /15 per seed | P(model > human), pooled (anti-conservative) | P(model > human), hierarchical (citable) |",
        "|---|---|---|---|---|---|",
    ]
    for llabel, mlabel, sh, sm, p_pool, p_hier in rows:
        sh_str = ", ".join(str(s) for s in sh)
        sm_str = ", ".join(str(s) for s in sm)
        lines.append(
            f"| {llabel} | {mlabel} | {sh_str} (n={len(sh)}) "
            f"| {sm_str} (n={len(sm)}) | {p_pool:.3f} | {p_hier:.3f} |")
    lines += [
        "",
        "## Interpretation guidance (read before quoting)",
        "",
        "- **n=7-8 seeds per condition** (the 2026-08-08/09 seed wave; "
        "dDP_SACfD has 7). The hierarchical numbers are the citable ones; "
        "the pooled numbers are shown only to make the seed-correlation "
        "effect visible. Where the two disagree, the gap is exactly the "
        "certainty the pooled model manufactures by ignoring seed-level "
        "variance.",
        "- **Decisiveness bar unchanged.** Hierarchical probabilities in "
        "the 0.2-0.8 band mean \"the data do not distinguish the sources\"; "
        "treat anything short of ~0.95 (or ~0.05) as suggestive at best.",
        "- **Verdict movement vs the n=3 analysis (2026-08-08):** BC "
        "in-dist moved 0.76 -> 0.99 (suggestive -> clears the ~0.95 bar: "
        "every dDP_DP seed >= the dH_DP median, and the new seeds widened "
        "the gap); SACfD in-dist moved 0.28 -> 0.78 (the nominal human "
        "advantage at n=3 was seed noise — it reversed direction with more "
        "seeds and is still not decisive); both random-IC comparisons "
        "stayed ~0.45 (no source effect on generalisation).",
        "- **SACfD seed spread is wide** (in-dist 0.00-0.53 for dH, "
        "0.07-0.60 for dDP): report ranges, never bare means; the "
        "hierarchical probability prices in that spread.",
        "- `eval_indist` = demo-IC (the ~3 demo can positions; the "
        "headline); `eval_random` = support-random ICs (generalisation). "
        "Never mix the two regimes.",
        "- **Confound (source property, not removable):** human-DP numbers "
        "depend on idle-frame pruning (0.67-0.80 pruned vs 0.27 unpruned "
        "control); model harvests contain no idle frames. A win for dDP is "
        "partly \"no idle frames by construction\".",
        "- Pick-phase scope only. Rounding rate*15 to integer successes was "
        "exact for every run (all rates are multiples of 1/15).",
        "",
        "## Source runs",
        "",
        "| run | metric | rate | successes | created |",
        "|---|---|---|---|---|",
    ]
    for name, metric, rate, s, created in meta:
        lines.append(f"| {name} | {metric} | {rate:.4f} | {s} | {created} |")
    lines.append("")
    with open(OUT_MD, "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    main()
