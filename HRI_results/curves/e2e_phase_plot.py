#!/usr/bin/env python3
"""Per-phase acquisition curves for the e2e long runs (amendment (x) ladder).

usage: e2e_phase_plot.py <curves.csv> <out_prefix>

The stock plot_curves.py has a fixed panel layout with no e2e placed_v2 / contact_push /
slide panels, so the stages that answer "acquisition of pick, placement and accepted slide"
were computed but never drawn. This draws them.

These are ONLINE TRAINING ROLLOUTS from the exploring policy resetting from the TRAINING
bank -- NOT the evaluation protocol. An endpoint here is not a table number.
"""
import csv, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

src, prefix = sys.argv[1], sys.argv[2]
rows = [r for r in csv.DictReader(l for l in open(src) if not l.startswith('#'))
        if r['family'] == 'e2e']
# Stage aliases: r2dreamer's reader names slide/nested one way, the RLPD record another.
# Accept either so ONE plotter draws both learners rather than two divergent figures.
STAGES = [(('picked',), 'pick'), (('placed_v2',), 'placement'), (('contact',), 'contact (legacy)'),
          (('contact_push',), 'contact_push'),
          (('slide_within_episode_diagnostic', 'slide_success'), 'slide'),
          (('nested_proxy', 'nested'), 'nested (proxy)')]
LEARNER = sys.argv[3] if len(sys.argv) > 3 else 'r2dreamer'
fig, axes = plt.subplots(2, 3, figsize=(16, 8.5))
for ax, (names, title) in zip(axes.ravel(), STAGES):
    for arm, colour in (('human', '#1f5fa8'), ('machine', '#c1392b')):
        d = sorted((float(r['step']), float(r['mean']), float(r['se']))
                   for r in rows if r['stage'] in names and r['arm'] == arm)
        if not d:
            continue
        x = [p[0] for p in d]; m = [p[1] for p in d]; se = [p[2] for p in d]
        n = {r['n_seeds'] for r in rows if r['stage'] in names and r['arm'] == arm}
        ax.plot(x, m, color=colour, lw=2, label=f'{arm} (mean ± SE, {sorted(n)[0]} seeds)')
        ax.fill_between(x, [a - b for a, b in zip(m, se)], [a + b for a, b in zip(m, se)],
                        color=colour, alpha=0.18, lw=0)
    ax.set_title(f'e2e — {title}'); ax.set_xlabel('env step (training)')
    ax.set_ylabel('online rollout rate'); ax.set_ylim(-0.02, 1.02)
    ax.grid(alpha=0.3); ax.legend(fontsize=8, loc='upper left')
fig.suptitle('{%s} end-to-end phase acquisition' % LEARNER + '  — ONLINE TRAINING ROLLOUTS, not evaluation',
             fontsize=13)
_prov = (' Records carry no record_valid certificate (its emitter was never deployed); validated structurally instead — 60,233 rows, 0 missing stages, 0 non-binary, 0 implication violations.'
         if LEARNER == 'r2dreamer' else
         ' Records are the amendment (u) per-episode rollout log (episode_rollouts.jsonl); sticky flags are accumulated across each episode, so a stage counts wherever it occurred, not only at the final step.')
fig.text(0.5, 0.005,
         f'{LEARNER}: ONLINE TRAINING ROLLOUTS from the exploring policy, resetting from the TRAINING bank. '
         'DP is offline (no online rollouts) and cannot produce this curve.\n'
         'Endpoints are NOT the table numbers (eval uses mode/sampled actions on rnd30/spots60, final checkpoint).\n'
         + _prov,
         ha='center', fontsize=8, color='#444')
fig.tight_layout(rect=(0, 0.06, 1, 0.96))
for ext in ('png', 'pdf'):
    fig.savefig(f'{prefix}.{ext}', dpi=150)
print(f'wrote {prefix}.png/.pdf')
