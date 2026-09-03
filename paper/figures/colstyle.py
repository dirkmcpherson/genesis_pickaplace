"""Shared single-column style for every paper figure (added 2026-09-02, user Q6).

Target: one column of a two-column conference paper = 3.35 in wide, <= 3 in tall unless the
content needs two rows. Base font 8 pt, nothing below 7 pt, titles one line, short axis labels,
no redundant legends. Conventions unchanged (memory: run-naming-convention): COLOUR = algorithm
(DP blue, RLPD red, r2dreamer/WM green, dv3 purple), MARKER = demo source (human circle,
machine triangle), every seed drawn as a point, mean = thick bar, violin only when n >= 6.

The human-demo label for DP carries an asterisk (user Q5): DP trains on the leading-idle-PRUNED
human tapes (dHpruned = dHv2), RLPD/WM on the RAW tapes (dH = dHv2raw) in the v2 blocks; the
frozen blocks fed the pruned base to every learner (PREREG A24/A25). FOOT_DP is the footnote.
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

W = 3.35                                     # column width, inches
C = dict(DP='#1f77b4', RLPD='#d62728', R2D='#2ca02c', WM='#2ca02c', DV3='#9467bd',
         old='#7f7f7f', w3='#2ca02c', real='#d62728',          # colour = world (tape figures)
         human='#d95f02', machine='#7570b3')                   # fig10 only (source colours)
MK = dict(human='o', machine='^', dH='o', dDP='^')
HUMAN_DP = 'human*'                          # DP's human arm is the pruned set
FOOT_DP = ('*DP: leading-idle-pruned human tapes (frozen blocks: all\n'
           'learners pruned; v2: DP pruned, RLPD/WM raw)')
SRC_LEGEND = '○ human   △ machine'


def setup():
    plt.rcParams.update({
        'font.size': 8, 'axes.titlesize': 8.5, 'axes.labelsize': 8,
        'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5, 'legend.fontsize': 7,
        'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': 0.7,
        'xtick.major.size': 2.5, 'ytick.major.size': 2.5, 'xtick.major.pad': 2, 'ytick.major.pad': 2,
        'xtick.major.width': 0.7, 'ytick.major.width': 0.7, 'lines.linewidth': 1.2,
        'figure.dpi': 200, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
        'axes.titlepad': 4, 'legend.frameon': False, 'legend.handletextpad': 0.4,
        'legend.columnspacing': 0.8, 'legend.borderaxespad': 0.2, 'pdf.fonttype': 42})
    return np.random.default_rng(0)


def fig(h=2.4, w=W, **kw):
    return plt.subplots(figsize=(w, h), **kw)


def save(fig_, name, out='paper/figures'):
    fig_.savefig(f'{out}/{name}.png'); fig_.savefig(f'{out}/{name}.pdf'); plt.close(fig_)
    print('wrote', name)


def seeds_pts(ax, x, vals, colour, source, denom, rng, s=22, jit=0.09, half=0.22):
    """Every seed as a hollow marker (shape = source) + thick mean bar."""
    v = np.asarray(vals, float) / denom
    ax.scatter(x + rng.uniform(-jit, jit, len(v)), v, marker=MK[source], s=s,
               facecolor='white', edgecolor=colour, linewidth=1.0, zorder=3)
    ax.hlines(v.mean(), x - half, x + half, color=colour, linewidth=2.0, zorder=4)
    return v


def violin(ax, x, vals, colour, alpha, denom, width=0.6):
    v = np.asarray(vals, float) / denom
    if len(v) >= 6:
        p = ax.violinplot([v], positions=[x], widths=width, showextrema=False)
        for b in p['bodies']:
            b.set_facecolor(colour); b.set_alpha(alpha); b.set_edgecolor('none')


def strip(ax, x, vals, colour, marker='o', rng=None, s=9, jit=0.08, half=0.2):
    """Per-tape strip (small markers) + mean bar; used by the tape figures."""
    rng = rng or np.random.default_rng(0)
    v = np.asarray(vals, float)
    ax.scatter(x + rng.uniform(-jit, jit, len(v)), v, marker=marker, s=s,
               facecolor='white', edgecolor=colour, linewidth=0.6, zorder=3)
    ax.hlines(v.mean(), x - half, x + half, color=colour, lw=1.8, zorder=4)
    return v


def bracket(ax, x0, x1, y, txt, dy=0.015, fs=7):
    ax.plot([x0, x0, x1, x1], [y - dy, y, y, y - dy], color='0.35', lw=0.8, clip_on=False)
    ax.text((x0 + x1) / 2, y + 0.4 * dy, txt, ha='center', va='bottom', fontsize=fs, color='0.25')


def group_label(ax, x, txt, y=-0.2, fs=7.5, color='0.25'):
    ax.text(x, y, txt, ha='center', va='top', fontsize=fs, color=color,
            transform=ax.get_xaxis_transform())


def footnote(fig_, txt=FOOT_DP, y=-0.02, fs=7):
    fig_.text(0.0, y, txt, ha='left', va='top', fontsize=fs, color='0.3', transform=fig_.transFigure)


def panel_tag(ax, tag, x=-0.18, y=1.02):
    ax.text(x, y, tag, transform=ax.transAxes, fontsize=8.5, fontweight='bold', va='bottom')
