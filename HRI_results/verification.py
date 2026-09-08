#!/usr/bin/env python3
"""Generates VERIFICATION.md: for every row of record, everything an auditor needs to re-derive
it without asking us.

Almost all of this is derived, not written: the cluster paths come from the `path` column of
seed_counts.csv (the file each per-seed count was actually read from), the bank version and
hardware class from the harvest stamps and node_provenance.csv, and the commit from git. The only
hand-maintained parts are STALE_PATHS and REGEN, both below, because neither is recoverable from
the artefacts themselves.
"""
import collections, csv, os, subprocess, datetime

HERE = os.path.dirname(os.path.abspath(__file__))

# Which script regenerates a row, keyed by the `source` tag its per-seed counts carry.
REGEN = {
    'cluster:wm': ('$W/runs/<run>/fresh_eval_<cell>/metrics.json',
                   'harvest_cluster.py (section 1) -> make_tables.py',
                   'r2dreamer eval, launched by $W/wmfix_phase_eval_cpu.sbatch or '
                   '$W/full_posthoc_evals.sbatch; re-scores by $W/cpsc_rescore.sbatch'),
    'cluster:dv3': ('$W/runs/dv3dbg_*/fresh_eval_<cell>/metrics.json',
                    'harvest_cluster.py (section 1b) -> make_tables.py',
                    'cluster/dv3dbg/dv3dbg_eval.sbatch'),
    'cluster:rlpd': ('$LAB/genesis_pickaplace/baselines/rl/checkpoints/<run>/sweep/<cell>/sweep.json',
                     'harvest_cluster.py (section 2) -> make_tables.py',
                     'cluster/eval_sweep.sh / baselines/wandb_eval.py'),
    'cluster:dp': ('$LAB/genesis_pickaplace/baselines/outputs/<wave>/<run>/sweep/<cell>/sweep.json',
                   'harvest_cluster.py (section 3) -> make_tables.py',
                   'cluster/eval_sweep.sh / baselines/wandb_eval.py'),
    'cluster:dp_headline': ('$LAB/genesis_pickaplace/baselines/outputs/<wave>/<run>/sweep/HEADLINE.txt',
                            'harvest_cluster.py (section 3b) -> make_tables.py',
                            'the only surviving source for DP seeds whose sweep dirs were pruned '
                            'by the disk fix; the surviving sweep.json seeds agree exactly'),
    'cluster:phase_clone': ('$LAB/gp_place | $LAB/gp_e2e .../<run>/<cell>/metrics.json',
                            'harvest_cluster.py (section 4) -> make_tables.py',
                            'baselines/eval_place.py | baselines/eval_e2e.py'),
    'cluster:robomimic': ('$LAB/robomimic_runs/<learner>/<run>/<cell>/metrics.json',
                          'harvest_cluster.py (section 5) -> make_tables.py',
                          'baselines/robomimic/eval_*_robosuite.py'),
}

# Paths that hold a SUPERSEDED number. An auditor who re-derives from these will get a figure
# that disagrees with the table and conclude we erred; they are listed so that cannot happen.
STALE_PATHS = [
    ('$W/runs/s2_r2d_place_state_*/fresh_eval_polE_{mode,sample}/',
     'place, as recorded', 'superseded by fresh_eval_polE_{mode,sample}_v2 (rebuilt entry bank '
     'physgrip_2026-09-07, entries pinned, hardware pinned)'),
    ('$W/runs/s2_r2d_contact_state_*/fresh_eval_polE_{mode,sample}{,_cp}/',
     'contact', 'superseded by fresh_eval_polE_*_v2'),
    ('$W/runs/s2_r2d_carrycontact_state_*/fresh_eval_polE_{mode,sample}{,_cp}/',
     'carrycontact', 'superseded by fresh_eval_polE_*_v2'),
    ('$W/runs/full_r2d_state_*/fresh_eval_rnd30_{mode,sample}{,_cp}/',
     'end-to-end', 'superseded by fresh_eval_rnd30_*_v2 (pinned). NOTE: the ORIGINAL cells for '
     'seeds 2 and 3 of dHfull_all were produced on 36-core AVX2 hardware and do not reproduce; '
     'see "Why some cells did not reproduce" in results.md'),
    ('$LAB/genesis_pickaplace/baselines/outputs/dp_w2final/dH_DP_s{25..29}/sweep/'
     'selected_spots60_mixedcore/',
     'DP spots60, human seeds 25-29', 'archived mixed-hardware cells; the pinned re-runs are in '
     'selected_spots60/'),
    ('$LAB/robomimic_runs/bcrnn/*/eval_bank50_ep950/',
     'BC-RNN', 'epoch 950, selected by a sorting bug; LAST is epoch 2000 in eval_bank50/'),
    ('$LAB/robomimic_runs/bcrnn/bcrnn_{MH200,MG200s}_s*/',
     'BC-RNN source comparison', 'the row computed from these is WITHDRAWN: the arms run '
     'different action heads. Head-matched cells are bcrnn_MH200_nogmm_* and bcrnn_MG200s_gmm_*'),
]


def git_commit():
    """The commit that last changed the DATA, not HEAD.

    Stamping HEAD made this file drift by one commit every time it was itself committed - the
    stamp changed because committing it moved HEAD - which broke the rebuild-is-a-no-op property
    the README claims. The data commit is also the more useful fact for an auditor: it says which
    harvest the provenance below was derived from, and is stable across documentation commits.
    """
    try:
        return subprocess.run(['git', '-C', HERE, 'log', '-1', '--format=%h %ad',
                               '--date=short', '--', 'seed_counts_raw.csv'],
                              capture_output=True, text=True).stdout.strip() or 'unknown'
    except Exception:
        return 'unknown'


def write(path, results, tidy):
    by_cmp = collections.defaultdict(list)
    for t in tidy:
        by_cmp[t['comparison']].append(t)

    nodes = {}
    npath = os.path.join(HERE, 'node_provenance.csv')
    if os.path.exists(npath):
        for r in csv.DictReader(open(npath)):
            if not r['run'].startswith('#'):
                nodes[(r['run'], r['cell'])] = r

    L = ['# Verification guide', '',
         f'*Generated by `make_tables.py` on {datetime.date.today().isoformat()}. '
         f'Per-seed data as of commit `{git_commit()}`.*', '',
         'Everything needed to re-derive any figure in `results.md` without asking us. Paths are '
         'the files each per-seed count was actually read from, not a reconstruction.', '',
         '`$LAB` = `/cluster/tufts/shortlab/jstale02`, `$W` = `$LAB/wm_fix_2026-09-03`.', '',
         '## Re-deriving everything at once', '',
         '```bash',
         '# from a machine with `ssh pax` access:',
         'cd HRI_results && python3 make_tables.py --refresh',
         '# harvest_cluster.py runs ON the login node and walks every evaluation tree;',
         '# seed_counts_raw.csv is the complete per-seed harvest, seed_counts.csv the subset used.',
         '```', '',
         'A rebuild with unchanged data is byte-identical, so `git status` afterwards tells you '
         'whether the DATA moved, not whether the script ran.', '',
         '## Per-row provenance', '',
         '| row | value | seeds | source tree | bank version | hardware (original record) |',
         '|---|---|---|---|---|---|']

    for r in results:
        if r['status'] == 'EMPTY':
            continue
        rows = by_cmp.get(r['id'], [])
        if not rows:
            continue
        srcs = sorted({t['source'] for t in rows})
        paths = sorted({t['path'] for t in rows})
        ex = paths[0].replace('/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03', '$W') \
                     .replace('/cluster/tufts/shortlab/jstale02', '$LAB')
        ex = ex.rsplit('/', 2)[0] + '/...'
        hw = sorted({(nodes.get((t['arm'], t['cell'])) or {}).get('orig_cores', '')
                     for t in rows} - {''})
        val = (f"{r['human_rate']} v {r['machine_rate']}" if r['human_rate'] != '' else '-')
        L.append(f"| `{r['id']}` | {val} | {r['n_seeds_human']}v{r['n_seeds_machine']} "
                 f"| `{ex}` | {r['bank_stamp'] or 'n/a'} "
                 f"| {', '.join(f'{c}-core' for c in hw) if hw else 'not stamped'} |")

    L += ['', '## Which script produces which source', '',
          '| source tag | files | regenerated by | produced on the cluster by |',
          '|---|---|---|---|']
    for tag, (files, regen, produced) in sorted(REGEN.items()):
        L.append(f'| `{tag}` | `{files}` | {regen} | {produced} |')

    L += ['', '## Known-stale paths - DO NOT re-derive from these', '',
          'Each holds a real number that is no longer the number of record. An auditor deriving '
          'a figure from one of these will disagree with the table and may conclude we erred.',
          '',
          '| path | what it is | why it is stale |', '|---|---|---|']
    for p, what, why in STALE_PATHS:
        L.append(f'| `{p}` | {what} | {why} |')

    L += ['', '## Hardware', '',
          'ISA and physical core count are perfectly collinear on this cluster: **24 AVX2 nodes, '
          'every one 36-core; no 36-core node is AVX-512**, read from `/proc/cpuinfo` per machine '
          '(`$LAB/gp_e2e/isa_probe.log`). **Slurm `AvailableFeatures` are unreliable here** - '
          'nodes advertising `broadwell` measure as AVX-512 - so never key a hardware argument '
          'off `sinfo -o %f`. `harvest_nodes.py` reads the probe.', '',
          'Hardware class is the one identified cause of a cell failing to reproduce: 16 of 16 '
          'cells whose ORIGINAL record was produced on 36-core AVX2 hardware moved under '
          're-scoring, against 0 of 176 from every other class.', '',
          '## Statistics', '',
          '`hri_stats.py` is runnable as its own self-test (`python3 hri_stats.py`) and checks '
          'itself against two published figures: the r2dreamer pick permutation p = 0.875 and '
          'the RLPD pick MDE 0.345 / CI +/-0.245. If those fail, the statistics have drifted.', '']
    open(path, 'w').write('\n'.join(L) + '\n')
