#!/usr/bin/env python
"""Regenerate THE canonical results table from job artifacts. Never quote results from memory.

Reads SWEEP-HEADLINE (RLPD) / DP-HEADLINE (DP) / R2D-RESULT (r2dreamer sel-only) lines out of job
.out files, HEADLINE.txt files, and RESCORE-RESULT lines out of re-score .log files, and prints the
per-cell table. Excludes smoke runs (seed 0) explicitly -- including them silently dragged the dH rows
down in a hand re-derivation on 08-25.

  python analysis/results_table.py --artifacts ~/workspace/final_rr_artifacts_2026-08-24 [--md out.md]
  python analysis/results_table.py --artifacts <root> --selftest      # fails loudly on any world '?'

WORLD IS PARSED, NEVER GUESSED (AUDIT_approach_2026-09-02 finding 3: the former seed-range heuristic
mislabelled every block launched after 08-31). Provenance lines, all printed by the launchers:
  [sim-variant] <name>: ...                         sbatch_rlpd.sh / sbatch_dp.sh (non-base variants only)
  R2D-DEMOSET-V2-OK <dir> ... sim_variant=<name>    sbatch_r2dreamer.sh
  demo=<path> | dataset=<path> | raw=<path> | [demos] N npz in <path>   demo-set root (matched_w3 | matched_v2)
  DEMO-SHA <arm> <fmt> n=<N> sha=<sha16>            demo fingerprint (carried into the table)
World = sim_variant when printed (gc_kp4_riser3_shelf6 -> corrected, base -> old, anything else -> its
name), cross-checked against the demo root (matched_w3 -> corrected, matched_v2 -> old); a disagreement
prints CONFLICT. HEADLINE.txt and RESCORE-RESULT rows carry no provenance of their own: they are joined
to (a) a checkpoint sidecar `*.action_mode.json` next to the HEADLINE (sim_variant key) or (b) the
(learner, arm, seed) map built from every .out under the root. Unresolved rows print world='?' and
make --selftest exit 1. Cluster command (the v2/A27/A32/A33 .out files are not on the laptop):
  cd $LAB/genesis_pickaplace && python analysis/results_table.py --artifacts . --selftest
"""
import argparse, collections, glob, json, os, re, statistics as st, sys

CORRECTED_VARIANT = 'gc_kp4_riser3_shelf6'
ROOT_WORLD = {'matched_w3': 'corrected', 'matched_v2': 'old', 'matched_w2_pilot': 'pilot-w2',
              'matched_v1_pilot': 'legacy', 'lerobot_dR2D_pick': 'legacy'}

RE_SV = re.compile(r'\[sim-variant\] (\S+?):|sim_variant=(\S+)')
RE_ROOT = re.compile(r'(?:demo|dataset|raw)=\S*?/?(?:baselines|demonstrations)/([A-Za-z0-9_]+)/?(?:\s|$)|(?:demo|dataset|raw)=\S*?/?baselines/([A-Za-z0-9_]+)/|\[demos\] \d+ npz in \S*?baselines/([A-Za-z0-9_]+)/')
RE_SHA = re.compile(r'DEMO-SHA (\S+) (\S+) n=(\d+) sha=([0-9a-f]+)')
RE_START = re.compile(r'^== (RLPD|DP) (\S+?)_(?:RLPD|DP)-(\S+?)_s(\d+)')
RE_R2D_START = re.compile(r'^== r2dreamer (\S+) seed (\d+) start .*?demo=(\S+)')
RE_ARMSEED = re.compile(r'arm=(\S+) seed=(\d+)')


def world_from_variant(sv):
    return 'corrected' if sv == CORRECTED_VARIANT else ('old' if sv == 'base' else sv)


def parse_provenance(text):
    """-> dict(world, sim_variant, root, sha, arm_seed=set((learner, arm, seed)), wave). Pure parsing."""
    sv = None; roots = set(); sha = None; arm_seed = set(); wave = None; learner = None; has_result = False
    for line in text.splitlines():
        m = RE_SV.search(line)
        if m and sv is None: sv = m.group(1) or m.group(2)
        for a, b, c in RE_ROOT.findall(line):
            roots.add(a or b or c)
        m = RE_SHA.search(line)
        if m and sha is None: sha = m.group(4)[:12]
        m = RE_START.match(line)
        if m:
            learner, arm, wave, seed = m.group(1), m.group(2), m.group(3), int(m.group(4))
            arm_seed.add((learner, arm, seed))
        m = RE_R2D_START.match(line)
        if m:
            learner = 'r2d'; wave = m.group(1)          # arm comes from the R2D-RESULT line
        if line.startswith(('SWEEP-HEADLINE', 'DP-HEADLINE', 'R2D-RESULT')):
            has_result = True
            m = RE_ARMSEED.search(line)
            if m:
                lr = 'RLPD' if line.startswith('SWEEP') else ('DP' if line.startswith('DP-') else 'r2d')
                learner = learner or lr
                arm_seed.add((lr, m.group(1), int(m.group(2))))
    w_sv = world_from_variant(sv) if sv else None
    w_root = {ROOT_WORLD[r] for r in roots if r in ROOT_WORLD}
    if not w_root and roots and not sv:                 # pre-matrix demo dirs: parsed root, no world claim
        w_root = {'unmatched:' + '|'.join(sorted(roots))}
    if w_sv and w_root and w_root != {w_sv}:
        world = f'CONFLICT({w_sv}/{"|".join(sorted(w_root))})'
    elif w_sv: world = w_sv
    elif len(w_root) == 1: world = next(iter(w_root))
    elif w_root: world = 'CONFLICT(' + '|'.join(sorted(w_root)) + ')'
    else: world = '?'
    return dict(world=world, sim_variant=sv, roots=roots, sha=sha, arm_seed=arm_seed, wave=wave, learner=learner,
                has_result=has_result)


def sidecar_world(path):
    """World from a checkpoint sidecar next to a HEADLINE.txt (<RUN>/sweep/HEADLINE.txt -> <RUN>/**/*.action_mode.json)."""
    run = os.path.dirname(os.path.dirname(path))
    svs = set()
    for sc in glob.glob(os.path.join(run, '**', '*.action_mode.json'), recursive=True):
        if not os.path.isfile(sc): SKIPPED.append(sc); continue
        try:
            sv = json.load(open(sc)).get('sim_variant')
            if sv: svs.add(world_from_variant(sv))
        except Exception:
            pass
    return next(iter(svs)) if len(svs) == 1 else (None if not svs else 'CONFLICT(' + '|'.join(sorted(svs)) + ')')


SKIP_DIRS = ('wandb', '.git', '__pycache__')
SKIPPED = []                                      # unreadable / dangling paths seen by the last collect()


def _files(root, pattern):
    """Regular, readable files matching pattern under root. Dangling symlinks (wandb `latest-run`) and
    unreadable files are COUNTED into SKIPPED and reported, never raised, never silently dropped."""
    for f in sorted(glob.glob(os.path.join(root, '**', pattern), recursive=True)):
        if any(f'{os.sep}{d}{os.sep}' in f for d in SKIP_DIRS):
            continue
        if not os.path.isfile(f):                 # False for dangling symlinks
            SKIPPED.append(f); continue
        yield f


def _read(f):
    try:
        return open(f, errors='ignore').read()
    except OSError:
        SKIPPED.append(f); return ''


def collect(root, verbose=False):
    rows = collections.defaultdict(dict)          # key -> {seed: (seed, hold, rnd, sha)}; dedup by seed
    provmap = collections.defaultdict(set)        # (learner, arm, seed) -> {world}
    outs = {}                                     # .out path -> provenance dict (for --selftest)
    SKIPPED.clear()

    def put(key, seed, h, r, sha=None):
        if seed == 0: return                        # smoke
        rows[key].setdefault(seed, (seed, h, r, sha or ''))   # first sighting wins (mirror copies are identical)

    for f in _files(root, '*.out'):
        text = _read(f)
        if not text: continue
        p = parse_provenance(text); outs[f] = p
        for k in p['arm_seed']:
            provmap[k].add(p['world'])
        for line in text.splitlines():
            m = re.search(r'SWEEP-HEADLINE arm=(\S+) seed=(\d+) reward=(\S+).*? hold=(\d+)/(\d+) rnd=(\d+)/(\d+)', line)
            if m:
                arm, seed, rw, h, hn, r, rn = m.groups(); seed = int(seed)
                put(('RLPD', p['world'], rw, arm), seed, int(h)/int(hn), int(r)/int(rn), p['sha']); continue
            m = re.search(r'DP-HEADLINE arm=(\S+) seed=(\d+).*? hold=(\d+)/(\d+) rnd=(\d+)/(\d+)', line)
            if m:
                arm, seed, h, hn, r, rn = m.groups(); seed = int(seed)
                put(('DP', p['world'], '-', arm), seed, int(h)/int(hn), int(r)/int(rn), p['sha']); continue
            m = re.search(r'R2D-RESULT arm=(\S+) seed=(\d+) .*?reward=(\S+) .*?picked=([0-9.]+)', line)
            if m:                                   # sel readout only: selection set, 14/15 ceiling
                arm, seed, rw, pk = m.groups(); seed = int(seed)
                put(('r2d(SEL-ONLY, not a headline)', p['world'], rw, arm), seed, float(pk), float('nan'), p['sha'])

    def lookup(learner, arm, seed):
        ws = provmap.get((learner, arm, seed), set())
        return next(iter(ws)) if len(ws) == 1 else ('?' if not ws else 'CONFLICT(' + '|'.join(sorted(ws)) + ')')

    for f in _files(root, 'HEADLINE.txt'):
        line = _read(f)
        m = re.search(r'(DP-HEADLINE|SWEEP-HEADLINE) arm=(\S+) seed=(\d+)(?: reward=(\S+))?.*? hold=(\d+)/(\d+) rnd=(\d+)/(\d+)', line)
        if not m: continue
        kind, arm, seed, rw, h, hn, r, rn = m.groups(); seed = int(seed)
        learner = 'DP' if kind == 'DP-HEADLINE' else 'RLPD'
        world = sidecar_world(f) or lookup(learner, arm, seed)
        put((learner, world, rw or '-', arm), seed, int(h)/int(hn), int(r)/int(rn))

    resc = collections.defaultdict(dict)          # (arm, seed, w3) -> {set: frac}
    for f in _files(root, '*.log'):               # RESCORE-RESULT lines (n12_rescore/*.log and successors)
        for line in _read(f).splitlines():
            m = re.search(r'RESCORE-RESULT tag=\S*?_(dR2DDPfails|dR2D|dH|dDP|dHv2raw|dDPv2)_s(\d+)(_W3)?\S* set=(hold|rnd) picked=(\d+)/(\d+) expected=(\d+)', line)
            if not m: continue
            arm, seed, w3, st_, k, n, exp = m.groups()
            if int(n) != int(exp): continue          # asserted denominator failed
            resc[(arm, int(seed), w3 or '')][st_] = int(k)/int(n)
    for (arm, seed, w3), d in resc.items():
        if 'hold' in d and 'rnd' in d:
            world = 'corrected' if w3 else lookup('r2d', arm, seed)
            put(('r2dreamer', world, 'dense', arm), seed, d['hold'], d['rnd'])
    return {k: list(v.values()) for k, v in rows.items()}, outs


def selftest(root):
    rows, outs = collect(root)
    bad = []
    groups = {}
    for f, p in outs.items():
        key = (p['learner'] or os.path.basename(f).split('_')[0], p['wave'] or '?')
        groups.setdefault(key, (f, p['world']))
        if p['world'].startswith(('?', 'CONFLICT')) and p['has_result']:
            bad.append(f'{f}: world={p["world"]} arm_seed={sorted(p["arm_seed"])[:2]}')
    print(f'selftest: {len(outs)} .out files, {len(groups)} (learner, wave) groups; '
          f'skipped {len(SKIPPED)} dangling/unreadable paths' + (f' (first: {os.path.relpath(SKIPPED[0], root)})' if SKIPPED else ''))
    for (lr, wv), (f, w) in sorted(groups.items()):
        print(f'  {lr:<6} {wv:<40} world={w:<12} e.g. {os.path.relpath(f, root)}')
    for k, v in sorted(rows.items()):
        if k[1].startswith(('?', 'CONFLICT')):
            bad.append(f'table row {k} n={len(v)} world unresolved')
    if bad:
        print('SELFTEST FAILED -- unresolved world:\n  ' + '\n  '.join(bad)); sys.exit(1)
    print('selftest OK: every .out with a result line and every table row has a parsed world (unmatched:* = pre-matrix demo dir, labelled not guessed)')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--artifacts', required=True); ap.add_argument('--md')
    ap.add_argument('--selftest', action='store_true', help='fail loudly if any world is ? or CONFLICT')
    a = ap.parse_args(); root = os.path.expanduser(a.artifacts)
    if a.selftest: return selftest(root)
    rows, _ = collect(root)
    if SKIPPED:
        print(f'WARN: skipped {len(SKIPPED)} dangling/unreadable paths (first: {SKIPPED[0]})', file=sys.stderr)
    L = ['| learner | world | reward | arm | n | hold | rnd | per-seed hold | demo sha |', '|---|---|---|---|---|---|---|---|---|']
    for k in sorted(rows):
        v = sorted(rows[k]); h = [x[1] for x in v]
        rnd = f'{st.mean([x[2] for x in v]):.2f}' if v[0][2] == v[0][2] else 'n/a'
        shas = sorted({x[3] for x in v if x[3]})
        L.append(f'| {k[0]} | {k[1]} | {k[2]} | {k[3]} | {len(v)} | {st.mean(h):.2f} | {rnd} | '
                 + ' '.join(f'{x:.2f}' for x in h) + f' | {",".join(shas) or "-"} |')
    out = '\n'.join(L)
    print(out)
    if a.md:
        open(a.md, 'w').write('# Canonical results table (regenerate: analysis/results_table.py)\n\n'
                              'Smoke runs (seed 0) excluded. World PARSED from each job .out (sim_variant / demo root), '
                              'never from the seed number; r2dreamer rows are RESCORE (hold/rnd) readouts; '
                              'r2d(SEL-ONLY) rows are the selection-set readout and are NOT headlines. Dedup by (arm, seed).\n\n' + out + '\n')


if __name__ == '__main__':
    main()
