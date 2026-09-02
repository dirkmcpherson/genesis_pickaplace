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
to (a) json files in their own run directory (ckpt sidecar `sim_variant`, lerobot train_config `root`), else (b) the
(learner, arm, seed, wave) map built from every .out under the root (seeds are reused across worlds in v2, so the wave
in the run-dir path disambiguates). Unresolved rows print world='?' and
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
# RESCORE tag = <config>_<arm>_s<seed>[_W3][<rule suffix>]; the suffix (e.g. _F20_sample = BEST-of-5 fraction ckpts,
# _LAST_hold) is a DIFFERENT selection rule and must never be merged into the BEST-of-K rows -> it becomes part of the key.
RE_RESCORE = re.compile(r'RESCORE-RESULT tag=\S*?_(dR2DDPfails|dR2D|dH|dDP|dHv2raw|dDPv2|dDPretimed|dHsmoothed|dDPnoised)_s(\d+)(_W3)?(\S*) set=(hold|rnd) picked=(\d+)/(\d+) expected=(\d+)')


def _rescore(resc, m):
    arm, seed, w3, rule, st_, k, n, exp = m.groups()
    if int(n) != int(exp): return                  # asserted denominator failed
    resc[(arm, int(seed), w3 or '', rule)][st_] = int(k) / int(n)


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


def rundir_world(path, max_depth=3):
    """World for a HEADLINE.txt from ITS OWN run directory (<RUN>/sweep/HEADLINE.txt): any json up to max_depth
    below <RUN> carrying a `sim_variant` key (RLPD/DP ckpt sidecars) or a demo-set root string (lerobot
    train_config.json `root`). Unique world -> that; several -> CONFLICT; none -> None."""
    run = os.path.dirname(os.path.dirname(path))
    found = set()
    for d in range(max_depth + 1):
        for jf in glob.glob(os.path.join(run, *(['*'] * d), '*.json')):
            if not os.path.isfile(jf) or any(f'{os.sep}{sd}{os.sep}' in jf for sd in SKIP_DIRS):
                continue
            try:
                txt = open(jf, errors='ignore').read(200000)
            except OSError:
                SKIPPED.append(jf); continue
            m = re.search(r'"sim_variant"\s*:\s*"([^"]+)"', txt)
            if m: found.add(world_from_variant(m.group(1)))
            for r, w in ROOT_WORLD.items():
                if f'{r}/' in txt: found.add(w)
    return next(iter(found)) if len(found) == 1 else (None if not found else 'CONFLICT(' + '|'.join(sorted(found)) + ')')


def collect(root, verbose=False):
    rows = collections.defaultdict(dict)          # key -> {seed: (seed, hold, rnd, sha)}; dedup by seed
    provmap = collections.defaultdict(set)        # (learner, arm, seed) -> {world}
    wavemap = collections.defaultdict(set)        # (learner, arm, seed, wave) -> {world}; seeds are REUSED across worlds (v2 s50-67)
    outs = {}                                     # .out path -> provenance dict (for --selftest)
    resc = collections.defaultdict(dict)          # (arm, seed, w3, rule) -> {set: frac}; rule = tag suffix after _s<seed>
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
            if p['wave']: wavemap[k + (p['wave'],)].add(p['world'])
        for line in text.splitlines():
            if not line.startswith(('SWEEP-HEADLINE', 'DP-HEADLINE', 'R2D-RESULT', 'RESCORE-RESULT')):
                continue                            # pack/harvest .out files ECHO other runs' lines with a "[file] " prefix
            m = RE_RESCORE.search(line)
            if m:
                _rescore(resc, m); continue
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

    def lookup(learner, arm, seed, path=''):
        ws = provmap.get((learner, arm, seed), set())
        if len(ws) > 1 and path:                    # same (arm, seed) trained in two worlds: disambiguate by the wave in the path
            ws2 = set()
            for (lr, a, sd, wave), w in wavemap.items():
                if (lr, a, sd) == (learner, arm, seed) and wave and re.search(r'(^|[_/.-])' + re.escape(wave) + r'([_/.-]|$)', path):
                    ws2 |= w
            if len(ws2) == 1: ws = ws2
        return next(iter(ws)) if len(ws) == 1 else ('?' if not ws else 'CONFLICT(' + '|'.join(sorted(ws)) + ')')

    for f in _files(root, 'HEADLINE.txt'):
        line = _read(f)
        m = re.search(r'(DP-HEADLINE|SWEEP-HEADLINE) arm=(\S+) seed=(\d+)(?: reward=(\S+))?.*? hold=(\d+)/(\d+) rnd=(\d+)/(\d+)', line)
        if not m: continue
        kind, arm, seed, rw, h, hn, r, rn = m.groups(); seed = int(seed)
        learner = 'DP' if kind == 'DP-HEADLINE' else 'RLPD'
        world = rundir_world(f) or lookup(learner, arm, seed, f)
        put((learner, world, rw or '-', arm), seed, int(h)/int(hn), int(r)/int(rn))

    for f in _files(root, '*.log'):               # RESCORE-RESULT lines (n12_rescore/*.log and successors)
        for line in _read(f).splitlines():
            m = RE_RESCORE.search(line)
            if m: _rescore(resc, m)
    for (arm, seed, w3, rule), d in resc.items():
        if 'hold' in d and 'rnd' in d:
            world = 'corrected' if w3 else lookup('r2d', arm, seed)
            put((f'r2dreamer{rule}', world, 'dense', arm), seed, d['hold'], d['rnd'])
    return {k: list(v.values()) for k, v in rows.items()}, outs


def selftest(root):
    rows, outs = collect(root)
    bad = []
    groups = {}
    for f, p in outs.items():
        key = (p['learner'] or os.path.basename(f).split('_')[0], p['wave'] or '?')
        groups.setdefault(key, (f, p['world']))
        cells = {k for k in p['arm_seed'] if k[1] != 'legacy'}   # ARM unset -> launcher prints arm=legacy (demo-free probes)
        if p['world'].startswith(('?', 'CONFLICT')) and p['has_result'] and cells:
            bad.append(f'{f}: world={p["world"]} arm_seed={sorted(cells)[:2]}')
    print(f'selftest: {len(outs)} .out files, {len(groups)} (learner, wave) groups; '
          f'skipped {len(SKIPPED)} dangling/unreadable paths' + (f' (first: {os.path.relpath(SKIPPED[0], root)})' if SKIPPED else ''))
    for (lr, wv), (f, w) in sorted(groups.items()):
        print(f'  {lr:<6} {wv:<40} world={w:<12} e.g. {os.path.relpath(f, root)}')
    for k, v in sorted(rows.items()):
        if k[1].startswith(('?', 'CONFLICT')) and k[3] != 'legacy':
            bad.append(f'table row {k} n={len(v)} world unresolved: seeds {sorted(x[0] for x in v)}')
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
