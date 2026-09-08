#!/usr/bin/env python3
"""Cluster-side harvester for HRI_results. Runs ON the cluster (login node), walks every
evaluation artefact tree, and writes ONE tidy CSV of per-seed counts to stdout.

It is deliberately dumb and total: it emits everything it can parse and makes no decisions
about which cell is "of record". The selection of cells of record lives in make_tables.py,
so that when new runs land this script needs no edit.

Row schema (one row per run x eval-cell x statistic):
  source, learner, phase, run, arm, seed, cell, action_mode, statistic, k, n, n_expected,
  bank_version, bank_sha, evaluator, path, extra

`bank_version` / `bank_sha` / `evaluator` are the COMPARABILITY STAMPS. They are emitted verbatim
and left EMPTY when the artefact carries no stamp -- an absent stamp is not a match, and
make_tables.py treats it as a blocker to cross-learner comparison.

Counts: WM/robomimic metrics.json store RATES; k is reconstructed as round(rate * episodes)
(the same reconstruction morning_table.py/phase_table.py use). Robomimic metrics.json also
carry a per-episode list, so k is counted EXACTLY there and the reconstruction is cross-checked.

usage: python3 harvest_cluster.py > seed_counts_raw.csv
"""
import csv, glob, json, os, re, sys

LAB = os.environ.get("LAB", "/cluster/tufts/shortlab/jstale02")
W = os.environ.get("W", os.path.join(LAB, "wm_fix_2026-09-03"))
GP = os.path.join(LAB, "genesis_pickaplace")

rows = []
def emit(**kw):
    kw.setdefault("extra", "")
    for f in ("bank_version", "bank_sha", "evaluator"):
        kw.setdefault(f, "")
    kw.setdefault("n_expected", kw.get("n", ""))
    rows.append(kw)

def jload(p):
    try:
        with open(p) as f:
            return json.load(f)
    except Exception:
        return None

def r2k(rate, n):
    """rate -> integer count. Returns None if the rate is not a clean multiple of 1/n."""
    if rate is None or not n:
        return None
    k = round(float(rate) * int(n))
    if abs(float(rate) * int(n) - k) > 1e-6:
        return None
    return int(k)

# ---------------------------------------------------------------- 1. Genesis world model
# $W/runs/<run>/fresh_eval_<cell>_<mode>[_cp]/metrics.json
WM_RUN = re.compile(r"^(?P<kind>s2_r2d|full_r2d|dv3dbg)_(?:(?P<phase>pick|place|contact|carrycontact)_state_|state_|pick_)"
                    r"(?P<arm>[A-Za-z0-9]+)_(?P<setting>.*?)_s(?P<seed>\d+)$")
# cell name must be matched longest-first: rnd300 before rnd30, polEdDP before polE
WM_CELLS = ["rnd300", "rnd30", "hold15", "holdv2", "alldemo", "polEdDP", "polE_contact", "polE", "holdE"]

for mpath in sorted(glob.glob(os.path.join(W, "runs", "*", "fresh_eval_*", "metrics.json"))):
    run = mpath.split(os.sep)[-3]
    ev = mpath.split(os.sep)[-2][len("fresh_eval_"):]
    m = WM_RUN.match(run)
    if not m:
        continue
    d = jload(mpath)
    if not d or "episodes" not in d:
        continue
    cell = next((c for c in WM_CELLS if ev.startswith(c)), None)
    if cell is None:
        continue
    rest = ev[len(cell):].lstrip("_")
    cp = rest.endswith("_cp")
    if cp:
        rest = rest[:-3]
    # eval dir suffix is usually the action mode, but recovery re-evals use their own tag
    # (e.g. fresh_eval_rnd30_recov for dv3); fall back to the mode recorded inside the file.
    mm = re.match(r"^(sample|mode)(?:_(.+))?$", rest)
    if mm:
        # "mode" -> mode cell; "mode_v2" -> mode cell of the v2 RE-SCORE, tagged _v2 so the
        # registry can prefer the re-scored cell while the as-recorded one stays selectable.
        mode, tag = mm.group(1), (mm.group(2) or "")
    else:
        mode, tag = d.get("mode", "?"), rest
    if mode not in ("sample", "mode"):
        continue
    n = int(d["episodes"])
    kind, phase, arm, setting, seed = (m.group("kind"), m.group("phase"), m.group("arm"),
                                       m.group("setting"), int(m.group("seed")))
    learner = "dv3" if kind == "dv3dbg" else "r2dreamer"
    if kind == "full_r2d":
        phase = "e2e"
    elif kind == "dv3dbg":
        phase = "pick"
    # statistics available: top-level scalars + the `stages` dict (full task)
    stats = {}
    for key in ("picked", "placed_v2", "contact", "contact_push", "nested", "nested_honest",
                "nested_proxy", "slide_success", "restore_failed"):
        if key in d and not isinstance(d[key], (list, dict)):
            stats[key] = d[key]
    for key, v in (d.get("stages") or {}).items():
        stats[key] = v
    for stat, rate in stats.items():
        k = r2k(rate, n)
        if k is None:
            continue
        emit(source="cluster:wm", learner=learner, phase=phase, run=run, arm=arm, seed=seed,
             cell=cell + ("_cp" if cp else "") + (("_" + tag) if tag else ""),
             action_mode=mode, statistic=stat, k=k, n=n,
             path=mpath, extra="setting=%s" % setting,
             bank_version=str(d.get("bank_version") or ""),
             bank_sha=str(d.get("bank_sha256") or "")[:12],
             evaluator="wm/eval_genesis.py")

# ---------------------------------------------------------------- 1b. dv3 (DreamerV3-torch)
# dv3 writes a different schema: {"policy_eval/<stat>": rate, "policy_eval/n": N} and has no
# mode/sample split -- its single cell SAMPLES a latent inside the policy call. RESULTS §7 item 9
# records that the sidecar's "deterministic" label is a known mislabel, so every dv3 cell is
# tagged action_mode="sample" here and must be compared with r2dreamer's SAMPLE row.
DV3_RUN = re.compile(r"^dv3dbg_(?P<phase>pick|reach)_(?P<arm>[A-Za-z0-9.]+)_(?P<setting>.*?)_s(?P<seed>\d+)$")
for mpath in sorted(glob.glob(os.path.join(W, "runs", "dv3dbg_*", "fresh_eval_*", "metrics.json"))):
    run = mpath.split(os.sep)[-3]
    ev = mpath.split(os.sep)[-2][len("fresh_eval_"):]
    m = DV3_RUN.match(run)
    d = jload(mpath)
    if not m or not d:
        continue
    n = d.get("policy_eval/n")
    if not n:
        continue
    cell = next((c for c in WM_CELLS if ev.startswith(c)), None)
    if cell is None:
        continue
    tag = ev[len(cell):].lstrip("_")
    for key, rate in d.items():
        if not key.startswith("policy_eval/") or key == "policy_eval/n":
            continue
        stat = key.split("/", 1)[1]
        k = r2k(rate, n)
        if k is None:
            continue
        emit(source="cluster:dv3", learner="dv3", phase=m.group("phase"), run=run,
             arm=m.group("arm"), seed=int(m.group("seed")),
             cell=cell + (("_" + tag) if tag else ""), action_mode="sample",
             statistic=stat, k=k, n=int(n), path=mpath,
             extra="setting=%s" % m.group("setting"), evaluator="dv3/eval")

# ---------------------------------------------------------------- 2. Genesis RLPD
# $GP/baselines/rl/checkpoints/<wave>_<arm>_s<seed>/sweep/<cell>/sweep.json  (also final_sweep.json)
RL_RUN = re.compile(r"^(?P<wave>rlpd_[A-Za-z0-9]+)_(?P<arm>[A-Za-z0-9]+)_s(?P<seed>\d+)(?P<suffix>_\w+)?$")
for sw in sorted(glob.glob(os.path.join(GP, "baselines/rl/checkpoints/*/sweep/*/sweep.json"))
                 + glob.glob(os.path.join(GP, "baselines/rl/checkpoints/*/sweep/final_sweep.json"))
                 + glob.glob(os.path.join(LAB, "gp_place/baselines/rl/checkpoints/*/*/sweep/*/sweep.json"))):
    parts = sw.split(os.sep)
    cell = parts[-2] if parts[-2] != "sweep" else "final"
    run = parts[parts.index("sweep") - 1]
    m = RL_RUN.match(run)
    if not m:
        continue
    j = jload(sw)
    if not j or "sets" not in j:
        continue
    act = j.get("act_selection") or ""
    for setname, r in (j["sets"] or {}).items():
        if not isinstance(r, dict) or "picked" not in r:
            continue
        emit(source="cluster:rlpd", learner="RLPD", phase="pick", run=run,
             arm=m.group("arm"), seed=int(m.group("seed")), cell="%s/%s" % (cell, setname),
             action_mode={"deterministic": "mode", "sampled": "sample"}.get(act, act or "?"),
             statistic="picked", k=int(r["picked"]), n=int(r.get("n_present", 0)),
             n_expected=int(r.get("n_expected", 0)), path=sw,
             extra="wave=%s;ckpt=%s" % (m.group("wave"), j.get("ckpt_step", "")),
             evaluator="eval_sweep.sh/wandb_eval.py",
             bank_version=str(j.get("bank_version") or ""),
             bank_sha=str(j.get("bank_sha256") or "")[:12])

# ---------------------------------------------------------------- 3. Genesis Diffusion Policy
# $GP/baselines/outputs/<wave>/<arm>_DP_s<seed>/sweep/<cell>/sweep.json
DP_RUN = re.compile(r"^(?P<arm>[A-Za-z0-9]+)_DP_s(?P<seed>\d+)$")
for sw in sorted(glob.glob(os.path.join(GP, "baselines/outputs/*/*/sweep/*/sweep.json"))
                 + glob.glob(os.path.join(GP, "baselines/outputs/*/*/sweep/final_sweep.json"))
                 + glob.glob(os.path.join(LAB, "gp_place/baselines/outputs/*/*/sweep/*/sweep.json"))):
    parts = sw.split(os.sep)
    cell = parts[-2] if parts[-2] != "sweep" else "final"
    run = parts[parts.index("sweep") - 1]
    wave = parts[parts.index("sweep") - 2]
    m = DP_RUN.match(run)
    if not m:
        continue
    j = jload(sw)
    if not j or "sets" not in j:
        continue
    for setname, r in (j["sets"] or {}).items():
        if not isinstance(r, dict) or "picked" not in r:
            continue
        emit(source="cluster:dp", learner="DiffusionPolicy", phase="pick", run=run,
             arm=m.group("arm"), seed=int(m.group("seed")), cell="%s/%s" % (cell, setname),
             action_mode="sample", statistic="picked", k=int(r["picked"]),
             n=int(r.get("n_present", 0)), n_expected=int(r.get("n_expected", 0)), path=sw,
             extra="wave=%s;ckpt=%s" % (wave, j.get("ckpt_step", "")),
             evaluator="eval_sweep.sh/wandb_eval.py",
             bank_version=str(j.get("bank_version") or ""),
             bank_sha=str(j.get("bank_sha256") or "")[:12])

# --------------------------------------------------- 3b. DP HEADLINE.txt (LAST cell, pruned runs)
# The disk-pressure fix prunes superseded checkpoints AND their sweep dirs, so several DP runs no
# longer hold a final/ sweep.json even though the run completed. sweep/HEADLINE.txt survives and
# carries the same LAST numbers; it is the only source for those seeds. Rows from here are tagged
# cell="HEADLINE/<set>" so make_tables.py can prefer sweep.json and fall back to this.
RE_HL = re.compile(r"DP-HEADLINE arm=(?P<arm>\S+) seed=(?P<seed>\d+).*?"
                   r"hold=(?P<sh>\d+)/(?P<shn>\d+) rnd=(?P<sr>\d+)/(?P<srn>\d+).*?"
                   r"final_hold=(?P<fh>\d+)/(?P<fhn>\d+) final_rnd=(?P<fr>\d+)/(?P<frn>\d+)")
for hl in sorted(glob.glob(os.path.join(GP, "baselines/outputs/*/*/sweep/HEADLINE.txt"))):
    parts = hl.split(os.sep)
    wave, run = parts[-4], parts[-3]
    try:
        txt = open(hl).read()
    except Exception:
        continue
    m = RE_HL.search(txt)
    if not m:
        continue
    node = re.search(r"node=(\S+)", txt)
    for tag, kk, nn in (("selected/hold", "sh", "shn"), ("selected/rnd", "sr", "srn"),
                        ("final/hold", "fh", "fhn"), ("final/rnd", "fr", "frn")):
        emit(source="cluster:dp_headline", learner="DiffusionPolicy", phase="pick", run=run,
             arm=m.group("arm"), seed=int(m.group("seed")), cell="HEADLINE/" + tag,
             action_mode="sample", statistic="picked", k=int(m.group(kk)), n=int(m.group(nn)),
             path=hl, extra="wave=%s;node=%s" % (wave, node.group(1) if node else ""),
             evaluator="sweep/HEADLINE.txt")

# ---------------------------------------------------------------- 4. Genesis place/e2e clones (DP+RLPD phase agents)
for pat, learner_of in ((os.path.join(LAB, "gp_place", "**", "*eval*", "metrics.json"), None),
                        (os.path.join(LAB, "gp_e2e", "**", "*eval*", "metrics.json"), None)):
    for mpath in sorted(glob.glob(pat, recursive=True)):
        d = jload(mpath)
        if not d or "episodes" not in d:
            continue
        n = int(d["episodes"])
        run = mpath.split(os.sep)[-3]
        learner = ("RLPD" if "rlpd" in run.lower() else
                   "DiffusionPolicy" if run.lower().startswith("dp") or "_dp" in run.lower() else "?")
        phase = "place" if "gp_place" in mpath else "e2e"
        stats = {k: v for k, v in (d.get("stages") or {}).items()}
        for key in ("picked", "placed_v2", "contact", "contact_push", "nested",
                    "nested_honest", "slide_success"):
            if key in d and not isinstance(d[key], (list, dict)):
                stats.setdefault(key, d[key])
        sm = re.search(r"_s(\d+)$", run)
        for stat, rate in stats.items():
            k = r2k(rate, n)
            if k is None:
                continue
            emit(source="cluster:phase_clone", learner=learner, phase=phase, run=run,
                 arm=run, seed=int(sm.group(1)) if sm else -1,
                 cell=mpath.split(os.sep)[-2], action_mode=d.get("mode", "?"),
                 statistic=stat, k=k, n=n, path=mpath,
                 extra="protocol=each_entry_once",
                 bank_version=str(d.get("bank_version") or ""),
                 bank_sha=str(d.get("bank_sha256") or d.get("bank_used_sha256") or "")[:12],
                 evaluator="baselines/eval_place.py|eval_e2e.py")

# ---------------------------------------------------------------- 5. robomimic Can
RB_RUN = re.compile(r"^(?P<learner>rlpd|dp|r2d|bcrnn)_(?P<arm>.+?)_s(?P<seed>\d+)$")
RB_LEARNER = {"rlpd": "RLPD", "dp": "DiffusionPolicy", "r2d": "r2dreamer", "bcrnn": "BC-RNN"}
for mpath in sorted(glob.glob(os.path.join(LAB, "robomimic_runs", "*", "*", "eval_*", "metrics.json"))):
    run = mpath.split(os.sep)[-3]
    cell = mpath.split(os.sep)[-2]
    m = RB_RUN.match(run)
    if not m:
        continue
    d = jload(mpath)
    if not d:
        continue
    eps = d.get("episodes")
    ep_list = eps if isinstance(eps, list) else d.get("results")
    if isinstance(ep_list, list) and ep_list and isinstance(ep_list[0], dict):
        n = len(ep_list)
        k = sum(1 for e in ep_list if e.get("success"))
    else:
        n = int(eps) if isinstance(eps, int) else 0
        k = r2k(d.get("success_rate", d.get("success")), n)
    if k is None or not n:
        continue
    mode = ("mode" if cell.endswith("_mode") else "sample" if cell.endswith("_sample")
            else d.get("mode", "mode"))
    emit(source="cluster:robomimic", learner=RB_LEARNER[m.group("learner")], phase="robomimic_can",
         run=run, arm=m.group("arm"), seed=int(m.group("seed")), cell=cell, action_mode=mode,
         statistic="success", k=k, n=n, path=mpath,
         extra="ckpt=%s" % (d.get("ckpt_step", ""),),
         bank_version="bank_can50", bank_sha=str(d.get("bank_sha256") or "")[:12],
         evaluator="baselines/robomimic/eval_*_robosuite.py")

w = csv.DictWriter(sys.stdout, fieldnames=["source", "learner", "phase", "run", "arm", "seed",
                                           "cell", "action_mode", "statistic", "k", "n",
                                           "n_expected", "bank_version", "bank_sha",
                                           "evaluator", "path", "extra"])
w.writeheader()
for r in sorted(rows, key=lambda r: (r["source"], r["phase"], r["learner"], r["arm"], r["seed"],
                                     r["cell"], r["statistic"])):
    w.writerow(r)
print("# harvested %d rows" % len(rows), file=sys.stderr)
