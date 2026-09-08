#!/usr/bin/env python3
"""Cluster-side: per-cell node provenance and re-score movement for the world-model cells.

ATTRIBUTION IS PER DIRECTORY. Every evaluation job logs a line

    [eval] wrote <.../runs/<run>/fresh_eval_<cell>/metrics.json + N mp4s

so the job that produced a given cell is named explicitly, and its node comes from `sacct`.
An earlier version of this script attributed a cell to the node in the run's
`events.out.tfevents.*` filename -- that is the TRAINING node, while the evaluations ran as
separate CPU jobs, so it mis-attributed every post-hoc cell and inverted the conclusion. Do not
reintroduce a first-match or per-run heuristic here.

ISA comes from the /proc/cpuinfo probe ($LAB/gp_e2e/isa_probe.log), never from Slurm's
`AvailableFeatures`, which are wrong on this cluster (nodes advertising "broadwell" measure as
AVX-512).

MOVEMENT IS EPISODE-LEVEL. A cell counts as moved only when episodes reach different terminal
states between the original and its `_cp` re-score, read from the per-episode mp4 names. A
column that was structurally zero and becomes non-zero with ZERO differing episodes is a
predicate becoming earnable, not a reproduction failure, and is reported separately.

usage: python3 harvest_nodes.py > node_provenance.csv
"""
import collections, csv, glob, json, os, re, subprocess, sys

L = os.environ.get("LAB", "/cluster/tufts/shortlab/jstale02")
W = os.environ.get("W", os.path.join(L, "wm_fix_2026-09-03"))

WROTE = re.compile(r"\[eval\] wrote (\S+?/runs/[^/]+/fresh_eval_[^/]+)/metrics\.json")

dir2job = {}
for f in (glob.glob(W + "/slurm/*.out") + glob.glob(W + "/cpsc_logs/*.log")
          + glob.glob(W + "/*.log")):
    jid = None
    mm = re.search(r"_(\d+)\.out$", os.path.basename(f))
    if mm:
        jid = mm.group(1)
    try:
        txt = open(f, errors="ignore").read()
    except OSError:
        continue
    hm = re.search(r"host=(pax\d+)", txt)
    for d in WROTE.findall(txt):
        dir2job.setdefault(d, []).append((jid, hm.group(1) if hm else None))

jids = sorted({j for v in dir2job.values() for j, _ in v if j})
node = {}
for i in range(0, len(jids), 400):
    out = subprocess.run(["sacct", "-X", "-n", "-P", "-j", ",".join(jids[i:i + 400]),
                          "-o", "JobID,NodeList"], capture_output=True, text=True).stdout
    for ln in out.splitlines():
        p = ln.split("|")
        if len(p) == 2 and p[1] and not p[1].startswith("None"):
            node[p[0].split(".")[0]] = p[1]

cores = {}
for ln in subprocess.run(["sinfo", "-h", "-N", "-o", "%n %c"],
                         capture_output=True, text=True).stdout.splitlines():
    p = ln.split()
    if len(p) == 2:
        cores[p[0]] = int(p[1])
isa = {}
try:
    for ln in open(os.path.join(L, "gp_e2e", "isa_probe.log")):
        m = re.match(r"(pax\d+)\s+(avx2|avx512)\b", ln)
        if m:
            isa[m.group(1)] = m.group(2)
except OSError:
    pass


def host_of(d):
    for j, h in dir2job.get(d, []):
        n = node.get(j) or h
        if n:
            return n
    return ""


def outcomes(d):
    m = {}
    for f in glob.glob(d + "/*.mp4"):
        mm = re.match(r"ep(\d+)_(\w+?)_(\w+)\.mp4$", os.path.basename(f))
        if mm:
            m[int(mm.group(1))] = mm.group(3)
    return m


rows = []
for run in sorted(os.listdir(os.path.join(W, "runs"))):
    d0 = os.path.join(W, "runs", run)
    for op in sorted(glob.glob(d0 + "/fresh_eval_*/metrics.json")):
        cell = op.split(os.sep)[-2]
        if cell.endswith("_cp"):
            continue
        od = op.rsplit("/", 1)[0]
        cpd = d0 + "/" + cell + "_cp"
        if not os.path.exists(cpd + "/metrics.json"):
            continue
        try:
            a = json.load(open(op)); b = json.load(open(cpd + "/metrics.json"))
        except Exception:
            continue
        n = a.get("episodes")
        sa = dict(a.get("stages") or {}); sb = dict(b.get("stages") or {})
        for k in ("picked", "placed", "placed_v2", "contact", "nested"):
            if k in a and not isinstance(a[k], (list, dict)):
                sa.setdefault(k, a[k])
            if k in b and not isinstance(b[k], (list, dict)):
                sb.setdefault(k, b[k])
        colmoved = [k for k in sorted(sa) if k in sb and round(sa[k] * n) != round(sb[k] * n)]
        struct = [k for k in colmoved if round(sa[k] * n) == 0 and round(sb[k] * n) > 0]
        oa, ob = outcomes(od), outcomes(cpd)
        ep_diff = sum(1 for k in oa if k in ob and oa[k] != ob[k])
        oh, rh = host_of(od), host_of(cpd)
        rows.append(dict(run=run, cell=cell, arm=("human" if "_dH" in run else "machine"),
                         episodes=n, ep_diff=ep_diff,
                         moved=int(ep_diff > 0),
                         structural_only=int(ep_diff == 0 and bool(struct)),
                         cols_changed="|".join(colmoved), structural_cols="|".join(struct),
                         orig_host=oh, orig_cores=cores.get(oh, ""), orig_isa=isa.get(oh, ""),
                         rescore_host=rh, rescore_cores=cores.get(rh, ""),
                         rescore_isa=isa.get(rh, "")))

w = csv.DictWriter(sys.stdout, fieldnames=list(rows[0].keys()))
w.writeheader()
for r in rows:
    w.writerow(r)
c = collections.Counter((r["orig_cores"], r["moved"]) for r in rows)
print("# episode-level movement by original-record core count: " +
      "; ".join(f"{k}-core {c[(k,1)]}/{c[(k,1)]+c[(k,0)]}"
                for k in sorted({r['orig_cores'] for r in rows}, key=lambda x: (x == '', x))),
      file=sys.stderr)
