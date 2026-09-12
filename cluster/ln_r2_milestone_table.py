#!/usr/bin/env python3
"""One line per {r2dreamer} milestone CELL, from $W/ln_milestone_cells/<run>/<milestone>/<cell>/metrics.json.

    python3 cluster/ln_r2_milestone_table.py [--root DIR] [--md]

Counts, not rates: `home` at these n is an IGNITION read (PHASE_PLAN (aa) rev 3), and a rate
printed at n=15 or n=30 invites a comparison the sample cannot support. `k/N` keeps N in view.
An absent cell is printed as absent; it is never a zero.
"""
import argparse, glob, json, os, sys

CELLS = ("rnd30_mode", "hold15_mode", "rnd30_sample")
COLS = ("picked", "placed_v2", "farside", "slide_event", "home", "nested_v2", "nested_honest")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/ln_milestone_cells")
    ap.add_argument("--md", action="store_true", help="markdown table instead of TSV")
    a = ap.parse_args()

    rows, missing = [], []
    for run in sorted(d for d in glob.glob(os.path.join(a.root, "full_r2d_state_*")) if os.path.isdir(d)):
        for ms in sorted(d for d in glob.glob(os.path.join(run, "*")) if os.path.isdir(d)):
            for cell in CELLS:
                p = os.path.join(ms, cell, "metrics.json")
                if not os.path.exists(p):
                    missing.append((os.path.basename(run), os.path.basename(ms), cell))
                    continue
                m = json.load(open(p))
                n = int(m["episodes"])
                st = m.get("stages", {})
                prov = {}
                pj = os.path.join(ms, "provenance.json")
                if os.path.exists(pj):
                    prov = json.load(open(pj))
                side = prov.get("sidecar") or {}
                rows.append(dict(
                    run=os.path.basename(run).replace("full_r2d_state_", ""),
                    milestone=os.path.basename(ms), cell=cell, n=n,
                    step=side.get("online_sim_steps"),
                    ladder=(m.get("ladder_provenance") or {}).get("ladder"),
                    node=m.get("node"), cores=m.get("ncpus_machine"),
                    tipped=round(float(m.get("tipped", 0.0)) * n),
                    **{c: (round(float(st[c]) * n) if c in st else None) for c in COLS}))

    hdr = ["run", "milestone", "cell", "n", "ladder"] + list(COLS) + ["tipped", "node", "cores"]
    def cellstr(r, k):
        v = r.get(k)
        if v is None:
            return "absent"
        return f"{v}/{r['n']}" if k in COLS or k == "tipped" else str(v)

    if a.md:
        print("| " + " | ".join(hdr) + " |")
        print("|" + "---|" * len(hdr))
        for r in rows:
            print("| " + " | ".join(cellstr(r, k) for k in hdr) + " |")
    else:
        print("\t".join(hdr))
        for r in rows:
            print("\t".join(cellstr(r, k) for k in hdr))
    print(f"\n# {len(rows)} cell(s); {len(missing)} (run, milestone, cell) not yet written", file=sys.stderr)
    for t in missing:
        print("# absent: " + " ".join(t), file=sys.stderr)
    # per-run home totals over the cells that exist -- the headline of this table
    tot = {}
    for r in rows:
        if r.get("home") is not None:
            k = r["run"]
            h, e = tot.get(k, (0, 0))
            tot[k] = (h + r["home"], e + r["n"])
    print("\n# home, pooled over this run's existing cells (ignition read, not a rate):", file=sys.stderr)
    for k in sorted(tot):
        print(f"#   {k}: {tot[k][0]}/{tot[k][1]}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
