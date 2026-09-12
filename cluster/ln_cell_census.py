"""Slide diagnostics + ramp stamp + reward histogram, for r2dreamer milestone cells AND RLPD cells.
args: metrics.json files, run directories (globbed for metrics.json to depth 3), or episode_rollouts.jsonl
"""
import json, sys, collections, glob, os

def rhist(vals):
    return sorted(collections.Counter(round(float(v), 2) for v in vals).items())

def cell(path):
    m = json.load(open(path))
    pe = m.get("per_episode") or (m.get("episodes") if isinstance(m.get("episodes"), list) else None) or []
    print("=" * 100)
    print(path.replace("/cluster/tufts/shortlab/jstale02/", "$LAB/"))
    print("stamp:", str(m.get("ladder_stamp"))[:200])
    lp = m.get("ladder_provenance") or {}
    if lp:
        print("  ramp:", lp.get("ramp"), "tip_guard:", lp.get("tip_guard"), "max_return:", lp.get("max_return"))
    hs = m.get("headline_stages") or m.get("stages") or {}
    print("headline:", {k: round(v, 3) for k, v in hs.items()} if isinstance(hs, dict) else hs)
    for k in ("node", "cores", "checkpoint", "ic_mode", "protocol", "role", "cpu_model"):
        if k in m:
            print(f"  {k}: {str(m[k])[:120]}")
    if not pe:
        print("  (no per-episode list; keys:", sorted(m.keys())[:30], ")")
        return
    print("  n episodes:", len(pe), "episode keys:", sorted(pe[0].keys())[:30])
    if "reward" in pe[0]:
        print("  reward histogram:", rhist(e["reward"] for e in pe))
    se = [e for e in pe if (e.get("stages") or e).get("slide_event")]
    if se:
        rr = collections.Counter(str(e.get("slide_fail_reason")) for e in se)
        print("  slide_event eps:", len(se), "fail_reason:", sorted(rr.items()))
        oc = collections.Counter((e.get("outcome"), round(float(e.get("reward", -1)), 2)) for e in se)
        print("  (outcome, reward) over slide_event eps:", sorted(oc.items()))
        for e in se:
            if (e.get("stages") or e).get("home"):
                print("   HOME ep", e.get("ep"), "reward", round(float(e.get("reward", -1)), 2), "steps", e.get("steps"), "video", e.get("video"))

def rollouts(path):
    print("=" * 100)
    print(path.replace("/cluster/tufts/shortlab/jstale02/", "$LAB/"))
    rows = [json.loads(l) for l in open(path) if l.strip()]
    print("  n rows:", len(rows), "keys:", sorted(rows[0].keys()))
    rk = [k for k in rows[0] if "rew" in k.lower() or "return" in k.lower() or "score" in k.lower()]
    print("  reward-like keys:", rk)
    last = rows[-200:]
    for k in rk:
        vals = [r[k] for r in last if r.get(k) is not None]
        if vals:
            print(f"  {k} over last {len(vals)} eps: hist", rhist(vals)[:25])
    for k in sorted(rows[0]):
        if k.startswith("episode/train_ep_"):
            vals = [r.get(k) for r in last if r.get(k) is not None]
            if vals:
                print(f"  {k}: {sum(vals)/len(vals):.3f} (last {len(vals)})")

for a in sys.argv[1:]:
    if a.endswith(".jsonl"):
        rollouts(a)
    elif os.path.isdir(a):
        found = sorted(glob.glob(os.path.join(a, "fresh_eval_*", "metrics.json")) + glob.glob(os.path.join(a, "rec", "*", "metrics.json")))
        found = [f for f in found if "_iso" not in f]
        print("#", a, "->", len(found), "metrics.json")
        for f in found:
            cell(f)
    else:
        cell(a)
