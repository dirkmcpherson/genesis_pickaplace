"""Rise time per (ae) seed, from the TRAINING record (dense) and the series (sparse).
Training: first online step at which the rolling-30 rate of a stage first reaches a threshold (rolling over the
last 30 episodes; counter of the 30th episode minus the prefill origin). Series: first snapshot with the cell condition."""
import re, glob, json, os
runs = [("human s0","dv3px_sparse10_dHfull_all_rns10h_img_rlDreamer_s0","dv3px_sparse10_series"),
        ("machine s0","dv3px_sparse10_dDPfull_first_rns10h_img_rlDreamer_s0","dv3px_sparse10_series_dM_s0"),
        ("human s1","dv3px_sparse10_dHfull_all_rns10h_img_rlDreamer_s1","dv3px_sparse10_series_dH_s1"),
        ("machine s1","dv3px_sparse10_dDPfull_first_rns10h_img_rlDreamer_s1","dv3px_sparse10_series_dM_s1"),
        ("human s2","dv3px_sparse10_dHfull_all_rns10h_img_rlDreamer_s2","dv3px_sparse10_series_dH_s2"),
        ("machine s2","dv3px_sparse10_dDPfull_first_rns10h_img_rlDreamer_s2","dv3px_sparse10_series_dM_s2"),
        ("human s3","dv3px_sparse10_dHfull_all_rns10h_img_rlDreamer_s3","dv3px_sparse10_series_dH_s3"),
        ("machine s3","dv3px_sparse10_dDPfull_first_rns10h_img_rlDreamer_s3","dv3px_sparse10_series_dM_s3")]
R = "/home/j/runs_dv3_local"
def first_cross(eps, key, thr, w=30):
    for i in range(w-1, len(eps)):
        if sum(e[key] for e in eps[i-w+1:i+1]) / w >= thr:
            return eps[i]["counter"]
    return None
print("seed         | origin | picked>=.5 | home>=.1 | home>=.3 | home>=.5 | 1st home ep | series: 1st rnd30 home>=1 | 1st hold15 home>=5 | episodes so far")
for name, run, ser in runs:
    log = f"{R}/run_placeholder"
    con = f"{R}/{run}/console.log"
    if not os.path.exists(con):
        print(f"{name:12s} | (not started)"); continue
    origin = None; eps = []
    for line in open(con, errors="replace"):
        if origin is None:
            m = re.search(r"trainer starts at counter step (\d+)", line)
            if m: origin = int(m.group(1))
        if "episode/score" in line:
            m = re.match(r"\[(\d+)\]", line)
            if not m: continue
            c = int(m.group(1)); d = {"counter": c}
            for k in ("picked","home"):
                mm = re.search(rf"train_ep_{k} ([0-9.]+)", line); d[k] = float(mm.group(1)) if mm else 0.0
            eps.append(d)
    if origin is None: origin = 0
    def on(c): return None if c is None else c - origin
    first_home = next((e["counter"] for e in eps if e["home"] >= 1.0), None)
    # series
    s_rnd = s_hold = None
    for D in sorted(glob.glob(f"{R}/{ser}/ck_*"), key=lambda p: int(re.search(r"ck_(\d+)", p).group(1)) if re.search(r"ck_(\d+)$", p) else 0):
        mm = re.search(r"ck_(\d+)$", D)
        if not mm: continue
        c = int(mm.group(1))
        for cell, thr, slot in (("rnd30_mode", 1, "rnd"), ("hold15_mode", 5, "hold")):
            f = f"{D}/fresh_eval_{cell}/metrics.json"
            if os.path.exists(f):
                pe = json.load(open(f))["per_episode"]
                h = sum(1 for e in pe if e["stages"].get("home"))
                if slot == "rnd" and s_rnd is None and h >= thr: s_rnd = c - origin
                if slot == "hold" and s_hold is None and h >= thr: s_hold = c - origin
    fmt = lambda v: "   -   " if v is None else f"{v/1000:6.0f}k"
    print(f"{name:12s} | {origin:6d} | {fmt(on(first_cross(eps,'picked',0.5)))} | {fmt(on(first_cross(eps,'home',0.1)))} | {fmt(on(first_cross(eps,'home',0.3)))} | {fmt(on(first_cross(eps,'home',0.5)))} | {fmt(on(first_home))} | {fmt(s_rnd)} | {fmt(s_hold)} | {len(eps)}")
