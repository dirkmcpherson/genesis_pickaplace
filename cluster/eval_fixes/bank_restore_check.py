#!/usr/bin/env python3
"""Restore-survival of every bank entry, before (raw-grip bank) and after (physical-grip bank), in ONE world per scope
(PHASE_PLAN amendment (j)). Builds the r2dreamer adapter env exactly as eval_genesis does (scope, corrected world via
R2D_SIM_VARIANT), then calls the env's own restore for each entry of each bank and records: survived (the scope's
predicate: place/carrycontact = can centre above PLACE_HELD_Z; contact = can upright inside the shelf band), can z, grip_obs
and the commanded finger targets after the settle. CPU. usage: bank_restore_check.py <scope> <bank_before> <bank_after> <out.json>"""
import json, os, sys, time
import numpy as np
W = os.environ.get("W", "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03"); sys.path.insert(0, f"{W}/r2dreamer_fix")
scope, bank_before, bank_after, out = sys.argv[1:5]
from envs.genesis import GenesisPick
env = GenesisPick("genesis_pick", size=(64, 64), seed=0, scope=scope, action_repeat=4, place_entry_bank=bank_after,
                  action_mode="delta_joint", delta_cap=0.025, delta_leash_mult=5, state_obs=True)
env._build(); fe = env._env; genv = fe.genv
restore = fe._restore_contact_entry if scope == "contact" else fe._restore_place_entry
res = {}
for tag, path in (("before", bank_before), ("after", bank_after)):
    bank = json.load(open(path)); rows = []
    t0 = time.time()
    for uid, e in bank.items():
        e = dict(e, uid=int(uid))
        ok = bool(restore(e))
        st = genv._obs()["state"]; bp = np.asarray(genv.w["bottle"].get_pos()).reshape(-1)
        rows.append(dict(uid=int(uid), survived=ok, grip_cmd=float(e["grip_cmd"]), grip_obs_entry=float(e["grip_obs"]),
                         can_z_entry=float(e["can_pos"][2]), can_z_after=float(bp[2]), grip_obs_after=float(st[6])))
    n = len(rows); s = sum(r["survived"] for r in rows)
    res[tag] = dict(bank=path, n=n, survived=s, rate=s / n, failed_uids=[r["uid"] for r in rows if not r["survived"]], rows=rows,
                    seconds=round(time.time() - t0, 1))
    print(f"[restore-check] scope={scope} {tag}: {s}/{n} survived ({s/n:.3f}); failed {res[tag]['failed_uids']}", flush=True)
b, a = res["before"], res["after"]
fb, fa = set(b["failed_uids"]), set(a["failed_uids"])
res["summary"] = dict(scope=scope, n=b["n"], survived_before=b["survived"], survived_after=a["survived"],
                      fail_before_only=sorted(fb - fa), fail_after_only=sorted(fa - fb), fail_both=sorted(fb & fa))
print(f"[restore-check] summary: {json.dumps(res['summary'])}", flush=True)
json.dump(res, open(out, "w"), indent=1)
