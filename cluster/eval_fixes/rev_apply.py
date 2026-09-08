#!/usr/bin/env python3
"""Reverse-apply amendments (j) and (l) to a COPY of the trees, to isolate whether those edits changed evaluation
dynamics. Uses the same exact-anchor pairs as the forward patches, swapped (new -> old), (l) undone before (j).
usage: rev_apply.py --r2d <copy> --gp <copy>"""
import argparse, importlib.util, pathlib, sys

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); sys.modules[name] = m
    spec.loader.exec_module(m)  # patch modules only define constants + apply() under __main__ guard
    return m

def rev(path, edits, label):
    p = pathlib.Path(path); s = p.read_text(); n = 0
    for old, new in edits:                     # swapped: remove `new`, restore `old`
        if new not in s:
            print(f"  [{p.name}] {label}: already reverted / absent: {old.splitlines()[0].strip()[:60]}"); continue
        assert s.count(new) == 1, (p, s.count(new))
        s = s.replace(new, old); n += 1
    p.write_text(s); print(f"  [{p.name}] {label}: reverted {n} edit(s)")

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--r2d", required=True); ap.add_argument("--gp", required=True)
    ap.add_argument("--dir", default=str(pathlib.Path(__file__).parent))
    a = ap.parse_args()
    J = load("jpatch", f"{a.dir}/eval_fixes_patch.py"); L = load("lpatch", f"{a.dir}/slide_success_patch.py")
    r2d, gp = pathlib.Path(a.r2d), pathlib.Path(a.gp)
    rev(r2d / "eval_genesis.py", L.EVAL_EDITS, "l"); rev(r2d / "envs/genesis.py", L.ADAPTER_EDITS, "l")
    rev(gp / "baselines/genesis_can_env.py", L.CAN_ENV_EDITS, "l"); rev(gp / "baselines/rl/full_env.py", L.FULL_ENV_EDITS, "l")
    rev(r2d / "eval_genesis.py", J.EVAL_EDITS, "j"); rev(r2d / "envs/genesis.py", J.ADAPTER_EDITS, "j")
    rev(gp / "baselines/rl/full_env.py", J.FULL_ENV_EDITS, "j")
    print("reverted trees ready")
