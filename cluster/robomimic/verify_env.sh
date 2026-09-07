#!/bin/bash
# Robomimic leg: print the versions of the two NEW envs (brief deliverable 1). Exit non-zero if a hard pin is off.
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
rc=0
for V in $LAB/robo_venv $LAB/r2d_venv_robo; do
  echo "== $V"
  [ -x "$V/bin/python" ] || { echo "   MISSING"; rc=1; continue; }
  "$V/bin/python" - <<'PY' || rc=1
import sys, importlib
mods = ["torch", "numpy", "mujoco", "robosuite", "robomimic", "h5py", "gymnasium", "stable_baselines3", "lerobot", "tensordict", "torchrl", "hydra"]
row = [f"python {sys.version.split()[0]}"]
for m in mods:
    try:
        mod = importlib.import_module(m); row.append(f"{m} {getattr(mod, '__version__', '?')}")
    except Exception as e:
        row.append(f"{m} -")
print("   " + " | ".join(row))
import robosuite, torch
assert robosuite.__version__ == "1.5.1", robosuite.__version__
print("   cuda build", torch.version.cuda, "| cuda available", torch.cuda.is_available())
PY
done
exit $rc
