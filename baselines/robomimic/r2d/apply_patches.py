"""Apply the robomimic-leg edits to the COPY of the r2dreamer tree ($LAB/robomimic_r2d; never the Genesis tree).
Idempotent (each edit is guarded by a marker). Edits (plan §3/§4):
  1. demo_prefill.py: the hard-coded state (T,17) and image (T,64,64,6) asserts become config-driven
     (env.state_dim, env.size x env.image_channels); a `robosuite` suite branch for the log_* columns.
  2. envs/__init__.py: a `suite == "robosuite"` branch constructing envs.robosuite.RobosuiteCan.

  python apply_patches.py $LAB/robomimic_r2d
"""
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1]).resolve()
MARK = "# ROBOMIMIC-LEG"

# ---------------- demo_prefill.py ----------------
p = root / "demo_prefill.py"; s = p.read_text(); orig = s
if MARK not in s:
    s = s.replace(
        'def _load_episode(path, downsample=1, reward_scale=1.0, act_dim=7):',
        f'def _load_episode(path, downsample=1, reward_scale=1.0, act_dim=7, state_dim=17, image_shape=(64, 64, 6)):  {MARK}: config-driven dims')
    s = s.replace(
        '        assert ep["state"].shape == (T, 17), (path, ep["state"].shape)\n    assert ep["image"].shape == (T, 64, 64, 6) and ep["image"].dtype == np.uint8, path',
        f'        assert ep["state"].shape == (T, state_dim), (path, ep["state"].shape, state_dim)  {MARK}\n'
        f'    assert ep["image"].shape == (T,) + tuple(image_shape) and ep["image"].dtype == np.uint8, (path, ep["image"].shape, image_shape)  {MARK}')
    s = s.replace(
        '    eps = [_load_episode(f, downsample=ds, reward_scale=scale, act_dim=act_dim) for f in files]',
        f'    _sd = int(config.env.get("state_dim", 17) or 17); _ish = tuple(int(x) for x in config.env.size) + (int(config.env.get("image_channels", 6) or 6),)  {MARK}\n'
        f'    eps = [_load_episode(f, downsample=ds, reward_scale=scale, act_dim=act_dim, state_dim=_sd, image_shape=_ish) for f in files]')
    s = s.replace(
        '        assert state.shape[-1] == 17 and state.dtype == np.float32, state.shape',
        f'        assert state.shape[-1] == int(config.env.get("state_dim", 17) or 17) and state.dtype == np.float32, state.shape  {MARK}')
    s = s.replace(
        'MANISKILL_LOG_KEYS = ("log_success", "log_native_success")',
        f'MANISKILL_LOG_KEYS = ("log_success", "log_native_success")\nROBOSUITE_LOG_KEYS = ("log_success",)  {MARK}: envs/robosuite.py emits exactly this column')
    s = s.replace(
        '        if suite == "maniskill":\n            log_keys = MANISKILL_LOG_KEYS\n',
        f'        if suite == "maniskill":\n            log_keys = MANISKILL_LOG_KEYS\n        elif suite == "robosuite":  {MARK}\n            log_keys = ROBOSUITE_LOG_KEYS\n')
    s = s.replace(
        '        if suite == "maniskill":\n            # maniskill demos',
        f'        if suite == "robosuite":  {MARK}: the reward-bearing (terminal) frame is the success\n'
        '            row["log_success"] = (row["reward"] > 0).to(torch.float32)\n'
        '        elif suite == "maniskill":\n            # maniskill demos')
    n_marks = s.count(MARK)
    assert n_marks == 8, f"demo_prefill.py: expected 8 edits, applied {n_marks} -- the source drifted; inspect"
    p.write_text(s); print(f"[patch] demo_prefill.py: {n_marks} edits")
else:
    print("[patch] demo_prefill.py already patched")

# ---------------- envs/__init__.py ----------------
p = root / "envs" / "__init__.py"; s = p.read_text()
if MARK not in s:
    branch = f'''    elif suite == "robosuite":  {MARK}: robomimic Can (state input; envs/robosuite.py)
        import envs.robosuite as robosuite_env

        env = robosuite_env.RobosuiteCan(
            task,
            size=tuple(config.size),
            seed=config.seed + id,
            hdf5=config.get("hdf5", None),
            reward_scale=config.get("reward_scale", 1.0),
            image_channels=config.get("image_channels", 3),
        )
    else:
        raise NotImplementedError(suite)
'''
    assert s.count('    else:\n        raise NotImplementedError(suite)\n') == 1
    s = s.replace('    else:\n        raise NotImplementedError(suite)\n', branch)
    p.write_text(s); print("[patch] envs/__init__.py: robosuite branch added")
else:
    print("[patch] envs/__init__.py already patched")
print("[patch] done")
