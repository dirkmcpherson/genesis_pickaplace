"""ONE way to run a sim variant everywhere (recorder / RLPD trainer / eval).

The variant realizes the SAME MDP in a different Genesis world (arm gains, gravity comp,
riser, ...; baselines/sim_variants.py; paper/real2sim_follower_lab_2026-08-23.md). The rule
(silent-default family): every process that builds a world calls apply_pre(name) BEFORE and
apply_post(env, name) AFTER; every artifact (tape, sidecar, manifest, registry) carries the
name; every consumer asserts it. 'base' is the unpatched world and the default everywhere.

Resolution order for eval: explicit CLI > checkpoint sidecar > 'base' (mismatch = refuse).
"""
import os, sys, pathlib as pl
REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[1]))

def apply_pre(name):
    if not name or name == 'base':
        return 'base'
    sys.path.insert(0, str(REPO / 'baselines'))
    import sim_variants
    assert name in sim_variants.VARIANTS, (name, sorted(sim_variants.VARIANTS))
    sim_variants.install(name)
    return name

def apply_post(env, name):
    """env = FullTaskEnv (has .genv.w) or a raw world dict."""
    if not name or name == 'base':
        return dict(name='base')
    import sim_variants
    w = env.genv.w if hasattr(env, 'genv') else (env.w if hasattr(env, 'w') else env)
    info = sim_variants.post_build(w, name)
    print(f'[sim-variant] {name}: kp={info["kp"]} gc={info["gravity_comp"]} riser={info["riser"]}', flush=True)
    return info
