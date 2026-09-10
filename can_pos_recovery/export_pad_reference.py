"""Export actual processed finger collision geometry; no simulation actions."""
import os, sys, json, hashlib
from pathlib import Path
import numpy as np
REPO=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(REPO/'baselines'),str(REPO/'can_pos_recovery')]
os.environ['TI_CPU_MAX_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
from sim_variant_hook import apply_pre, apply_post
from genesis_can_env import GenesisCanEnv
meta=json.loads((REPO/'paper/eef_recovery_2026-09-09/timestamp_full_pool/233/source.json').read_text())
apply_pre(meta['variant'])
env=GenesisCanEnv(backend='cpu',max_steps=1)
apply_post(env,meta['variant'])
out=REPO/'paper/eef_recovery_2026-09-09/localized_pads/reference'
out.mkdir(parents=True,exist_ok=False)
records=[]
for link in env.w['kinova'].links:
    if 'finger' not in link.name:continue
    assert len(link.geoms)==1
    g=link.geoms[0]
    assert np.allclose(g._init_pos,0) and np.allclose(g._init_quat,[1,0,0,0])
    m=g.get_trimesh();path=out/f'{link.name}.ply'
    m.export(path)
    records.append(dict(link=link.name,path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        vertices=len(m.vertices),faces=len(m.faces),convex=bool(m.is_convex),
        bounds_m=m.bounds.tolist(),geom_idx=g.idx))
assert len(records)==4
(out/'manifest.json').write_text(json.dumps(dict(variant=meta['variant'],records=records),indent=2))
print(json.dumps(records,indent=2))
