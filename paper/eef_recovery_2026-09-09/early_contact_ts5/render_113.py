from pathlib import Path
import json,sys,runpy
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
sys.path[:0]=[str(REPO/'baselines'),str(REPO/'can_pos_recovery')]
import sim_variants
m=json.loads((ROOT/'113/source.json').read_text());sim_variants.VARIANTS[m['variant']]=dict(sim_variants.VARIANTS[m['parent_variant']],grasp_timeconst=.005,n_grasp_geoms=5)
sys.argv=['render_eef_trace.py',str(ROOT/'113/collection/113_eef_delta.npz'),'--stop','1000','--out',str(ROOT/'113/contact_probe_first30s.mp4')]
runpy.run_path(str(REPO/'can_pos_recovery/render_eef_trace.py'),run_name='__main__')
