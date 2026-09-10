"""Pin Genesis's supported deterministic solver selection for the frozen benchmark."""
from pathlib import Path
import json,runpy,sys
R=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(R/'baselines'),str(R/'can_pos_recovery')]
import genesis as gs
import replay_harness as h
original_init=gs.init;original_build=h.build_world;settings={}
def initialize(*args,**kwargs):
 kwargs['use_deterministic_algorithms']=True
 return original_init(*args,**kwargs)
gs.init=initialize
def build(*args,**kwargs):
 w=original_build(*args,**kwargs);cfg=w['scene'].sim.rigid_solver.rigid_config
 assert gs.use_deterministic_algorithms and cfg.prefer_decomposed_solver==1
 settings.update(use_deterministic_algorithms=True,prefer_decomposed_solver=cfg.prefer_decomposed_solver,
  para_level=int(cfg.para_level),n_dofs=w['scene'].sim.rigid_solver.n_dofs,
  explanation='Supported Genesis flag pins numerically distinct solver dispatch instead of timing-based selection. Other benchmark code and physical parameters remain frozen.')
 return w
h.build_world=build
out=Path(sys.argv[sys.argv.index('--out')+1])
runpy.run_path(str(R/'can_pos_recovery/bench_native_batch_g14_v2.py'),run_name='__main__')
report=json.loads((out/'report.json').read_text());report['deterministic_dispatch']=settings
(out/'report.json').write_text(json.dumps(report,indent=2))
print('DETERMINISTIC_DISPATCH',json.dumps(settings),flush=True)
