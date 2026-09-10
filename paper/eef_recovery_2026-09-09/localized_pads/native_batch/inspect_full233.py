"""Locate full-source CPU/GPU divergence using observed physical states."""
from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
names=['full_cpu233','full_gpu16_233'];traces=[np.load(D/n/'trace.npz')['state'][:,0] for n in names]
diag=[np.load(D/n/'full_task_diagnostic.npz') for n in names]
t=(np.arange(len(traces[0]))+1)*.03;delta=1000*np.linalg.norm(traces[0][:,10:13]-traces[1][:,10:13],axis=-1)
thresholds={}
for mm in [1,5,10,50,100]:
 ids=np.flatnonzero(delta>mm);thresholds[str(mm)]=float(t[ids[0]]) if len(ids) else None
fig,axes=plt.subplots(4,1,figsize=(10,10),sharex=True)
for label,z,d in zip(['CPU','GPU: deterministic, 16 environments'],traces,diag):
 gap=1000*(np.linalg.norm(z[:,10:12]-z[:,17:19],axis=-1)-.066)
 axes[0].plot(t,gap,label=label);axes[1].plot(t,z[:,12]);axes[2].plot(t,d['trajectory'][:,0,20])
axes[0].set_ylabel('Can–goal surface gap (mm)');axes[0].axhline(0,color='k',ls=':',lw=1);axes[0].legend()
axes[1].set_ylabel('Can center height (m)');axes[1].axhline(.2205,color='k',ls=':',lw=1)
axes[2].set_ylabel('Can tilt (degrees)');axes[2].axhline(20,color='k',ls=':',lw=1)
axes[3].plot(t,delta,color='purple');axes[3].set_ylabel('CPU/GPU can difference (mm)');axes[3].set_xlabel('Recorded replay time (s)')
for ax in axes:ax.set_xlim(18,t[-1]);ax.grid(alpha=.2)
fig.suptitle('Trial233: identical recorded commands; full-task CPU/GPU disagreement\nNo added endpoint holds or changed success thresholds')
fig.tight_layout();fig.savefig(D/'full233_cpu_gpu_divergence.png',dpi=160);plt.close(fig)
rows=[]
for time in [12,18,20,22,22.56,23,24,25,26,27,28.86]:
 i=min(len(t)-1,round(time/.03)-1);r=dict(time_s=float(t[i]),can_difference_mm=float(delta[i]),cases={})
 for label,z,d in zip(names,traces,diag):
  r['cases'][label]=dict(can_xyz=z[i,10:13].tolist(),goal_xyz=z[i,17:20].tolist(),tilt_deg=float(d['trajectory'][i,0,20]),contacts=d['contact_counts'][i,0].tolist())
 rows.append(r)
out=dict(first_difference_above_mm_s=thresholds,rows=rows,source_sha256={n:hashlib.sha256((D/n/'full_task_diagnostic.npz').read_bytes()).hexdigest() for n in names},qualification='Diagnostic association, not a causal localization of solver error. Same intended physical model and commands; CPU/GPU numerical implementations differ.')
(D/'full233_cpu_gpu_divergence.json').write_text(json.dumps(out,indent=2));print(thresholds)
