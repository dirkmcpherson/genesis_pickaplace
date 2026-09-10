"""Verify the observer and summarize friction utilization at approach divergence."""
from pathlib import Path
import json
import numpy as np
ROOT=Path(__file__).resolve().parent
run=ROOT/'113_critical_soft'
plan=json.loads((ROOT/'plan.json').read_text())
ref=Path(plan['reference'])
z=np.load(run/'113_eef_delta.npz');b=np.load(ref)
exact={k:bool(np.array_equal(z[k],b[k])) for k in z.files}
assert all(exact.values()),exact
audit=json.loads((run/'contact_dynamics_audit.json').read_text())
stats=json.loads((run/'transmission_stats.json').read_text())
assert audit['substep_calls']==stats['substep_calls']==8+24*len(z['trajectory'])
c=np.load(run/'contact_dynamics.npz');v=c['values'];cols=c['columns'].tolist()
def col(name):return v[:,cols.index(name)]
t=(col('substep')-8)*.00125
finger_can=((col('kind_a')==1)&(col('kind_b')==2))|((col('kind_a')==2)&(col('kind_b')==1))
fn=col('normal_force_N');vt=col('relative_tangent_speed_m_s');u=col('pyramid_utilization')
loaded=finger_can&(fn>.05)
assert np.isfinite(v).all()
summary=dict(all_reference_arrays_exact=exact,substep_calls=audit['substep_calls'],contact_rows=len(v),finger_can_rows=int(finger_can.sum()),loaded_finger_can_rows=int(loaded.sum()),
    qualification='Dependent contact samples at 10 ms; Fn > 0.05 N excludes near-zero denominators for diagnostics only. Speeds are before current integration. Saturation does not by itself prove a higher-friction treatment helps. No scoring thresholds changed.',intervals=[])
for lo,hi in [(0,5),(5,6),(6,8),(8,float(t.max())+.001)]:
    mask=loaded&(t>=lo)&(t<hi)
    row=dict(start_s=lo,end_s=hi,n=int(mask.sum()))
    if mask.any():
        sliding=mask&(vt>.001)
        row.update(normal_force_N_p50_p95=np.quantile(fn[mask],[.5,.95]).tolist(),
                   tangent_speed_mm_s_p50_p95=(np.quantile(vt[mask],[.5,.95])*1000).tolist(),
                   utilization_p50_p95=np.quantile(u[mask],[.5,.95]).tolist(),
                   saturation_fraction=float(np.mean(u[mask]>=.95)),
                   moving_fraction=float(np.mean(vt[mask]>.001)),
                   saturation_fraction_among_moving=float(np.mean(u[sliding]>=.95)) if sliding.any() else None,
                   effective_mu_values=np.unique(col('friction')[mask]).tolist())
    summary['intervals'].append(row)
if loaded.any():summary['first_loaded_finger_contact_s']=float(t[loaded].min())
(ROOT/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='all_reference_arrays_exact'},indent=2))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True,layout='constrained')
time=(np.arange(len(z['trajectory']))+1)*.03
tr=z['trajectory']
axs[0].plot(time,(tr[:,13]-tr[0,13])*1000,label='Can x displacement')
axs[0].plot(time,(tr[:,14]-tr[0,14])*1000,label='Can y displacement')
axs[0].plot(time,(tr[:,15]-tr[0,15])*1000,label='Can z displacement')
axs[0].set_ylabel('Displacement (mm)');axs[0].legend(loc='upper left',fontsize=8)
axs[1].scatter(t[loaded],vt[loaded]*1000,c=u[loaded],vmin=0,vmax=1,s=8,cmap='viridis')
axs[1].set_ylabel('Pre-integration tangent\nspeed (mm/s)')
sc=axs[2].scatter(t[loaded],u[loaded],c=fn[loaded],s=8,cmap='plasma')
axs[2].axhline(1,color='black',lw=.6);axs[2].set_ylabel('Friction pyramid\nutilization');axs[2].set_xlabel('Replay time (s)')
fig.colorbar(sc,ax=axs[2],label='Normal force (N)')
for ax in axs:ax.set_xlim(4.5,8);ax.grid(alpha=.2)
fig.suptitle('113 soft hand: observed approach contacts; trajectory reproduced exactly')
fig.savefig(ROOT/'113_approach_contact_dynamics.png',dpi=170)
fig.savefig(ROOT/'113_approach_contact_dynamics.svg')
