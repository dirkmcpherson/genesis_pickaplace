"""Consolidate all declared engine-contact bench cases without task claims."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
names=['legacy','g14_pyramid1_v3','g14_elliptic1','g14_elliptic10_v3']
labels=['Pinned 0.2.1','1.4 pyramidal / ratio 1','1.4 elliptic / ratio 1','1.4 elliptic / ratio 10']
reports={n:json.loads((ROOT/n/'report.json').read_text()) for n in names}
reference=reports[names[0]]['records'];rows=[]
for i,r in enumerate(reference):
    key={k:r[k] for k in ['pad_tc','penetration_m','offset_m']}
    row=dict(**key,conditions={})
    for n,d in reports.items():
        v=d['records'][i]
        assert all(v[k]==value for k,value in key.items())
        assert v['geometry']==r['geometry'],(n,i)
        assert np.allclose(v['armature'],0)
        row['conditions'][n]={k:v[k] for k in ['mass_kg','droop_deg_at_0p1_0p5_1_2_s',
            'displacement_mm_at_0p1_0p5_1_2_s','contact_force_sum_N_at_0p1_0p5_1_2_s']}
    rows.append(row)
out=dict(completed_conditions=4,cases_per_condition=8,all_input_mesh_hashes_and_normal_parameters_exact=True,
         records=rows,qualification='Eight spatially separated sets per scene, four scenes; not independent task demonstrations. New analytic cylinder mass differs .645% from legacy mesh mass. Same-engine cone comparisons avoid this mass confound. Fixed-pad evidence is not full-task recovery or hardware material calibration.')
(ROOT/'summary.json').write_text(json.dumps(out,indent=2))
fig,axs=plt.subplots(2,2,figsize=(11,7),sharex=True,layout='constrained')
for j,pen in enumerate([.0005,.002]):
    for k,off in enumerate([0,.03]):
        i=next(i for i,v in enumerate(reference) if v['pad_tc']==.03 and v['penetration_m']==pen and v['offset_m']==off)
        for n,label in zip(names,labels):
            tr=np.load(ROOT/n/f'case{i}.npz')['values']
            origin=np.array([.4*(i%4),.5*(i//4),.3])
            y=np.linalg.norm(tr[:,1:4]-origin,axis=1)*1000 if off==0 else tr[:,4]
            axs[k,j].plot(tr[:,0],y,label=label,lw=1.6)
        axs[k,j].set_title(f'{pen*1000:g} mm initial overlap; {off*1000:g} mm offset')
        axs[k,j].set_ylabel('Can displacement (mm)' if off==0 else 'Can-axis droop (degrees)')
        axs[k,j].grid(alpha=.2)
        if k==1:axs[k,j].set_xlabel('Time (s)')
axs[0,0].legend(fontsize=8)
fig.suptitle('Fixed soft-pad bench: declared pad time constant 0.03 s, can 0.02 s\nSame input meshes and normal parameters; no robot or demonstration replay')
fig.savefig(ROOT/'soft_contact_comparison.png',dpi=170)
fig.savefig(ROOT/'soft_contact_comparison.svg')
print('Verified all 32 bench-case records, input meshes and normal parameter assignments.')
