"""Rebuild the mass-inclusive bench summary and standalone plot."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
n=np.array([-1,1,-2,-2.]);D=np.diag([.05,.05,.002,.002]);stiffness=.7011904;rows=[]
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
for case,color,label in [('default','#b2182b','Added inertia 0.1 kg m²/joint'),('all_1e-4','#2166ac','Added inertia 0.0001 kg m²/joint')]:
 z=np.load(ROOT/case/'trace.npz')['values'];r=json.loads((ROOT/case/'report.json').read_text());assert r['error'] is None and len(z)==r['planned_frames']
 M=np.asarray(r['mass_matrix_at_end']);physical=M-np.diag(r['armature_after'])-.00125*D
 mass=float(n@(physical+np.diag(r['armature_after']))@n);damp=float(n@D@n);wn=(stiffness/mass)**.5
 rows.append(dict(treatment=case,internal_mode_effective_mass=mass,natural_period_s=2*np.pi/wn,damping_ratio=damp/(2*(stiffness*mass)**.5),max_free_shape_error_deg=float(np.rad2deg(abs(z[:,2:6]-z[:,10:14])).max()),max_hold_checkpoint_error_deg=max(float(np.rad2deg(abs(np.asarray(c['target_error']))).max()) for c in r['checkpoints'] if c['time_s'] in [3.5,6,9.5])))
 axes[1].plot(z[:,0],np.rad2deg(z[:,3]),label=label,color=color);axes[2].plot(z[:,0],np.rad2deg(z[:,4:6].mean(axis=1)),color=color)
axes[0].plot(z[:,0],z[:,1],color='black');axes[0].set_ylabel('Motor input (%)')
axes[1].plot(z[:,0],np.rad2deg(z[:,11]),'k--',label='Unloaded relation (model)');axes[2].plot(z[:,0],np.rad2deg(z[:,12]),'k--')
axes[1].set_ylabel('Right proximal angle (deg)');axes[2].set_ylabel('Mean distal angle (deg)');axes[2].set_xlabel('Bench time (s)')
for ax in axes:ax.grid(alpha=.2)
axes[1].legend(fontsize=9);fig.suptitle('Unloaded hand: effect of generic added joint inertia\nSame transmission; no contact, gravity, arm motion or material treatment');fig.tight_layout()
fig.savefig(ROOT/'unloaded_response.png',dpi=180);fig.savefig(ROOT/'unloaded_response.svg');plt.close(fig)
(ROOT/'mode_readout.json').write_text(json.dumps(dict(records=rows,mode=n.tolist(),modal_stiffness=stiffness,qualification='Linearized internal mode at fixed assumed actuator coordinate. Effective mass at end pose; includes URDF linkage mass and added joint inertia, subtracts solver damping*dt. Not a hardware measurement.'),indent=2))
print(json.dumps(rows,indent=2))
