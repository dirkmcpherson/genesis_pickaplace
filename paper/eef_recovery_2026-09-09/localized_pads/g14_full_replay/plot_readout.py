from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent
names=[('Pinned rigid',R.parent/'finger_inertia_alignment/233_tc0.02'),('1.4 pyramid rigid',R/'233_pyramid_rigid_port3'),('1.4 pyramid soft',R/'233_pyramid_soft_port3'),('1.4 elliptic10 rigid',R/'233_elliptic10_rigid_port3'),('1.4 elliptic10 soft',R/'233_elliptic10_soft_port3')]
fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained');rows=[]
for label,p in names:
 z=np.load(p/'233_eef_delta.npz');t=(np.arange(len(z['source_grip']))+1)*.03
 sel=(t>=.3)&(t<=3)
 q=z['finger_joint'];theta=.96-1.05*z['source_grip']/100
 target=np.stack([-theta,theta,.149-.676*theta,.149-.676*theta],axis=-1)
 error=np.rad2deg(abs(q-target))
 axes[0].plot(t[t<=4],np.rad2deg(q[t<=4,1]),label=label,lw=1)
 axes[1].plot(t,z['trajectory'][:,15],label=label,lw=1)
 rows.append(dict(condition=label,window_s=[.3,3],max_can_hand_contact_count=int(z['contact_counts'][sel,1].max()),finger_unloaded_relation_error_deg_p50_p95_max=np.percentile(error[sel],[50,95,100]).tolist()))
axes[0].plot(t[t<=4],np.rad2deg(theta[t<=4]),'k--',label='Unloaded target',lw=1.8)
axes[0].set(xlabel='Source time (s)',ylabel='Right proximal joint (degrees)',title='Approach: changed finger response before can grasp');axes[0].legend(fontsize=8,ncol=2)
axes[1].set(xlabel='Source time (s)',ylabel='Can center height (m)',title='Full source replay: new-engine controls fail pickup')
for ax in axes:ax.grid(alpha=.2)
fig.suptitle('Trial 233: full-world engine compatibility experiment\nSame tool targets and recorded grip; no added motion')
fig.savefig(R/'233_hand_and_carry.png',dpi=160);fig.savefig(R/'233_hand_and_carry.svg')
(R/'approach_readout.json').write_text(json.dumps({'records':rows,'qualification':'Sampled approach contains zero can-hand contacts in the reported window. This is not a separate collision-disabled or zero-gravity bench; robot self-contact and other world forces have not been excluded.'},indent=2))
print(json.dumps(rows,indent=2))
