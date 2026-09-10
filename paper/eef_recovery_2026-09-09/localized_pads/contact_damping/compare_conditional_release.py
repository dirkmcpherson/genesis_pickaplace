"""Compare saved conditional113 release trajectories; no physics or scoring edits."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;P=D.parent
fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True,layout='constrained')
records=[]
for material in ['rigid','soft']:
 for gain in [1,2]:
  folder=P/f'vision_initial_probe/{material}' if gain==1 else D/f'corrected/113_conditional_{material}_g2'
  z=np.load(folder/'113_eef_delta.npz');tr=z['trajectory'];t=(np.arange(len(tr))+1)*.03
  mask=(t>=19.8)&(t<=21.5);x=tr[mask];label=f'{material}, damping {gain}x'
  style='-' if gain==1 else '--';color='tab:blue' if material=='rigid' else 'tab:orange'
  axes[0].plot(t[mask],1000*(x[:,13]-.55),ls=style,color=color,label=label)
  axes[1].plot(t[mask],1000*x[:,15],ls=style,color=color,label=label)
  axes[2].plot(t[mask],x[:,20],ls=style,color=color,label=label)
  samples=[]
  for time in [8,18,20.04,20.16,20.34,21]:
   i=int(np.argmin(abs(t-time)))
   samples.append(dict(time_s=float(t[i]),can_xyz_m=tr[i,13:16].tolist(),tilt_deg=float(tr[i,20]),contacts=z['contact_counts'][i].tolist()))
  records.append(dict(material=material,gain=gain,path=str(folder/'113_eef_delta.npz'),samples=samples))
axes[0].axhline(0,color='black',ls=':',label='Shelf front edge')
axes[1].axhline(220.5,color='black',ls=':',label='Upright supported center height')
axes[0].set_ylabel('Can center past front (mm)');axes[1].set_ylabel('Can center height (mm)')
axes[2].set(ylabel='Tilt (degrees)',xlabel='Replay time (s)')
for ax in axes:ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.suptitle('113 conditional image-derived start: added damping does not recover release\nSame source motion and elastic coefficient; both normal and tangent damping change.')
fig.savefig(D/'conditional_release_comparison.png',dpi=150)
(D/'conditional_release_comparison.json').write_text(json.dumps(dict(records=records,qualification='Saved decision-time states. Conditional starting position remains unadopted. Treatment acts throughout approach/carry, so release starts from different contact configurations even though the elastic coefficient at fixed penetration is preserved.'),indent=2))
