"""Real-image comparison using the same fixed annotations and camera hypothesis."""
from pathlib import Path
import json
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
ROOT=Path(__file__).resolve().parent
paths=[ROOT.parent/'adaptive_transmission/233_active/can_seating_comparison.json',
       ROOT/'233_fingers/can_seating_comparison.json',ROOT/'233_can/can_seating_comparison.json']
data=[json.loads(p.read_text()) for p in paths]
labels=['Adaptive hand, original contact','Softer fingers','Softer can']
fig,axes=plt.subplots(3,3,figsize=(13,12))
for row,index in enumerate([0,1,3]):
 for col,(dataset,label) in enumerate(zip(data,labels)):
  rec=dataset['records'][index];ax=axes[row,col]
  img=cv2.cvtColor(cv2.imread(rec['image']),cv2.COLOR_BGR2RGB)
  cx,cy=rec['real_rim_ellipse']['center_px'];pred=rec['nominal']['top_rim_ellipse']
  ax.imshow(img)
  ax.add_patch(Ellipse(pred['center_px'],*pred['axes_px'],angle=pred['angle_deg'],fill=False,color='#ff5733',lw=2))
  ax.plot(cx,cy,'yo',ms=4)
  ax.set_xlim(cx-165,cx+145);ax.set_ylim(cy+185,cy-125);ax.axis('off')
  ax.set_title(f'{label}\n{rec["time_s"]:.2f} s: {rec["nominal"]["cap_relative_error_norm_px"]:.1f} px error',fontsize=10)
fig.suptitle('Trial 233: softer contact improves the loaded can position, then worsens placement\nRed = simulated top rim attached to real wrist pose; yellow = annotated real top center\nErrors are relative to gripper-cap landmarks. Conditional camera comparison, not measured 3D accuracy.',fontsize=11)
fig.tight_layout(rect=[0,0,1,.92]);fig.savefig(ROOT/'233_softness_real_comparison.png',dpi=160);plt.close(fig)
