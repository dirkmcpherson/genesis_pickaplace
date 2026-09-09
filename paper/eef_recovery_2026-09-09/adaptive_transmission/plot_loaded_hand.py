"""Orthographic mesh/trace plot, not a photograph or physics rerun."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np,cv2,trimesh
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[2]
tree=ET.parse(REPO/'gen3_lite_2f_robotiq_85.urdf').getroot()
links={x.attrib['name']:x for x in tree.findall('link')}
joints={x.find('child').attrib['link']:x for x in tree.findall('joint')}
def T(x,r):
 t=np.eye(4);t[:3,:3]=r;t[:3,3]=x;return t
def pose(row,offset):
 return T(row[offset:offset+3],Rotation.from_quat(row[[offset+4,offset+5,offset+6,offset+3]]).as_matrix())
names=['left_finger_prox_link','right_finger_prox_link','left_finger_dist_link','right_finger_dist_link']
paths=[ROOT.parent/'timestamp_full_pool/233/collection/233_eef_delta.npz',ROOT/'233_active/233_eef_delta.npz']
fig,axes=plt.subplots(1,2,figsize=(10,6))
for ax,path,label in zip(axes,paths,['Existing fixed coupling','Adaptive candidate']):
 z=np.load(path);i=464;q=z['finger_joint'][i];row=z['trajectory'][i]
 transforms={'gripper_base_link':np.eye(4)}
 for name,angle in zip(names,q):
  joint=joints[name];origin=joint.find('origin')
  xyz=np.fromstring(origin.attrib['xyz'],sep=' ');rpy=np.fromstring(origin.attrib['rpy'],sep=' ')
  axis=np.fromstring(joint.find('axis').attrib['xyz'],sep=' ')
  transforms[name]=transforms[joint.find('parent').attrib['link']]@T(xyz,Rotation.from_euler('xyz',rpy).as_matrix())@T(np.zeros(3),Rotation.from_rotvec(axis*angle).as_matrix())
 for name,t in transforms.items():
  mesh=links[name].find('visual/geometry/mesh').attrib['filename'].replace('package://','')
  vertices=np.asarray(trimesh.load(REPO/mesh,process=False).vertices)
  points=((t[:3,:3]@vertices.T).T+t[:3,3])[:,[1,2]]*1000
  hull=cv2.convexHull(points.astype(np.float32)).reshape(-1,2)
  color='#457b9d' if 'dist' in name else '#9ca3af'
  ax.fill(hull[:,0],hull[:,1],color=color,alpha=.65)
  if name!='gripper_base_link':ax.plot(t[1,3]*1000,t[2,3]*1000,'ko',ms=4)
 gripper_tool=T([0,0,.13],Rotation.from_euler('z',np.pi/2).as_matrix())
 gripper_can=gripper_tool@np.linalg.inv(pose(row,6))@pose(row,13)
 a=np.linspace(0,2*np.pi,128)
 points=np.vstack([np.c_[.033*np.cos(a),.033*np.sin(a),np.full(len(a),h)] for h in [-.0505,.0505]])
 points=((gripper_can[:3,:3]@points.T).T+gripper_can[:3,3])[:,[1,2]]*1000
 hull=cv2.convexHull(points.astype(np.float32)).reshape(-1,2);hull=np.vstack([hull,hull[:1]])
 ax.plot(hull[:,0],hull[:,1],color='#e76f51',lw=2,label='Simulated can projection')
 curl=-np.rad2deg(q[2:]-(.149-.676*q[1])).mean()
 ax.set_title(f'{label}\nIndependent inward curl: {curl:.1f}°')
 ax.set_aspect('equal');ax.set_xlim(-85,85);ax.set_ylim(30,200)
 ax.set_xlabel('Across the gripper (mm)');ax.set_ylabel('Distance along gripper from wrist origin (mm)')
 ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=8)
fig.suptitle('Trial 233 loaded grasp at approximately 14 s\nSaved simulated finger poses and can/tool transforms; blue = distal links',fontsize=11)
fig.tight_layout(rect=[0,0,1,.92]);fig.savefig(ROOT/'loaded_hand_comparison.png',dpi=160)
