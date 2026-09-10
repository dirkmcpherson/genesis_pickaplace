"""Inspect saved finger poses at the real wrist, with the cap-only camera fixed."""
from pathlib import Path
import json,sys,xml.etree.ElementTree as ET
import cv2,numpy as np,trimesh
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent;REPO=R.parents[3]
sys.path.insert(0,str(REPO/'baselines'))
from eef_delta_control import ArmKinematics,transform
report=json.loads((R/'113_conditional_seating.json').read_text())
c={k:np.array(v) for k,v in report['nominal_camera'].items()}
path=R.parent/'g14_feedback_full/113_elliptic10_soft/113_eef_delta.npz'
z=np.load(path);source=np.load(json.loads(path.with_suffix('.json').read_text())['timing_reconstruction']['source'])
fk=ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf')
root=ET.parse(REPO/'gen3_lite_2f_robotiq_85.urdf').getroot()
links={l.attrib['name']:l for l in root.findall('link')}
joints={j.find('child').attrib['link']:j for j in root.findall('joint')}
names=['left_finger_prox_link','right_finger_prox_link','left_finger_dist_link','right_finger_dist_link']
meshes={}
for name in names:
    mesh=links[name].find('visual/geometry/mesh').attrib['filename'].replace('package://','')
    meshes[name]=np.asarray(trimesh.load(REPO/mesh,process=False).vertices)
tool_from_gripper=transform(np.array([0,0,.13]),Rotation.from_euler('z',np.pi/2).as_matrix())
times=float(source['t_frame'][0])+(np.arange(len(z['finger_joint']))+1)*.03
fig,axes=plt.subplots(3,2,figsize=(11,13),layout='constrained');out=[]
for row,obs in enumerate([r for r in report['records'] if r['time_s'] in [8,14,20.04]]):
    t=obs['actual_camera_time_s'];q=np.array([np.interp(t,source['t_frame'],source['q_frame'][:,j]) for j in range(6)])
    pose=fk.tool(q,np.eye(4))@np.linalg.inv(tool_from_gripper)
    grip=np.interp(t,source['t_frame'],source['g_frame']);theta=.96-1.05*grip/100
    target=np.array([-theta,theta,.149-.676*theta,.149-.676*theta])
    actual=np.array([np.interp(t,times,z['finger_joint'][:,j]) for j in range(4)])
    img=cv2.cvtColor(cv2.imread(str(R/f'113_{obs["time_s"]:g}_cam4.jpg')),cv2.COLOR_BGR2RGB)
    for ax,label,angles in zip(axes[row],['Commanded unloaded relation','Saved simulated finger pose'],[target,actual]):
        ax.imshow(img);positions={'gripper_base_link':pose}
        for name,angle in zip(names,angles):
            j=joints[name];o=j.find('origin');xyz=np.fromstring(o.attrib['xyz'],sep=' ');rpy=np.fromstring(o.attrib['rpy'],sep=' ');axis=np.fromstring(j.find('axis').attrib['xyz'],sep=' ')
            positions[name]=positions[j.find('parent').attrib['link']]@transform(xyz,Rotation.from_euler('xyz',rpy).as_matrix())@transform(np.zeros(3),Rotation.from_rotvec(axis*angle).as_matrix())
            p=positions[name];world=meshes[name]@p[:3,:3].T+p[:3,3]
            uv=cv2.projectPoints(world,c['rv'],c['tv'],c['K'],c['dist'])[0].reshape(-1,2);uv=np.c_[uv[:,1],1279-uv[:,0]].astype(np.float32)
            hull=cv2.convexHull(uv).reshape(-1,2);hull=np.vstack([hull,hull[:1]])
            ax.plot(*hull.T,color='cyan' if 'prox' in name else 'magenta',lw=1)
        center=np.array(obs['caps']).mean(axis=0)
        ax.set(xlim=(center[0]-130,center[0]+130),ylim=(center[1]+110,center[1]-150),title=f'{obs["time_s"]:g}s: {label}');ax.axis('off')
    out.append(dict(time_s=obs['time_s'],target_rad=target.tolist(),simulated_rad=actual.tolist()))
fig.suptitle('113: original CAD outlines at the real wrist pose\nCyan proximal; magenta distal. Convex outlines include hidden surfaces.\nReal blue wrapping is not represented by the CAD; camera/cap assumptions remain.')
fig.savefig(R/'113_finger_projection.png',dpi=150)
(R/'113_finger_projection.json').write_text(json.dumps(dict(records=out,qualification='Read-only geometry diagnostic. No real finger angle fitting or padding thickness inference.'),indent=2))
