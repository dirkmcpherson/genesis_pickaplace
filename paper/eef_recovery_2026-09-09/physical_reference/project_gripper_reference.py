"""Project recorded-motor URDF geometry onto calibration images, without fitting fingers.

This is a kinematic hypothesis check, not a simulated contact outcome or a joint-angle
measurement. Camera alternatives are fit only to the original three cap poses.
"""
from pathlib import Path
import json, sys, hashlib, xml.etree.ElementTree as ET
import cv2, numpy as np, trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

R = Path(__file__).resolve().parent
REPO = R.parents[2]
sys.path.insert(0, str(REPO / 'baselines'))
from eef_delta_control import ArmKinematics, transform

pilot = json.loads((R/'233/rigid_cap_camera_pilot.json').read_text())
checks = json.loads((R/'233/cap_camera_sensitivity.json').read_text())['unused_pose_annotations']
meta = json.loads((R.parent/'timestamp_full_pool/233/collection/233_eef_delta.json').read_text())
source = Path(meta['timing_reconstruction']['source'])
z = np.load(source)
trace_path = R.parent/'timestamp_full_pool/233/collection/233_eef_delta.npz'
trace = np.load(trace_path)
trace_times = z['t_frame'][0] + (np.arange(len(trace['finger_joint']))+1)*.03
fk = ArmKinematics(REPO/'gen3_lite_2f_robotiq_85.urdf')
tool = transform(np.array([0, 0, .13]), Rotation.from_euler('z', np.pi/2).as_matrix())
def pose(t):
    q = np.array([np.interp(t, z['t_frame'], z['q_frame'][:,j]) for j in range(6)])
    return fk.tool(q, np.eye(4)) @ np.linalg.inv(tool)

local = np.array([[.0152, -.0305, .070003], [.0152, .0305, .070003]])
train = np.concatenate([(p[:3,:3]@local.T).T+p[:3,3] for p in map(pose, pilot['camera_times_s'])])
clicks = np.array(list(pilot['manual_cap_centers_rotated_pixels'].values())).reshape(-1,2)
pixels = np.ascontiguousarray(np.c_[1279-clicks[:,1], clicks[:,0]], dtype=np.float64)
fx, fy, cx, cy, kold = pilot['intrinsics_prior']
cameras = []
for scale in [.8, .9, 1., 1.1, 1.2]:
    for k in [-.25, kold, 0.]:
        K = np.array([[fx*scale,0,cx],[0,fy*scale,cy],[0,0,1.]])
        dist = np.array([k,0,0,0,0.])
        ok, rv, tv = cv2.solvePnP(train,pixels,K,dist,flags=cv2.SOLVEPNP_SQPNP)
        assert ok
        rv,tv = cv2.solvePnPRefineLM(train,pixels,K,dist,rv,tv)
        cameras.append((scale,k,K,dist,rv,tv))

urdf = ET.parse(REPO/'gen3_lite_2f_robotiq_85.urdf').getroot()
links = {x.attrib['name']:x for x in urdf.findall('link')}
joints = {x.find('child').attrib['link']:x for x in urdf.findall('joint')}
names = ['gripper_base_link','left_finger_prox_link','right_finger_prox_link',
         'left_finger_dist_link','right_finger_dist_link']
meshes = {}
for name in names:
    v = links[name].find('visual')
    assert v.find('origin').attrib == {'rpy':'0 0 0','xyz':'0 0 0'}
    mesh = REPO/v.find('geometry/mesh').attrib['filename'].replace('package://','')
    meshes[name] = np.asarray(trimesh.load(mesh,process=False).vertices)

def geometry(t, simulated=False):
    g = float(np.interp(t,z['t_frame'],z['g_frame']))
    theta = -.09 + (1-np.clip(g/100,0,1))*1.05
    tip = -.676*theta+.149
    angles = [-theta,theta,tip,tip]
    if simulated:
        angles = [float(np.interp(t,trace_times,trace['finger_joint'][:,j])) for j in range(4)]
    poses = {'gripper_base_link':pose(t)}
    for name,angle in zip(names[1:],angles):
        joint = joints[name]
        origin = joint.find('origin')
        xyz = np.fromstring(origin.attrib['xyz'],sep=' ')
        rpy = np.fromstring(origin.attrib['rpy'],sep=' ')
        axis = np.fromstring(joint.find('axis').attrib['xyz'],sep=' ')
        poses[name] = poses[joint.find('parent').attrib['link']] @ transform(xyz,Rotation.from_euler('xyz',rpy).as_matrix()) @ transform(np.zeros(3),Rotation.from_rotvec(axis*angle).as_matrix())
    world = {n:(poses[n][:3,:3]@meshes[n].T).T+poses[n][:3,3] for n in names}
    return g, angles, world

fig,axes = plt.subplots(4,3,figsize=(15,14))
records=[]
for row,check in enumerate(checks):
    t=check['time_s']; label=check['label']
    path=R/'233'/('rigid_cap_unused_14s.jpg' if label=='14s' else f'cap_check_{label}.jpg')
    img=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB)
    g,angles,world=geometry(t)
    centers=np.array(check['centers']); center=centers.mean(axis=0)
    for ax in axes[row]:
        ax.imshow(img);ax.set_xlim(center[0]-150,center[0]+150);ax.set_ylim(center[1]+125,center[1]-160);ax.axis('off')
    axes[row,0].set_title(f'Real {t:.3f} s; motor {g:.2f}%')
    axes[row,1].set_title('Commanded fingers (contact excluded)')
    axes[row,2].set_title('Simulated fingers at real wrist pose')
    contours={}
    sensitivity={}
    sim_angles=geometry(t,True)[1]
    for col,simulated in [(1,False),(2,True)]:
        world=geometry(t,simulated)[2];contours[str(col)]={}
        projected={name:[] for name in world}
        nominal_index=None
        for camera_index,(scale,k,K,dist,rv,tv) in enumerate(cameras):
            nominal=scale==1 and k==kold
            if nominal:nominal_index=camera_index
            for name,xyz in world.items():
                raw=cv2.projectPoints(xyz,rv,tv,K,dist)[0].reshape(-1,2)
                uv=np.c_[raw[:,1],1279-raw[:,0]].astype(np.float32)
                projected[name].append(uv)
                hull=cv2.convexHull(uv).reshape(-1,2);hull=np.vstack([hull,hull[:1]])
                color='lime' if 'base' in name else ('cyan' if 'prox' in name else 'magenta')
                axes[row,col].plot(hull[:,0],hull[:,1],color=color,lw=1.3 if nominal else .4,alpha=1 if nominal else .10)
                if nominal:contours[str(col)][name]=hull.tolist()
        axes[row,col].plot(centers[:,0],centers[:,1],'yo',ms=3)
        sensitivity[str(col)]={name:float(np.linalg.norm(np.array(views)-views[nominal_index],axis=2).max()) for name,views in projected.items()}
    records.append(dict(time_s=t,image=str(path),image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),motor_feedback=g,target_angles_rad=angles,simulated_angles_rad=sim_angles,lens_sensitivity_max_vertex_displacement_px=sensitivity,nominal_projected_convex_outlines=contours))
fig.suptitle('233 camera 4: fixed mimic hypothesis, no finger fitting\nGreen base; cyan proximal; magenta distal; yellow observed caps. Faint lines: lens sensitivity.\nConvex bounds include hidden surfaces. Both projections use the real wrist pose; neither fits real fingers.',fontsize=11)
fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(R/'233/gripper_projection_check.png',dpi=160);plt.close(fig)
report=dict(records=records,source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),simulated_trace=str(trace_path),simulated_trace_sha256=hashlib.sha256(trace_path.read_bytes()).hexdigest(),simulated_clock='Raw first t_frame plus (index+1)*0.03; linear interpolation of simulated joint observations',camera_fit='Original three cap poses only; no check images used in fitting',scope='Middle: recorded motor mapped through inverted replay targets and fixed URDF mimic, contact deflection excluded. Right: observed simulated finger angles mounted at real FK wrist pose to isolate hand shape. Lens alternatives are assumption sensitivity, not confidence bounds. Real padding/tape may differ from CAD. No simulation parameter changed.')
(R/'233/gripper_projection_check.json').write_text(json.dumps(report,indent=2))
print(json.dumps([{'time_s':r['time_s'],'motor_feedback':r['motor_feedback'],'target_angles_rad':r['target_angles_rad']} for r in records],indent=2))
