"""Check URDF finger mass properties against their actual watertight meshes."""
from pathlib import Path
import json,hashlib,xml.etree.ElementTree as ET
import numpy as np,trimesh
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3];source=REPO/'gen3_lite_2f_robotiq_85.urdf';tree=ET.parse(source);records=[]
for link in tree.getroot().findall('link'):
 name=link.get('name')
 if 'finger' not in name:continue
 mesh_path=REPO/link.find('visual/geometry/mesh').get('filename').removeprefix('package://');mesh=trimesh.load(mesh_path,force='mesh');assert mesh.is_watertight
 inertial=link.find('inertial');mass=float(inertial.find('mass').get('value'));com=np.array(list(map(float,inertial.find('origin').get('xyz').split())))
 assert np.allclose(list(map(float,inertial.find('origin').get('rpy').split())),0)
 d=inertial.find('inertia').attrib;I=np.array([[float(d['ixx']),float(d['ixy']),float(d['ixz'])],[float(d['ixy']),float(d['iyy']),float(d['iyz'])],[float(d['ixz']),float(d['iyz']),float(d['izz'])]])
 meshI=mesh.moment_inertia*(mass/mesh.mass);new_com=np.copysign(abs(com),mesh.center_mass);new_I=I.copy()
 # Preserve published diagonal/magnitudes; only flip nonzero cross terms whose
 # signs disagree with the mirrored mesh. Do not reflect an already-correct tensor.
 for i in range(3):
  for j in range(i):
   if I[i,j]!=0:new_I[i,j]=new_I[j,i]=np.copysign(abs(I[i,j]),meshI[i,j])
 assert np.linalg.eigvalsh(new_I).min()>0
 records.append(dict(link=name,mesh=str(mesh_path),mesh_sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),mass_kg=mass,mesh_bounds=mesh.bounds.tolist(),published_com=com.tolist(),mesh_uniform_density_com=mesh.center_mass.tolist(),published_com_outside_mesh_aabb=bool(np.any(com<mesh.bounds[0]) or np.any(com>mesh.bounds[1])),proposed_com=new_com.tolist(),proposed_com_mesh_difference_m=float(np.linalg.norm(new_com-mesh.center_mass)),published_inertia=I.tolist(),mesh_uniform_density_inertia=meshI.tolist(),proposed_inertia=new_I.tolist(),relative_inertia_error_before=float(np.linalg.norm(I-meshI)/np.linalg.norm(meshI)),relative_inertia_error_after=float(np.linalg.norm(new_I-meshI)/np.linalg.norm(meshI))))
result=dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),records=records,qualification='Mesh uniform density is corroborating evidence, not a material-density measurement. Proposed correction preserves published mass, COM magnitudes and inertia magnitudes, aligning signs with corresponding mesh/mirror geometry. Three COMs outside mesh bounds are impossible for mass confined to those meshes; undocumented extra mass outside geometry would be an alternative explanation.')
(ROOT/'audit.json').write_text(json.dumps(result,indent=2))
for r in records:print(r['link'],'outside',r['published_com_outside_mesh_aabb'],'COM residual um',round(r['proposed_com_mesh_difference_m']*1e6,2),'I error after',round(r['relative_inertia_error_after'],4))
