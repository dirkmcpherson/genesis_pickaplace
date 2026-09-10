"""Convex pad/backing partition with an unchanged outer collision envelope.

Three-millimetre maximum cap depth and pad boundary are explicit hypotheses,
not measured Kinova material dimensions. No additional mass or joints.
"""
from pathlib import Path
import hashlib, json, copy
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from adaptive_gripper_candidate import make_urdf, REPO

REFERENCE=REPO/'paper/eef_recovery_2026-09-09/localized_pads/reference'

def hull(points):
    # Preserve tiny facets; trimesh's default processing can delete one and leave
    # a small hole in this engine's left distal reference mesh.
    q=ConvexHull(points)
    mesh=trimesh.Trimesh(vertices=points,faces=q.simplices,process=False)
    trimesh.repair.fix_normals(mesh)
    return mesh

def clip(mesh, normal, offset):
    """Closed convex portion n.x <= offset using retained vertices and edge cuts."""
    v=np.asarray(mesh.vertices);d=v@normal-offset
    edges=mesh.edges_unique;a,b=edges.T
    crossing=(d[a]*d[b]<0)
    a=a[crossing];b=b[crossing]
    intersections=v[a]+(v[b]-v[a])*(d[a]/(d[a]-d[b]))[:,None]
    points=np.vstack([v[d<=1e-12],intersections])
    return hull(points)

def make_pad_urdf(path,geometry='split',depth=.003):
    assert geometry in ('hull','split') and 0<depth<.006
    path=Path(path);info=make_urdf(path)
    tree=ET.parse(path);root=tree.getroot()
    assets=path.parent/'pad_candidate_assets';assets.mkdir(exist_ok=False)
    records=[]
    for link in root.findall('link'):
        name=link.attrib['name']
        if 'finger' not in name:continue
        reference=trimesh.load(REFERENCE/f'{name}.ply',process=False)
        original=hull(reference.vertices)
        assert original.is_convex and original.is_watertight
        source=link.find('collision');assert len(link.findall('collision'))==1
        normal=np.array([.42 if 'prox' in name else .1,1 if 'right' in name else -1,0.])
        normal/=np.linalg.norm(normal)
        offset=float((original.vertices@normal).max()-depth)
        if geometry=='split':
            backing=clip(original,normal,offset)
            cap=clip(original,-normal,-offset)
            pad=clip(cap,np.array([1.,0,0]),-.008)
            knuckle=clip(cap,np.array([-1.,0,0]),.008)
            pieces=[('backing',backing),('pad',pad),('knuckle',knuckle)]
        else:pieces=[('hull',original)]
        volume_error=abs(sum(m.volume for _,m in pieces)-original.volume)/original.volume
        assert volume_error<1e-6,volume_error
        rng=np.random.default_rng(9);directions=rng.normal(size=(2000,3));directions/=np.linalg.norm(directions,axis=1)[:,None]
        combined=np.vstack([m.vertices for _,m in pieces])
        support_error=float(np.max(abs(np.max(combined@directions.T,axis=0)-np.max(original.vertices@directions.T,axis=0))))
        assert support_error<1e-8,support_error
        link.remove(source);outputs=[]
        for label,mesh in pieces:
            asset=assets/f'{name}_{label}.ply';mesh.export(asset)
            element=copy.deepcopy(source);element.set('name',f'{name}_{label}')
            element.find('geometry/mesh').set('filename',str(asset.resolve()))
            link.append(element)
            outputs.append(dict(label=label,path=str(asset.resolve()),sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
                faces=len(mesh.faces),volume_m3=mesh.volume,bounds_m=mesh.bounds.tolist()))
        records.append(dict(link=name,normal=normal.tolist(),offset_m=offset,
            volume_relative_error=volume_error,support_max_error_m=support_error,pieces=outputs))
    assert len(records)==4
    tree.write(path,encoding='utf-8',xml_declaration=True)
    info.update(candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),geometry=geometry,
        max_cap_depth_m=depth,pad_x_limit_m=-.008,regions=records,
        interpretation='Assumed inner cap material region; geometric partition of processed reference hull. No added thickness or mass; depth is not calibrated material compression.')
    (path.parent/'pad_geometry.json').write_text(json.dumps(info,indent=2))
    return info

def preserve_pad_meshes():
    """Avoid re-decimating already processed/split reference geometry only."""
    from genesis.utils import mesh as mu
    original=mu.postprocess_collision_geoms
    def process(g_infos,decimate,*args,**kwargs):
        selected=['pad_candidate_assets' in str(g['mesh'].metadata.get('mesh_path','')) for g in g_infos]
        if any(selected):
            assert all(selected)
            decimate=False
        return original(g_infos,decimate,*args,**kwargs)
    mu.postprocess_collision_geoms=process
