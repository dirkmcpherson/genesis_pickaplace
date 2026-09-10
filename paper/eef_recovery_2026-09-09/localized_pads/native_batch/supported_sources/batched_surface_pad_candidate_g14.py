"""Batched port of the existing localized pad law; runtime equivalence under verification."""
import numpy as np
import quadrants as qd
import genesis as gs
from genesis.utils import geom as gu

@qd.data_oriented
class BatchedSurfacePads:
    def __init__(self,world,pad_timeconst=.03,depth=.003,compliant_part='all'):
        assert gs.__version__=='1.4.0'
        assert .02<=pad_timeconst<=.04 and 0<depth<=.005
        assert compliant_part in ('all','proximal','distal')
        self.compliant_part=compliant_part
        self.solver=world['scene'].sim.rigid_solver
        self.collider=self.solver.collider
        self.contacts=self.collider.collider_state
        self.state=self.solver.dyn_state
        self.info=self.solver.dyn_info
        n=self.solver.n_geoms
        self.region=qd.Vector.field(4,dtype=gs.qd_float,shape=n)
        self.enabled=qd.field(dtype=qd.i32,shape=n)
        self.compliant=qd.field(dtype=qd.i32,shape=n)
        self.link_index=qd.field(dtype=qd.i32,shape=n)
        self.counts=qd.field(dtype=qd.i32,shape=(3,self.solver._B))
        self.activation_by_geom=qd.field(dtype=qd.i32,shape=(n,self.solver._B))
        self.pad_tc=float(pad_timeconst)
        fields=np.zeros((n,4),dtype=np.float32);enabled=np.zeros(n,dtype=np.int32)
        compliant=np.zeros(n,dtype=np.int32)
        links=np.zeros(n,dtype=np.int32);self.regions=[]
        for link in world['kinova'].links:
            if 'finger' not in link.name:continue
            assert len(link.geoms)==1
            g=link.geoms[0]
            normal=np.array([.42 if 'prox' in link.name else .1,1 if 'right' in link.name else -1,0.])
            normal/=np.linalg.norm(normal)
            # Material definition is in the finger link frame. New Genesis
            # recenters/reorients collision meshes; their raw verts are not
            # expressed in that frame.
            verts=g._init_verts@gu.quat_to_R(g.init_quat).T+g.init_pos
            offset=float((verts@normal).max()-depth)
            fields[g.idx]=np.r_[normal,offset];enabled[g.idx]=1
            compliant[g.idx]=int(compliant_part=='all' or
                (compliant_part=='proximal' and 'prox' in link.name) or
                (compliant_part=='distal' and 'dist' in link.name))
            links[g.idx]=link.idx
            self.regions.append(dict(link=link.name,geom=g.idx,normal=normal.tolist(),offset_m=offset,
                receives_compliance=bool(compliant[g.idx]),
                geom_local_pos=g.init_pos.tolist(),geom_local_quat_wxyz=g.init_quat.tolist(),
                link_frame_bounds=[verts.min(axis=0).tolist(),verts.max(axis=0).tolist()]))
        assert len(self.regions)==4
        self.region.from_numpy(fields);self.enabled.from_numpy(enabled)
        self.compliant.from_numpy(compliant)
        self.link_index.from_numpy(links)
        self.calls=0;self.original=self.collider.detection
        def detection(*args,**kwargs):
            result=self.original(*args,**kwargs);self.apply();self.calls+=1;return result
        self.collider.detection=detection

    @qd.kernel
    def apply(self):
        for j,b in qd.ndrange(3,self.solver._B):self.counts[j,b]=0
        for g,b in self.activation_by_geom:self.activation_by_geom[g,b]=0
        for b in range(self.solver._B):
            for i in range(self.contacts.n_contacts[b]):
                params=self.contacts.contact_data.sol_params[i,b]
                best_tc=params[0];classified=0
                ga=self.contacts.contact_data.geom_a[i,b];gb=self.contacts.contact_data.geom_b[i,b]
                cn=self.contacts.contact_data.normal[i,b];cp=self.contacts.contact_data.pos[i,b]
                penetration=self.contacts.contact_data.penetration[i,b]
                for side in qd.static(range(2)):
                    g=ga if side==0 else gb;other=gb if side==0 else ga
                    if self.enabled[g]:
                        outward=-cn if side==0 else cn
                        surface=cp+outward*penetration*.5
                        link=self.link_index[g]
                        local=gu.qd_inv_transform_by_trans_quat(surface,self.state.links.pos[link,b],self.state.links.quat[link,b])
                        local_n=gu.qd_inv_transform_by_quat(outward,self.state.links.quat[link,b])
                        region=self.region[g];normal=qd.Vector([region[0],region[1],region[2]])
                        cosine=normal.dot(local_n)
                        thickness=(normal.dot(local)-region[3])/qd.max(cosine,1e-6)
                        if local[0]<-.008 and cosine>.5 and thickness>0:
                            classified=1
                            if qd.static(self.pad_tc!=.02):
                                if self.compliant[g]:
                                    base=params[0];soft=(self.pad_tc+self.info.geoms.sol_params[other][0])*.5
                                    p=qd.max(penetration,1e-9);soft_ratio=(base/soft)**2
                                    ratio=(soft_ratio*qd.min(p,thickness)+qd.max(p-thickness,0.))/p
                                    best_tc=qd.max(best_tc,base/qd.sqrt(ratio))
                                    if base/qd.sqrt(ratio)>base:
                                        qd.atomic_add(self.activation_by_geom[g,b],1)
                                    if p>thickness:qd.atomic_add(self.counts[2,b],1)
                if classified:
                    qd.atomic_add(self.counts[0,b],1)
                    if best_tc>params[0]:qd.atomic_add(self.counts[1,b],1)
                    self.contacts.contact_data.sol_params[i,b][0]=best_tc

    def audit(self):
        return dict(pad_timeconst_s=self.pad_tc,compliant_part=self.compliant_part,regions=self.regions,
            geometry='Original candidate meshes, same normal/position material classification as pinned surface_pad_candidate.py.',
            law='Same layered secant normal timeconstant law; no friction coefficient or geometry changes. Engine cone and impratio are separate declared treatments.',
            qualification='Uncalibrated rigid-contact approximation, not measured rubber deformation.')
