"""Localized layered contact on original geometry, without extra contact pairs.

An exploratory contact law: soft response over a bounded compression travel, then the
original stiffness increment beyond the backing plane. This is not calibrated
material deformation. All objects contacting a designated pad receive the law.
"""
import numpy as np
import taichi as ti
import genesis as gs
from genesis.utils import geom as gu

@ti.data_oriented
class FiniteCompressionPads:
    def __init__(self,world,pad_timeconst=.03,depth=.003,compression_limit=.0005):
        assert 0<=compression_limit<=depth
        self.compression_limit=float(compression_limit)
        assert .02<=pad_timeconst<=.04 and 0<depth<=.005
        self.solver=world['scene'].sim.rigid_solver;self.collider=self.solver.collider
        n=self.solver.n_geoms
        self.region=ti.Vector.field(4,dtype=gs.ti_float,shape=n)
        self.enabled=ti.field(dtype=ti.i32,shape=n)
        self.counts=ti.field(dtype=ti.i32,shape=3)
        self.pad_tc=float(pad_timeconst)
        fields=np.zeros((n,4));enabled=np.zeros(n,dtype=np.int32);self.regions=[]
        for link in world['kinova'].links:
            if 'finger' not in link.name:continue
            assert len(link.geoms)==1
            g=link.geoms[0];normal=np.array([.42 if 'prox' in link.name else .1,1 if 'right' in link.name else -1,0.])
            normal/=np.linalg.norm(normal);offset=float((g._init_verts@normal).max()-depth)
            assert np.allclose(g._init_pos,0) and np.allclose(g._init_quat,[1,0,0,0])
            fields[g.idx]=np.r_[normal,offset];enabled[g.idx]=1
            self.regions.append(dict(link=link.name,geom=g.idx,normal=normal.tolist(),offset_m=offset))
        assert len(self.regions)==4
        self.region.from_numpy(fields);self.enabled.from_numpy(enabled)
        self.calls=0;self.original=self.collider.detection
        def detection():
            result=self.original()
            self.apply()
            self.calls+=1
            return result
        self.collider.detection=detection

    @ti.kernel
    def apply(self):
        for j in range(3):self.counts[j]=0
        for b in range(self.solver._B):
            for i in range(self.collider.n_contacts[b]):
                c=self.collider.contact_data[i,b]
                best_tc=c.sol_params[0]
                classified=0
                for side in ti.static(range(2)):
                    g=c.geom_a if side==0 else c.geom_b
                    other=c.geom_b if side==0 else c.geom_a
                    if self.enabled[g]:
                        outward=-c.normal if side==0 else c.normal
                        # Contact point lies midway through overlap: recover the
                        # finger surface point before classifying its material.
                        surface=c.pos+outward*c.penetration*.5
                        pose=self.solver.geoms_state[g,b]
                        local=gu.ti_inv_transform_by_trans_quat(surface,pose.pos,pose.quat)
                        local_n=gu.ti_inv_transform_by_quat(outward,pose.quat)
                        region=self.region[g];normal=ti.Vector([region[0],region[1],region[2]])
                        cosine=normal.dot(local_n)
                        thickness=(normal.dot(local)-region[3])/ti.max(cosine,1e-6)
                        if local[0]<-.008 and cosine>.5 and thickness>0:
                            classified=1
                            if ti.static(self.pad_tc!=.02):
                                base=c.sol_params[0]
                                soft=(self.pad_tc+self.solver.geoms_info[other].sol_params[0])*.5
                                thickness=ti.min(thickness,self.compression_limit)
                                p=ti.max(c.penetration,1e-9)
                                soft_ratio=(base/soft)**2
                                # F(p) = k_soft*min(p,h)+k_base*max(p-h,0).
                                # Match that restoring term via its secant stiffness.
                                ratio=(soft_ratio*ti.min(p,thickness)+ti.max(p-thickness,0.))/p
                                best_tc=ti.max(best_tc,base/ti.sqrt(ratio))
                                if p>thickness:ti.atomic_add(self.counts[2],1)
                if classified:
                    ti.atomic_add(self.counts[0],1)
                    if best_tc>c.sol_params[0]:ti.atomic_add(self.counts[1],1)
                    self.collider.contact_data[i,b].sol_params[0]=best_tc

    def audit(self):
        return dict(pad_timeconst_s=self.pad_tc,soft_compression_limit_m=self.compression_limit,regions=self.regions,
            geometry='Original reference URDF collision meshes; no subdivision or reconstruction',
            law='Soft restoring stiffness until the lesser of local layer thickness and declared compression limit; original incremental stiffness thereafter. Material region unchanged. Damping follows effective time constant; friction unchanged.',
            qualification='Uncalibrated layered contact approximation. Material classification uses local surface position/normal, not task phase, can identity or goal distance.')
