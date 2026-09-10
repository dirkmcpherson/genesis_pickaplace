"""Finite pad contact-area spin resistance, without changing sliding friction or source motion."""
from pathlib import Path
import argparse,json,runpy,sys
import numpy as np
import quadrants as qd
import genesis as gs
from genesis.utils import geom as gu
from genesis.utils.misc import qd_to_numpy
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'baselines'))
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
p.add_argument('--preset',type=Path,required=True)
p.add_argument('--spin-length',type=float,choices=[0.,.002],required=True)
a=p.parse_args()
import genesis_can_env
build=genesis_can_env.build_world
def world(*args,**kwargs):
    opts=dict(kwargs['rigid_extra']);opts['enable_torsional_friction']=True
    kwargs['rigid_extra']=opts;w=build(*args,**kwargs)
    solver=w['scene'].sim.rigid_solver
    solver.set_geoms_friction_torsional(np.zeros(solver.n_geoms))
    np.testing.assert_array_equal(qd_to_numpy(solver.dyn_info.geoms.friction_torsional),np.zeros(solver.n_geoms))
    return w
genesis_can_env.build_world=world
import surface_pad_candidate_g14 as material
Base=material.SurfacePads;instance=None;rows=[]
@qd.data_oriented
class SpinPads(Base):
    def __init__(self,*args,**kwargs):
        global instance
        self.spin_length=a.spin_length
        super().__init__(*args,**kwargs)
        cfg=self.solver.rigid_config
        assert cfg.enable_elliptic_friction and cfg.rows_per_contact==4
        assert cfg.enable_torsional_friction and not cfg.enable_rolling_friction
        assert not cfg.enable_per_island_solve
        self.cd=self.contacts.contact_data;self.cs=self.solver.constraint_solver.constraint_state
        self.spin_counts=qd.Vector.field(4,dtype=qd.i32,shape=())
        self.selected=qd.field(dtype=qd.i32,shape=self.cd.geom_a.shape[0])
        detect=self.collider.detection
        def detection(*args,**kwargs):
            result=detect(*args,**kwargs);self.assign_spin()
            if self.spin_counts[None][3]:raise RuntimeError('Unexpected pre-existing spin friction')
            return result
        self.collider.detection=detection
        self.force_calls=0;constraint=self.solver._func_constraint_force
        def observe(*args,**kwargs):
            result=constraint(*args,**kwargs);self.force_calls+=1
            if self.force_calls%24==8:
                n=int(qd_to_numpy(self.contacts.n_contacts)[0])
                assert int(qd_to_numpy(self.cs.n_constraints_cone)[0])==4*n
                offset=int(qd_to_numpy(self.cs.n_constraints_equality)[0]+qd_to_numpy(self.cs.n_constraints_frictionloss)[0])
                order=qd_to_numpy(self.contacts.contact_sort_idx)[:n,0]
                selected=self.selected.to_numpy()[:n]
                mu=qd_to_numpy(self.cd.friction_torsional)[:n,0]
                np.testing.assert_allclose(mu,selected*self.spin_length,rtol=1e-6,atol=0)
                force=qd_to_numpy(self.cs.efc_force)[:,0]
                normal=qd_to_numpy(self.cd.normal)[:n,0]
                la=qd_to_numpy(self.cd.link_a)[:n,0];lb=qd_to_numpy(self.cd.link_b)[:n,0]
                ang=qd_to_numpy(self.state.links.cd_ang)[:,0]
                ga=qd_to_numpy(self.cd.geom_a)[:n,0];gb=qd_to_numpy(self.cd.geom_b)[:n,0]
                for k,i in enumerate(order):
                    if selected[i]:
                        f=force[offset+4*k:offset+4*k+4]
                        wa=ang[la[i]] if la[i]>=0 else np.zeros(3)
                        wb=ang[lb[i]] if lb[i]>=0 else np.zeros(3)
                        spin=float((wb-wa)@normal[i])
                        rows.append([(self.force_calls-8)*self.solver._substep_dt,int(ga[i]),int(gb[i]),float(mu[i]),*f,spin])
            return result
        self.solver._func_constraint_force=observe;instance=self
    @qd.kernel
    def assign_spin(self):
        for i in range(self.contacts.n_contacts[0]):
            qd.atomic_add(self.spin_counts[None][0],1)
            if self.cd.friction_torsional[i,0]!=0:qd.atomic_add(self.spin_counts[None][3],1)
            self.selected[i]=0
            ga,gb=self.cd.geom_a[i,0],self.cd.geom_b[i,0]
            cn,cp,pen=self.cd.normal[i,0],self.cd.pos[i,0],self.cd.penetration[i,0]
            for side in qd.static(range(2)):
                g=ga if side==0 else gb
                if self.enabled[g] and self.compliant[g]:
                    outward=-cn if side==0 else cn;surface=cp+outward*pen*.5
                    link=self.link_index[g]
                    local=gu.qd_inv_transform_by_trans_quat(surface,self.state.links.pos[link,0],self.state.links.quat[link,0])
                    local_n=gu.qd_inv_transform_by_quat(outward,self.state.links.quat[link,0])
                    r=self.region[g];normal=qd.Vector([r[0],r[1],r[2]])
                    cosine=normal.dot(local_n);thickness=(normal.dot(local)-r[3])/qd.max(cosine,1e-6)
                    if local[0]<-.008 and cosine>.5 and thickness>0:self.selected[i]=1
            if self.selected[i]:
                qd.atomic_add(self.spin_counts[None][1],1)
                if qd.static(self.spin_length>0):
                    self.cd.friction_torsional[i,0]=self.spin_length
                    qd.atomic_add(self.spin_counts[None][2],1)
    def audit(self):
        r=super().audit();r['finite_pad_spin']=dict(spin_length_m=self.spin_length,
            rows_per_contact=4,rolling=False,normal_damping_gain=1,
            scope='Same inner-pad material regions, every contacting object, all replay times. All geometry torsional coefficients explicitly zero; only selected contact rows receive spin friction.',
            physical_hypothesis='Unmeasured effective contact patch moment capacity tau <=0.002m*normal_force. A uniform circular patch with sliding mu1 and radius3mm has this integrated capacity; patch size is an assumption, not hardware calibration.',
            fixed='Existing sliding friction, layered normal softness, source commands, poses, geometry, mass, armature and scoring.',
            engine_options_override='Actual solver enable_torsional_friction=True; generic parent engine_options snapshot predates this build override. Runtime rows and flags asserted here.')
        return r
material.SurfacePads=SpinPads
sys.argv=['run_g14_hand.py',str(a.source),'--out',str(a.out),'--preset',str(a.preset),'--cone','elliptic','--impratio','10']
try:runpy.run_path(str(R/'can_pos_recovery/run_g14_hand.py'),run_name='__main__')
finally:
    if instance is not None:
        (a.out/'pad_spin_audit.json').write_text(json.dumps(dict(spin_length_m=a.spin_length,force_calls=instance.force_calls,
            all_selected_changed_unexpected_counts=instance.spin_counts.to_numpy().tolist(),actual_rows_per_contact=instance.solver.rigid_config.rows_per_contact),indent=2))
        np.savez_compressed(a.out/'pad_spin_contacts.npz',contacts=np.asarray(rows).reshape(-1,9),columns=np.array(['time_s','geom_a','geom_b','spin_length_m','normal_N','tangent1_N','tangent2_N','spin_torque_Nm','relative_spin_rad_s']))
