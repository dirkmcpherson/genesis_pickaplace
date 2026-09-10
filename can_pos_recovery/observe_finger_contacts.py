"""Read-only contact snapshots during an otherwise identical full hand replay.

Samples after constraint resolution and before integration at the last substep
of each scene step. Velocities therefore precede the current impulse. The
four-ray solver uses a pyramidal friction bound; L1 utilization is reported.
"""
from pathlib import Path
import argparse, json, runpy, sys
import numpy as np
import genesis as gs
import taichi as ti
from genesis.utils import geom as gu
import adaptive_gripper_candidate as adaptive

REPO=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(add_help=False)
p.add_argument('--aligned-inertia',action='store_true')
p.add_argument('--contact-window',type=float,nargs=2,default=[0,float('inf')])
p.add_argument('--every-substep',action='store_true')
args,rest=p.parse_known_args()
out=Path(rest[rest.index('--out')+1])
original_install=adaptive.install
observer=None

@ti.data_oriented
class Contacts:
    def __init__(self, world):
        self.solver=world['scene'].sim.rigid_solver
        self.collider=self.solver.collider
        self.kinds=ti.field(ti.i32,shape=self.solver.n_geoms)
        labels=np.zeros(self.solver.n_geoms,dtype=np.int32)
        for link in world['kinova'].links:
            if 'finger' in link.name:
                for g in link.geoms:labels[g.idx]=1
        labels[world['bottle'].geom_start:world['bottle'].geom_end]=2
        self.kinds.from_numpy(labels)
        self.buffer=ti.field(gs.ti_float,shape=(self.collider.contact_data.shape[0],25))
        self.rows=[];self.calls=0
        self.original=self.solver._func_constraint_force
        def observed():
            result=self.original()
            self.calls+=1
            t=(self.calls-8)*self.solver._substep_dt
            if args.contact_window[0]<=t<=args.contact_window[1] and (args.every_substep or self.calls%world['scene'].sim.substeps==0):
                self.sample()
                z=self.buffer.to_numpy();z=z[z[:,0]>=0]
                if len(z):self.rows.append(np.c_[np.full(len(z),self.calls),z])
            return result
        self.solver._func_constraint_force=observed

    @ti.kernel
    def sample(self):
        for i in range(self.buffer.shape[0]):
            self.buffer[i,0]=-1
            if i<self.collider.n_contacts[0]:
                c=self.collider.contact_data[i,0]
                ka,kb=self.kinds[c.geom_a],self.kinds[c.geom_b]
                if ka>0 or kb>0:
                    n=c.normal;f=c.force;d1,d2=gu.orthogonals(n)
                    fn=-f.dot(n);ft1=f.dot(d1);ft2=f.dot(d2)
                    va=self.solver._func_vel_at_point(c.pos,c.link_a,0)
                    vb=self.solver._func_vel_at_point(c.pos,c.link_b,0)
                    rel=vb-va;vn=rel.dot(n);vt=rel-vn*n
                    utilization=(ti.abs(ft1)+ti.abs(ft2))/ti.max(c.friction*fn,1e-12)
                    vals=ti.Vector([ti.cast(c.geom_a,gs.ti_float),ti.cast(c.geom_b,gs.ti_float),
                        ti.cast(ka,gs.ti_float),ti.cast(kb,gs.ti_float),c.penetration,c.friction,
                        fn,ft1,ft2,utilization,vn,vt.norm(),f.dot(rel),
                        c.pos[0],c.pos[1],c.pos[2],n[0],n[1],n[2],
                        rel[0],rel[1],rel[2],f[0],f[1],f[2]])
                    for j in ti.static(range(25)):self.buffer[i,j]=vals[j]

    def save(self):
        columns=['substep','geom_a','geom_b','kind_a','kind_b','penetration_m','friction',
                 'normal_force_N','tangent1_force_N','tangent2_force_N','pyramid_utilization',
                 'relative_normal_velocity_m_s','relative_tangent_speed_m_s','force_dot_relative_velocity_W',
                 'contact_x','contact_y','contact_z','normal_x','normal_y','normal_z',
                 'relative_vx','relative_vy','relative_vz','force_x','force_y','force_z']
        np.savez_compressed(out/'contact_dynamics.npz',values=np.concatenate(self.rows) if self.rows else np.empty((0,26)),columns=np.array(columns))
        (out/'contact_dynamics_audit.json').write_text(json.dumps(dict(substep_calls=self.calls,
            sampling='Snapshots after constraint solve, before integration. Relative velocities precede current force integration.',
            every_substep=args.every_substep,contact_window_s=args.contact_window,
            kinds={'0':'other geometry','1':'finger geometry','2':'manipulated can'},
            force_convention='force acts on link_b; normal force = -force dot collision normal',
            utilization='(abs(F dot d1)+abs(F dot d2))/(mu*Fn), using the pinned solver orthogonals; ignore near-zero Fn when interpreting ratios.',
            qualification='Observer only. Must reproduce all saved reference arrays before diagnostic use. No physical coefficient measurement; contact rows are dependent samples.'),indent=2))

def install(world,*a,**kw):
    global observer
    result=original_install(world,*a,**kw)
    observer=Contacts(world)
    return result

adaptive.install=install
entry='run_aligned_finger_inertia.py' if args.aligned_inertia else 'run_hand_preset.py'
sys.argv=[entry]+rest
try:runpy.run_path(str(REPO/'can_pos_recovery'/entry),run_name='__main__')
finally:
    if observer is not None:observer.save()
