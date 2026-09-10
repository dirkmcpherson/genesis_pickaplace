"""Normal damping with cancellation-aware input-scaled audit; physical update unchanged."""
from pathlib import Path
import argparse,json,runpy
import quadrants as qd
import genesis as gs
from genesis.utils import geom as gu
import surface_pad_candidate_g14 as material

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
p.add_argument('--preset',type=Path,required=True);p.add_argument('--gain',type=float,choices=[1.,2.],required=True)
p.add_argument('--verify-actions',action='store_true')
a=p.parse_args();Base=material.SurfacePads;instance=None

@qd.data_oriented
class NormalPads(Base):
    def __init__(self,*args,**kwargs):
        global instance
        self.gain=a.gain
        super().__init__(*args,**kwargs)
        self.cd=self.contacts.contact_data
        self.cs=self.solver.constraint_solver.constraint_state
        cfg=self.solver.rigid_config
        assert cfg.enable_elliptic_friction and cfg.rows_per_contact==3
        assert not cfg.enable_per_island_solve and not self.solver._use_hibernation
        assert not cfg.batch_links_info and not cfg.enable_torsional_friction and not cfg.enable_rolling_friction
        self.before=qd.field(dtype=gs.qd_float,shape=self.cs.aref.shape[0])
        self.changed=qd.field(dtype=qd.i32,shape=self.cs.aref.shape[0])
        self.before_params=qd.Vector.field(7,dtype=gs.qd_float,shape=self.cd.sol_params.shape[0])
        self.before_mu=qd.field(dtype=gs.qd_float,shape=self.cd.sol_params.shape[0])
        self.errors=qd.Vector.field(3,dtype=gs.qd_float,shape=())
        self.legacy_errors=qd.Vector.field(2,dtype=gs.qd_float,shape=())
        self.legacy_exceedances=qd.field(dtype=qd.i32,shape=())
        self.backward_errors=qd.Vector.field(3,dtype=gs.qd_float,shape=())
        self.jacobian_error=qd.field(dtype=gs.qd_float,shape=())
        self.max_cancellation_scale=qd.field(dtype=gs.qd_float,shape=())
        self.counts_check=qd.Vector.field(4,dtype=qd.i32,shape=())
        self.resolve_calls=0
        original=self.solver.constraint_solver.resolve
        def resolve(*args,**kwargs):
            self.snapshot();self.adjust();self.verify()
            self.resolve_calls+=1
            # Refuse to solve with a mismapped row or an unintended mutation.
            e=self.errors.to_numpy();n=self.counts_check.to_numpy()
            if max(self.backward_errors.to_numpy())>1e-5 or self.jacobian_error[None]>1e-6 or n[3]:
                raise RuntimeError(f'Normal damping row verification failed: errors={e}, counts={n}')
            return original(*args,**kwargs)
        self.solver.constraint_solver.resolve=resolve
        instance=self

    @qd.kernel
    def snapshot(self):
        for i in range(self.cs.n_constraints[0]):
            self.before[i]=self.cs.aref[i,0];self.changed[i]=0
        for i in range(self.contacts.n_contacts[0]):
            self.before_params[i]=self.cd.sol_params[i,0]
            self.before_mu[i]=self.cd.friction[i,0]

    @qd.kernel
    def adjust(self):
        offset=self.cs.n_constraints_equality[0]+self.cs.n_constraints_frictionloss[0]
        if self.cs.n_constraints_cone[0]!=3*self.contacts.n_contacts[0]:
            qd.atomic_add(self.counts_check[None][3],1)
        for order in range(self.contacts.n_contacts[0]):
            i=self.contacts.contact_sort_idx[order,0]
            ga,gb=self.cd.geom_a[i,0],self.cd.geom_b[i,0]
            cn,cp,pen=self.cd.normal[i,0],self.cd.pos[i,0],self.cd.penetration[i,0]
            selected=0
            for side in qd.static(range(2)):
                g=ga if side==0 else gb
                if self.enabled[g] and self.compliant[g]:
                    outward=-cn if side==0 else cn;surface=cp+outward*pen*.5
                    link=self.link_index[g]
                    local=gu.qd_inv_transform_by_trans_quat(surface,self.state.links.pos[link,0],self.state.links.quat[link,0])
                    local_n=gu.qd_inv_transform_by_quat(outward,self.state.links.quat[link,0])
                    r=self.region[g];normal=qd.Vector([r[0],r[1],r[2]])
                    cosine=normal.dot(local_n)
                    thickness=(normal.dot(local)-r[3])/qd.max(cosine,1e-6)
                    if local[0]<-.008 and cosine>.5 and thickness>0:selected=1
            if selected:
                row=offset+3*order
                params=self.cd.sol_params[i,0]
                for direction in qd.static(range(3)):
                    idx=row+direction;vel=gs.qd_float(0.)
                    for dof in range(self.solver.n_dofs):
                        vel+=self.cs.jac[idx,dof,0]*self.state.dofs.vel[dof,0]
                    position=-pen if direction==0 else 0.
                    _,expected=gu.imp_aref(params,-pen,vel,position)
                    _,elastic=gu.imp_aref(params,-pen,0.,position)
                    scale=qd.max(1.,qd.abs(elastic),qd.abs(expected),qd.abs(self.before[idx]))
                    legacy_error=qd.abs(self.before[idx]-expected)/scale
                    qd.atomic_max(self.legacy_errors[None][0 if direction==0 else 1],legacy_error)
                    if legacy_error>1e-5:qd.atomic_add(self.legacy_exceedances[None],1)
                    # Reconstruct the engine's A-then-B, child-to-parent, descending
                    # DOF accumulation. J@qvel above combines shared ancestors and
                    # sums ascending, so float32 cancellation can differ. Preserve
                    # that original velocity for the physical normal update below.
                    d1,d2=gu.qd_orthogonals(cn)
                    n=-cn
                    if direction==1:n=d1
                    if direction==2:n=d2
                    native_vel=gs.qd_float(0.)
                    sum_abs=gs.qd_float(0.)
                    native_jac=qd.Vector.zero(gs.qd_float,self.solver.n_dofs)
                    for side in range(2):
                        sign=gs.qd_float(-1.)
                        link=self.cd.link_a[i,0]
                        if side==1:
                            sign=gs.qd_float(1.)
                            link=self.cd.link_b[i,0]
                        while link>-1:
                            for dd in range(self.info.links.n_dofs[link]):
                                dof=self.info.links.dof_end[link]-1-dd
                                pos=cp-self.state.links.root_COM[link,0]
                                _,motion=gu.qd_transform_motion_by_trans_quat(
                                    self.state.dofs.cdof_ang[dof,0],self.state.dofs.cdof_vel[dof,0],pos,gu.qd_identity_quat())
                                jac=(sign*motion)@n
                                native_vel=native_vel+jac*self.state.dofs.vel[dof,0]
                                sum_abs+=qd.abs(jac*self.state.dofs.vel[dof,0])
                                native_jac[dof]+=jac
                            link=self.info.links.parent_idx[link]
                    for dof in range(self.solver.n_dofs):
                        jd=qd.abs(native_jac[dof]-self.cs.jac[idx,dof,0])/qd.max(1.,qd.abs(native_jac[dof]),qd.abs(self.cs.jac[idx,dof,0]))
                        qd.atomic_max(self.jacobian_error[None],jd)
                    _,native_expected=gu.imp_aref(params,-pen,native_vel,position)
                    error=qd.abs(self.before[idx]-native_expected)/qd.max(1.,qd.abs(elastic),qd.abs(native_expected),qd.abs(self.before[idx]))
                    # Scale by the magnitudes entering the sum, not its possibly
                    # cancelled result. Same1e-5 backward-error tolerance. A wrong
                    # Jacobian entry is checked independently above at1e-6.
                    input_scale=qd.max(scale,2./(params[3]*params[0])*sum_abs)
                    qd.atomic_max(self.max_cancellation_scale[None],input_scale/scale)
                    back_error=qd.abs(self.before[idx]-native_expected)/input_scale
                    qd.atomic_max(self.backward_errors[None][0 if direction==0 else 1],back_error)
                    if direction==0:
                        qd.atomic_max(self.errors[None][0],error)
                        qd.atomic_add(self.counts_check[None][0],1)
                        if qd.static(self.gain!=1.):
                            delta=-(self.gain-1.)*2./(params[3]*params[0])*vel
                            self.cs.aref[idx,0]=self.before[idx]+delta
                            self.changed[idx]=1
                            qd.atomic_add(self.counts_check[None][1],1)
                            target=elastic+self.gain*(expected-elastic)
                            qd.atomic_max(self.backward_errors[None][2],qd.abs(self.cs.aref[idx,0]-target)/qd.max(input_scale*self.gain,qd.abs(target)))
                            qd.atomic_max(self.errors[None][2],qd.abs(self.cs.aref[idx,0]-target)/qd.max(scale,qd.abs(target)))
                    else:
                        qd.atomic_max(self.errors[None][1],error)

    @qd.kernel
    def verify(self):
        for i in range(self.cs.n_constraints[0]):
            if not self.changed[i]:
                qd.atomic_add(self.counts_check[None][2],1)
                if self.cs.aref[i,0]!=self.before[i]:qd.atomic_add(self.counts_check[None][3],1)
        for i in range(self.contacts.n_contacts[0]):
            for j in qd.static(range(7)):
                if self.cd.sol_params[i,0][j]!=self.before_params[i][j]:qd.atomic_add(self.counts_check[None][3],1)
            if self.cd.friction[i,0]!=self.before_mu[i]:qd.atomic_add(self.counts_check[None][3],1)

    def audit(self):
        result=super().audit()
        result['normal_only_damping']=dict(gain=self.gain,
            law='Immediately before solve: selected normal aref -= (gain-1)*b*(normal Jacobian dot generalized velocity).',
            invariant='Normal elastic term, all tangent references, every unselected constraint reference, contact sol_params and friction coefficient unchanged by this hook.',
            scope='Existing inner-pad regions against every object throughout the whole replay. No source, geometry or initial-pose change.',
            qualification='Uncalibrated forward-only normal damping hypothesis; no autodiff/backward claim. Actual row mapping and unchanged references checked every substep.')
        return result

material.SurfacePads=NormalPads
import sys
sys.argv=['run_g14_hand.py',str(a.source),'--out',str(a.out),'--preset',str(a.preset),'--cone','elliptic','--impratio','10']
if a.verify_actions:sys.argv.append('--verify-actions')
try:runpy.run_path(str(ROOT/'can_pos_recovery/run_g14_hand.py'),run_name='__main__')
finally:
    if instance is not None:
        report=dict(gain=a.gain,resolve_calls=instance.resolve_calls,
            counts_selected_normal_changed_normal_checked_unchanged_violations=instance.counts_check.to_numpy().tolist(),
            max_normal_mapping_tangent_mapping_normal_target_relative_errors=instance.errors.to_numpy().tolist(),
            audit_version='native_order_backward_error_v3',
            max_input_scaled_normal_tangent_target_errors=instance.backward_errors.to_numpy().tolist(),
            max_jacobian_entry_relative_error=float(instance.jacobian_error[None]),
            max_input_to_output_scale_ratio=float(instance.max_cancellation_scale[None]),
            max_legacy_ascending_normal_tangent_mapping_errors=instance.legacy_errors.to_numpy().tolist(),
            legacy_mapping_exceedances=int(instance.legacy_exceedances[None]),
            qualification='Selected reference errors use native ancestor accumulation and sum-of-input-magnitudes scale at1e-5; native Jacobian entries compared independently at1e-6. Output-scaled and legacy errors retained. Physical update unchanged. Legacy ascending errors retained. Physical delta and normal target check unchanged from v1; all unselected references and contact parameters checked for exact nonmutation. Identity performs no writes.')
        (a.out/'normal_damping_audit.json').write_text(json.dumps(report,indent=2))
