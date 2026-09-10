"""Observe solved contact wrenches during unchanged full-source replays."""
from pathlib import Path
import argparse,json,runpy,sys
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'can_pos_recovery'))
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
p.add_argument('--preset',type=Path,required=True);p.add_argument('--windows',type=float,nargs='+',required=True)
a=p.parse_args();assert len(a.windows)%2==0;windows=np.array(a.windows).reshape(-1,2)
from genesis.utils.misc import qd_to_numpy
import surface_pad_candidate_g14 as module
Original=module.SurfacePads;contacts=[];states=[];observer=None
class ObservedPads(Original):
    def __init__(self,world,*args,**kwargs):
        global observer
        super().__init__(world,*args,**kwargs);observer=self
        self.observer_calls=0
        self.geom_labels={g.idx:link.name for link in world['kinova'].links for g in link.geoms}
        for name in ['bottle','goal','shelf']:
            if name in world:
                for link in world[name].links:
                    for g in link.geoms:self.geom_labels[g.idx]=name
        can=world['bottle'];can_geoms={g.idx for link in can.links for g in link.geoms};can_link=can.base_link_idx
        self.can_mass=float(can.get_mass());original=self.solver._func_constraint_force
        def observed(*args,**kwargs):
            result=original(*args,**kwargs);self.observer_calls+=1
            t=(self.observer_calls-8)*self.solver._substep_dt
            if not any(lo<=t<=hi for lo,hi in windows):return result
            c=self.contacts.contact_data;n=int(qd_to_numpy(self.contacts.n_contacts)[0])
            ga=qd_to_numpy(c.geom_a)[:n,0];gb=qd_to_numpy(c.geom_b)[:n,0]
            selected=[i for i in range(n) if int(ga[i]) in can_geoms or int(gb[i]) in can_geoms]
            ls=self.state.links
            com=qd_to_numpy(ls.root_COM)[:,0];vel=qd_to_numpy(ls.cd_vel)[:,0];ang=qd_to_numpy(ls.cd_ang)[:,0]
            center=com[can_link];total=np.zeros(3);moment=np.zeros(3);finger=np.zeros(3);other=np.zeros(3)
            if selected:
                la=qd_to_numpy(c.link_a)[:n,0];lb=qd_to_numpy(c.link_b)[:n,0]
                pos=qd_to_numpy(c.pos)[:n,0];normal=qd_to_numpy(c.normal)[:n,0]
                force=qd_to_numpy(c.force)[:n,0];pen=qd_to_numpy(c.penetration)[:n,0];mu=qd_to_numpy(c.friction)[:n,0]
                for i in selected:
                    va=vel[la[i]]+np.cross(ang[la[i]],pos[i]-com[la[i]])
                    vb=vel[lb[i]]+np.cross(ang[lb[i]],pos[i]-com[lb[i]])
                    rel=vb-va;vn=float(rel@normal[i]);tangent=rel-vn*normal[i]
                    fn=float(-force[i]@normal[i]);ft=force[i]+fn*normal[i]
                    oncan=force[i] if int(gb[i]) in can_geoms else -force[i]
                    othergeom=int(ga[i] if int(gb[i]) in can_geoms else gb[i]);label=self.geom_labels.get(othergeom,'other')
                    torque=np.cross(pos[i]-center,oncan);total+=oncan;moment+=torque
                    if 'finger' in label:finger+=oncan
                    else:other+=oncan
                    contacts.append([t,int(ga[i]),int(gb[i]),othergeom,float(pen[i]),float(mu[i]),fn,float(np.linalg.norm(ft)),float(np.linalg.norm(ft)/max(mu[i]*fn,1e-12)),vn,float(np.linalg.norm(tangent)),float(force[i]@rel),float((ang[lb[i]]-ang[la[i]])@normal[i]),*pos[i],*normal[i],*oncan,*torque])
            states.append([t,*center,*vel[can_link],*ang[can_link],*total,*moment,*finger,*other,len(selected)])
            return result
        self.solver._func_constraint_force=observed
module.SurfacePads=ObservedPads
sys.argv=['run_g14_hand.py',str(a.source),'--out',str(a.out),'--preset',str(a.preset),'--cone','elliptic','--impratio','10']
try:runpy.run_path(str(R/'can_pos_recovery/run_g14_hand.py'),run_name='__main__')
finally:
    if observer is not None:
        cc=['time_s','geom_a','geom_b','other_geom','penetration_m','mu','normal_N','tangent_N','elliptic_utilization','relative_normal_m_s','relative_tangent_m_s','force_dot_relative_velocity_W','relative_spin_rad_s','contact_x','contact_y','contact_z','normal_x','normal_y','normal_z','can_force_x','can_force_y','can_force_z','can_torque_x','can_torque_y','can_torque_z']
        sc=['time_s','can_com_x','can_com_y','can_com_z','can_vx','can_vy','can_vz','can_wx','can_wy','can_wz','contact_Fx','contact_Fy','contact_Fz','contact_Tx','contact_Ty','contact_Tz','finger_Fx','finger_Fy','finger_Fz','other_Fx','other_Fy','other_Fz','contacts']
        np.savez_compressed(a.out/'release_forces.npz',contacts=np.asarray(contacts).reshape(-1,len(cc)),states=np.asarray(states).reshape(-1,len(sc)),contact_columns=np.array(cc),state_columns=np.array(sc))
        (a.out/'release_forces_audit.json').write_text(json.dumps(dict(windows_s=windows.tolist(),substep_dt_s=observer.solver._substep_dt,observer_calls=observer.observer_calls,can_mass_kg=observer.can_mass,geom_labels=observer.geom_labels,
            qualification='Every substep in declared windows, after constraint solve and before integration. Velocities precede current impulse. Force acts on link_b; can-force sign chosen by geometry side. Net contact force and next-step linear momentum must agree before interpretation. Exact saved-reference replay required. No physical parameter or source change.'),indent=2))
