"""Read-only contact-location observer around the unchanged candidate replay."""
from pathlib import Path
import argparse,json,runpy,sys
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'can_pos_recovery'))
p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True);p.add_argument('--preset',type=Path,required=True);a=p.parse_args()
from genesis.utils.misc import qd_to_numpy
from genesis.utils.geom import quat_to_R
import surface_pad_candidate_g14 as module
Original=module.SurfacePads;rows=[];samples=[]
class ObservedPads(Original):
    def __init__(self,world,*args,**kwargs):
        super().__init__(world,*args,**kwargs)
        mapping={r['geom']:r for r in self.regions}
        can_geoms={g.idx for link in world['bottle'].links for g in link.geoms}
        detection=self.collider.detection
        def observed(*args,**kwargs):
            result=detection(*args,**kwargs)
            t=(self.calls-8)*.00125
            if self.calls%8 or not 7<=t<=21:return result
            c=self.contacts.contact_data;n=int(qd_to_numpy(self.contacts.n_contacts)[0])
            ga=qd_to_numpy(c.geom_a)[:n,0];gb=qd_to_numpy(c.geom_b)[:n,0]
            indices=[i for i in range(n) if (int(ga[i]) in mapping and int(gb[i]) in can_geoms) or (int(gb[i]) in mapping and int(ga[i]) in can_geoms)]
            samples.append(dict(time_s=t,can_finger_contacts=len(indices)))
            if not indices:return result
            pos=qd_to_numpy(c.pos)[:n,0];normal=qd_to_numpy(c.normal)[:n,0];pen=qd_to_numpy(c.penetration)[:n,0]
            lp=qd_to_numpy(self.state.links.pos)[:,0];lq=qd_to_numpy(self.state.links.quat)[:,0]
            linkids=self.link_index.to_numpy()
            for i in indices:
                first=int(ga[i]) in mapping;g=int(ga[i] if first else gb[i]);r=mapping[g];link=linkids[g]
                outward=-normal[i] if first else normal[i];surface=pos[i]+outward*pen[i]*.5
                rotation=quat_to_R(lq[link]);local=(surface-lp[link])@rotation;localnormal=outward@rotation
                cosine=float(np.array(r['normal'])@localnormal)
                thickness=float((np.array(r['normal'])@local-r['offset_m'])/max(cosine,1e-6))
                rows.append(dict(time_s=t,link=r['link'],geom=g,local_surface_m=local.tolist(),local_outward_normal=localnormal.tolist(),penetration_m=float(pen[i]),normal_cosine=cosine,layer_thickness_along_normal_m=thickness,within_x_region=bool(local[0]<-.008),within_normal_region=bool(cosine>.5),within_depth_region=bool(thickness>0),classified=bool(local[0]<-.008 and cosine>.5 and thickness>0)))
            return result
        self.collider.detection=observed
module.SurfacePads=ObservedPads
sys.argv=['run_g14_hand.py',str(a.source),'--out',str(a.out),'--preset',str(a.preset),'--cone','elliptic','--impratio','10']
try:runpy.run_path(str(R/'can_pos_recovery/run_g14_hand.py'),run_name='__main__')
finally:
    if a.out.exists():(a.out/'contact_locations.json').write_text(json.dumps(dict(samples=samples,records=rows,qualification='Read-only observation after collision detection and pad assignment, at each original10ms scene-step final substep during7–21s. Before current constraint-force solve. Includes every detected can-finger contact; no force threshold. Not every physics substep, and not evidence all detected contacts carry load.'),indent=2))
