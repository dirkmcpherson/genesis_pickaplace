"""Apply the audited mesh-consistent signs to an isolated candidate URDF.

Preserves published masses and all inertia/COM magnitudes. No geometry, joint,
arm dynamics or source URDF is changed. This is not a measured density model.
"""
from pathlib import Path
import json,hashlib,xml.etree.ElementTree as ET
import numpy as np
REPO=Path(__file__).resolve().parents[1]
AUDIT=REPO/'paper/eef_recovery_2026-09-09/localized_pads/finger_inertia_alignment/audit.json'
def align(path):
 path=Path(path).resolve()
 if path==REPO/'gen3_lite_2f_robotiq_85.urdf':raise ValueError('Only isolated candidate URDFs may be changed')
 report=json.loads(AUDIT.read_text());tree=ET.parse(path);root=tree.getroot();before=hashlib.sha256(path.read_bytes()).hexdigest();changes=[]
 for row in report['records']:
  inertial=root.find(f"link[@name='{row['link']}']/inertial");assert inertial is not None
  assert np.allclose(np.fromstring(inertial.find('origin').get('xyz'),sep=' '),row['published_com'],atol=1e-12)
  assert float(inertial.find('mass').get('value'))==row['mass_kg']
  inertial.find('origin').set('xyz',' '.join(format(v,'.12g') for v in row['proposed_com']))
  I=np.asarray(row['proposed_inertia'])
  for key,i,j in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]:
   old=float(inertial.find('inertia').get(key));assert abs(old)==abs(I[i,j])
   inertial.find('inertia').set(key,format(I[i,j],'.12g'))
  changes.append(dict(link=row['link'],before_com=row['published_com'],after_com=row['proposed_com'],before_inertia=row['published_inertia'],after_inertia=row['proposed_inertia']))
 tree.write(path,encoding='utf-8',xml_declaration=True)
 return dict(audit=str(AUDIT),audit_sha256=hashlib.sha256(AUDIT.read_bytes()).hexdigest(),before_sha256=before,after_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),changes=changes,qualification=report['qualification'])
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('urdf',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.write_text(json.dumps(align(a.urdf),indent=2))
