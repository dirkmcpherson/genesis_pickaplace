"""Prepare all declared early candidates from bag timestamps; preserve archived files."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
import extract_real_timed as reader

archive=Path('/home/james/wm_fix_2026-09-03/recover_early')
winners=json.loads((archive/'winners.json').read_text())
seeds=json.loads((archive/'seeds.json').read_text())
uids=sorted(map(int,seeds))
plan=dict(uids=uids,scope='89 early-day candidates; December 16 and 17 only',
          yaw_deg={'12-16':-9.7,'12-17':-19.2},
          ic='88 archived outcome-fitted winners, already world-rotated; 183 uses rotated seed',
          command='Bag-window measurements resampled at 30ms; no og4 or added push',
          world='Full pre and post hooks; shelf/goal/table remain fixed',
          provenance='New full-world test; old picked/contact/nested labels are not completion evidence',
          workers=2,winners_sha256=hashlib.sha256((archive/'winners.json').read_bytes()).hexdigest())
(ROOT/'plan.json').write_text(json.dumps(plan,indent=2))
for name in ('winners','seeds'):
    (ROOT/f'archived_{name}.json').write_text((archive/f'{name}.json').read_text())
timed=ROOT/'timed';timed.mkdir(exist_ok=True);reader.OUT=timed
for uid in uids:
    dst=ROOT/str(uid);dst.mkdir(exist_ok=False)
    s=seeds[str(uid)];angle=np.deg2rad(plan['yaw_deg'][s['day']]);rot=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    assert np.allclose(rot@np.asarray(s['can_xy_armframe']),s['can_xy'],atol=1e-12)
    d=reader.extract(uid);assert d is not None
    old=np.load(REPO/f'inthewild_trials/{uid}_episodes.npy',allow_pickle=True).item()
    exact=np.array_equal(d['q_frame'],old['vel_cmd']) and np.array_equal(d['g_frame'],np.asarray(old['gripper_pos'])[:,0])
    assert exact or uid==207,uid  # documented ROS1 arithmetic correction
    t=d['t_frame']-d['t_frame'][0];assert np.all(np.diff(t)>0)
    sample=np.arange(int(np.ceil(t[-1]/.03))+1)*.03
    q=d['q_frame'];assert np.max(np.abs(np.diff(q,axis=0)))<np.pi
    np.savez_compressed(dst/'source.npz',joint_waypoints=np.stack([np.interp(sample,t,q[:,j]) for j in range(6)],axis=1),
                        source_grip=np.interp(sample,t,d['g_frame']),sample_time_s=sample,original_frame_time_s=t)
    w=winners.get(str(uid));pos=w['can_pos'] if w else [*s['can_xy'],.113]
    meta=dict(uid=uid,day=s['day'],variant='gc_kp4_riser3_shelf6_yaw'+s['day'][-2:],variant_post_hook=True,
              source_kind='kinematic_tape',can_pos=pos,can_quat=[1,0,0,0],source_frames=len(sample),
              source_sha256=hashlib.sha256((timed/f'{uid}_timed.npz').read_bytes()).hexdigest(),
              initial_pose_provenance='archived pre-only outcome-fitted world position' if w else 'rotated unfitted seed',
              source_matches_archived_episode=exact,
              timing_reconstruction=dict(method='ROS1 window timestamps, linear measurement interpolation at action starts',
                  original_span_s=float(t[-1]),original_frames=len(t),replay_duration_s=len(sample)*.03,
                  source=str(timed/f'{uid}_timed.npz'),decision_dt_s=.03))
    (dst/'source.json').write_text(json.dumps(meta,indent=2));print(uid,flush=True)
(ROOT/'preparation_complete.json').write_text(json.dumps(dict(uids=uids),indent=2))
