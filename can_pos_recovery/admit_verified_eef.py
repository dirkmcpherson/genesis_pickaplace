"""Package visually approved census traces only after exact independent replay."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path)
    p.add_argument('--bank',type=Path,default=Path('baselines/demos_eef_recovery_2026-09-09'))
    a=p.parse_args()
    manifest_path=a.bank/'manifest.json';manifest=json.loads(manifest_path.read_text())
    assert manifest['reconstruction_class']=='unmodified_commands'
    reviews=json.loads((a.root/'visual_review.json').read_text())
    admitted=[];pending=[]
    for review in reviews:
        if not review['approved']:continue
        uid=review['uid'];folder=a.root/str(uid)
        source=folder/'collection'/f'{uid}_eef_delta.npz'
        verified=folder/'verification'/source.name
        if not verified.with_suffix('.json').exists():pending.append(uid);continue
        meta=json.loads(verified.with_suffix('.json').read_text())
        assert meta['sequence']['complete'] and meta['action_replay_verification'] and meta['precision_ik']
        assert 'physics_treatment' not in meta and meta['extension_m']==0
        assert not meta.get('grip_transform') and not meta.get('terminal_hold_frames')
        original=json.loads((folder/'source.json').read_text())
        assert not original.get('timing_reconstruction') and not meta.get('timing_reconstruction')
        assert meta['can_pos']==original['can_pos'] and meta['can_quat']==original['can_quat']
        with np.load(source) as collected,np.load(verified) as replay:
            for key in ('trajectory','actions_eef','observations'):
                assert np.array_equal(collected[key],replay[key]),(uid,key)
            assert replay['observations'].dtype==np.float32
            assert replay['observations'].shape==(len(replay['actions_eef'])+1,17)
        video=Path(review['video']);sheet=Path(review['contact_sheet'])
        assert video.exists() and sheet.exists()
        existing=[r for r in manifest['records'] if r['uid']==uid and r['role']=='primary']
        if existing:
            assert len(existing)==1 and existing[0]['sha256']==digest(verified)
            continue
        target=a.bank/f'{uid}_primary.npz'
        if target.exists():assert digest(target)==digest(verified)
        else:shutil.copyfile(verified,target)
        record=dict(uid=uid,role='primary',reconstruction_class='unmodified_commands',file=target.name,sha256=digest(target),frames=meta['frames'],
                    can_pos=meta['can_pos'],can_quat=meta['can_quat'],can_offset_from_archived_xy=[0,0],
                    precision_ik=True,added_slide_translation_m=0.,added_frames=0,sequence=meta['sequence'],
                    generation=str(source.with_suffix('.json').resolve()),verification=str(verified.with_suffix('.json').resolve()),
                    video=str(video.resolve()),visual_review=review['note'],visual_contact_sheet=str(sheet.resolve()),
                    source_sha256=meta['source_sha256'])
        manifest['records'].append(record);admitted.append(uid)
    manifest['independent_source_trials']=len({r['uid'] for r in manifest['records']})
    manifest['primary_count']=sum(r['role']=='primary' for r in manifest['records'])
    manifest['alternative_count']=sum(r['role']!='primary' for r in manifest['records'])
    temp=manifest_path.with_suffix('.json.tmp');temp.write_text(json.dumps(manifest,indent=2));temp.replace(manifest_path)
    print(json.dumps(dict(admitted=admitted,pending_verification=pending,independent_source_trials=manifest['independent_source_trials'])))


if __name__=='__main__':main()
