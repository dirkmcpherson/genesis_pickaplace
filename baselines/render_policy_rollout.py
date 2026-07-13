"""Render a trained DP rollout to mp4 -- the visual sanity-check that eval numbers are real.
Runs the policy closed-loop in the camera env and writes baselines/policy_videos/<uid>_<outcome>.mp4.
Usage: render_policy_rollout.py <checkpoint> <uid> [uid...]
"""
import sys, pathlib as pl, numpy as np, torch, cv2
REPO = pl.Path('/home/james/workspace/genesis_pickaplace')
sys.path.insert(0, str(REPO/'baselines'))
from genesis_can_env import GenesisCanEnv
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.policies.factory import make_pre_post_processors
CK = sys.argv[1]; OUT = REPO/'baselines/policy_videos'; OUT.mkdir(exist_ok=True)
TASK = 'pick the can and slide it against the can on the shelf'
pol = DiffusionPolicy.from_pretrained(CK); pol.eval()
dev = 'cuda' if torch.cuda.is_available() else 'cpu'; pol.to(dev)
pre, post = make_pre_post_processors(policy_cfg=pol.config, pretrained_path=CK)
PROPRIO = pol.config.input_features['observation.state'].shape[0]  # v1-v3: 7, v4+: 8
ENVDIM = 17  # current env obs width
# 16-dim (pre grip-effort) checkpoints: drop grip-effort at index 7 from the env's obs
DROP_GRIP = (PROPRIO == 7)
env = GenesisCanEnv(backend='cpu', render_size=(480, 640))
def obsvec(o):
    s = o['state']
    return np.delete(s, 7) if DROP_GRIP else s
for uid in [int(u) for u in sys.argv[2:]]:
    obs = env.reset(uid=uid); pol.reset(); frames=[]; done=False; info={}
    while not done:
        sv = obsvec(obs)
        b={'observation.state': torch.from_numpy(sv[:PROPRIO]).float().unsqueeze(0),
           'observation.environment_state': torch.from_numpy(sv[PROPRIO:]).float().unsqueeze(0),
           'task':[TASK]}
        b=pre(b); b={k:(v.to(dev) if torch.is_tensor(v) else v) for k,v in b.items()}
        with torch.no_grad(): a=post(pol.select_action(b)).squeeze(0).cpu().numpy()
        obs,done,info=env.step(a)
        frames.append(np.asarray(env.w['cam'].render()[0])[:,:,::-1])
    oc=('S-nested' if info.get('nested') else 'S' if info['contact'] else
        'placed' if info['placed'] else 'picked' if info['picked'] else 'fail')
    f=OUT/f'{uid}_policy_{oc}.mp4'
    vw=cv2.VideoWriter(str(f),cv2.VideoWriter_fourcc(*'mp4v'),20,(frames[0].shape[1],frames[0].shape[0]))
    for fr in frames: vw.write(fr.astype(np.uint8))
    vw.release(); print(f'{uid}: {oc} ({len(frames)}f) -> {f.name}', flush=True)
