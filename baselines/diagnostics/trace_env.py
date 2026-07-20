import sys, numpy as np, pathlib as pl, torch
sys.path.insert(0, 'baselines'); sys.path.insert(0, 'can_pos_recovery')
from genesis_can_env import GenesisCanEnv, np_
from replay_harness import load_episode
UID = int(sys.argv[1])
env = GenesisCanEnv(backend='cpu')
vel, gp = load_episode(UID)
obs = env.reset(uid=UID)
rows=[]
for i in range(len(vel)-1):
    a = np.concatenate([vel[i], [np.clip(gp[i]/100.0,0,1)]]).astype(np.float32)
    obs, done, info = env.step(a)
    bp=np_(env.w['bottle'].get_pos()); ep=np_(env.w['eef'].get_pos())
    rows.append([bp[0],bp[1],bp[2],ep[0],ep[1],ep[2]])
np.save(sys.argv[2], np.array(rows))
print("env-path done", len(rows))
