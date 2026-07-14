"""Single Diffusion Policy loader -> a policy_action(obs) closure.

Extracted from eval_policy.py:35-67 (and dedups render_policy_rollout.py:13-32). lerobot
0.4.x moved (un)normalization OUT of the policy into processor pipelines, so we wrap
select_action with the saved pre/post processors -- otherwise the arm gets [-1,1]-scale
joint targets. PROPRIO is read back from the checkpoint's own config (v1-v3: 7, v4+: 8
incl grip effort), and the env's 17-dim obs is split state[:PROPRIO] -> observation.state,
state[PROPRIO:] -> observation.environment_state.
"""
import numpy as np
import torch

TASK = 'pick the can and slide it against the can on the shelf'


def load_dp_runner(checkpoint, image=False, device=None):
    """-> (policy_action, policy_reset, proprio).

    policy_action(obs) returns a PHYSICAL 7-vector action. policy_reset() clears the
    diffusion action-chunking queue and MUST be called at the start of each episode.
    """
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
    from lerobot.policies.factory import make_pre_post_processors

    device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
    policy = DiffusionPolicy.from_pretrained(checkpoint)
    policy.eval()
    policy.to(device)
    pre, post = make_pre_post_processors(policy_cfg=policy.config,
                                         pretrained_path=checkpoint)
    proprio = policy.config.input_features['observation.state'].shape[0]

    def policy_action(obs):
        s = obs['state']
        batch = {
            'observation.state':
                torch.from_numpy(s[:proprio]).float().unsqueeze(0).to(device),
            'observation.environment_state':
                torch.from_numpy(s[proprio:]).float().unsqueeze(0).to(device),
            'task': [TASK],
        }
        if image:
            img = torch.from_numpy(obs['image']).permute(2, 0, 1).float() / 255.0
            batch['observation.images.cam'] = img.unsqueeze(0).to(device)
        batch = pre(batch)
        batch = {k: (v.to(device) if torch.is_tensor(v) else v)
                 for k, v in batch.items()}
        with torch.no_grad():
            action = policy.select_action(batch)
        action = post(action)
        return action.squeeze(0).cpu().numpy()

    return policy_action, policy.reset, proprio
