"""BC-RNN control config (paper/ROBOMIMIC_PLAN_2026-09-05.md §4 "BC reference row").

robomimic's OWN low-dim BC-RNN paper settings, reproduced field-by-field from
robomimic/scripts/generate_paper_configs.py (v0.5.0: modify_config_for_default_low_dim_exp lines 28-96 and
modify_bc_rnn_config_for_dataset lines 380-432) on top of the bc.json template (config_factory("bc")):
  epoch_every_n_steps 100, num_epochs 2000 (= 200k gradient steps), batch 100, seq_length 10, LSTM 2x400,
  actor_layer_dims (), lr 1e-4, GMM head ON for ph/mh, OFF for mg, obs = eef_pos + eef_quat + gripper_qpos + object,
  hdf5_cache_mode all.
Deviation (disclosed, plan §5 statistic): in-training rollouts are DISABLED (the paper scores MAX over
checkpoints with 50 rollouts every 50 epochs); we score the LAST checkpoint on the shared 50-state bank with
eval_bcrnn_robosuite.py. The arm's tape list enters through train.hdf5_filter_key = <ARM> on the masked copy
(make_arms.py) -- exactly the tapes the other learners see.

  $LAB/robo_venv/bin/python baselines/robomimic/make_bcrnn_config.py --arm PH200 --seed 0 --out <run>/config.json
"""
import argparse
import json
import pathlib as pl
import sys

sys.path.insert(0, str(pl.Path(__file__).resolve().parent))
from robo_common import DATA_ROOT, STATE_KEYS, HORIZON  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", required=True, choices=["PH200", "MH200", "MG200s", "MH300", "MGall", "MG718s"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True, help="config json path")
    ap.add_argument("--output-dir", required=True, help="robomimic train.output_dir")
    ap.add_argument("--name", default=None)
    ap.add_argument("--num-epochs", type=int, default=2000)
    ap.add_argument("--gmm", choices=["recipe", "on", "off"], default="recipe",
                    help="policy head. 'recipe' (default) = robomimic's per-dataset setting (GMM on for ph/mh, OFF for mg) "
                         "-- the setting every 2026-09-07 BC-RNN cell used. 'on'/'off' FORCE the head, which is what "
                         "amendment A5 (2026-09-08) needs: the two arms of the published row used DIFFERENT policy "
                         "classes, so a head-matched contrast has to override the recipe explicitly.")
    args = ap.parse_args()
    from robomimic.config import config_factory
    idx = json.loads((DATA_ROOT / "arms" / "arms_index.json").read_text())
    arm_man = json.loads((DATA_ROOT / "arms" / args.arm / "manifest.json").read_text())
    src = arm_man["src"]; masked = idx["masked"][src]
    dtype = {"ph": "ph", "mh": "mh", "mg": "mg"}[src]
    config = config_factory("bc")
    with config.values_unlocked():
        config.experiment.name = args.name or f"bcrnn_{args.arm}_s{args.seed}"
        config.experiment.validate = False
        config.experiment.logging.terminal_output_to_txt = True
        config.experiment.logging.log_tb = False
        config.experiment.save.enabled = True
        config.experiment.save.every_n_epochs = 50
        config.experiment.save.on_best_rollout_success_rate = False
        config.experiment.save.on_best_validation = False
        config.experiment.epoch_every_n_steps = 100
        config.experiment.validation_epoch_every_n_steps = 10
        config.experiment.rollout.enabled = False          # DISCLOSED deviation: scored LAST on the bank instead
        config.experiment.rollout.n = 50
        config.experiment.rollout.horizon = HORIZON
        config.experiment.rollout.rate = 50
        config.experiment.rollout.terminate_on_success = True
        config.experiment.render_video = False
        config.train.data = [{"path": masked}]
        config.train.hdf5_filter_key = args.arm
        config.train.output_dir = args.output_dir
        config.train.num_data_workers = 0
        config.train.hdf5_cache_mode = "all"
        config.train.batch_size = 100
        config.train.num_epochs = args.num_epochs
        config.train.seq_length = 10
        config.train.seed = args.seed
        config.algo.rnn.enabled = True
        config.algo.rnn.horizon = 10
        config.algo.rnn.hidden_dim = 400
        config.algo.optim_params.policy.learning_rate.initial = 1e-4
        config.algo.actor_layer_dims = ()
        gmm_recipe = (dtype != "mg")
        config.algo.gmm.enabled = gmm_recipe if args.gmm == "recipe" else (args.gmm == "on")
        config.observation.modalities.obs.low_dim = list(STATE_KEYS)
        config.observation.modalities.obs.rgb = []
    out = pl.Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(json.loads(config.dump()), indent=2))
    print(f"[bcrnn-config] arm {args.arm} src {src} filter_key {args.arm} data {masked} gmm {config.algo.gmm.enabled} "
          f"(flag={args.gmm}, recipe would be {gmm_recipe}) epochs {args.num_epochs} -> {out}")


if __name__ == "__main__":
    main()
