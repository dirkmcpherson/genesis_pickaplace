# genesis_pickaplace

Replays in-the-wild Kinova gen3-lite pick-and-place demos in the Genesis simulator.

- Entry point: `python example.py -v` (viewer), `-t <uid>` for specific trials, `-p 0|1|2` to filter by can position.
- Data: `./inthewild_trials/<uid>_episodes.npy` (gitignored; pickled dicts with `vel_cmd` + `gripper_pos`). Regenerable from ROS bags via `trial_reader.py` (needs ROS1).
- Env: use `~/workspace/genesis_sim2real/venv` (python 3.10, genesis-world 0.2.1 editable → `~/workspace/Genesis`, torch 2.7.0+cu126, cv2 4.11). E.g. `~/workspace/genesis_sim2real/venv/bin/python example.py -t 232`.
- See `CAN_STARTING_POSITION.md` for the can-position recovery effort status.

## Agent Status

- **Status:** 🟢 active
- **Last session:** 2026-07-06
- **Current branch:** main
- **What happened:** Finished the placement pipeline end-to-end: 58/75 legit successful demos solved (77% coverage), validated contact 0.53 / nested 0.28 per run (see CAN_STARTING_POSITION.md FINAL NUMBERS). Key late finds: drag demos (seed at closure-start, from bags), fallen-can demos (8 solved with lying spawns — real videos show cans starting on their side), GPU->CPU winner transfer cliff (CPU re-pick pass recovers), nested metric scored at first-contact + settle (19/19 negative-control specificity). Success gallery in can_pos_recovery/videos (31/39 renders complete the task). lerobot DP baseline scaffolding in baselines/ (collector -> LeRobotDataset v3 w/ proprio+environment_state split -> eval incl --random ICs), smoke-tested.
- **What's next:** (1) full dataset collection + overnight state-only DP train (task list #9), eval on demo ICs + randomized ICs; (2) genesis upgrade in fresh venv (+ lerobot in same venv), match 0.2.1 baseline via probes -> sweep -> validation; (3) deferred: substeps=8 world (rescues 6 more drop-class trials — probe results in scratchpad probe_s8.json ... rerun post-upgrade), failure-video renders.
- **Blocked on:** nothing
- **Key decisions made:** substeps=8 decision deferred to post-upgrade world (probe: 6/8 drop-class rescued at 2x cost). Fail-labeled demos never get relocated goals (would manufacture success); example.py applies placements to success-labeled trials only. 290/322 are mislabeled stubs (gripper never closes) — excluded. Nested metric = upright+contact+at-rest after settle at first contact-success. Videos/datasets gitignored (reproducible).
