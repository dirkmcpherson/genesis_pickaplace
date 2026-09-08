# Entry-bank state images (2026-09-07)

Every entry of the five phase entry-banks rendered as a picture of the restored scene, grouped
**one PNG per originating initial condition** so the user can flip through a bank with
`mpv ~/wm_fix_2026-09-03/bank_images/<bank>/`. Images only — no numbers here are new results.

## What was rendered

| bank | source bank file | version | scope | group key | entries | images | restore-failed |
|---|---|---|---|---|---|---|---|
| `polE_place` | `polE_place_physgrip.json` | physgrip_2026-09-07 (from `polE_place_rawgrip.json`) | place | `ic_index` | 148 | **23** | 5 (900002, 900007, 900055, 900058, 900126) |
| `polE_place_dDP` | `polE_place_dDP_physgrip.json` | physgrip_2026-09-07 | place | `ic_index` | 149 | **25** | 3 (910043, 910051, 910103) |
| `polE_contact` | `polE_contact_physgrip.json` | physgrip_2026-09-07 | contact | `ic_index` | 160 | **89** | 0 |
| `holdE_place` | `holdE_place.json` | as-recorded (no physgrip rebuild) | place | `uid` | 13 | **13** | 0 |
| `holdE_contact` | `holdE_contact.json` | as-recorded (no physgrip rebuild) | contact | `uid` | 11 | **11** | 0 |

161 PNGs, 67 MB, at `~/wm_fix_2026-09-03/bank_images/<bank>/` (LOCAL box; not in the repo, not on
the cluster). Filenames `ic_<key:04d>_n<entries>.png`, fixed pixel size per directory
(2785x771 for the policy banks, 700x449 for the holdE banks) so mpv flips cleanly. Each
directory also carries an `index.json` with the per-entry numbers behind its tiles.

The restore-failure sets reproduce `rs2_prep`'s `bank_restore_check.py` verdicts exactly
(148: 143 survive; 149: 146; 160: 160) — the failures are labelled in the images, not hidden.

## Render recipe

Renderer `~/wm_fix_2026-09-03/bank_images.py` (also at `$W/bank_images.py` on the cluster; not
committed). Per bank, one process:

1. `sim_variant_hook.apply_pre('gc_kp4_riser3_shelf6')`, `GENESIS_SIM_VARIANT`/`R2D_SIM_VARIANT`
   exported (full_env asserts its shelf band against the built world), rig cameras widened to
   256 px via `genesis_can_env.build_world = partial(..., rig_res=256)`.
2. `FullTaskEnv(backend='cpu', max_steps=1e9, scope=<place|contact>, render_size=(64,64),
   camera_rig=True, entry_bank=<one-entry temp copy>, phase_sparse=True)` + `apply_post` — the
   r2dreamer adapter's own build (`envs/genesis.py::_build`). The banks are read-only; nothing is
   written back to `phase_banks/`.
3. Per entry, the env's own restore — `_restore_place_entry` / `_restore_contact_entry`, i.e. the
   call the evaluator makes — then capture the 640x560 review cam (cropped), the rig top cam and
   the through-gripper wrist cam.

Per tile: `ic_index`/`uid`, the bank's own id, `frame`, can xyz, `grip_cmd` (physical) and
`grip_obs`, plus **RESTORE OK / RESTORE FAILED** with the can z and grip_obs after the settle;
failed entries are drawn dimmed. Per header: bank, bank file + version + grip units, IC key,
entry count, scope, world, and the spread within that IC (max pairwise can-pos cm, max pairwise
qpos rad).

## Notes

- Rendered on the local box (CPU sim backend, genesis 0.2.1) against a symlink-followed copy of
  the cluster's private `$W/gp_root`, after a parity check against the same code on a cluster CPU
  node: identical restore verdicts on all five banks and can-z agreement < 0.11 mm. Local was
  preferred because the cluster's `cv2` 5.0.0 double-draws `putText` labels (ghosted text on every
  caption); its 4.11 does not. No GPU, no cluster storage used.
- `holdE_place`/`holdE_contact` have one entry per uid, so their images are single tiles and their
  spread rows read 0.
