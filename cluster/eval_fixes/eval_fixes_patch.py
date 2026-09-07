#!/usr/bin/env python3
"""Evaluation fixes after ADVERSARIAL_REVIEW_eval_env_2026-09-07 (PHASE_PLAN amendment (j)). Exact-anchor, idempotent
replacements (asserts every anchor is unique), applied identically to the r2dreamer tree(s) and the genesis_pickaplace
tree(s). Nothing here changes a reward, a termination or a training data row.

r2dreamer  envs/genesis.py : GenesisPick.grip_phys (the ONE policy->physical grip map; step() uses it), FULL_EXTRA_KEYS
                             (log_placed_v2 / log_nested_honest for scope=full), nested_honest = genv._nested() at the
                             terminal step AFTER the terminal obs is captured (training rows unchanged).
r2dreamer  eval_genesis.py : (1) EVERY bank scope pins the enumerated entry (place drew with replacement + substituted
                             failing entries: S1-3); restored uid + entry frame recorded per episode and asserted equal to
                             the enumerated uid; (2) --dump-entries stores grip_cmd = grip_phys(action) (S2-4) with
                             grip_cmd_raw / grip_units / bank_version; (3) scope=full reports nested_honest (settle at the
                             episode end) next to nested_proxy (== the old `nested`; `outcome` keeps the old taxonomy so
                             the contact_push reproduction check still applies) and outcome_honest; (4) stages.placed_v2
                             is real in scope=full (full_env below), `placed` flagged stale; summary stamps
                             eval_fixes/bank sha/bank_version/world shelf top.
genesis_pickaplace baselines/rl/full_env.py : (4) placed_v2 computed (LOGGED ONLY) in scope=full; _pv2_run reset on every
                             reset path; (5) shelf band asserted against the BUILT world's shelf box in __init__ (S3-7).
usage: eval_fixes_patch.py --r2d <r2dreamer_root> [--r2d ...] --gp <genesis_pickaplace_root> [--gp ...]"""
import argparse, pathlib

ADAPTER_EDITS = [
    ('PLACE_EXTRA_KEYS = ("placed_v2",)\n',
     'PLACE_EXTRA_KEYS = ("placed_v2",)\n'
     '# scope=\'full\' (amendment (j), 2026-09-07): the phase-scope release predicate placed_v2 is computed (logged only) in\n'
     '# full scope too, and nested_honest = the env\'s settled proximity predicate at the terminal step (log keys only).\n'
     'FULL_EXTRA_KEYS = ("placed_v2", "nested_honest")\n'),
    ('        self._stage_keys = STAGE_KEYS + (PLACE_EXTRA_KEYS if scope in ("place", "contact", "carrycontact") else ())\n',
     '        self._stage_keys = STAGE_KEYS + (PLACE_EXTRA_KEYS if scope in ("place", "contact", "carrycontact") else ()) \\\n'
     '            + (FULL_EXTRA_KEYS if scope == "full" else ())   # amendment (j)\n'),
    ('    def sync_delta_target(self):\n',
     '    @staticmethod\n'
     '    def grip_phys(a):\n'
     '        """The ONE policy->physical grip map: normalized action a[6] in [-1, 1] -> physical 0..1 (what FullTaskEnv\n'
     '        reads as a_phys[6]; pick_env.denormalize_action is the same affine map). step() uses it, and so does\n'
     '        eval_genesis --dump-entries (ADVERSARIAL_REVIEW_eval_env_2026-09-07 S2-4: the raw [-1,1] value used to be\n'
     '        stored as grip_cmd and read back by the restore as physical 0..1)."""\n'
     '        return (float(np.clip(np.asarray(a, dtype=np.float64).reshape(-1)[6], -1.0, 1.0)) + 1.0) / 2.0\n'
     '\n'
     '    def sync_delta_target(self):\n'),
    ('            grip01 = (float(np.clip(a[6], -1.0, 1.0)) + 1.0) / 2.0  # grip ABSOLUTE\n',
     '            grip01 = self.grip_phys(a)  # grip ABSOLUTE -- the ONE map (amendment (j)); numerically identical to before\n'),
    ('        if getattr(self, "_state_obs", False):\n'
     '            obs["state"] = self._last_state\n'
     '        # Stage flags: zero on every step except the final one (trainer.eval\n',
     '        if getattr(self, "_state_obs", False):\n'
     '            obs["state"] = self._last_state\n'
     '        if done and self._scope == "full":\n'
     '            # nested_honest (amendment (j), review S1-1): the env\'s own settled predicate (100 settle steps, proximity\n'
     '            # <= NESTED_TOUCH_DIST, picked, both upright) -- what the DP/RLPD path reports at its horizon; this adapter\n'
     '            # disables the env\'s own settle (max_steps=1e9). Runs AFTER the terminal obs/reward/termination were taken,\n'
     '            # so rewards, `is_terminal` and every stored row are unchanged; only the log key differs. The old per-step\n'
     '            # `nested` (sticky contact + grip commanded open + both upright) stays the training reward/terminal and is\n'
     '            # reported as nested_proxy. A truncation by the caller\'s own horizon never reaches here (eval_genesis then\n'
     '            # calls genv._nested() itself).\n'
     '            info["nested_honest"] = bool(self._env.genv._nested())\n'
     '            info["nested_proxy"] = bool("nested" in self._env._granted or info.get("nested"))\n'
     '        # Stage flags: zero on every step except the final one (trainer.eval\n'),
]

EVAL_EDITS = [
    # (1) pin the entry in EVERY scope; return the restored uid/frame
    ('def reset_to_uid(uid):\n'
     '    """Adapter reset, but pinned to an explicit demo uid (reportable ICs)."""\n'
     '    if env._env is None:\n'
     '        env._build()                              # gs.init + world build (once)\n'
     '    # place scope: do NOT pin the uid -- the within-uid resample can loop on a\n'
     '    # single non-surviving entry (seen: 333@721 x30 -> RuntimeError). Whole-bank\n'
     '    # sampling matches training\'s reset distribution anyway.\n'
     '    if getattr(env._env, \'scope\', \'full\') == \'place\':\n'
     '        env._env.reset()\n'
     '    else:\n'
     '        env._env.reset(options={"uid": int(uid)})  # demo IC: that trial\'s recorded placement\n',
     'def reset_to_uid(uid):\n'
     '    """Adapter reset, pinned to an explicit demo uid / bank entry (reportable ICs). Returns (obs, (restored_uid, entry_frame)).\n'
     '\n'
     '    EVERY scope pins (amendment (j), 2026-09-07; ADVERSARIAL_REVIEW_eval_env S1-3): the place scope used to call\n'
     '    env._env.reset() unpinned, i.e. it drew from the WHOLE bank with replacement (~94 of 148 distinct entries per\n'
     '    cell) and silently substituted any entry that failed to restore, while contact/carrycontact pinned -- the only\n'
     '    difference behind the "restores in one scope but not the other" chase. Now a pinned entry that does not survive\n'
     '    the restore raises (FullTaskEnv retries the same entry PLACE_MAX_TRIES times) -> recorded as restore_failed by\n'
     '    the caller, never substituted; the restored uid is returned and asserted equal to the enumerated one."""\n'
     '    if env._env is None:\n'
     '        env._build()                              # gs.init + world build (once)\n'
     '    _ret = env._env.reset(options={"uid": int(uid)})  # demo IC / bank entry of THAT uid only\n'
     '    _rinfo = _ret[1] if (isinstance(_ret, tuple) and len(_ret) == 2 and isinstance(_ret[1], dict)) else {}\n'
     '    _restored = (int(_rinfo["uid"]) if _rinfo.get("uid") is not None else None,\n'
     '                 (int(_rinfo["entry_frame"]) if _rinfo.get("entry_frame") is not None else None))\n'),
    ('    for k in STAGE_KEYS + ("task_success",):\n'
     '        obs[f"log_{k}"] = np.float32(0.0)\n'
     '    return obs\n'
     '\n'
     '\n'
     'import cv2  # noqa: E402\n',
     '    for k in STAGE_KEYS + ("task_success",):\n'
     '        obs[f"log_{k}"] = np.float32(0.0)\n'
     '    return obs, _restored\n'
     '\n'
     '\n'
     'import cv2  # noqa: E402\n'
     'import hashlib  # noqa: E402\n'
     '\n'
     'BANK_VERSION = "physgrip_2026-09-07"   # stamped into every --dump-entries entry (physical grip units)\n'
     'BANK_SHA256 = hashlib.sha256(open(args.entry_bank, "rb").read()).hexdigest() if args.entry_bank else None\n'
     'BANK_VERSION_IN = None\n'
     'if args.entry_bank:\n'
     '    _bj = json.load(open(args.entry_bank)); _b0 = (next(iter(_bj.values())) if isinstance(_bj, dict) else _bj[0]) if _bj else {}\n'
     '    BANK_VERSION_IN = _b0.get("bank_version") if isinstance(_b0, dict) else None\n'
     '    _bg = [float(e["grip_cmd"]) for e in (_bj.values() if isinstance(_bj, dict) else _bj)]\n'
     '    print(f"[eval] entry bank {args.entry_bank}: {len(_bg)} entries, bank_version={BANK_VERSION_IN!r}, grip_cmd range "\n'
     '          f"{min(_bg):.3f}..{max(_bg):.3f}, sha256 {BANK_SHA256[:12]}", flush=True)\n'
     '    if min(_bg) < 0.0:\n'
     '        print("[eval] WARNING: bank carries grip_cmd < 0 (raw [-1,1] units; review S2-4) -- the restore clips it to 0 (fingers open)", flush=True)\n'),
    ('    for k in STAGE_KEYS + ("task_success",):\n'
     '        obs[f"log_{k}"] = np.float32(0.0)\n'
     '    return obs\n'
     '\n'
     '\n'
     'def _ic_label(ic, ep):\n',
     '    for k in STAGE_KEYS + ("task_success",):\n'
     '        obs[f"log_{k}"] = np.float32(0.0)\n'
     '    return obs, (None, None)\n'
     '\n'
     '\n'
     'def _ic_label(ic, ep):\n'),
    # stage keys + honest counters
    ('STAGES = ("picked", "placed", "placed_v2", "contact", "contact_push", "nested")   # END-TO-END arm: success-by-stage (every stage granted during the episode), reported alongside the scope success; contact_push = stricter contact, logged only (2026-09-07, amendment (g))\n'
     'stage_counts = {k: 0 for k in STAGES}   # hang = --ic-skip episodes (deterministic stalls), counted as failures\n',
     '# amendment (j) 2026-09-07: nested_proxy == the old `nested` (training proxy: sticky contact + grip commanded open + both upright,\n'
     '# terminating); nested_honest = genv._nested() after 100 settle steps at the episode end (scope=full only; False elsewhere);\n'
     '# placed_v2 is now computed in scope=full too (full_env, logged only); `placed` is the STALE base-world band (0.12-0.18 m).\n'
     'STAGES = ("picked", "placed", "placed_v2", "contact", "contact_push", "nested", "nested_proxy", "nested_honest")   # END-TO-END arm: success-by-stage (every stage granted during the episode), reported alongside the scope success; contact_push = stricter contact, logged only (2026-09-07, amendment (g))\n'
     'stage_counts = {k: 0 for k in STAGES}   # hang = --ic-skip episodes (deterministic stalls), counted as failures\n'
     'counts_honest = {"nested_honest": 0, "proxy_only": 0, "tipped": 0, "timeout": 0}   # scope=full outcome taxonomy under the honest predicate (amendment (j))\n'
     'pin_stats = {"n_enumerated": 0, "n_restored_match": 0, "n_restore_failed": 0, "n_hang": 0}   # amendment (j): every enumerated bank uid must be the restored one\n'),
    ('        counts["hang"] += 1\n'
     '        ep_stages = {k: False for k in STAGES}',
     '        counts["hang"] += 1\n'
     '        pin_stats["n_hang"] += 1; pin_stats["n_enumerated"] += int(not isinstance(ics[ep], dict))\n'
     '        ep_stages = {k: False for k in STAGES}'),
    ('    try:\n'
     '        obs = reset_to_ic(ics[ep])\n'
     '    except RuntimeError as _e:\n'
     '        if "no entry survived restore" not in str(_e):\n'
     '            raise\n'
     '        counts["restore_failed"] += 1\n',
     '    _restored, _entry_frame = None, None\n'
     '    try:\n'
     '        obs, (_restored, _entry_frame) = reset_to_ic(ics[ep])\n'
     '    except RuntimeError as _e:\n'
     '        if "no entry survived restore" not in str(_e):\n'
     '            raise\n'
     '        counts["restore_failed"] += 1\n'
     '        pin_stats["n_restore_failed"] += 1; pin_stats["n_enumerated"] += int(not isinstance(ics[ep], dict))\n'),
    ('    state = agent.get_initial_state(1)\n'
     '    trans = pack(obs, 0.0)\n',
     '    if not isinstance(ics[ep], dict):\n'
     '        # amendment (j): the episode runs from EXACTLY the enumerated entry (no substitution, ever)\n'
     '        pin_stats["n_enumerated"] += 1\n'
     '        assert _restored == int(ics[ep]), f"ep{ep}: enumerated uid {int(ics[ep])} but the env restored uid {_restored} (substitution)"\n'
     '        pin_stats["n_restored_match"] += 1\n'
     '    state = agent.get_initial_state(1)\n'
     '    trans = pack(obs, 0.0)\n'),
    ('    success = success_key in env._env._granted or bool(info.get(success_key))\n'
     '    tipped = bool(info.get("tipped"))\n'
     '    outcome = success_key if success else ("tipped" if tipped else "timeout")\n',
     '    _outcome_honest = None\n'
     '    if SCOPE == "full":\n'
     '        # nested_honest (amendment (j), review S1-1): if the episode terminated inside env.step the adapter already ran the\n'
     '        # settle; if it ended on THIS loop\'s horizon the adapter saw no `done`, so run the env\'s settled predicate now.\n'
     '        # Either way exactly one 100-step settle per episode, after the last decision (steps/outcome/reward unchanged).\n'
     '        if info.get("nested_honest") is None:\n'
     '            info["nested_honest"] = bool(env._env.genv._nested())\n'
     '        info["nested_proxy"] = bool("nested" in env._env._granted or info.get("nested"))\n'
     '        _outcome_honest = ("nested_honest" if info["nested_honest"] else "proxy_only" if info["nested_proxy"]\n'
     '                           else "tipped" if bool(info.get("tipped")) else "timeout")\n'
     '        counts_honest[_outcome_honest] += 1\n'
     '    success = success_key in env._env._granted or bool(info.get(success_key))\n'
     '    tipped = bool(info.get("tipped"))\n'
     '    outcome = success_key if success else ("tipped" if tipped else "timeout")   # scope=full: `nested` here is the PROXY (kept so per-episode outcome/steps reproduce the cells of record); see outcome_honest\n'),
    ('                          qpos=[float(x) for x in _sv[:6]], grip_cmd=float(a[6]) if len(a) >= 7 else float(_sv[6]),\n',
     '                          qpos=[float(x) for x in _sv[:6]],\n'
     '                          # amendment (j) / review S2-4: PHYSICAL grip (the adapter\'s own map), the units the restore reads\n'
     '                          grip_cmd=(env.grip_phys(a) if len(a) >= 7 else float(_sv[6])), grip_cmd_raw=(float(a[6]) if len(a) >= 7 else None),\n'
     '                          grip_units="physical01", bank_version=BANK_VERSION,\n'),
    ('    results.append(dict(ep=ep, stages=ep_stages, contact_diag=ep_cdiag, uid=(int(ics[ep]) if not isinstance(ics[ep], dict) else None),\n',
     '    results.append(dict(ep=ep, stages=ep_stages, contact_diag=ep_cdiag, restored_uid=_restored, entry_frame=_entry_frame, outcome_honest=_outcome_honest,\n'
     '                        uid=(int(ics[ep]) if not isinstance(ics[ep], dict) else None),\n'),
    ('    contact_push_diag=dict(n_contact=stage_counts["contact"], n_contact_push=stage_counts["contact_push"], **cpush_diag),   # 2026-09-07\n',
     '    contact_push_diag=dict(n_contact=stage_counts["contact"], n_contact_push=stage_counts["contact_push"], **cpush_diag),   # 2026-09-07\n'
     '    # amendment (j) 2026-09-07 -------------------------------------------------------------------------------------\n'
     '    nested_proxy=(stage_counts["nested_proxy"] / n if SCOPE == "full" else None),\n'
     '    nested_honest=(stage_counts["nested_honest"] / n if SCOPE == "full" else None),\n'
     '    outcomes_honest=(dict(counts_honest) if SCOPE == "full" else None),\n'
     '    eval_fixes="j", entries_pinned=True, pin_stats=dict(pin_stats),\n'
     '    bank_path=(str(args.entry_bank) if args.entry_bank else None), bank_sha256=BANK_SHA256, bank_version=BANK_VERSION_IN,\n'
     '    world_shelf_top_z=float(getattr(env._env, "shelf_top_z", float("nan"))),\n'
     '    stage_notes={"placed": "STALE base-world band BOX_TOP_Z+[0.01,0.07] = 0.12-0.18 m (unearnable in gc_kp4_riser3_shelf6; CONFOUNDS row 47) -- use placed_v2",\n'
     '                 "placed_v2": "release predicate on the WORLD shelf band (shelf_top_z+[0.01,0.07]), sustained 10 frames; computed in every scope incl. full (logged only there)",\n'
     '                 "nested": "TRAINING PROXY (== nested_proxy): sticky contact + grip commanded open + both upright; terminates the episode",\n'
     '                 "nested_honest": "genv._nested(): 100 settle steps at the episode end, centre distance <= NESTED_TOUCH_DIST, picked, both upright (the DP/RLPD path\'s predicate); scope=full only"},\n'),
    ('(OUT / "metrics.json").write_text(json.dumps(summary, indent=1))\n',
     '# amendment (j): the enumerated bank uids ARE the restored ones (or restore_failed / hang), never a substitute\n'
     'assert pin_stats["n_restored_match"] + pin_stats["n_restore_failed"] + pin_stats["n_hang"] == pin_stats["n_enumerated"], pin_stats\n'
     'print(f"[eval] pinned entries: {pin_stats}", flush=True)\n'
     '(OUT / "metrics.json").write_text(json.dumps(summary, indent=1))\n'),
]

FULL_ENV_EDITS = [
    # (5) shelf band asserted against the BUILT world (every scope, every path that constructs FullTaskEnv)
    ("            print(f'[phase] variant {_vn}: shelf_top_z {self.shelf_top_z:.3f} (band {self.shelf_top_z+0.01:.3f}..{self.shelf_top_z+0.07:.3f})', flush=True)\n",
     "            print(f'[phase] variant {_vn}: shelf_top_z {self.shelf_top_z:.3f} (band {self.shelf_top_z+0.01:.3f}..{self.shelf_top_z+0.07:.3f})', flush=True)\n"
     "        # amendment (j) 2026-09-07 (ADVERSARIAL_REVIEW_eval_env S3-7): the band above is derived from an ENV VAR while the\n"
     "        # world is built by sim_variant_hook.apply_pre() -- nothing tied them together. Assert against the BUILT shelf box\n"
     "        # (sim_variants.install() moves it by shelf_dz at build time) on EVERY path that constructs this env.\n"
     "        _built_top = self._world_shelf_top()\n"
     "        assert abs(_built_top - self.shelf_top_z) < 1e-6, (\n"
     "            f'shelf band mismatch: variant env var {_vn!r} gives shelf_top_z {self.shelf_top_z:.4f} but the BUILT world\\'s shelf top is '\n"
     "            f'{_built_top:.4f} -- export R2D_SIM_VARIANT/GENESIS_SIM_VARIANT to the variant the world was built with')\n"),
    ("    def _sync_dj_target(self):\n",
     "    def _world_shelf_top(self):\n"
     "        \"\"\"Top z of the shelf box AS BUILT (the Box entity whose morph size is replay_harness.BOX_SIZE; its base-link\n"
     "        position reflects sim_variants.install()'s shelf_dz shift). amendment (j) 2026-09-07.\"\"\"\n"
     "        from replay_harness import BOX_SIZE\n"
     "        for ent in self.genv.w['scene'].entities:\n"
     "            m = getattr(ent, 'morph', None)\n"
     "            if m is not None and type(m).__name__ == 'Box' and getattr(m, 'size', None) is not None \\\n"
     "                    and np.allclose(np.asarray(m.size, float), np.asarray(BOX_SIZE, float), atol=1e-9):\n"
     "                return float(np.asarray(np_(ent.get_pos()), dtype=np.float64).reshape(-1)[2]) + float(BOX_SIZE[2]) / 2.0\n"
     "        raise RuntimeError('shelf box entity (Box morph of size BOX_SIZE) not found in the built world')\n"
     "\n"
     "    def _sync_dj_target(self):\n"),
    # _pv2_run reset on the full/pick reset paths (it was only reset by _reset_place)
    ("        obs = self.genv.reset(uid=int(uid))\n"
     "        self._t = 0\n"
     "        self._granted = set()\n"
     "        self._hold_run = 0\n"
     "        self._pick_phi_prev = self._pick_phi() if self.pick_shaping else 0.0\n"
     "        self._sync_dj_target()\n"
     "        return obs['state'].astype(np.float32), {'uid': int(uid)}\n",
     "        obs = self.genv.reset(uid=int(uid))\n"
     "        self._t = 0\n"
     "        self._granted = set()\n"
     "        self._hold_run = 0\n"
     "        self._pv2_run = 0   # amendment (j): placed_v2 is computed in scope=full too\n"
     "        self._pick_phi_prev = self._pick_phi() if self.pick_shaping else 0.0\n"
     "        self._sync_dj_target()\n"
     "        return obs['state'].astype(np.float32), {'uid': int(uid)}\n"),
    ("        obs = self.genv.reset(**ic)\n"
     "        self._t = 0\n"
     "        self._granted = set()\n"
     "        self._hold_run = 0\n"
     "        self._pick_phi_prev = self._pick_phi() if self.pick_shaping else 0.0\n",
     "        obs = self.genv.reset(**ic)\n"
     "        self._t = 0\n"
     "        self._granted = set()\n"
     "        self._hold_run = 0\n"
     "        self._pv2_run = 0   # amendment (j)\n"
     "        self._pick_phi_prev = self._pick_phi() if self.pick_shaping else 0.0\n"),
    # (4) placed_v2 in scope=full: LOGGED ONLY (no reward, no termination; the staged ladder is untouched)
    ("        terminated = bool(info.get('nested')) and self.scope != 'place'\n",
     "        if self.scope == 'full':\n"
     "            # amendment (j) 2026-09-07 (ADVERSARIAL_REVIEW_eval_env S1-2): the phase-scope release predicate placed_v2\n"
     "            # (grip commanded open < PLACE_RELEASE, can inside the shelf footprint and the WORLD's shelf band, tilt <\n"
     "            # PLACE_TILT_DEG, sustained PLACE_SUSTAIN frames) is computed here too, LOGGED ONLY: no reward, no termination,\n"
     "            # so the staged ladder, every running job and every stored row are unchanged. The legacy `placed` (STAGE_REWARD)\n"
     "            # keeps the stale base-world band and is reported as stale by the evaluator.\n"
     "            _bp = np_(self.genv.w['bottle'].get_pos())\n"
     "            _ok = (float(a_phys[6]) < self.PLACE_RELEASE and in_shelf_footprint(_bp)\n"
     "                   and self.shelf_top_z + 0.01 < _bp[2] < self.shelf_top_z + 0.07\n"
     "                   and tilt_deg(np_(self.genv.w['bottle'].get_quat())) < self.PLACE_TILT_DEG)\n"
     "            self._pv2_run = self._pv2_run + 1 if _ok else 0\n"
     "            if self._pv2_run >= self.PLACE_SUSTAIN:\n"
     "                info['placed_v2'] = True\n"
     "                self._granted.add('placed_v2')\n"
     "        terminated = bool(info.get('nested')) and self.scope != 'place'\n"),
]


def apply(path, edits):
    p = pathlib.Path(path); s = p.read_text(); n_app = 0
    for old, new in edits:
        if new in s:
            print(f"  [{p.name}] already applied: {old.splitlines()[0].strip()[:70]}"); continue
        assert s.count(old) == 1, (p, s.count(old), old.splitlines()[0])
        s = s.replace(old, new); n_app += 1
        print(f"  [{p.name}] applied: {old.splitlines()[0].strip()[:70]}")
    p.write_text(s)
    return n_app


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--r2d", action="append", default=[], help="r2dreamer root(s): envs/genesis.py + eval_genesis.py")
    ap.add_argument("--gp", action="append", default=[], help="genesis_pickaplace root(s): baselines/rl/full_env.py")
    a = ap.parse_args()
    for root in a.r2d:
        root = pathlib.Path(root).expanduser(); print(f"== r2dreamer {root}")
        apply(root / "envs/genesis.py", ADAPTER_EDITS)
        apply(root / "eval_genesis.py", EVAL_EDITS)
    for root in a.gp:
        root = pathlib.Path(root).expanduser(); print(f"== genesis_pickaplace {root}")
        apply(root / "baselines/rl/full_env.py", FULL_ENV_EDITS)
