#!/usr/bin/env python3
"""Two fixes on top of amendment (l)'s first implementation, found while smoke-testing (neither has run in any cell):

FIX 1 (settle guard). The evaluator decided whether to run the end-of-episode settle with
`info.get("slide_success") is None`, but genesis_can_env.step now writes that key on EVERY step, so the guard was always
False and the settle was skipped for every episode that ended on the evaluator's horizon (i.e. every timeout) -- exactly
the episodes where the in-episode counter cannot have granted it either. slide_success was therefore structurally 0 for
timeouts. The settle is now flagged explicitly by whoever runs it (`info['end_of_episode'] = True`) and the evaluator
keys off that flag.

FIX 2 (per-clause diagnostics). A failed window recorded nothing about WHY, so a zero column could not be explained.
`_slide_clauses()` now returns (ok, reason) with reason in {not_picked, grip_closed, no_contact, off_shelf_or_tilted},
and end_of_episode reports `slide_fail_reason` / `slide_fail_frame` (the first frame of the window that broke it).
usage: slide_guard_fix.py --r2d <root> ... --gp <root> ..."""
import argparse, pathlib

CAN_ENV_EDITS = [
    ("    def _slide_clauses(self):\n"
     "        \"\"\"amendment (l): the four slide_success clauses evaluated on the CURRENT world state, with the last\n"
     "        COMMANDED grip (the controller target is unchanged through the settle, so the command still stands).\"\"\"\n"
     "        w = self.w\n"
     "        if self._last_grip_cmd is None or float(self._last_grip_cmd) >= GRIP_OPEN_CMD or not self._picked:\n"
     "            return False\n"
     "        c = np_(w['bottle'].get_contacts(w['goal'])['position'])\n"
     "        if not (c.size and c.shape[0]):\n"
     "            return False\n"
     "        bp = np_(w['bottle'].get_pos())\n"
     "        return bool(in_shelf_footprint(bp) and tilt_deg(np_(w['bottle'].get_quat())) < 20.0)\n",
     "    def _slide_clauses(self):\n"
     "        \"\"\"amendment (l): the four slide_success clauses on the CURRENT world state, with the last COMMANDED grip\n"
     "        (the controller target is unchanged through the settle, so the command still stands).\n"
     "        Returns (ok, reason); reason names the FIRST clause that failed, so a zero column can be explained.\"\"\"\n"
     "        w = self.w\n"
     "        if not self._picked:\n"
     "            return False, 'not_picked'\n"
     "        if self._last_grip_cmd is None or float(self._last_grip_cmd) >= GRIP_OPEN_CMD:\n"
     "            return False, 'grip_closed'\n"
     "        c = np_(w['bottle'].get_contacts(w['goal'])['position'])\n"
     "        if not (c.size and c.shape[0]):\n"
     "            return False, 'no_contact'\n"
     "        bp = np_(w['bottle'].get_pos())\n"
     "        if not (in_shelf_footprint(bp) and tilt_deg(np_(w['bottle'].get_quat())) < 20.0):\n"
     "            return False, 'off_shelf_or_tilted'\n"
     "        return True, None\n"),
    ("        w = self.w\n"
     "        slide, route, steps = bool(self._slide_success), self._slide_route, 0\n"
     "        if not slide:\n"
     "            held = True\n"
     "            for _ in range(SLIDE_SUSTAIN):\n"
     "                for _ in range(3):\n"
     "                    w['scene'].step()\n"
     "                steps += 3\n"
     "                if not self._slide_clauses():\n"
     "                    held = False\n"
     "                    break\n"
     "            if held:\n"
     "                slide, route = True, 'settle'\n"
     "                self._slide_success = True; self._slide_route = route; self._slide_frame = self._t\n"
     "        for _ in range(SETTLE_STEPS - steps):\n"
     "            w['scene'].step()\n",
     "        w = self.w\n"
     "        slide, route, steps = bool(self._slide_success), self._slide_route, 0\n"
     "        reason, fail_frame = None, None\n"
     "        if not slide:\n"
     "            held = True\n"
     "            for f in range(SLIDE_SUSTAIN):\n"
     "                for _ in range(3):\n"
     "                    w['scene'].step()\n"
     "                steps += 3\n"
     "                ok, why = self._slide_clauses()\n"
     "                if not ok:\n"
     "                    held, reason, fail_frame = False, why, f\n"
     "                    break\n"
     "            if held:\n"
     "                slide, route = True, 'settle'\n"
     "                self._slide_success = True; self._slide_route = route; self._slide_frame = self._t\n"
     "        for _ in range(SETTLE_STEPS - steps):\n"
     "            w['scene'].step()\n"),
    ("        return dict(nested=nested, slide_success=bool(slide), slide_route=route)\n",
     "        return dict(nested=nested, slide_success=bool(slide), slide_route=route,\n"
     "                    slide_fail_reason=reason, slide_fail_frame=fail_frame)\n"),
    ("            _e = self.end_of_episode()\n"
     "            info['nested'] = _e['nested']\n"
     "            info['slide_success'] = _e['slide_success']; info['slide_route'] = _e['slide_route']\n",
     "            _e = self.end_of_episode()\n"
     "            info['nested'] = _e['nested']\n"
     "            info['slide_success'] = _e['slide_success']; info['slide_route'] = _e['slide_route']\n"
     "            info['slide_fail_reason'] = _e['slide_fail_reason']; info['slide_fail_frame'] = _e['slide_fail_frame']\n"
     "            info['end_of_episode'] = True\n"),
]

ADAPTER_EDITS = [
    ('            _e = self._env.genv.end_of_episode()\n'
     '            info["slide_success"] = _e["slide_success"]; info["slide_route"] = _e["slide_route"]\n'
     '        if done and self._scope == "full":\n',
     '            _e = self._env.genv.end_of_episode()\n'
     '            info["slide_success"] = _e["slide_success"]; info["slide_route"] = _e["slide_route"]\n'
     '            info["slide_fail_reason"] = _e["slide_fail_reason"]; info["slide_fail_frame"] = _e["slide_fail_frame"]\n'
     '            info["end_of_episode"] = True   # the settle ran here; the evaluator must not run a second one\n'
     '        if done and self._scope == "full":\n'),
    ('            _e = self._env.genv.end_of_episode()   # amendments (j) + (l): ONE settle, both readings\n'
     '            info["nested_honest"] = bool(_e["nested"])\n'
     '            info["slide_success"] = _e["slide_success"]; info["slide_route"] = _e["slide_route"]\n',
     '            _e = self._env.genv.end_of_episode()   # amendments (j) + (l): ONE settle, both readings\n'
     '            info["nested_honest"] = bool(_e["nested"])\n'
     '            info["slide_success"] = _e["slide_success"]; info["slide_route"] = _e["slide_route"]\n'
     '            info["slide_fail_reason"] = _e["slide_fail_reason"]; info["slide_fail_frame"] = _e["slide_fail_frame"]\n'
     '            info["end_of_episode"] = True\n'),
]

EVAL_EDITS = [
    ('    if SCOPE in ("full", "contact", "carrycontact") and info.get("slide_success") is None:\n'
     '        # amendment (l): the episode ended on THIS loop\'s horizon (the adapter never saw `done`), so the held\n'
     '        # continuation has not run yet. One settle per episode either way.\n'
     '        _es = env._env.genv.end_of_episode()\n'
     '        info["slide_success"] = _es["slide_success"]; info["slide_route"] = _es["slide_route"]\n'
     '        if SCOPE == "full":\n'
     '            info["nested_honest"] = _es["nested"]\n',
     '    if SCOPE in ("full", "contact", "carrycontact") and not info.get("end_of_episode"):\n'
     '        # amendment (l) + guard fix: the episode ended on THIS loop\'s horizon (the adapter never saw `done`), so the\n'
     '        # held continuation has not run yet. The guard keys off the explicit end_of_episode flag -- keying it off\n'
     '        # info["slide_success"] was ALWAYS False, because genesis_can_env.step writes that key on every step, so the\n'
     '        # settle was skipped for every timeout episode and slide_success was structurally 0 there.\n'
     '        _es = env._env.genv.end_of_episode()\n'
     '        info["slide_success"] = _es["slide_success"]; info["slide_route"] = _es["slide_route"]\n'
     '        info["slide_fail_reason"] = _es["slide_fail_reason"]; info["slide_fail_frame"] = _es["slide_fail_frame"]\n'
     '        info["end_of_episode"] = True\n'
     '        if SCOPE == "full":\n'
     '            info["nested_honest"] = _es["nested"]\n'),
    ('    if info.get("slide_route"):\n'
     '        slide_routes[info["slide_route"]] = slide_routes.get(info["slide_route"], 0) + 1\n',
     '    if info.get("slide_route"):\n'
     '        slide_routes[info["slide_route"]] = slide_routes.get(info["slide_route"], 0) + 1\n'
     '    elif info.get("slide_fail_reason"):\n'
     '        slide_fails[info["slide_fail_reason"]] = slide_fails.get(info["slide_fail_reason"], 0) + 1\n'),
    ('slide_routes = {"sustained": 0, "settle": 0}   # amendment (l): how each slide_success grant was earned\n',
     'slide_routes = {"sustained": 0, "settle": 0}   # amendment (l): how each slide_success grant was earned\n'
     'slide_fails = {}   # amendment (l): first clause that broke the window, per episode that did not earn it\n'),
    ('    slide_routes=dict(slide_routes),\n',
     '    slide_routes=dict(slide_routes), slide_fails=dict(slide_fails),\n'),
    ('    results.append(dict(ep=ep, stages=ep_stages, contact_diag=ep_cdiag, restored_uid=_restored, entry_frame=_entry_frame, outcome_honest=_outcome_honest,\n',
     '    results.append(dict(ep=ep, stages=ep_stages, contact_diag=ep_cdiag, restored_uid=_restored, entry_frame=_entry_frame, outcome_honest=_outcome_honest,\n'
     '                        slide_route=info.get("slide_route"), slide_fail_reason=info.get("slide_fail_reason"), slide_fail_frame=info.get("slide_fail_frame"),\n'),
]


def apply(path, edits):
    p = pathlib.Path(path); s = p.read_text()
    for old, new in edits:
        if new in s:
            print(f"  [{p.name}] already applied: {old.splitlines()[0].strip()[:66]}"); continue
        assert s.count(old) == 1, (p, s.count(old), old.splitlines()[0])
        s = s.replace(old, new); print(f"  [{p.name}] applied: {old.splitlines()[0].strip()[:66]}")
    p.write_text(s)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--r2d", action="append", default=[]); ap.add_argument("--gp", action="append", default=[])
    a = ap.parse_args()
    for r in a.r2d:
        r = pathlib.Path(r).expanduser(); print(f"== r2dreamer {r}")
        apply(r / "envs/genesis.py", ADAPTER_EDITS); apply(r / "eval_genesis.py", EVAL_EDITS)
    for g in a.gp:
        g = pathlib.Path(g).expanduser(); print(f"== genesis_pickaplace {g}")
        apply(g / "baselines/genesis_can_env.py", CAN_ENV_EDITS)
