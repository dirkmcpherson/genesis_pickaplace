#!/usr/bin/env python3
"""PHASE_PLAN amendment (l): `slide_success` = picked (earlier) AND pick-can<->goal solver contact AND gripper COMMANDED
OPEN (< 0.3) AND the pick-can in the shelf footprint with tilt < 20 deg, sustained 3 decisions. Logged beside every
existing key; no reward, no termination, no training row changes. Exact-anchor, idempotent; applied to the same four
files as amendment (j) (repo / $W/gp_root / $W/r2dreamer_fix / local mirror).

THE SUSTAIN WINDOW (disclosed design decision, coordinator informed): both scopes where this is the statistic of record
TERMINATE at the first frame the predicate can hold -- scope='contact' returns on the first `contact` frame
(full_env.py:766) and scope='full' returns on the nested proxy (:817), which fires on contact AND grip commanded open AND
both upright, i.e. at or before slide's clauses. An in-episode counter therefore reaches 1, never 3 decisions, and a
literal implementation would report slide_success == 0 everywhere for a mechanical reason. So the window is evaluated in
the post-episode continuation that ALREADY exists: `_nested()`'s 100-step settle, during which the last commanded action
is held (the controller targets are unchanged). The first 12 frames (3 decisions x action_repeat 4, one frame = the 3
scene steps env.step takes) of that settle must satisfy all four clauses continuously. Total settle steps stay exactly
100 and the nested measurement is taken at the same point as before, so `nested` / `nested_honest` are unchanged.
Two routes are recorded separately: 'sustained' (granted inside the episode, possible only when nothing terminated) and
'settle' (granted over the held continuation). Route counts are reported per cell.
usage: slide_success_patch.py --r2d <r2dreamer_root> ... --gp <genesis_pickaplace_root> ..."""
import argparse, pathlib

CAN_ENV_EDITS = [
    ("PICK_EEF_DIST = 0.20\nPICK_SUSTAIN = 10\n",
     "PICK_EEF_DIST = 0.20\nPICK_SUSTAIN = 10\n"
     "# --- amendment (l) 2026-09-07: slide_success = the task as demonstrated (can ON THE SHELF, RELEASED, touching the\n"
     "# goal), sustained 3 decisions. One decision = action_repeat 4 env frames; one env frame = the 3 scene steps step()\n"
     "# takes -> SLIDE_SUSTAIN frames. GRIP_OPEN_CMD is the same threshold the legacy nested proxy uses.\n"
     "SLIDE_SUSTAIN_DECISIONS = 3\n"
     "SLIDE_SUSTAIN = SLIDE_SUSTAIN_DECISIONS * 4   # env frames\n"
     "GRIP_OPEN_CMD = 0.3\n"
     "SETTLE_STEPS = 100        # the post-episode settle _nested() has always run (scene steps)\n"),
    ("        self._contact_farside_wrist = False  # ... the WRIST on the far side (the withdrawn first definition)\n"
     "        self._pick_run = 0   # consecutive frames satisfying the held-can guard\n",
     "        self._contact_farside_wrist = False  # ... the WRIST on the far side (the withdrawn first definition)\n"
     "        # slide_success (amendment (l), logged only): sticky; route 'sustained' (in-episode) or 'settle' (held continuation)\n"
     "        self._slide_success = False; self._slide_run = 0; self._slide_frame = None; self._slide_route = None\n"
     "        self._last_grip_cmd = None   # last COMMANDED grip (physical 0..1); held through the post-episode settle\n"
     "        self._pick_run = 0   # consecutive frames satisfying the held-can guard\n"),
    ("                self._contact_push_frame = self._t\n"
     "        done = self._t >= self.max_steps\n",
     "                self._contact_push_frame = self._t\n"
     "        # --- slide_success (amendment (l)): all four clauses on THIS frame; sticky once sustained --------------\n"
     "        self._last_grip_cmd = float(grip)\n"
     "        slide_ok = bool(self._picked and bg_touch and float(grip) < GRIP_OPEN_CMD\n"
     "                        and in_shelf_footprint(bp) and tilt_deg(np_(w['bottle'].get_quat())) < 20.0)\n"
     "        self._slide_run = self._slide_run + 1 if slide_ok else 0\n"
     "        if self._slide_run >= SLIDE_SUSTAIN and not self._slide_success:\n"
     "            self._slide_success = True; self._slide_frame = self._t; self._slide_route = 'sustained'\n"
     "        done = self._t >= self.max_steps\n"),
    ("                    t=self._t, uid=self._uid, ws_blocked=ws_blocked,\n"
     "                    ws_violations=self.ws_violations)\n"
     "        if done:\n"
     "            info['nested'] = self._nested()\n"
     "        return self._obs(), done, info\n",
     "                    slide_success=self._slide_success, slide_ok=slide_ok, slide_run=self._slide_run,\n"
     "                    slide_frame=self._slide_frame, slide_route=self._slide_route,\n"
     "                    t=self._t, uid=self._uid, ws_blocked=ws_blocked,\n"
     "                    ws_violations=self.ws_violations)\n"
     "        if done:\n"
     "            # ONE post-episode settle yields both the settled `nested` and slide_success's held window (amendment (l));\n"
     "            # identical step count and identical nested measurement point as the old info['nested'] = self._nested().\n"
     "            _e = self.end_of_episode()\n"
     "            info['nested'] = _e['nested']\n"
     "            info['slide_success'] = _e['slide_success']; info['slide_route'] = _e['slide_route']\n"
     "        return self._obs(), done, info\n"),
    ("        return bool(self._picked and touch and tilt_deg(np_(w['bottle'].get_quat())) < 20\n"
     "                    and tilt_deg(np_(w['goal'].get_quat())) < 20)\n"
     "\n"
     "    def _obs(self):\n",
     "        return bool(self._picked and touch and tilt_deg(np_(w['bottle'].get_quat())) < 20\n"
     "                    and tilt_deg(np_(w['goal'].get_quat())) < 20)\n"
     "\n"
     "    def _slide_clauses(self):\n"
     "        \"\"\"amendment (l): the four slide_success clauses evaluated on the CURRENT world state, with the last\n"
     "        COMMANDED grip (the controller target is unchanged through the settle, so the command still stands).\"\"\"\n"
     "        w = self.w\n"
     "        if self._last_grip_cmd is None or float(self._last_grip_cmd) >= GRIP_OPEN_CMD or not self._picked:\n"
     "            return False\n"
     "        c = np_(w['bottle'].get_contacts(w['goal'])['position'])\n"
     "        if not (c.size and c.shape[0]):\n"
     "            return False\n"
     "        bp = np_(w['bottle'].get_pos())\n"
     "        return bool(in_shelf_footprint(bp) and tilt_deg(np_(w['bottle'].get_quat())) < 20.0)\n"
     "\n"
     "    def end_of_episode(self):\n"
     "        \"\"\"The single post-episode settle (SETTLE_STEPS scene steps, last command held), yielding BOTH the honest\n"
     "        settled `nested` (amendment (j)/S1-1) and slide_success's held window (amendment (l)).\n"
     "\n"
     "        Why the window lives here: scope='contact' terminates on the first contact frame and scope='full' on the\n"
     "        nested proxy, so an in-episode counter can never reach 3 decisions in the two scopes where slide_success is\n"
     "        the statistic of record. The clauses are therefore required to hold continuously over the first\n"
     "        SLIDE_SUSTAIN frames (3 scene steps each) of this continuation. The step BUDGET and the point at which\n"
     "        nested is measured are unchanged (always SETTLE_STEPS steps before the nested read), so nested is bit-identical\n"
     "        to the previous _nested() path; only extra state READS happen during the window (reads do not perturb the\n"
     "        solver -- established by the #26 trace ablation).\"\"\"\n"
     "        w = self.w\n"
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
     "            w['scene'].step()\n"
     "        bp = np_(w['bottle'].get_pos()); gp_ = np_(w['goal'].get_pos())\n"
     "        touch = float(np.hypot(bp[0] - gp_[0], bp[1] - gp_[1])) <= NESTED_TOUCH_DIST\n"
     "        nested = bool(self._picked and touch and tilt_deg(np_(w['bottle'].get_quat())) < 20\n"
     "                      and tilt_deg(np_(w['goal'].get_quat())) < 20)\n"
     "        return dict(nested=nested, slide_success=bool(slide), slide_route=route)\n"
     "\n"
     "    def _obs(self):\n"),
]

FULL_ENV_EDITS = [
    ("        if info.get('contact_push'):\n"
     "            # contact_push (2026-09-07): logged grant only -- never rewarded, never terminates (amendment (g))\n"
     "            self._granted.add('contact_push')\n",
     "        if info.get('contact_push'):\n"
     "            # contact_push (2026-09-07): logged grant only -- never rewarded, never terminates (amendment (g))\n"
     "            self._granted.add('contact_push')\n"
     "        if info.get('slide_success'):\n"
     "            # slide_success (amendment (l)): logged grant only -- never rewarded, never terminates\n"
     "            self._granted.add('slide_success')\n"),
]

ADAPTER_EDITS = [
    ('FULL_EXTRA_KEYS = ("placed_v2", "nested_honest")\n',
     'FULL_EXTRA_KEYS = ("placed_v2", "nested_honest", "slide_success")\n'
     '# amendment (l): slide_success is the success statistic of record for the slide phase (scope=\'contact\') and the\n'
     '# end-to-end scope; logged in the contact scopes too (carrycontact keeps bare `contact` as its own statistic).\n'
     'SLIDE_EXTRA_KEYS = ("slide_success",)\n'),
    ('        self._stage_keys = STAGE_KEYS + (PLACE_EXTRA_KEYS if scope in ("place", "contact", "carrycontact") else ()) \\\n'
     '            + (FULL_EXTRA_KEYS if scope == "full" else ())   # amendment (j)\n',
     '        self._stage_keys = STAGE_KEYS + (PLACE_EXTRA_KEYS if scope in ("place", "contact", "carrycontact") else ()) \\\n'
     '            + (FULL_EXTRA_KEYS if scope == "full" else ()) \\\n'
     '            + (SLIDE_EXTRA_KEYS if scope in ("contact", "carrycontact") else ())   # amendments (j), (l)\n'),
    ('        if done and self._scope == "full":\n',
     '        if done and self._scope in ("contact", "carrycontact") :\n'
     '            # amendment (l): the phase scopes terminate on the first contact frame, so slide_success\'s window is\n'
     '            # evaluated in the held continuation (env.end_of_episode). Runs AFTER the terminal obs/reward were taken.\n'
     '            _e = self._env.genv.end_of_episode()\n'
     '            info["slide_success"] = _e["slide_success"]; info["slide_route"] = _e["slide_route"]\n'
     '        if done and self._scope == "full":\n'),
    ('            info["nested_honest"] = bool(self._env.genv._nested())\n'
     '            info["nested_proxy"] = bool("nested" in self._env._granted or info.get("nested"))\n',
     '            _e = self._env.genv.end_of_episode()   # amendments (j) + (l): ONE settle, both readings\n'
     '            info["nested_honest"] = bool(_e["nested"])\n'
     '            info["slide_success"] = _e["slide_success"]; info["slide_route"] = _e["slide_route"]\n'
     '            info["nested_proxy"] = bool("nested" in self._env._granted or info.get("nested"))\n'),
]

EVAL_EDITS = [
    ('STAGES = ("picked", "placed", "placed_v2", "contact", "contact_push", "nested", "nested_proxy", "nested_honest")',
     'STAGES = ("picked", "placed", "placed_v2", "contact", "contact_push", "nested", "nested_proxy", "nested_honest", "slide_success")'),
    ('counts_honest = {"nested_honest": 0, "proxy_only": 0, "tipped": 0, "timeout": 0}   # scope=full outcome taxonomy under the honest predicate (amendment (j))\n',
     'counts_honest = {"nested_honest": 0, "proxy_only": 0, "tipped": 0, "timeout": 0}   # scope=full outcome taxonomy under the honest predicate (amendment (j))\n'
     'slide_routes = {"sustained": 0, "settle": 0}   # amendment (l): how each slide_success grant was earned\n'),
    ('    _outcome_honest = None\n'
     '    if SCOPE == "full":\n',
     '    _outcome_honest = None\n'
     '    if SCOPE in ("full", "contact", "carrycontact") and info.get("slide_success") is None:\n'
     '        # amendment (l): the episode ended on THIS loop\'s horizon (the adapter never saw `done`), so the held\n'
     '        # continuation has not run yet. One settle per episode either way.\n'
     '        _es = env._env.genv.end_of_episode()\n'
     '        info["slide_success"] = _es["slide_success"]; info["slide_route"] = _es["slide_route"]\n'
     '        if SCOPE == "full":\n'
     '            info["nested_honest"] = _es["nested"]\n'
     '    if info.get("slide_route"):\n'
     '        slide_routes[info["slide_route"]] = slide_routes.get(info["slide_route"], 0) + 1\n'
     '    if SCOPE == "full":\n'),
    ('    nested_proxy=(stage_counts["nested_proxy"] / n if SCOPE == "full" else None),\n',
     '    slide_success=(stage_counts["slide_success"] / n if SCOPE in ("full", "contact", "carrycontact") else None),   # amendment (l): statistic of record for the slide phase and end-to-end\n'
     '    slide_routes=dict(slide_routes),\n'
     '    nested_proxy=(stage_counts["nested_proxy"] / n if SCOPE == "full" else None),\n'),
    ('                 "nested_honest": "genv._nested(): 100 settle steps at the episode end, centre distance <= NESTED_TOUCH_DIST, picked, both upright (the DP/RLPD path\'s predicate); scope=full only"},\n',
     '                 "nested_honest": "genv._nested(): 100 settle steps at the episode end, centre distance <= NESTED_TOUCH_DIST, picked, both upright (the DP/RLPD path\'s predicate); scope=full only",\n'
     '                 "slide_success": "amendment (l): picked AND pick-can/goal solver contact AND grip commanded < 0.3 AND can in the shelf footprint with tilt < 20 deg, sustained 3 decisions (12 env frames). Both scopes where it is the statistic of record terminate at the first frame it could hold, so the window is evaluated over the first 12 frames of the post-episode settle with the last command held; route sustained|settle per episode"},\n'),
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--r2d", action="append", default=[]); ap.add_argument("--gp", action="append", default=[])
    a = ap.parse_args()
    for root in a.r2d:
        root = pathlib.Path(root).expanduser(); print(f"== r2dreamer {root}")
        apply(root / "envs/genesis.py", ADAPTER_EDITS); apply(root / "eval_genesis.py", EVAL_EDITS)
    for root in a.gp:
        root = pathlib.Path(root).expanduser(); print(f"== genesis_pickaplace {root}")
        apply(root / "baselines/genesis_can_env.py", CAN_ENV_EDITS); apply(root / "baselines/rl/full_env.py", FULL_ENV_EDITS)
