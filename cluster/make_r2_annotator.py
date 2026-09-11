"""Build eval_genesis_annot.py: a COPY of the r2dreamer evaluator that overlays per-decision
phase state, matching baselines/eval_e2e_annot.py for the {RLPD}/DP path.

A copy, not an edit: eval_genesis.py is read at runtime by in-flight eval stages. The copy adds a
per-decision snapshot of the SAME union the evaluator itself scores with (line ~369,
`_granted | {k for k in STAGES if info[k]}`) and replaces only the video writer. It never
recomputes a predicate, so the overlay shows exactly what was scored.

usage: make_r2_annotator.py [SRC eval_genesis.py] [DST eval_genesis_annot.py]

2026-09-10 (LADDER_UNIFY_BRIEF): the chips follow the UNIFIED LADDER -- PICK / PLACE /
PUSH are the three non-terminal rungs, NEST2 is nested_v2 and SLIDE is the paid terminal.
The withdrawn `nested` proxy is deliberately NOT a chip any more: an overlay that lights it
invites reading nesting off a predicate whose precision is 0.114 (human) / 0.029 (machine)
and which REVERSES the arm ordering. It is still in the evaluator's legacy columns.

SRC/DST default to the in-flight cluster tree for backward compatibility, but note that
writing into $W/r2dreamer_fix TOUCHES a tree in-flight jobs import at runtime -- pass
explicit paths (e.g. into $LAB/gp_unified's r2dreamer) once that tree exists.
"""
import shutil, sys
src = sys.argv[1] if len(sys.argv) > 1 else \
    '/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/r2dreamer_fix/eval_genesis.py'
dst = sys.argv[2] if len(sys.argv) > 2 else \
    '/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/r2dreamer_fix/eval_genesis_annot.py'
s = open(src).read()

old_init = """    frames = [obs["image"]]
    done, info, ep_reward, t = False, {}, 0.0, 0"""
new_init = """    frames = [obs["image"]]
    snaps = [frozenset()]          # sticky stage set, aligned 1:1 with frames
    done, info, ep_reward, t = False, {}, 0.0, 0"""
assert s.count(old_init) == 1, 'init anchor'
s = s.replace(old_init, new_init)

old_app = """        obs, reward, done, info = env.step(a)
        frames.append(obs["image"])"""
new_app = """        obs, reward, done, info = env.step(a)
        frames.append(obs["image"])
        _acc = set(snaps[-1]) | set(getattr(env._env, "_granted", set()))
        for _k in STAGES:                       # same union the scorer uses; read, never recompute
            if info.get(_k):
                _acc.add(_k)
        snaps.append(frozenset(_acc))"""
assert s.count(old_app) == 1, 'append anchor'
s = s.replace(old_app, new_app)

old_vid = """    vw = cv2.VideoWriter(str(vid), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
    for f in frames:
        tile = np.hstack([f[..., :3], f[..., 3:]])[:, :, ::-1]   # top|wrist, BGR
        vw.write(cv2.resize(tile, (w, h), interpolation=cv2.INTER_NEAREST))
    vw.release()"""
new_vid = """    PANEL = 78
    CHIPS = [("PICK", "picked"), ("PLACE", "placed_v2"), ("PUSH", "contact_push"),
             ("NEST2", "nested_v2"), ("SLIDE", "slide_success"),
             ("FAR", "farside"), ("HOME", "home")]
    first = {}
    for _i, _sn in enumerate(snaps):
        for _, _key in CHIPS:
            if _key in _sn and _key not in first:
                first[_key] = _i
    _F = cv2.FONT_HERSHEY_SIMPLEX
    _verdict = "slide=%d  nested_v2=%d  nested_honest=%d  tipped=%d" % (
        int(bool(info.get("slide_success"))), int(bool(info.get("nested_v2"))),
        int(bool(info.get("nested_honest"))), int(bool(tipped)))
    vw = cv2.VideoWriter(str(vid), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h + PANEL))
    for _i, f in enumerate(frames):
        tile = np.hstack([f[..., :3], f[..., 3:]])[:, :, ::-1]   # top|wrist, BGR
        im = cv2.resize(tile, (w, h), interpolation=cv2.INTER_NEAREST)
        pan = np.full((PANEL, w, 3), 24, np.uint8)
        sn = snaps[_i] if _i < len(snaps) else snaps[-1]
        x = 6
        for label, key in CHIPS:
            on = key in sn
            fresh = on and 0 <= _i - first.get(key, -99) < 6
            col = (60, 255, 255) if fresh else ((90, 240, 90) if on else (90, 90, 90))
            wid = 11 * len(label) + 12
            cv2.rectangle(pan, (x, 6), (x + wid, 28), col, -1 if on else 1)
            cv2.putText(pan, label, (x + 6, 23), _F, 0.44,
                        (20, 20, 20) if on else (170, 170, 170), 1, cv2.LINE_AA)
            if key in first:
                cv2.putText(pan, "d%d" % first[key], (x + 6, 42), _F, 0.34, (150, 220, 150), 1, cv2.LINE_AA)
            x += wid + 7
        cv2.putText(pan, "decision %d/%d" % (_i, len(frames) - 1), (6, 62), _F, 0.40,
                    (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(pan, _verdict, (170, 62), _F, 0.40,
                    (60, 255, 255) if info.get("slide_success") else (190, 190, 190), 1, cv2.LINE_AA)
        by = PANEL - 6
        cv2.line(pan, (6, by), (w - 6, by), (70, 70, 70), 2)
        if len(frames) > 1:
            px = int(6 + (w - 12) * _i / (len(frames) - 1))
            cv2.line(pan, (px, by - 3), (px, by + 3), (240, 240, 240), 2)
            for _key, _fi in first.items():
                fx = int(6 + (w - 12) * _fi / (len(frames) - 1))
                cv2.line(pan, (fx, by - 4), (fx, by + 4), (90, 240, 90), 1)
        vw.write(np.ascontiguousarray(np.vstack([im, pan])))
    vw.release()"""
assert s.count(old_vid) == 1, 'video anchor'
s = s.replace(old_vid, new_vid)
open(dst, 'w').write(s)
import ast; ast.parse(s)
print('wrote', dst, '(syntax OK)')
