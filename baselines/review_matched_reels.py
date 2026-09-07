#!/usr/bin/env python3
"""Matched-start review reels for the human-vs-machine comparisons of record (2026-09-07).

For each comparison (arm H = human-demo policy, arm M = machine-demo policy) and each deterministic ("mode") eval cell,
put the two arms on the SAME evaluation starts, stratified by joint outcome (both-succeed / human-only / machine-only /
both-fail), so a reviewer can see whether the arms behave the same or merely score the same.  Built ONLY from the
evaluation videos + metrics.json already on the cluster: no simulation, no learner runs, cluster read-only.

Selection rule (deterministic, written in the INDEX):
  * per arm and cell, the MEDIAN seed by that cell's success count: median count = sorted(counts)[(n-1)//2] (the lower
    median for even n); ties -> the lowest seed with that count (typical policy, not the best one);
  * over the shared starts (episode index k = start; uid / ic checked equal across arms) classify each start by
    (H outcome, M outcome); starts where either arm reports restore_failed or has no video are skipped;
  * sample up to 3 starts per class uniformly with random.Random(0), one generator per (comparison, cell), classes drawn
    in the fixed order both-succeed, human-only, machine-only, both-fail;
  * reel: one 2-row block per class (row 1 = H, row 2 = M, columns = sampled starts), blocks concatenated left->right with
    a header "<class> <count>/<n>"; every tile "H s<seed> ep<k> <outcome> <steps>"; scale 1, 4 fps (half speed),
    cap 150 frames, ended clips dimmed (style of ~/wm_fix_2026-09-03/tile_review.py);
  * contact sheet (OOD cell only): the same starts, first / middle / last frame of both arms side by side.

usage (default root ~/wm_fix_2026-09-03/review_matched):
  review_matched_reels.py plan   [--root R]            # metrics.json under R/src/<run>/<cell>/ -> R/selection.json, R/INDEX.md, R/src/videos_list.txt
  review_matched_reels.py fetch  [--root R]            # rsync the SELECTED clips only (read-only on the cluster)
  review_matched_reels.py build  [--root R] [--only pick,place] [--no-h264]
"""
import argparse, json, os, random, subprocess, sys
from math import comb
import cv2, numpy as np

W_REMOTE = "pax:/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/runs/"
W_ABS = "/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/runs/"
DEFAULT_ROOT = os.path.expanduser("~/wm_fix_2026-09-03/review_matched")
S2 = ["fresh_eval_rnd30_mode", "fresh_eval_hold15_mode"]
PH = ["fresh_eval_polE_mode", "fresh_eval_holdE_mode"]
# name, H run pattern, M run pattern, seeds, [OOD cell, ID cell], success outcome, one-line description
COMPARISONS = [
    ("pick", "s2_r2d_pick_state_dHv2raw_bnormclamp1ent5_s{s}", "s2_r2d_pick_state_dDP_bnormclamp1ent5_s{s}", range(8), S2, "picked",
     "pick scope; rnd30 = 30 shared random starts (OOD), hold15 = 15 held-out human starts (ID)"),
    ("pick_r1", "s2_r2d_pick_state_dHv2raw_r1_bnormclamp1ent5_s{s}", "s2_r2d_pick_state_dDP_r1_bnormclamp1ent5_s{s}", range(4), S2, "picked",
     "repeat-1 pick (4 seeds per arm); same cells as pick"),
    ("place", "s2_r2d_place_state_dH_bnormclamp1ent5_s{s}", "s2_r2d_place_state_dDP_bnormclamp1ent5_n39_s{s}", range(8), PH, "placed_v2",
     "place scope, matched 39; polE = 148 shared policy-generated entries (OOD), holdE = 13 shared human entries (ID)"),
    ("contact", "s2_r2d_contact_state_dH_bnormclamp1ent5_subfloor_s{s}", "s2_r2d_contact_state_dDP_bnormclamp1ent5_n11_s{s}", range(8), PH, "contact",
     "contact after release, matched 11; polE = 160 shared entries (OOD), holdE = 11 (ID)"),
    ("carrycontact", "s2_r2d_carrycontact_state_dH_bnormclamp1ent5_s{s}", "s2_r2d_carrycontact_state_dDP_bnormclamp1ent5_n21_s{s}", range(8), PH, "contact",
     "carrycontact, matched 21; polE = 148 shared entries (OOD; entries that fail restore are skipped), holdE = 13 (ID)"),
    ("full", "full_r2d_state_dHfull_all_bnormclampS8ent5_s{s}", "full_r2d_state_dDPfull_bnormclampS8ent5_s{s}", range(8), S2, "nested",
     "end-to-end; success = nested; tiles carry the stages flags [p=picked c=contact n=nested]"),
]
CLASSES = [("both", "both-succeed"), ("H_only", "human-only"), ("M_only", "machine-only"), ("neither", "both-fail")]
PER_CLASS, T_CAP, OUT_FPS, DIM = 3, 150, 4.0, 0.6
FONT = cv2.FONT_HERSHEY_SIMPLEX


def load_cell(root, run, cell):
    p = f"{root}/src/{run}/{cell}/metrics.json"
    if not os.path.exists(p):
        return None
    m = json.load(open(p))
    return {e["ep"]: e for e in m["per_episode"]}


def start_key(e):
    if e.get("uid") is not None:
        return ("uid", e["uid"])
    ic = e.get("ic") or {}
    return ("ic", tuple(round(float(x), 6) for x in (ic.get("can_pos") or [])))


def ep_label(e, scope):
    lab = e["outcome"]
    if scope == "full" and e.get("stages"):
        st = e["stages"]
        flags = "".join(c for c, k in (("p", "picked"), ("c", "contact"), ("n", "nested")) if st.get(k))
        lab += f"[{flags or '-'}]"
    return lab


def joint(epsA, epsB, success):
    """Classify shared starts; returns dict class -> sorted list of starts, plus skipped list and mismatches."""
    out = {c: [] for c, _ in CLASSES}
    skipped, mismatch = [], []
    for k in sorted(set(epsA) & set(epsB)):
        a, b = epsA[k], epsB[k]
        if start_key(a) != start_key(b):
            mismatch.append(k)
            continue
        if a["outcome"] == "restore_failed" or b["outcome"] == "restore_failed" or not a.get("video") or not b.get("video"):
            skipped.append(k)
            continue
        sa, sb = a["outcome"] == success, b["outcome"] == success
        out["both" if sa and sb else "H_only" if sa else "M_only" if sb else "neither"].append(k)
    return out, skipped, mismatch


def median_seed(counts):
    """counts: {seed: n_success}. Lower median count, ties -> lowest seed."""
    med = sorted(counts.values())[(len(counts) - 1) // 2]
    return min(s for s, c in counts.items() if c == med)


def binom_two_sided(x, n):
    if n == 0:
        return float("nan")
    pl = sum(comb(n, i) for i in range(0, min(x, n - x) + 1)) / 2 ** n
    return min(1.0, 2 * pl)


def plan(root):
    sel, index = {}, []
    index.append("# Matched-start review reels — 2026-09-07\n")
    index.append("Arm H = human-demo policy, arm M = machine-demo policy; cells are the deterministic (mode) evals; runs under "
                 f"`{W_ABS}`. Built by `baselines/review_matched_reels.py` from `metrics.json` + the episode videos only "
                 "(no simulation). Numbers below come from `metrics.json` only.\n")
    index.append("**Selection rule.** Per arm and cell the *median* seed by that cell's success count (lower median for even n, "
                 "ties → lowest seed) so the reel shows a typical policy. Shared starts = episode index k (uid / can_pos checked "
                 "equal across arms); starts with `restore_failed` on either arm are skipped. Up to 3 starts per joint-outcome "
                 "class, sampled uniformly with `random.Random(0)` (one generator per comparison×cell, classes drawn in the "
                 "order both / human-only / machine-only / both-fail). Reels: 4 fps (half speed), cap 150 frames, ended clips dimmed.\n")
    index.append("**How to read the joint-outcome table.** `both` / `H-only` / `M-only` / `neither` count the shared starts by "
                 "(H outcome, M outcome). If the two arms were interchangeable policies, H-only ≈ M-only. The all-seed-pairs row "
                 "sums seed i vs seed i over all seeds; its units are start×seed-pair (starts repeat across seeds, so the exact "
                 "binomial p on the discordant counts is descriptive, not an independent-sample test).\n")
    vlist = []
    for name, pa, pb, seeds, cells, success, desc in COMPARISONS:
        index.append(f"\n## {name}\n\n{desc}. H = `{pa}`, M = `{pb}`, seeds {list(seeds)}, success = `{success}`.\n")
        sel[name] = {"success": success, "cells": {}}
        for ci, cell in enumerate(cells):
            kind = "OOD" if ci == 0 else "ID"
            A = {s: load_cell(root, pa.format(s=s), cell) for s in seeds}
            B = {s: load_cell(root, pb.format(s=s), cell) for s in seeds}
            missA = [s for s in seeds if A[s] is None]
            missB = [s for s in seeds if B[s] is None]
            index.append(f"### {cell} ({kind})\n")
            if missA or missB:
                index.append(f"MISSING metrics.json: H seeds {missA}, M seeds {missB} — these seeds are excluded below.\n")
            okA = {s: A[s] for s in seeds if A[s] is not None}
            okB = {s: B[s] for s in seeds if B[s] is not None}
            if not okA or not okB:
                index.append("Cell not built (an arm has no metrics).\n")
                continue
            cntA = {s: sum(e["outcome"] == success for e in okA[s].values()) for s in okA}
            cntB = {s: sum(e["outcome"] == success for e in okB[s].values()) for s in okB}
            sA, sB = median_seed(cntA), median_seed(cntB)
            nepA, nepB = len(okA[sA]), len(okB[sB])
            jc, skipped, mism = joint(okA[sA], okB[sB], success)
            n_shared = sum(len(v) for v in jc.values())
            # all seed pairs i vs i
            tot = {c: 0 for c, _ in CLASSES}
            pair_rows, tot_skipped = [], 0
            for s in seeds:
                if s in okA and s in okB:
                    j, sk, _ = joint(okA[s], okB[s], success)
                    for c in tot:
                        tot[c] += len(j[c])
                    tot_skipped += len(sk)
                    pair_rows.append(f"s{s}: {len(j['both'])}/{len(j['H_only'])}/{len(j['M_only'])}/{len(j['neither'])}")
            n_tot = sum(tot.values())
            rng = random.Random(0)
            picks = {c: rng.sample(sorted(jc[c]), min(PER_CLASS, len(jc[c]))) for c, _ in CLASSES}
            index.append(f"Per-seed successes — H: {', '.join(f's{s}={cntA[s]}/{len(okA[s])}' for s in okA)}; "
                         f"M: {', '.join(f's{s}={cntB[s]}/{len(okB[s])}' for s in okB)}.  ")
            index.append(f"**Chosen (median) seeds: H s{sA} ({cntA[sA]}/{nepA}), M s{sB} ({cntB[sB]}/{nepB}).**"
                         + (f" Skipped starts (restore_failed / no video) for this pair: {skipped}." if skipped else "")
                         + (f" START MISMATCH at episodes {mism} (excluded)." if mism else "") + "\n")
            index.append("| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |\n|---|---|---|---|---|---|---|")
            d = len(jc["H_only"]) + len(jc["M_only"])
            index.append(f"| chosen seeds (H s{sA} vs M s{sB}) | {len(jc['both'])} | {len(jc['H_only'])} | {len(jc['M_only'])} | "
                         f"{len(jc['neither'])} | {n_shared} | {binom_two_sided(len(jc['H_only']), d):.3f} |")
            dt = tot["H_only"] + tot["M_only"]
            index.append(f"| all seed pairs (i vs i, {len(pair_rows)} pairs) | {tot['both']} | {tot['H_only']} | {tot['M_only']} | "
                         f"{tot['neither']} | {n_tot} | {binom_two_sided(tot['H_only'], dt):.3f} |\n")
            index.append("Per pair both/H-only/M-only/neither: " + "; ".join(pair_rows)
                         + (f" (skipped over all pairs: {tot_skipped})" if tot_skipped else "") + ".\n")
            index.append("Sampled starts: " + "; ".join(f"{lab} {picks[c]}" for c, lab in CLASSES) + ".\n")
            reel = f"{name}_{cell}_matched.mp4"
            files = [f"review_matched/{reel}"] + ([f"review_matched/{name}_contact_sheet.png"] if kind == "OOD" else [])
            index.append("Files: " + ", ".join(f"`{f}`" for f in files) + "\n")
            if name == "place" and kind == "OOD":
                ctx = {s: load_cell(root, f"s2_r2d_place_state_dDP_bnormclamp1ent5_s{s}", cell) for s in range(8)}
                ctx = {s: v for s, v in ctx.items() if v}
                if ctx:
                    cr = {s: sum(e["outcome"] == success for e in v.values()) / len(v) for s, v in ctx.items()}
                    mH = sum(cntA[s] / len(okA[s]) for s in okA) / len(okA); mM = sum(cntB[s] / len(okB[s]) for s in okB) / len(okB)
                    index.append(f"Context (from the same `metrics.json` files): this reel uses the matched-39 machine arm "
                                 f"(`{pb}`), polE mean {mM:.3f} vs H {mH:.3f}. The UNCAPPED machine arm "
                                 f"`s2_r2d_place_state_dDP_bnormclamp1ent5_s{{0..7}}` (104 demos, not in this reel) scores "
                                 f"{sum(cr.values())/len(cr):.3f} on the same cell (per seed "
                                 f"{', '.join(f's{s}={cr[s]:.3f}' for s in sorted(cr))}) — that is the 0.709 the 09-05 status "
                                 "line quotes; the matched-39 arm is the one the brief names.\n")
            tiles = {}
            for c, _ in CLASSES:
                tiles[c] = []
                for k in picks[c]:
                    ea, eb = okA[sA][k], okB[sB][k]
                    ta = dict(arm="H", seed=sA, ep=k, label=ep_label(ea, name), steps=ea["steps"],
                              rel=os.path.relpath(ea["video"], W_ABS))
                    tb = dict(arm="M", seed=sB, ep=k, label=ep_label(eb, name), steps=eb["steps"],
                              rel=os.path.relpath(eb["video"], W_ABS))
                    tiles[c].append((ta, tb))
                    vlist += [ta["rel"], tb["rel"]]
            sel[name]["cells"][cell] = dict(kind=kind, seedH=sA, seedM=sB, counts={c: len(jc[c]) for c, _ in CLASSES},
                                            n=n_shared, all_pairs=tot, n_all=n_tot, tiles=tiles, reel=reel)
    os.makedirs(f"{root}/src", exist_ok=True)
    json.dump(sel, open(f"{root}/selection.json", "w"), indent=1)
    open(f"{root}/INDEX.md", "w").write("\n".join(index) + "\n")
    open(f"{root}/src/videos_list.txt", "w").write("\n".join(sorted(set(vlist))) + "\n")
    print(f"planned: {sum(len(v['cells']) for v in sel.values())} cells, {len(set(vlist))} clips -> {root}/selection.json, INDEX.md, src/videos_list.txt")


def fetch(root):
    cmd = ["rsync", "-a", f"--files-from={root}/src/videos_list.txt", W_REMOTE, f"{root}/src/"]
    print(" ".join(cmd)); subprocess.check_call(cmd)
    missing = [l.strip() for l in open(f"{root}/src/videos_list.txt") if l.strip() and not os.path.exists(f"{root}/src/{l.strip()}")]
    print(f"fetched; missing after rsync: {len(missing)}" + (f" {missing[:5]}" if missing else ""))


def n_frames(path):
    cap = cv2.VideoCapture(path); n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0); cap.release()
    return n


def caption(fr, text):
    fr = fr.copy()
    cv2.rectangle(fr, (0, 0), (fr.shape[1], 14), (0, 0, 0), -1)
    cv2.putText(fr, text, (2, 11), FONT, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
    return fr


def tile_text(t):
    return f"{t['arm']} s{t['seed']} ep{t['ep']} {t['label']} {t['steps']}"


def build_reel(root, name, cell, info, h264=True):
    src = f"{root}/src"
    # open every clip as a stream; keep the last frame for ended clips (memory stays at one frame per clip)
    blocks = []  # (header, [(capA, capB, ta, tb)])
    for c, lab in CLASSES:
        blocks.append((f"{lab} {info['counts'][c]}/{info['n']}", info["tiles"][c]))
    caps = {}
    lengths = []
    for _, tiles in blocks:
        for ta, tb in tiles:
            for t in (ta, tb):
                p = f"{src}/{t['rel']}"
                if not os.path.exists(p):
                    print(f"  missing clip {p}"); continue
                caps[t["rel"]] = [cv2.VideoCapture(p), None, 0]  # cap, last frame, frames read
                lengths.append(n_frames(p))
    if not caps:
        print(f"  {name} {cell}: no clips, skipped"); return None
    th, tw = 256, 512
    T = min(T_CAP, max(lengths)) if lengths else T_CAP
    hdr, top, empty_w = 18, 16, 176
    widths = [max(len(tiles), 0) * tw or empty_w for _, tiles in blocks]
    Wd = sum(widths); H = top + hdr + 2 * th
    Wd += Wd % 2; H += H % 2
    tmp = f"{root}/{info['reel']}.mp4v.mp4"; out = f"{root}/{info['reel']}"
    vw = cv2.VideoWriter(tmp, cv2.VideoWriter_fourcc(*"mp4v"), OUT_FPS, (Wd, H))
    title = f"{name} | {cell} ({info['kind']}) | H s{info['seedH']} (row 1) vs M s{info['seedM']} (row 2) | same start per column"
    for t in range(T):
        canvas = np.zeros((H, Wd, 3), np.uint8)
        cv2.putText(canvas, f"{title} | t={t}/{T}", (2, 12), FONT, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
        x0 = 0
        for (header, tiles), bw in zip(blocks, widths):
            cv2.rectangle(canvas, (x0, top), (x0 + bw - 1, top + hdr - 1), (40, 40, 40), -1)
            cv2.putText(canvas, header, (x0 + 3, top + 13), FONT, 0.42, (255, 255, 120), 1, cv2.LINE_AA)
            for k, (ta, tb) in enumerate(tiles):
                for r, ti in enumerate((ta, tb)):
                    st = caps.get(ti["rel"])
                    if st is None: continue
                    cap, last, nread = st
                    fr = None
                    if cap is not None:
                        ok, fr = cap.read()
                        if ok:
                            st[1] = fr; st[2] = nread + 1
                        else:
                            cap.release(); st[0] = None; fr = None
                    ended = fr is None
                    fr = st[1]
                    if fr is None: continue
                    if fr.shape[0] != th or fr.shape[1] != tw:
                        fr = cv2.resize(fr, (tw, th), interpolation=cv2.INTER_NEAREST)
                    fr = caption(fr, tile_text(ti))
                    if ended: fr = (fr * DIM).astype(np.uint8)
                    y0 = top + hdr + r * th
                    canvas[y0:y0 + th, x0 + k * tw:x0 + (k + 1) * tw] = fr
            if not tiles:
                cv2.putText(canvas, "(no starts)", (x0 + 6, top + hdr + 30), FONT, 0.4, (110, 110, 110), 1, cv2.LINE_AA)
            x0 += bw
        vw.write(canvas)
    vw.release()
    for st in caps.values():
        if st[0] is not None: st[0].release()
    if h264:
        subprocess.check_call(["ffmpeg", "-y", "-loglevel", "error", "-i", tmp, "-c:v", "libx264", "-pix_fmt", "yuv420p",
                               "-crf", "20", "-preset", "veryfast", "-movflags", "+faststart", out])
        os.remove(tmp)
    else:
        os.replace(tmp, out)
    print(f"  {out}: {Wd}x{H}, {T} frames @ {OUT_FPS} fps, {len(caps)} clips")
    return out


def read_all(path, cap_n=400):
    cap = cv2.VideoCapture(path); fr = []
    while len(fr) < cap_n:
        ok, im = cap.read()
        if not ok: break
        fr.append(im)
    cap.release(); return fr


def build_sheet(root, name, cell, info):
    src = f"{root}/src"; th, tw, lab_w, hdr = 256, 512, 150, 16
    rows = []
    for c, lab in CLASSES:
        for ta, tb in info["tiles"][c]:
            rows.append((lab, ta, tb))
    if not rows:
        return None
    Wd = lab_w + 6 * tw; H = hdr + len(rows) * (th + 2)
    canvas = np.full((H, Wd, 3), 20, np.uint8)
    cv2.putText(canvas, f"{name} | {cell} ({info['kind']}) | per row: H first / middle / last  |  M first / middle / last  (same start)",
                (2, 12), FONT, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
    for r, (lab, ta, tb) in enumerate(rows):
        y0 = hdr + r * (th + 2)
        cv2.putText(canvas, lab, (3, y0 + 30), FONT, 0.45, (255, 255, 120), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"ep{ta['ep']}", (3, y0 + 52), FONT, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
        for a, ti in enumerate((ta, tb)):
            frs = read_all(f"{src}/{ti['rel']}")
            if not frs: continue
            idx = [0, len(frs) // 2, len(frs) - 1]
            for j, i in enumerate(idx):
                fr = frs[i]
                if fr.shape[0] != th or fr.shape[1] != tw:
                    fr = cv2.resize(fr, (tw, th), interpolation=cv2.INTER_NEAREST)
                fr = caption(fr, f"{tile_text(ti)} | f{i}/{len(frs)}")
                x0 = lab_w + (a * 3 + j) * tw
                canvas[y0:y0 + th, x0:x0 + tw] = fr
    out = f"{root}/{name}_contact_sheet.png"
    cv2.imwrite(out, canvas)
    print(f"  {out}: {Wd}x{H}, {len(rows)} rows")
    return out


def build(root, only=None, h264=True):
    sel = json.load(open(f"{root}/selection.json"))
    for name, comp in sel.items():
        if only and name not in only: continue
        print(name)
        for cell, info in comp["cells"].items():
            build_reel(root, name, cell, info, h264)
            if info["kind"] == "OOD":
                build_sheet(root, name, cell, info)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["plan", "fetch", "build"])
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--only", default=None, help="comma-separated comparison names (build only)")
    ap.add_argument("--no-h264", action="store_true", help="keep the raw mp4v writer output (no ffmpeg transcode)")
    a = ap.parse_args()
    os.makedirs(a.root, exist_ok=True)
    if a.cmd == "plan": plan(a.root)
    elif a.cmd == "fetch": fetch(a.root)
    else: build(a.root, set(a.only.split(",")) if a.only else None, not a.no_h264)
