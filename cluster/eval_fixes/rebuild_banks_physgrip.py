#!/usr/bin/env python3
"""Rebuild the policy-generated entry banks with PHYSICAL grip units (PHASE_PLAN amendment (j); ADVERSARIAL_REVIEW_eval_env
2026-09-07 S2-4). `eval_genesis.py --dump-entries` stored grip_cmd = the raw policy action a[6] in [-1, 1]; the restore
(`FullTaskEnv._restore_place_entry` / `_restore_contact_entry`) reads grip_cmd as physical 0..1 (`gripper_targets(cmd*100)`,
clipped). The adapter's own map is grip_phys = (clip(a, -1, 1) + 1) / 2 (envs/genesis.py GenesisPick.grip_phys; identical to
pick_env.denormalize_action), so every stored value is converted with exactly that map. The merged banks are verbatim
copies of the per-checkpoint dump JSONs (polE_place: the union of polE_place_from_dHv2raw_s*.json; polE_place_dDP: the union
of polE_place_from_dDP_s*.json; polE_contact: 10 per source of polE_contact_from_*.json, rng seed 0) -- this script
re-derives each merged entry from its dump entry (same source checkpoint, ic_index, frame, qpos) and asserts the stored
grip_cmd matches before converting, so the rebuilt bank is "from the existing dump JSONs" in the literal sense.
Schema unchanged (dict pseudo-uid -> entry; FullTaskEnv + eval_place.py both read it); per entry: grip_cmd (physical),
grip_cmd_raw (the stored raw action), grip_units='physical01', bank_version='physgrip_2026-09-07', converted_from.
usage: rebuild_banks_physgrip.py <phase_banks dir> [--out-suffix _physgrip_staging] [--write]"""
import argparse, glob, json, os, sys
import numpy as np

BANK_VERSION = "physgrip_2026-09-07"


def grip_phys(a):
    return (float(np.clip(float(a), -1.0, 1.0)) + 1.0) / 2.0


def dump_index(files):
    idx = {}
    for f in files:
        for e in json.load(open(f)):
            key = (os.path.basename(os.path.dirname(e["source"])), int(e["ic_index"]), int(e["frame"]))
            idx.setdefault(key, []).append((f, e))
    return idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bank_dir"); ap.add_argument("--out-suffix", default="_physgrip_staging"); ap.add_argument("--write", action="store_true")
    a = ap.parse_args(); B = a.bank_dir
    banks = {"polE_place": "polE_place_from_dHv2raw_s*.json", "polE_place_dDP": "polE_place_from_dDP_s*.json",
             "polE_contact": "polE_contact_from_*.json"}
    report = {}
    for name, pat in banks.items():
        src = os.path.join(B, f"{name}_rawgrip.json") if os.path.exists(os.path.join(B, f"{name}_rawgrip.json")) else os.path.join(B, f"{name}.json")
        raw = json.load(open(src)); assert isinstance(raw, dict), src
        idx = dump_index(sorted(glob.glob(os.path.join(B, pat))))
        out, n_changed, n_neg, n_cmd_lt_obs_before, n_cmd_lt_obs_after, n_released_before, n_released_after = {}, 0, 0, 0, 0, 0, 0
        n_matched_dump = 0
        for uid, e in raw.items():
            e = dict(e)
            assert "grip_cmd_raw" not in e and e.get("bank_version") is None, (name, uid, "already converted")
            key = (os.path.basename(os.path.dirname(e["source"])), int(e["ic_index"]), int(e["frame"]))
            cands = [d for f, d in idx.get(key, []) if np.allclose(d["qpos"], e["qpos"]) and abs(float(d["grip_cmd"]) - float(e["grip_cmd"])) < 1e-12]
            assert cands, (name, uid, key, "no matching dump entry")
            n_matched_dump += 1
            g_raw = float(e["grip_cmd"]); g_phys = grip_phys(g_raw)
            n_changed += int(abs(g_phys - g_raw) > 1e-12); n_neg += int(g_raw < 0.0)
            n_cmd_lt_obs_before += int(float(np.clip(g_raw, 0.0, 1.0)) < float(e["grip_obs"]))   # what the restore actually commanded vs the measured closure
            n_cmd_lt_obs_after += int(g_phys < float(e["grip_obs"]))
            n_released_before += int(float(np.clip(g_raw, 0.0, 1.0)) < 0.45); n_released_after += int(g_phys < 0.45)
            e.update(grip_cmd=g_phys, grip_cmd_raw=g_raw, grip_units="physical01", bank_version=BANK_VERSION,
                     converted_from=os.path.basename(src))
            out[uid] = e
        rep = dict(source=os.path.basename(src), n=len(out), n_matched_dump=n_matched_dump, n_changed=n_changed, n_raw_negative=n_neg,
                   grip_cmd_range_raw=[min(float(e["grip_cmd_raw"]) for e in out.values()), max(float(e["grip_cmd_raw"]) for e in out.values())],
                   grip_cmd_range_phys=[min(float(e["grip_cmd"]) for e in out.values()), max(float(e["grip_cmd"]) for e in out.values())],
                   mean_phys=float(np.mean([e["grip_cmd"] for e in out.values()])),
                   n_cmd_below_obs_before=n_cmd_lt_obs_before, n_cmd_below_obs_after=n_cmd_lt_obs_after,
                   n_released_lt045_before=n_released_before, n_released_lt045_after=n_released_after)
        report[name] = rep
        print(f"== {name}: {json.dumps(rep)}")
        if a.write:
            dst = os.path.join(B, f"{name}{a.out_suffix}.json")
            json.dump(out, open(dst, "w"), indent=1); print(f"   wrote {dst} ({len(out)} entries)")
    if a.write:
        json.dump(report, open(os.path.join(B, f"rebuild_report{a.out_suffix}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
