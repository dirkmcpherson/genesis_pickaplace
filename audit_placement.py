"""Audit how the cans (bottle + goal_bottle) are placed for a given episode.

Run:  .venv/bin/python audit_placement.py [UID]
"""
import sys
import pathlib as pl
import numpy as np

# --- mirror the constants the simulator uses (example.py) ---
BOTTLE_RADIUS = 0.035
BOTTLE_HEIGHT = 0.075
BOX_WIDTH, BOX_HEIGHT = 0.75, 0.12
BOX_POS = (0.75, -BOX_WIDTH / 4, 0.05)
BOX_SIZE = (0.4, BOX_WIDTH, BOX_HEIGHT)

STATIC_BOTTLE_POSITION = (0.6, -0.2, 0.19)   # goal_bottle, fixed every episode
POSITION_0 = (0.4381, 0.1, 0.05)
POSITION_1 = (0.4381, -0.05, 0.05)
POSITION_2 = (0.4381, -0.2, 0.05)
POS = {0: POSITION_0, 1: POSITION_1, 2: POSITION_2}

# --- the raw lists, straight from kinova.py / trial_reader.py ---
p0_succ = [232, 235, 242, 245, 248, 251, 254, 257, 261, 265, 269, 273, 276, 279, 283, 293, 297, 301, 304, 308, 315, 316, 319, 320, 325, 328, 331, 335]
p0_fail = [238, 249, 260, 268, 282, 288, 307, 312, 324, 331, 334]
p1_succ = [233, 236, 239, 243, 246, 252, 255, 258, 262, 266, 274, 277, 280, 284, 287, 294, 298, 302, 305, 309, 317, 321, 326, 329, 322]
p1_fail = [270, 313]
p2_succ = [327, 330, 333, 300, 303, 306, 311, 318, 278, 281, 286, 290, 295, 299, 256, 259, 263, 267, 275, 234, 237, 244, 247, 250]
p2_fail = [240, 253, 285, 289, 296, 310, 314, 322]

# what example.py ACTUALLY uses (failed lists commented out):
ACTIVE = {0: p0_succ, 1: p1_succ, 2: p2_succ}


def classify(uid, lists):
    """Replicate example.py's if/elif/elif/else dispatch order: 0 wins, then 1, then 2."""
    for pos in (0, 1, 2):
        if uid in lists[pos]:
            return pos
    return None


def z_drop(pos_z, surface_z):
    """How far the can free-falls before resting (negative => starts embedded)."""
    rest_center = surface_z + BOTTLE_HEIGHT / 2
    return pos_z - rest_center


def report_episode(uid):
    print(f"\n================ EPISODE {uid} ================")
    # 1. where does the episode file say the can is? (spoiler: it doesn't)
    f = pl.Path("inthewild_trials") / f"{uid}_episodes.npy"
    if f.exists():
        d = np.load(f, allow_pickle=True).item()
        print(f"episode file keys : {list(d.keys())}")
        for k, v in d.items():
            arr = np.asarray(v)
            print(f"   {k:12s} shape={arr.shape} dtype={arr.dtype}")
        has_obj = any("bottle" in k or "obj" in k or "pose" in k or "pos" in k.replace("gripper_pos", "")
                      for k in d.keys())
        print(f"   -> contains any can/object position?  {has_obj}")
    else:
        print(f"episode file MISSING: {f}")

    # 2. where does example.py place it?
    pos_idx = classify(uid, ACTIVE)
    if pos_idx is None:
        print("classify(ACTIVE) -> None  => setup() returns False, episode SKIPPED, can never placed")
        return
    can = POS[pos_idx]
    print(f"classify(ACTIVE)  -> POSITION_{pos_idx} = {can}")
    print(f"   bottle placed at {can}, quat reset to [1,0,0,0]")
    print(f"   goal_bottle placed at {STATIC_BOTTLE_POSITION} (same every episode)")

    # 3. geometry sanity for the placement
    plane_drop = z_drop(can[2], 0.0)
    print(f"   bottle z-drop onto plane (z=0): {plane_drop:+.4f} m "
          f"({'starts above, will settle' if plane_drop >= 0 else 'STARTS EMBEDDED in ground'})")
    box_top = BOX_POS[2] + BOX_SIZE[2] / 2
    goal_drop = z_drop(STATIC_BOTTLE_POSITION[2], box_top)
    print(f"   goal_bottle z-drop onto box top (z={box_top:.3f}): {goal_drop:+.4f} m")
    # is the moving bottle clear of the box in x?
    box_x_min = BOX_POS[0] - BOX_SIZE[0] / 2
    print(f"   bottle x={can[0]:.4f} vs box front face x={box_x_min:.4f}: "
          f"{'clear of box' if can[0] + BOTTLE_RADIUS < box_x_min else 'OVERLAPS box footprint'}")


def integrity_audit():
    print("================ MAPPING INTEGRITY AUDIT ================")
    all_lists = {
        "p0_succ": p0_succ, "p0_fail": p0_fail,
        "p1_succ": p1_succ, "p1_fail": p1_fail,
        "p2_succ": p2_succ, "p2_fail": p2_fail,
    }
    # duplicates within a single list
    for name, lst in all_lists.items():
        dups = sorted({x for x in lst if lst.count(x) > 1})
        if dups:
            print(f"[DUP within {name}] {dups}")

    # same uid in >1 list (cross-bucket) — split active vs failed
    from collections import defaultdict
    where = defaultdict(list)
    for name, lst in all_lists.items():
        for uid in set(lst):
            where[uid].append(name)
    print("\n-- UIDs appearing in more than one bucket --")
    for uid in sorted(where):
        if len(where[uid]) > 1:
            print(f"   {uid}: {where[uid]}")

    # ambiguity that actually changes placement under ACTIVE (succ-only) lists
    print("\n-- ACTIVE (succ-only) cross-position collisions (change placement!) --")
    where_active = defaultdict(list)
    for pos, lst in ACTIVE.items():
        for uid in set(lst):
            where_active[uid].append(pos)
    any_active = False
    for uid in sorted(where_active):
        if len(set(where_active[uid])) > 1:
            any_active = True
            chosen = classify(uid, ACTIVE)
            print(f"   {uid}: in positions {sorted(set(where_active[uid]))} -> "
                  f"if/elif picks POSITION_{chosen}")
    if not any_active:
        print("   none — every UID maps to a single position under the active lists")

    # what if the failed lists were re-enabled (the commented '# + ..._failed')?
    print("\n-- IF failed lists re-enabled: cross-position collisions --")
    full = {0: p0_succ + p0_fail, 1: p1_succ + p1_fail, 2: p2_succ + p2_fail}
    where_full = defaultdict(list)
    for pos, lst in full.items():
        for uid in set(lst):
            where_full[uid].append(pos)
    for uid in sorted(where_full):
        if len(set(where_full[uid])) > 1:
            chosen = classify(uid, full)
            print(f"   {uid}: in positions {sorted(set(where_full[uid]))} -> would pick POSITION_{chosen}")

    # episodes on disk vs episodes that get a placement
    disk = sorted(int(p.name.split("_")[0]) for p in pl.Path("inthewild_trials").glob("*_episodes.npy"))
    placed = {uid for lst in ACTIVE.values() for uid in lst}
    on_disk_not_placed = [u for u in disk if classify(u, ACTIVE) is None]
    placed_not_on_disk = sorted(placed - set(disk))
    print(f"\n-- coverage --")
    print(f"   episode files on disk : {len(disk)}")
    print(f"   UIDs with a placement : {len(placed)}")
    print(f"   on disk but NOT placed (setup returns False, skipped): {on_disk_not_placed}")
    print(f"   placed but NO file on disk (never runs): {placed_not_on_disk}")


if __name__ == "__main__":
    integrity_audit()
    uids = [int(a) for a in sys.argv[1:]] or [235]  # default: the debug episode
    for uid in uids:
        report_episode(uid)
