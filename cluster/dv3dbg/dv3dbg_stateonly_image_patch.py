#!/usr/bin/env python3
"""DV3 DEBUG G2 (paper/DV3_DEBUG_2026-09-05.md): patch a dreamerv3-torch tree so STATE-ONLY genesis runs can mix the
state demo sets (placeholder image (T,64,64,6) from to_dreamer_native.py --with-state) with online episodes (adapter
vector mode emits (64,64,3) zeros). tools.sample_episodes appends chunks across episodes and from_generator stacks the
batch, so the channel mismatch raises at the first batch. The encoder never reads the image (cnn_keys '$^'), so the
demo placeholder is replaced by zeros of the ONLINE shape. usage: dv3dbg_stateonly_image_patch.py <tree>/dreamer.py"""
import sys
p = sys.argv[1]; s = open(p).read()
old = '''            img = ep.get("image")
            assert img is not None, f"Demo episode {epkey} missing 'image' key"
'''
new = old + '''            # DV3 DEBUG G2 (2026-09-05, paper/DV3_DEBUG_2026-09-05.md): state-only genesis runs -- the state demo
            # sets carry a (T,64,64,6) zero placeholder while the adapter in vector mode emits (64,64,3) zeros;
            # sample_episodes/from_generator would raise on the channel mismatch. Encoder cnn_keys is '$^' here.
            if (str(config.task).startswith("genesis") and not getattr(config, "genesis_pixels", False)
                    and img.shape[-1] != 3):
                if not globals().get("_DV3DBG_IMG_NOTE"):
                    print(f"[state-only] demo image placeholder {tuple(img.shape[1:])} -> "
                          f"({config.size[0]},{config.size[1]},3) zeros (encoder cnn_keys={config.encoder['cnn_keys']!r})")
                    globals()["_DV3DBG_IMG_NOTE"] = True
                ep["image"] = np.zeros((img.shape[0], config.size[0], config.size[1], 3), np.uint8)
                img = ep["image"]
'''
if "[state-only] demo image placeholder" in s:
    print("already patched"); sys.exit(0)
assert s.count(old) == 1, s.count(old)
open(p, "w").write(s.replace(old, new)); print("patched", p)
