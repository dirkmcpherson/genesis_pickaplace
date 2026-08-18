# Figure style rule (standing, user directive 2026-08-18)

Every d{src}_{algo} figure in this project encodes:
- **marker SHAPE = demo SOURCE**: dH = circle `o`, dDP = square `s`, dR2D = triangle `^`
- **COLOR = ALGORITHM**: DP `#1b7837`, RLPD `#4878cf`, r2dreamer `#cb181d`, dv3 `#8e44ad`

Fixed forever; the mapping lives in one dict (`SRC_MARKER`, `ALG_COLOR`) at the
top of `analysis/make_ignition_figs_20260818.py` and any successor script must
import or copy it verbatim. Why: the user tracks the whole matrix from these
figures — one visual grammar across all of them means a reader never re-learns
a legend. Secondary encodings (open marker = exactly 0.00; thick edge = in-loop
eval fallback; dashes = wave) never reuse shape or hue.
