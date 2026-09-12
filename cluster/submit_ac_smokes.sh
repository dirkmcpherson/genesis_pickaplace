#!/usr/bin/env bash
# The two nested_sparse10 smokes (PHASE_PLAN amendment (ac)), one per learner, on the human
# `_rns10h` set. Reuses submit_ln_smokes.sh's smoke_rl / smoke_r2 VERBATIM (sourced with
# ONLY=none so its own (aa) submissions do not fire) -- same queues, same budgets, same
# preflight -- so the only thing that differs from the (aa) smokes is the ladder and the set.
#
#   GP=$LAB/gp_ac bash cluster/submit_ac_smokes.sh          # GP must NOT be the pinned gp_ladderN
#   DRYRUN=1 GP=... bash cluster/submit_ac_smokes.sh
#
# Pass criteria (from each job's own stdout, per the coordinator 2026-09-12):
#   [ladder] ... ladder=nested_sparse10 ... home=10 | max_return=10 | terminal=home+tipped ... tip=tilt>60deg&not_in_hand@4f
#   {r2dreamer}: `return_clamp=10.0 (env and model agree)`
#   {RLPD}: the demo gate accepting the _rns10h set (set-ladder == run-ladder)
#   a ladder_provenance.json in the logdir with max_return / return_clamp_required 10.0
set -euo pipefail
: "${GP:?set GP to a FRESH clone at 5d92a09 or later; never the pinned gp_ladderN}"
case "$GP" in *gp_ladderN*) echo "FATAL: refusing the pinned tree $GP"; exit 1;; esac
ONLY=none source "$(dirname "$0")/submit_ln_smokes.sh"
[ -f "$DEMOS/dHfull_all_rns10h/repeat.json" ] || { echo "FATAL: $DEMOS/dHfull_all_rns10h missing (rsync it from the build box)"; exit 1; }
grep -q "'nested_sparse10'" "$GP/baselines/rl/full_env.py" || { echo "FATAL: $GP has no nested_sparse10 entry -- clone is older than 5d92a09"; exit 1; }
smoke_rl nested_sparse10 dHfull_all_rns10h 9984 sparse10
smoke_r2 nested_sparse10 dHfull_all_rns10h 9984 sparse10
