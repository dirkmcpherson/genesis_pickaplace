#!/bin/bash
# PHASE evaluation cells (place | contact) for one finished RLPD or DP run (PHASE_PLAN amendments (h)/(m), 2026-09-07); the re-runnable
# eval stage of cluster/sbatch_{rlpd,dp}_place.sh (in-job) and the post-hoc path (e.g. once the rebuilt polE bank lands).
#   cells: fresh_eval_{holdE,polE}_{sample,mode}/metrics.json under $OUT (r2dreamer layout: phase_table.py reads them)
#   sac: MODES="sample mode"; dp: MODES="sample" (no deterministic mode for a diffusion policy)
# Usage (env):
#   KIND=sac|dp CKPT=<rlpd_final.zip | .../pretrained_model> OUT=<run dir> ARM=<dH|dDP> SEED=<n> \
#   [HOLDE=$W/phase_banks/holdE_place.json] [POLE=$W/phase_banks/polE_place.json] [MODES="sample mode"] \
#   [SIM_VARIANT=gc_kp4_riser3_shelf6] [EVAL_SEED=0] [VIDEO=1] [REDO=0] [PAR=4] bash cluster/place_eval_cells.sh
# Rules: a cell whose metrics.json exists is kept (resume-safe; REDO=1 wipes the cells first); the polE cells are run
# ONLY when the bank carries `bank_version` (adversarial review 2026-09-07 S2-4: the raw dumps hold normalised grips) --
# otherwise they are DEFERRED (exit 0, line POLE-DEFERRED) and this script is re-run post hoc. eval_place.py itself
# refuses a raw-grip bank, so a stale bank can never produce a number.
set -uo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
W=${W:-/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03}
: "${KIND:?KIND sac|dp}"; : "${CKPT:?CKPT}"; : "${OUT:?OUT}"; : "${ARM:?ARM}"; : "${SEED:?SEED}"
PHASE=${PHASE:-place}
case "$PHASE" in place|contact) ;; *) echo "FATAL: PHASE must be place|contact (got $PHASE)"; exit 1 ;; esac
# polE banks: the eval-fixes agent's REBUILT physical-grip banks (bank_version=physgrip_2026-09-07) are the
# *_physgrip.json files; the canonical polE_*.json names currently hold the raw-grip originals (checked 2026-09-07
# 21:00: polE_place/polE_place_dDP/polE_contact all have no bank_version), and eval_place.py refuses those by design.
HOLDE=${HOLDE:-$W/phase_banks/holdE_${PHASE}.json}
POLE=${POLE:-$W/phase_banks/polE_$([ "$PHASE" = place ] && echo place || echo contact)_physgrip.json}
SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}; EVAL_SEED=${EVAL_SEED:-0}; VIDEO=${VIDEO:-1}; PAR=${PAR:-4}
if [ -z "${MODES:-}" ]; then MODES="sample mode"; [ "$KIND" = dp ] && MODES="sample"; fi
[ "$KIND" = dp ] && case " $MODES " in *" mode "*) echo "FATAL: dp has no mode cell"; exit 1;; esac
if [ "$KIND" = sac ]; then [ -f "$CKPT" ] || { echo "FATAL: checkpoint $CKPT missing"; exit 1; }; else [ -d "$CKPT" ] || { echo "FATAL: DP checkpoint dir $CKPT missing"; exit 1; }; fi
[ -s "$HOLDE" ] || { echo "FATAL: holdE bank $HOLDE missing"; exit 1; }
POLE_OK=0
if [ -s "$POLE" ]; then
  python3 - "$POLE" <<'PY' && POLE_OK=1
import json, sys
b = json.load(open(sys.argv[1])); ents = b['entries'] if isinstance(b, dict) and 'entries' in b else (list(b.values()) if isinstance(b, dict) else b)
bv = (b.get('bank_version') if isinstance(b, dict) else None) or (ents[0].get('bank_version') if ents and isinstance(ents[0], dict) else None)
bad = sum(1 for e in ents if not (0.0 <= float(e['grip_cmd']) <= 1.0))
if bv is None or bad:
    print(f'POLE-DEFERRED {sys.argv[1]}: bank_version={bv!r}, {bad}/{len(ents)} entries with grip outside [0,1] -- the rebuilt physical-grip bank is required (review 2026-09-07 S2-4); polE cells deferred to a post-hoc run'); sys.exit(1)
print(f'POLE-OK {sys.argv[1]}: bank_version={bv!r}, {len(ents)} entries, grips in [0,1]')
PY
else
  echo "POLE-DEFERRED: bank $POLE absent"
fi
# SLIDE FIX GATE (2026-09-07): the settle-gate fix a7de6a0 ships the per-clause `slide_fail_reason` diagnostics; a
# contact cell without them is not reportable (coordinator). Refuse EARLY and cleanly so a job that reaches its eval
# stage before the fix is live simply leaves the cells missing -- place_readout.sh re-runs them afterwards.
if [ "$PHASE" = contact ] && [ -z "${SLIDE_DIAG_OPTIONAL:-}" ]; then
  if ! grep -q "slide_fail_reason" baselines/genesis_can_env.py 2>/dev/null; then
    echo "SLIDE-FIX-MISSING: $(git rev-parse --short HEAD 2>/dev/null) predates a7de6a0 (no slide_fail_reason in baselines/genesis_can_env.py)."
    echo "  Contact cells are NOT written; re-run them with 'PHASE=contact bash cluster/place_readout.sh' once the fixed code is synced."
    exit 0
  fi
  [ -z "${CONTACT_GRANT:-}" ] && { echo "SLIDE-ON-HOLD: contact cells need CONTACT_GRANT (PHASE_PLAN (p)); no cells written."; exit 0; }
  echo "SLIDE-FIX-OK: slide_fail_reason present (git $(git rev-parse --short HEAD 2>/dev/null))"
fi
VF=(); [ "$VIDEO" = 1 ] && VF=(--video)
[ "${REDO:-0}" = 1 ] && rm -rf "$OUT"/fresh_eval_{holdE,polE}_{sample,mode}
echo "== place_eval_cells phase=$PHASE kind=$KIND ckpt=$CKPT out=$OUT arm=$ARM seed=$SEED modes='$MODES' holdE=$HOLDE polE=$POLE(ok=$POLE_OK) variant=$SIM_VARIANT eval_seed=$EVAL_SEED par=$PAR $(date)"
CELLS=()
for MODE in $MODES; do
  CELLS+=("holdE|$HOLDE|$MODE")
  [ "$POLE_OK" = 1 ] && CELLS+=("polE|$POLE|$MODE")
done
run_cell() {
  local SET=$1 BANK=$2 MODE=$3 D="$OUT/fresh_eval_${SET}_${MODE}"
  if [ -f "$D/metrics.json" ]; then echo "# cell $SET $MODE exists ($D/metrics.json), kept"; return 0; fi
  mkdir -p "$D"
  python baselines/eval_place.py --kind "$KIND" --checkpoint "$CKPT" --entry-bank "$BANK" --out "$D" --mode "$MODE" --seed "$EVAL_SEED" \
      --scope "$PHASE" ${CONTACT_GRANT:+--contact-grant "$CONTACT_GRANT"} --max-steps 600 --sim-variant "$SIM_VARIANT" --arm "$ARM" --tag "${SET}_${MODE}" "${VF[@]}" > "$D/eval.log" 2>&1
  local rc=$?; echo "# cell $SET $MODE rc=$rc $(date -Is)"; grep -E "^\[eval-place\] [0-9]+ episodes|FATAL|Traceback|Error" "$D/eval.log" | tail -3
  return $rc
}
for C in "${CELLS[@]}"; do
  IFS='|' read -r SET BANK MODE <<< "$C"
  while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 5; done
  run_cell "$SET" "$BANK" "$MODE" &
done
wait
python3 - "$OUT" "$KIND" "$ARM" "$SEED" "$PHASE" <<'PY'
import json, os, sys
out, kind, arm, seed, phase = sys.argv[1:6]
key = 'placed_v2' if phase == 'place' else 'slide_success'
parts = []; _loaded = []
for s in ('holdE', 'polE'):
    for m in ('sample', 'mode'):
        f = os.path.join(out, f'fresh_eval_{s}_{m}', 'metrics.json')
        if os.path.exists(f):
            d = json.load(open(f)); _loaded.append(d); n = int(d['episodes']); k = int(round(float(d[key]) * n)); rf = int(round(float(d.get('restore_failed', 0.0)) * n))
            extra = ''
            if phase == 'contact':
                extra = '[c%d]' % int(round(float(d.get('contact', 0.0)) * n))
            parts.append(f'{s}_{"S" if m == "sample" else "M"}={k}/{n}{extra}' + (f'(rf{rf})' if rf else ''))
        else:
            parts.append(f'{s}_{"S" if m == "sample" else "M"}=—')
banks = sorted({(os.path.basename(d.get('bank_path') or ''), (d.get('bank_sha256') or '')[:12], d.get('bank_version'))
                for d in _loaded})
bstr = ' '.join(f'{b}@{sha}/{bv}' for b, sha, bv in banks if b)
line = (f'{phase.upper()}-HEADLINE learner={kind} arm={arm} seed={seed} key={key} ' + ' '.join(parts)
        + (f' banks={bstr}' if bstr else '') + f' out={out}')
print(line); open(os.path.join(out, f'{phase.upper()}_HEADLINE.txt'), 'w').write(line + '\n')
PY
echo "== place_eval_cells done $(date)"
