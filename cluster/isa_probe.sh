#!/bin/bash
# Build the REAL node -> CPU-model / instruction-set-class map (PHASE_PLAN amendment (n)/(s), 2026-09-07).
#
# Why this exists: long-horizon full-scope episodes diverge between AVX2 and AVX-512 parts (same checkpoint, IC,
# mode, seed and horizon: Broadwell E5-2695 v4 reproduces the record bit-for-bit, Cascade Lake and Sapphire Rapids
# do not, at every thread pinning tried), and **Slurm's feature labels are unreliable for this** -- pax001 advertises
# `AvailableFeatures=broadwell` and is a Cascade Lake part. So `--constraint=broadwell` does NOT guarantee a
# reproducing node. The only trustworthy source is /proc/cpuinfo ON the node, which is what this reads. Pin with an
# explicit `--nodelist` built from this map, never with `--constraint`; and keep REQUIRE_ISA set so the job also
# fails loudly if it lands somewhere else anyway (belt and braces -- the label lied once already).
#
# usage:
#   bash cluster/isa_probe.sh                       # every node of $PART that is idle or mixed
#   bash cluster/isa_probe.sh pax109 pax154 pax001  # just these
#   PART=preempt GRES=gpu OUT=isa_map.json bash cluster/isa_probe.sh
# Output: one line per node `<node> <isa> avx512f=<0|1> <model string>`, plus a JSON map when OUT is set.
set -uo pipefail
PART=${PART:-preempt}
TIMEOUT=${TIMEOUT:-60}
if [ "$#" -gt 0 ]; then NODES="$*"
else NODES=$(sinfo -h -p "$PART" -t idle,mix -o "%N" | paste -sd, - | xargs -r scontrol show hostnames | tr '\n' ' '); fi
[ -n "${GRES:-}" ] && NODES=$(for n in $NODES; do scontrol show node "$n" 2>/dev/null | grep -q "Gres=.*${GRES}" && echo "$n"; done | tr '\n' ' ')
echo "== isa_probe: ${PART} -> $(echo "$NODES" | wc -w) node(s) $(date -Is)"
TMP=$(mktemp); : > "$TMP"
for N in $NODES; do
  L=$(timeout "$TIMEOUT" srun -p "$PART" --qos=preempt --overlap -w "$N" -n1 -c1 --mem=100M --time=00:01:00 \
        bash -c 'M=$(awk -F": " "/^model name/{print \$2; exit}" /proc/cpuinfo);
                 F=$(awk "/^flags/{print; exit}" /proc/cpuinfo);
                 case "$F" in *avx512f*) I=avx512; A=1;; *avx2*) I=avx2; A=0;; *) I=pre-avx2; A=0;; esac
                 echo "$(hostname) $I avx512f=$A $M"' 2>/dev/null | tail -1)
  if [ -n "$L" ]; then echo "$L" | tee -a "$TMP"; else echo "$N UNREACHABLE (busy/drained/timeout)"; fi
done
if [ -n "${OUT:-}" ]; then
  python3 - "$TMP" "$OUT" <<'PY'
import json, sys
m = {}
for line in open(sys.argv[1]):
    p = line.split()
    if len(p) >= 4:
        m[p[0]] = dict(isa=p[1], avx512f=p[2].endswith('1'), model=' '.join(p[3:]))
json.dump(m, open(sys.argv[2], 'w'), indent=1)
byisa = {}
for n, d in m.items():
    byisa.setdefault(d['isa'], []).append(n)
print(f'wrote {sys.argv[2]}: ' + '; '.join(f'{k} n={len(v)} ({",".join(sorted(v)[:6])}{"..." if len(v) > 6 else ""})'
                                          for k, v in sorted(byisa.items())))
print('nodelist for a pinned AVX2 re-score: --nodelist=' + ','.join(sorted(byisa.get('avx2', []))))
PY
fi
rm -f "$TMP"
echo "== isa_probe done $(date -Is)"
