#!/bin/bash
# Build the REAL node -> machine-size map: physical cores, sockets, logical CPUs, GPU and CPU model, read from
# /proc/cpuinfo ON each node (PHASE_PLAN amendment (n)/(s)/(t), 2026-09-07).
#
# Why this exists: long-horizon full-scope episodes diverge with MACHINE SIZE -- same checkpoint, IC, mode, seed and
# horizon, different physical core count, different outcome. Established on processor counts, which are reliable:
# 53 of 53 same-core-count comparisons bit-identical across nodes, labels and code versions, and every one of the 19
# disagreements with a 36-core machine on exactly one side (coordinator verdict 2026-09-07: `cores`). The
# instruction-set question is UNRESOLVED rather than ruled out: the CPU-family labels behind both the original AVX
# claim and its withdrawal came from Slurm's AvailableFeatures, which are wrong here. ISA is printed as a diagnostic
# only, so that a future re-check of families against /proc/cpuinfo remains possible.
#
# **Slurm's feature labels are unreliable** (pax001 advertises `AvailableFeatures=broadwell` and is a Cascade Lake
# part), so nothing here trusts them. But note the good news: a core/thread pin needs no `--nodelist` at all --
# eval_e2e.py --require-cores / --threads checks /proc/cpuinfo on arrival and dies if the machine is the wrong size,
# so any machine of the right size will do. This map is for choosing WHICH size to standardise on and for knowing
# how many machines of that size exist.
#
# CAUTION: run with a modest PAR. PAR=30 saturated the login node on 2026-09-07 and made ssh itself time out.
#
# usage:
#   bash cluster/hw_probe.sh                       # every node of $PART that is idle or mixed
#   bash cluster/hw_probe.sh pax109 pax154 pax001  # just these
#   PART=preempt GRES=gpu OUT=hw_map.json bash cluster/hw_probe.sh
# Output: one line per node `<node> <isa> avx512f=<0|1> <model string>`, plus a JSON map when OUT is set.
set -uo pipefail
PART=${PART:-preempt}
TIMEOUT=${TIMEOUT:-60}
if [ "$#" -gt 0 ]; then NODES="$*"
else NODES=$(sinfo -h -p "$PART" -t idle,mix -o "%N" | paste -sd, - | xargs -r scontrol show hostnames | tr '\n' ' '); fi
[ -n "${GRES:-}" ] && NODES=$(for n in $NODES; do scontrol show node "$n" 2>/dev/null | grep -q "Gres=.*${GRES}" && echo "$n"; done | tr '\n' ' ')
echo "== hw_probe: ${PART} -> $(echo "$NODES" | wc -w) node(s) $(date -Is)"
TMP=$(mktemp -d)
PAR=${PAR:-6}
probe_one() {
  local N=$1
  timeout "$TIMEOUT" srun -p "$PART" --qos=preempt --overlap -w "$N" -n1 -c1 --mem=100M --time=00:01:00 \
      bash -c 'M=$(awk -F": " "/^model name/{print $2; exit}" /proc/cpuinfo);
               F=$(awk "/^flags/{print; exit}" /proc/cpuinfo);
               case "$F" in *avx512f*) I=avx512; A=1;; *avx2*) I=avx2; A=0;; *) I=pre-avx2; A=0;; esac
               G=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | tr " " "_");
               C=$(awk -F": " "/^cpu cores/{print \$2; exit}" /proc/cpuinfo);
               S=$(grep -c "^physical id" /proc/cpuinfo >/dev/null 2>&1 && awk -F": " "/^physical id/{print \$2}" /proc/cpuinfo | sort -u | wc -l);
               L=$(grep -c "^processor" /proc/cpuinfo);
               echo "$(hostname) cores=$(( ${C:-0} * ${S:-1} )) sockets=${S:-1} logical=$L $I avx512f=$A gpu=${G:-none} $M"' 2>/dev/null | tail -1 > "$TMP/$N"
}
for N in $NODES; do
  while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 1; done
  probe_one "$N" &
done
wait
OUTFILE=$(mktemp); : > "$OUTFILE"
for N in $NODES; do
  L=$(cat "$TMP/$N" 2>/dev/null)
  if [ -n "$L" ]; then echo "$L" | tee -a "$OUTFILE"; else echo "$N UNREACHABLE (busy/drained/timeout)"; fi
done
rm -rf "$TMP"; TMP=$OUTFILE
if [ -n "${OUT:-}" ]; then
  python3 - "$TMP" "$OUT" <<'PY'
import json, sys
m = {}
for line in open(sys.argv[1]):
    p = line.split()
    if len(p) >= 8:
        m[p[0]] = dict(cores=int(p[1].split('=')[1]), sockets=int(p[2].split('=')[1]), logical=int(p[3].split('=')[1]),
                       isa=p[4], avx512f=p[5].endswith('1'), gpu=p[6].split('=', 1)[1], model=' '.join(p[7:]))
json.dump(m, open(sys.argv[2], 'w'), indent=1)
bysize = {}
for n, d in m.items():
    bysize.setdefault(d['cores'], []).append(n)
print(f'wrote {sys.argv[2]}: ' + '; '.join(f'{k}-core n={len(v)}' for k, v in sorted(bysize.items())))
for k, v in sorted(bysize.items(), key=lambda kv: -len(kv[1])):
    gp = sum(1 for n in v if m[n]['gpu'] not in ('none', ''))
    isas = sorted({m[n]['isa'] for n in v})
    print(f'  {k:>3} physical cores: {len(v):>3} machines ({gp} with a GPU), isa {isas} -- '
          f'{",".join(sorted(v)[:8])}{"..." if len(v) > 8 else ""}')
if 36 in bysize:
    print(f'\n!! {len(bysize[36])} machine(s) have 36 physical cores. Every recorded disagreement had a 36-core '
          f'machine on exactly one side, so do NOT standardise the pinned pass on 36 unless you mean to.')
cand = {k: v for k, v in bysize.items() if k != 36}
best = max(cand.items(), key=lambda kv: len(kv[1]))[0] if cand else (max(bysize, key=lambda k: len(bysize[k])) if bysize else None)
if best is not None:
    print(f'\nSuggested REQUIRE_CORES: {best} ({len(bysize[best])} machines, the most common NON-36-core size). A '
          f'pinned pass needs NO --nodelist: eval_e2e.py --require-cores {best} checks /proc/cpuinfo on arrival.')
PY
fi
rm -f "$TMP"
echo "== hw_probe done $(date -Is)"
