# Canonical {r2dreamer} milestone schedule (user, 2026-09-13): denser checkpoints for every NEW run, as a strict
# SUPERSET of the legacy schedule [500000,1000000,2000000,4000000] so new seeds stay comparable with the old ones
# at those four points. Source this and pass $R2_MILESTONES_4M (or _2M / _1M) to wmfix_full.sbatch.
# 119 MB per milestone; the 4M schedule = 16 checkpoints = 1.9 GB per run.
R2_MILESTONES_LEGACY_4M='[500000,1000000,2000000,4000000]'
R2_MILESTONES_4M='[250000,500000,750000,1000000,1250000,1500000,1750000,2000000,2250000,2500000,2750000,3000000,3250000,3500000,3750000,4000000]'
R2_MILESTONES_2M='[250000,500000,750000,1000000,1250000,1500000,1750000,2000000]'
R2_MILESTONES_1M='[250000,500000,750000,1000000]'

# {RLPD} (same rule, 2026-09-13). Legacy checkpoints in ABSOLUTE decisions: 40k, 100k, 250k, 500k (250k runs used
# fracs 0.16,0.4,1.0; 500k runs 0.2,0.5,1.0). Dense = every 50k plus the legacy points; 12 MB each.
RLPD_FRACS_LEGACY_250K='0.16,0.4,1.0'
RLPD_FRACS_LEGACY_500K='0.2,0.5,1.0'
RLPD_FRACS_250K='0.16,0.2,0.4,0.6,0.8,1.0'                              # 40k 50k 100k 150k 200k 250k
RLPD_FRACS_500K='0.08,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0'           # 40k 50k 100k ... 500k
RLPD_FRACS_1M='0.04,0.1,0.2,0.25,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0'        # 40k 100k 200k 250k 300k ... 1M (legacy 40k/100k/250k/500k kept)
