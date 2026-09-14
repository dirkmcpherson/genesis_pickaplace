# Canonical {r2dreamer} milestone schedule (user, 2026-09-13): denser checkpoints for every NEW run, as a strict
# SUPERSET of the legacy schedule [500000,1000000,2000000,4000000] so new seeds stay comparable with the old ones
# at those four points. Source this and pass $R2_MILESTONES_4M (or _2M / _1M) to wmfix_full.sbatch.
# 119 MB per milestone; the 4M schedule = 16 checkpoints = 1.9 GB per run.
R2_MILESTONES_LEGACY_4M='[500000,1000000,2000000,4000000]'
R2_MILESTONES_4M='[250000,500000,750000,1000000,1250000,1500000,1750000,2000000,2250000,2500000,2750000,3000000,3250000,3500000,3750000,4000000]'
R2_MILESTONES_2M='[250000,500000,750000,1000000,1250000,1500000,1750000,2000000]'
R2_MILESTONES_1M='[250000,500000,750000,1000000]'
