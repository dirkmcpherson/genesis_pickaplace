#!/usr/bin/env bash
# Cell list for rs2_rescore.sbatch (amendment (j)): every affected world-model cell x {mode, sample}, long cells first so the
# round-robin lanes balance. Columns: <run> <phase|full> <bank|icset> <tag> <mode>. Banks = the REBUILT physical-grip
# polE_place / polE_place_dDP / polE_contact (canonical names; the raw-grip originals live on as *_rawgrip.json).
W=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03; B=$W/phase_banks
{
# place: human s0-7, machine-39 s0-7, machine-63 s0-7 on polE (rebuilt) + polE-dDP (rebuilt); pinned entries
for m in mode sample; do for s in 0 1 2 3 4 5 6 7; do
  for r in s2_r2d_place_state_dH_bnormclamp1ent5_s$s s2_r2d_place_state_dDP_bnormclamp1ent5_n39_s$s s2_r2d_place_state_dDP_bnormclamp1ent5_s$s; do
    echo "$r phase $B/polE_place_physgrip.json polE $m"
    echo "$r phase $B/polE_place_dDP_physgrip.json polEdDP $m"
  done
done; done
# carrycontact: human s0-7, machine-21 s0-7 on polE (rebuilt) + polE-dDP (rebuilt)
for m in mode sample; do for s in 0 1 2 3 4 5 6 7; do
  for r in s2_r2d_carrycontact_state_dH_bnormclamp1ent5_s$s s2_r2d_carrycontact_state_dDP_bnormclamp1ent5_n21_s$s; do
    echo "$r phase $B/polE_place_physgrip.json polE $m"
    echo "$r phase $B/polE_place_dDP_physgrip.json polEdDP $m"
  done
done; done
# contact after release: human sub-floor s0-7, machine n11 s0-7, machine n25 s0-7 on polE_contact (rebuilt)
for m in mode sample; do for s in 0 1 2 3 4 5 6 7; do
  for r in s2_r2d_contact_state_dH_bnormclamp1ent5_subfloor_s$s s2_r2d_contact_state_dDP_bnormclamp1ent5_n11_s$s s2_r2d_contact_state_dDP_bnormclamp1ent5_s$s; do
    echo "$r phase $B/polE_contact_physgrip.json polE $m"
  done
done; done
# end-to-end: rnd30 + hold15, both arms, both modes (nested_honest / nested_proxy / placed_v2 / contact / picked)
for m in mode sample; do for s in 0 1 2 3 4 5 6 7; do
  echo "full_r2d_state_dHfull_all_bnormclampS8ent5_s$s full rnd rnd30 $m"
  echo "full_r2d_state_dDPfull_bnormclampS8ent5_s$s full rnd rnd30 $m"
done; done
for m in mode sample; do for s in 0 1 2 3 4 5 6 7; do
  echo "full_r2d_state_dHfull_all_bnormclampS8ent5_s$s full hold hold15 $m"
  echo "full_r2d_state_dDPfull_bnormclampS8ent5_s$s full hold hold15 $m"
done; done
# place holdE (human bank, physical already): re-run once, pinned, to confirm the bank identity (the record drew with replacement)
for m in mode sample; do for s in 0 1 2 3 4 5 6 7; do
  for r in s2_r2d_place_state_dH_bnormclamp1ent5_s$s s2_r2d_place_state_dDP_bnormclamp1ent5_n39_s$s s2_r2d_place_state_dDP_bnormclamp1ent5_s$s; do
    echo "$r phase $B/holdE_place.json holdE $m"
  done
done; done
} > $W/rs2_cells.txt
wc -l $W/rs2_cells.txt
