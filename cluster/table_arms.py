#!/usr/bin/env python3
"""PHASE_PLAN (v): make the RLPD and DP arm directory names overridable, as --wm-human/--wm-machine already are, so the
de-confounded arm can be tabulated without a third-arm refactor. Defaults reproduce today's behaviour exactly."""
import io, sys
p = sys.argv[1]
s = io.open(p, encoding='utf-8').read()
if '--rlpd-machine' in s:
    print(f'[table_arms] {p}: already patched'); sys.exit(0)

A = "    ap.add_argument('--sets', default='hold15 rnd30 spots60')\n"
B = ("    # PHASE_PLAN (v): the arm a learner reads is a NAME TEMPLATE, so the de-confounded machine arm\n"
     "    # (dDPfirst / dDPfull_first) can be tabulated against either the human arm (the de-confounded source\n"
     "    # contrast) or the selected machine arm (the selection effect on its own). Defaults are the arms of record.\n"
     "    ap.add_argument('--rlpd-human',   default='e2e_rlpd_dH_s{s}')\n"
     "    ap.add_argument('--rlpd-machine', default='e2e_rlpd_dDP_s{s}')\n"
     "    ap.add_argument('--dp-human',     default='e2e_dp_dH_s{s}')\n"
     "    ap.add_argument('--dp-machine',   default='e2e_dp_dDP_s{s}')\n") + A
assert s.count(A) == 1, s.count(A)
s = s.replace(A, B)

A2 = ("        ('RLPD', lambda arm, s: os.path.join(args.rlpd_runs, f'e2e_rlpd_{arm}_s{s}'), ('sample', 'mode')),\n"
      "        ('DP  ', lambda arm, s: os.path.join(args.dp_runs, f'e2e_dp_{arm}_s{s}'), ('sample',)),\n")
B2 = ("        ('RLPD', lambda arm, s: os.path.join(args.rlpd_runs, (args.rlpd_human if arm == 'dH' else args.rlpd_machine).format(s=s)), ('sample', 'mode')),\n"
      "        ('DP  ', lambda arm, s: os.path.join(args.dp_runs, (args.dp_human if arm == 'dH' else args.dp_machine).format(s=s)), ('sample',)),\n")
assert s.count(A2) == 1, s.count(A2)
s = s.replace(A2, B2)

A3 = "    for iset in sets:\n"
B3 = ("    defaults = {'wm_human': 'full_r2d_state_dHfull_all_bnormclampS8ent5_s{s}',\n"
      "                'wm_machine': 'full_r2d_state_dDPfull_bnormclampS8ent5_s{s}',\n"
      "                'rlpd_human': 'e2e_rlpd_dH_s{s}', 'rlpd_machine': 'e2e_rlpd_dDP_s{s}',\n"
      "                'dp_human': 'e2e_dp_dH_s{s}', 'dp_machine': 'e2e_dp_dDP_s{s}'}\n"
      "    moved = {k: getattr(args, k) for k, v in defaults.items() if getattr(args, k) != v}\n"
      "    if moved:\n"
      "        # never let a non-default arm be read as the comparison of record\n"
      "        print('NON-DEFAULT ARMS -- this is NOT the table of record: ' +\n"
      "              ', '.join(f'{k}={v}' for k, v in sorted(moved.items())))\n") + A3
assert s.count(A3) == 1, s.count(A3)
s = s.replace(A3, B3)
io.open(p, 'w', encoding='utf-8').write(s)
print(f'[table_arms] {p}: PATCHED')
