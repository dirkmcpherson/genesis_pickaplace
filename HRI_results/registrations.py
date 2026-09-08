#!/usr/bin/env python3
"""Generates REGISTRATIONS.md: every registered prediction with its outcome.

This is where fidelity is won or lost. The project's record is NOT clean, and a writer must not
be able to describe it as clean. Outcomes are one of:

  MET            the registered threshold or decision rule was satisfied
  FAILED         it was not - the prediction was wrong in the direction stated
  NOT EVALUABLE  the experiment ran but cannot bear on the prediction (e.g. both arms floored)
  DEMOTED        the statistic it names was withdrawn or redefined after registration
  WITHDRAWN      a later registered control invalidated the reading the prediction was about
  PENDING        registered, not yet read out

Every entry cites where it is registered and where the outcome is stated, so an auditor can check
that the registration preceded the readout.
"""
import datetime

REGISTRATIONS = [
# ================================================================= FAILED
dict(id='(d) P2 - in-distribution placed_v2', outcome='FAILED',
     registered='PHASE_PLAN_2026-09-04.md:204, amendment (d), registered 2026-09-05 10:05 '
                'before any run',
     prediction='"|D(human - machine)| < 0.10 at every stage that either arm reaches >= 0.2"',
     result='in-distribution (hold15) placed_v2: human 0.083 v machine 0.242, D -0.158 - OUTSIDE '
            'the margin, in the MACHINE arm\'s favour (MDE 0.254, p 0.106)',
     note='THE ONE OUTRIGHT FAILED MARGIN on record. Three documents disagree about it: '
          'EVAL_FIXES_2026-09-07.md:388 and PHASE_RESULTS_2026-09-05.md:239 both say "all met"; '
          'PHASE_RESULTS_2026-09-05.md:216 is the CORRECTING entry and says the earlier wording '
          '"was true only of the out-of-distribution cells". Cite :216. The same prediction is '
          'MET on the random-start cell (D -0.038, p 0.520).',
     verify='PHASE_RESULTS_2026-09-05.md:216'),

dict(id='A20 - RLPD source effect replicates in the corrected world', outcome='FAILED',
     registered='PREREG_final_round_robin_2026-08-23.md:376-386, before any readout of s40-47',
     prediction='"LAST rnd dH > dDP by >= 0.10; divergence dH <= 2/8, dDP >= 3/8"',
     result='BOTH clauses fail: no source effect on any statistic (dH ~ dDP ~ 0.50-0.52, '
            'symmetric 3/8 divergence). D -0.021, permutation p 0.983',
     note='The old-world RLPD source effect (A16, +0.205) does NOT survive the world correction. '
          'The doc\'s own section header reads "REGISTERED PREDICTIONS FAIL; the RLPD source '
          'effect is WORLD-DEPENDENT".',
     verify='RESULTS_for_writing_2026-08-30.md:142,149-152'),

dict(id='PAPER_PLAN H4 - world models benefit equally from all sources', outcome='FAILED',
     registered='PAPER_PLAN, 2026-07-31, before any world-model source data',
     prediction='world models benefit approximately equally from human and model demonstrations',
     result='FAILED, with a directional HUMAN preference at the time it was scored',
     note='THE PAPER\'S ORIGINAL HEADLINE HYPOTHESIS, recorded as failed in the project\'s own '
          'registration-timing table. The later null (stage 3, |D| < 0.10 MET) was registered '
          'AFTER that failure. A writer must not present the null as the confirmation of H4; it '
          'is a different, later prediction.',
     verify='PREREG_final_round_robin_2026-08-23.md:582-594'),

dict(id='dHv2all - the world model gains from human failures', outcome='FAILED',
     registered='PHASE_PLAN_2026-09-04.md:99-102, work order 2026-09-04 12:15',
     prediction='"r2dreamer dHv2all > dHv2raw on rnd30 MODE by >= 0.05"',
     result='0.588 v 0.617 (D -0.03, p 0.58); RLPD 0.554 v 0.600 (D -0.046, p 0.567)',
     note='A directional prediction that came out on the WRONG SIDE OF ZERO on both learners, '
          'not merely short of its margin. In-distribution the deficit is significant '
          '(holdv2 p 0.035, alldemo p 0.044).',
     verify='MORNING_TABLE_2026-09-04.md:124-129'),

dict(id='A32 - clamp-variant endpoints survive', outcome='FAILED',
     registered='PREREG_final_round_robin_2026-08-23.md:534-536',
     prediction='"C2000 and RS1 ignite at least as often as the clamped block (>= 3/4) and keep '
                '>= 3/4 endpoints alive"',
     result='C2000 LAST sel 0.00 x4; RS1 LAST sel 0.00 x4. Endpoints clause FAILS. The '
            'SPARSE-RS1 clause (<= 1/2 ignition) was met - it never ignited.',
     verify='RESULTS_for_writing_2026-08-30.md:262-268'),

dict(id='P-R2D (section 6) - dense-reward ignition replicates', outcome='FAILED',
     registered='PREREG_final_round_robin_2026-08-23.md:172-192, 2026-08-23',
     prediction='the dense 4/4-class ignition on dH "replicates and extends to dDP/dR2D"',
     result='FAILED in the corrected world (dDP 3/8)',
     verify='PREREG_final_round_robin_2026-08-23.md:584'),

dict(id='World-search A2 P1 - legacy world-fit transfers', outcome='FAILED',
     registered='WORLD_SEARCH_PREREG_2026-09-02.md:255-256, written before the baseline ran',
     prediction='"theta_L scores >= theta0 + 3 (contact+nested) on the fit set at gen 0"',
     result='theta_L is -9 score / -5 contact+nested versus theta0 - FALSIFIED',
     note='Consequence recorded in the doc: the legacy search result "must not be cited as '
          'evidence for any physics parameter on the paper world". P2 (finger_force <= 50) also '
          'falsified at 59.',
     verify='WORLD_SEARCH_PREREG_2026-09-02.md:307-311'),

dict(id='CREEP P1 - contact-constraint creep is the recorder-path mechanism', outcome='FAILED',
     registered='CREEP_PREREG_2026-09-03.md:86,91-94, before the candidate census runs',
     prediction='"contact >= 32, tipped <= 22, picked 65-69, nested >= 20"',
     result='contact 20, tipped 43 - FALSIFIED WITH THE WRONG SIGN ON EVERY METRIC '
            '(Dcontact -6, Dtipped +11, Dset-down -23, all far outside the 5.3 noise floor)',
     note='Disconfirm branch (c) fired as the verdict for the whole pre-registration. The two '
          'companion rows (gimp99, ts5_imp99) were also not adopted.',
     verify='CREEP_PREREG_2026-09-03.md:123-125,174-175'),

dict(id='SHELF_HEIGHT P1 - a taller shelf converts drops into contact', outcome='FAILED',
     registered='SHELF_HEIGHT_PREREG_2026-09-03.md:76,81-83, before any candidate census',
     prediction='"contact >= 29 (expect 28-34), tipped <= 18, picked 66-69, nested >= 19"',
     result='contact 18 (-8, the wrong sign); all three shelf rows NOT ADOPTED',
     note='The motivating finding F3 ("shelf 4 cm too low") was itself WITHDRAWN. Three '
          'independent lines later agreed the real plate is 11-13 cm = the existing shelf6.',
     verify='SHELF_HEIGHT_PREREG_2026-09-03.md:128-129,171-172'),

dict(id='(g) P2 - the contact_push failing fraction is small and symmetric',
     outcome='FAILED',
     registered='PHASE_PLAN_2026-09-04.md:240, registered before any re-score job',
     prediction='"the failing fraction ... is <= 0.15 of the contact credit, and does not differ '
                'between arms by more than 0.05"',
     result='the <= 0.15 clause is DECISIVELY VIOLATED (0.37-0.69); the symmetry clause holds at '
            'the phases (+0.028, -0.040) but not end-to-end (+0.103)',
     note='The registered wrong-side disconfirm branch FIRED: gripper-goal contact explains only '
          '11-33 % of failing episodes; the rest is the can still in the grasp. (g) P1 '
          '(|D| < 0.10 on contact_push) was met everywhere.',
     verify='CONTACT_PUSH_2026-09-07.md:131; PHASE_RESULTS_2026-09-05.md:159,175'),

dict(id='P-A4-3\' - the re-encoded control arms are neutral', outcome='FAILED',
     registered='ROBOMIMIC_PLAN_2026-09-05.md:217, before any A4 training',
     prediction='"re15 and re20 are within 0.10 of MH200@100k"',
     result='re15 0.080 versus MH200 0.455 - a -0.375 gap, far outside the registered band',
     note='Decision rule (iii) fired: the A4 pair may be read ONLY within itself, never against '
          'MH200@100k. The doc records the over-claiming sentence as STRUCK.',
     verify='ROBOMIMIC_LOG_2026-09-06.md:358-359,374-376'),

# ================================================================= NOT EVALUABLE
dict(id="P-A4-2' - action roughness explains the machine arm's weakness",
     outcome='NOT EVALUABLE',
     registered='ROBOMIMIC_PLAN_2026-09-05.md:216-217; the FLOOR RULE that governs it was '
                'registered separately at :303-311 while re15 was complete and rough15 had 3 of '
                '8 seeds in - i.e. before the contrast existed',
     prediction='"rough15 - re15 <= -0.10 (p < 0.05), and rough20 - re20 <= rough15 - re15"',
     result='the epsilon 0.15 pair has BOTH arms at or below the floor (0.080, 0.034), so the '
            'registered floor rule fires: counts reported, NO significance test, NOT EVALUABLE',
     note='Registered, run, and uninformative - which must be reported as such, never counted as '
          'a null. The epsilon 0.20 pair did clear the precondition and was tested (D +0.145, '
          'p 0.0047). A4\'s own question is recorded as answered NO, "not by this design". Its '
          'sibling P-A4-1 was WITHDRAWN UNRUN (build yields 0/200).',
     verify='ROBOMIMIC_LOG_2026-09-06.md:498-516'),

dict(id='N7 (3) - the dDP null control', outcome='NOT EVALUABLE',
     registered='the N7 density audit',
     prediction='dDP null control on the density ablation',
     result='NOT EVALUABLE at 3 of 6 seeds. In the same audit (1) and (2) FAILED.',
     note='A second, independent not-evaluable. The audit also records that '
          'NOTES_IN_PLAIN_ENGLISH "inverted the pre-registered reading" of the falsifier.',
     verify='AUDIT_results_2026-08-28.md:29,84'),

# ================================================================= DEMOTED
dict(id="(l)/(l') slide_success", outcome='DEMOTED',
     registered='PHASE_PLAN_2026-09-04.md:293-307 (l), :325-331 (l\')',
     prediction='"|D| < 0.10 on slide_success" and "slide_success <= nested_honest"',
     result='both MET as stated (D +0.025) - BUT FOR A PREDICATE THE PROJECT WITHDREW',
     note='Amendment (p) withdrew (l)\'s grip clause because it passes only 2 of 74 human '
          'demonstrations - 44 of the 72 failures are the grip clause alone, and median last '
          'commanded grip is 0.39. So the column scores a predicate the demonstrations '
          'themselves fail. DEMOTED from statistic of record to diagnostic. Amendment (o), which '
          'would have paid a reward on it, was STOPPED BEFORE LANDING and then measured '
          'unbuildable: at the registered threshold BOTH arms yield ZERO segments. No p-value '
          'may be attached (0-6 %, a floor).',
     verify='PHASE_RESULTS_2026-09-05.md:153-155; PHASE_PLAN_2026-09-04.md:451-462,470-485'),

# ================================================================= WITHDRAWN BY A LATER CONTROL
dict(id='robomimic source falsifier (a) - RLPD', outcome='WITHDRAWN',
     registered='ROBOMIMIC_PLAN_2026-09-05.md:64 (falsifier), :131-138 (the A2 controls and '
                'their escape clause, "written before any of them is built or submitted")',
     prediction='falsifier (a): source-indifference fails on an independent generator if '
                'PH/MH >> MG by >= 0.15, p < 0.05 - UNLESS the quantity/coverage control fires',
     result='the falsifier fired (D +0.307, p 0.008) and THEN the escape clause fired: A2 P1\'s '
            'second half FALSIFIED (MG718s predicted >= 0.15 below MH200; measured +0.020, '
            'p 0.886) and P2\'s consequence FALSIFIED (MGall is +0.155 ABOVE MH200)',
     note='The escape clause was registered in advance, so the falsifier does not count as '
          'fired. The doc\'s section header reads "the source reading is dead". Still untested: '
          'whether a NEUTRAL 200-tape draw would match.',
     verify='ROBOMIMIC_LOG_2026-09-06.md:315,325-334'),

dict(id='A5 P-A5-1 / P-A5-2 - the action head is not the story', outcome='WITHDRAWN',
     registered='ROBOMIMIC_PLAN_2026-09-05.md:229-273, registered BEFORE the runs, WITH its '
                'consequences pre-committed at :258-273',
     prediction='"MG200s_gmm within 0.10 of 0.393" and "MH200_nogmm within 0.10 of 0.927"',
     result='BOTH FALSIFIED. MG200s_gmm 0.093 (D -0.300, three times the band); MH200_nogmm '
            '0.560 (D -0.367). Matched-head source gaps: 0.834 mixture, 0.167 deterministic - '
            'the same comparison differs by 5.0x',
     note='Registered decision rules (ii) and (iii) both fired, so the BC-RNN row and the '
          'cross-learner ordering are withdrawn by pre-commitment, not by hindsight. Note the '
          'UPSTREAM predictions ("MG200s <= PH200 - 0.15"; "MH200 within 0.10 of PH200") were '
          'recorded as MET - a prediction can be met and still be uninformative, because the '
          'arms ran different heads.',
     verify='ROBOMIMIC_LOG_2026-09-06.md:403-411,432-441'),

dict(id='A27 - world-model v2 source effect at n=12v12', outcome='WITHDRAWN',
     registered='PREREG_final_round_robin_2026-08-23.md:465',
     prediction='"dHv2raw > dDPv2 on BEST rnd by >= 0.15; ignition difference >= 3/12"',
     result='CANCELLED BEFORE ANY SEED RAN by amendment A36: under return_clamp=100 the actor '
            'received no grasp credit, so the readout would have described a saturated-critic '
            'learner',
     note='Also carries the worst registration-timing exposure on record: A27 was registered '
          'AFTER the full frozen n=8v8 that motivated the reversal.',
     verify='PREREG_final_round_robin_2026-08-23.md:598-601,590'),

# ================================================================= UNSTATED / NEVER READ OUT
dict(id='TOST equivalence tests', outcome='NOT EVALUABLE',
     registered='ROBOMIMIC_PLAN_2026-09-05.md:61-62, "TOST +/-0.10 where equivalence is claimed"',
     prediction='a two-one-sided-tests equivalence procedure wherever equivalence is claimed',
     result='NO TOST RESULT IS REPORTED ANYWHERE, on any contrast, in any document',
     note='THE REGISTERED EQUIVALENCE PROCEDURE WAS NEVER RUN. Every equivalence-flavoured '
          'sentence in this project rests on "|D| < 0.10 was not exceeded" plus, in this '
          'directory, an MDE and a posterior - none of which is the registered test. This is a '
          'gap a writer must not paper over by calling a null an equivalence.',
     verify='no TOST output exists; searched the full paper/ corpus'),

dict(id='(k) spots60 in-distribution predictions', outcome='NOT EVALUABLE',
     registered='PHASE_PLAN_2026-09-04.md:268, registered 2026-09-07 18:05 before any '
                'evaluation on it',
     prediction='"every learner scores higher on spots60 than on rnd30; the human-vs-machine '
                'null holds on spots60 for the world model and RLPD at pick; DP\'s pruned-human '
                'arm exceeds 0.85"',
     result='the numbers exist but NO DOCUMENT STATES MET OR NOT MET for any of the three '
            'clauses, and the world-model clause is UNTESTABLE - that arm was never run',
     note='A registered prediction with numbers and no verdict is not a met prediction. Also '
          'registered post hoc against it: the cell is near saturation, so it is "a null by '
          'ceiling rather than a null by measurement".',
     verify='CELL_STATUS_2026-09-07.md:12; OVERNIGHT_STATE_2026-09-07.md:265'),

dict(id='robomimic primary matrix - r2dreamer arm', outcome='NOT EVALUABLE',
     registered='ROBOMIMIC_PLAN_2026-09-05.md:62, |D| < 0.10 on all three primary contrasts',
     prediction='r2dreamer shows |D| < 0.10 between sources on the independent task',
     result='NEVER READ OUT as a source contrast: the world model scored 0/400 and 1/400 at '
            '~541k steps - it did not learn the task',
     note='A failed LEARNER, not a failed prediction about demonstrations. But it is the reason '
          'the world-model source result rests on a single task.',
     verify='robo_r2d in results.md'),

dict(id='PREREG section 6 hypotheses (P-DP, P-RLPD dense, P-MECH, P-DV3, H4-prime), '
        'section 8 pilots, A21(ii)/(iii), A28, A36',
     outcome='NOT EVALUABLE',
     registered='PREREG_final_round_robin_2026-08-23.md:172-192, :203-211, :396-397, :481-483, '
                ':616-617',
     prediction='various',
     result='NO OUTCOME STATEMENT EXISTS for any of them. Five of the six section-8 pilots have '
            'no recorded outcome; A28 was never read out and its construction defect is still '
            'open; A36 was never read out',
     note='These are registered predictions that simply stopped. They must appear in any honest '
          'count of the registration record, not be silently dropped because they have no '
          'number.',
     verify='CONFOUNDS.md:35 (A28 defect); PREREG:582-594 (status table)'),

# ================================================================= PENDING
dict(id='(h) / (n) - DP and RLPD at place and end-to-end', outcome='PENDING',
     registered='PHASE_PLAN_2026-09-04.md:262 (h), :398-403 (n), both before any run',
     prediction='|D| < 0.10 on the statistic of record for BOTH learners at place and at every '
                'end-to-end stage either arm reaches 0.2',
     result='not read out; the results skeleton is written with every cell empty',
     note='(n) also registers a DP IDLE CONFOUND in advance: if DP shows |D| >= 0.10 WITH THE '
          'MACHINE AHEAD, a human-PRUNED control arm runs before the word "source" is used.',
     verify='PHASE_RESULTS_PLACE_DP_RLPD_2026-09-08.md:27-38; place_dp / e2e_dp in results.md'),

dict(id='(v) - does the source null survive de-selection', outcome='PENDING',
     registered='PHASE_PLAN_2026-09-04.md:660-671, registered 2026-09-08 BEFORE any build or run',
     prediction='P1 |D(human - machine-first)| < 0.10 at every stage either arm reaches 0.2; '
                'P2 machine-best >= machine-first; P3 (decisive) the learning-speed advantage '
                'either persists, shrinks toward zero, or reverses against machine-first',
     result='not read out',
     note='THE MOST IMPORTANT PENDING REGISTRATION. It is the registered test of the '
          'set-construction confound this directory flags on every end-to-end claim, and P3 (b) '
          'pre-commits to reporting that "the effect was substantially our filtering" if the '
          'advantage shrinks.',
     verify='PHASE_PLAN_2026-09-04.md:660-671'),

dict(id='(s) - shared versus isolated evaluation processes', outcome='PENDING',
     registered='PHASE_PLAN_2026-09-04.md:531, registered before either cell exists',
     prediction='"|shared - isolated| < 0.05 per arm per stage; if it exceeds that for either '
                'learner, section 5.1 is re-scored under isolation before the three-learner '
                'table is published"',
     result='not read out; depends on the pending (h)/(n) cells',
     verify='PHASE_PLAN_2026-09-04.md:531'),

dict(id='(u) - learning-speed / ignition-step difference', outcome='PENDING',
     registered='PHASE_PLAN_2026-09-04.md:551-558. The REPLICATION is registered before its data '
                'exists; the DISCOVERY is explicitly post hoc - "the threshold (0.2), the bin '
                'grid (40) and the stage (picked) were all chosen after seeing the data"',
     prediction='"the human arm ignites earlier. A result in the opposite direction, or a null, '
                'disconfirms."',
     result='replication not read out. The discovery magnitudes were CORRECTED AND PARTLY '
            'WITHDRAWN: "the originally reported ~262k-step difference was computed on '
            'contaminated flags and is withdrawn"; corrected picked D -156,249 (p 0.019)',
     note='Registered ordering rule: the selection control must run BEFORE, not after, any '
          'increase in seeds. Any learning-speed sentence must say the discovery was post hoc '
          'and that its first magnitudes were withdrawn.',
     verify='PHASE_PLAN_2026-09-04.md:547,596,602-604,618'),

dict(id='dv3 G3 - four-seed completion', outcome='PENDING',
     registered='DV3_DEBUG_2026-09-05.md:263-266',
     prediction='"|D rnd30| < 0.15 and p > 0.05" with both arms in 0.55-0.80',
     result='not read out; at n=2v2 the exact permutation p is 0.333, the floor at that n',
     note='TIMESTAMP DISCREPANCY worth one line in any methods audit: the G3 registration is '
          'stamped 20:20 while its submission is stamped 20:13:35 - registration appears to '
          'POSTDATE submission by ~7 minutes, unexplained in the file. G1 and G2 both have '
          'submission after registration.',
     verify='DV3_DEBUG_2026-09-05.md:257 versus :273'),

# ================================================================= MET
dict(id='Stage 3 - the world-model pick null', outcome='MET',
     registered='WM_FIX_PLAN_2026-09-03.md, registered 2026-09-03 23:05 before submission, with '
                'the statistic, n and prediction fixed',
     prediction='"|D| < 0.10"; the alternative "dHv2raw > dDP by >= 0.10" if the RLPD coverage '
                'mechanism generalises',
     result='0.617 v 0.608, D +0.008, exact permutation p 0.875 (n=8v8). Primary MET, '
            'alternative NOT met',
     note='The best-powered null in the project: this cell\'s MDE is below the margin, so '
          '"equivalent" is defensible HERE and almost nowhere else.',
     verify='pick_rnd30_r2d_mode in results.md'),

dict(id='(e) matched-N at place, contact and carrycontact', outcome='MET',
     registered='PHASE_PLAN_2026-09-04.md:221, registered 11:25 before any matched-N run',
     prediction='"the matched machine arm stays within |D| < 0.10 ... if it drops below human by '
                '>= 0.10 the section-2 null was carried by the extra machine demos"',
     result='place +0.056 -> +0.063 after re-scoring (p 0.112); contact -0.009 -> -0.018 '
            '(p 0.690); carrycontact +0.011 -> +0.013 (p 0.281). MET before and after',
     note='Met, but NOT powered to the margin at place, and both contact arms are sub-floor at '
          '11 demonstrations.',
     verify='place_r2d, contact_r2d_bare, carry_r2d_bare in results.md'),

dict(id='(i) bank-of-origin symmetry control', outcome='MET',
     registered='PHASE_PLAN_2026-09-04.md:244, "registered BEFORE submission ... before any of '
                'these cells exist"',
     prediction='"|D| < 0.10 on the machine-policy bank for both comparisons, and '
                '|D_polE - D_polEdDP| < 0.05 (the bank of origin does not carry the null)"',
     result='clause 1 met at 0.072; clause 2 met at 0.009, against 0.017 on the superseded cells',
     note='One of the cleanest results in the project: registered in advance, both clauses met, '
          'and it rules out a real alternative explanation.',
     verify='place_symmetry_r2d in results.md'),

dict(id='(c\') repeat-1 clock pilot', outcome='MET',
     registered='PHASE_PLAN_2026-09-04.md:183, registered 2026-09-05 09:55 before any pilot run',
     prediction='P1 |D| < 0.10 on rnd30 MODE at action-repeat 1',
     result='D 0.008 against the 0.10 margin; P2 (the clock hid a source effect) NOT met; the '
            'disconfirm branch did not fire - every seed of both arms learned',
     verify='PHASE_RESULTS_2026-09-05.md:256-266'),

dict(id='A29 - pruned beats raw for Diffusion Policy', outcome='MET',
     registered='PREREG_final_round_robin_2026-08-23.md:493-494',
     prediction='"pruned-dHv2 DP rnd exceeds dHv2raw DP rnd by >= 0.15 in both worlds"',
     result='MET in both worlds; this directory measures +0.31 on random starts and +0.19 '
            'in distribution, both p < 0.001',
     note='REGISTRATION-TIMING CAVEAT, self-disclosed: registered after 2 seeds were already '
          'seen. The effect is far larger than the margin, so the exposure is unlikely to be '
          'load-bearing, but it must be disclosed.',
     verify='prune_dp_rnd30, prune_dp_spots60 in results.md; PREREG:592, CONFOUNDS row 30'),

dict(id='A17 - RLPD gamma', outcome='MET',
     registered='PREREG_final_round_robin_2026-08-23.md, A17',
     prediction='gamma 0.99 stabilises RLPD where 0.998 diverges',
     result='69 of 74 runs diverged at 0.998; gamma 0.99 stable at UTD 10 and UTD 5',
     note='A recipe fix, not a result about demonstrations.',
     verify='METHODOLOGY.md, RLPD section'),

dict(id='dv3 G1 and G2', outcome='MET',
     registered='DV3_DEBUG_2026-09-05.md:68-75 (G1), :185-193 (G2), each submitted within a '
                'minute AFTER its registration',
     prediction='G1: target_max <= 1.0 in every row, no post-ignition bin below 0.6, fresh '
                'hold-15 >= 12/15 on 2/2 seeds. G2: rnd30 picked 0.5-0.7, arms indistinguishable '
                'at n=2',
     result='G1 all three met, 15/15 on 2/2 for both pairs; G2 all five met, 0.700 and 0.700',
     note='CORRECTION affecting every dv3 cell: they are SAMPLED-action cells; the sidecar\'s '
          '"deterministic" label is a mislabel, so they compare to r2dreamer\'s SAMPLE row '
          '(0.613 v 0.629), not its MODE row.',
     verify='DV3_DEBUG_2026-09-05.md:161-169,244-249,288-300'),

dict(id='robomimic gates G0, G1, G2a, G2b', outcome='MET',
     registered='ROBOMIMIC_PLAN_2026-09-05.md:63, before any install',
     prediction='G0 replay within 1 cm on >= 4/5; G1 BC-RNN on PH200 >= 0.90; G2a random policy '
                '<= 2/50; G2b no-demo RLPD <= 0.10',
     result='G0 PASS 4/5 (after a self-corrected FAIL that had scored the wrong slice); G1 PASS '
            'at 0.92 (after a checkpoint-selection bug was fixed); G2a 0/50; G2b 0/50',
     note='G2b was registered up front but RUN ONLY AFTER the primary readout, and it changed '
          'the reading: MG200s at 0.147 is ABOVE the demo-free floor of 0.000, so the finding is '
          '"MG helps less than MH", not "MG is worthless". G1 validates a row that is now '
          'withdrawn for a different reason.',
     verify='ROBOMIMIC_LOG_2026-09-06.md:51-62,114-115,136-137,279-286'),

dict(id='RELEASE_OPEN section 4 - the og4 release filter', outcome='MET',
     registered='RELEASE_OPEN_PREREG_2026-09-03.md:71-80, before the candidate census runs, with '
                'the filter constants fixed from earlier data',
     prediction='"adopt a row iff tipped <= 26 and contact >= 21 and picked >= 66 and '
                'nested >= 16, with the holdout moving the same way"',
     result='every clause met on both candidate rows; og4 adopted by the registered preference '
            'rule; the CLEAN holdout (17 uids the design never saw) confirms independently',
     note='Registered consequence still live: machine arms harvesting from teachers do NOT get '
          'this shaping. Handled by leaving both arms on the plain recorder, which is symmetric '
          'and registered - og4 is used by no reported arm.',
     verify='RELEASE_OPEN_PREREG_2026-09-03.md:112-122,117-119,131-133'),
]


def write(path):
    L = ['# Registration outcomes', '',
         f'*Generated by `make_tables.py` on {datetime.date.today().isoformat()} from the spec '
         f'in `registrations.py`.*', '',
         '**The record is not clean, and no writer should be able to describe it as clean.** '
         'This table exists so that a sentence like "every registered prediction was met" fails '
         'an audit immediately.', '']
    counts = {}
    for r in REGISTRATIONS:
        counts[r['outcome']] = counts.get(r['outcome'], 0) + 1
    L += ['| outcome | count |', '|---|---|']
    for k in ('MET', 'FAILED', 'NOT EVALUABLE', 'DEMOTED', 'WITHDRAWN', 'PENDING'):
        if k in counts:
            L.append(f'| {"**" + k + "**" if k != "MET" else k} | {counts[k]} |')
    L += ['',
          f'**{counts.get("FAILED", 0)} outright failed, '
          f'{counts.get("NOT EVALUABLE", 0)} not evaluable, '
          f'{counts.get("DEMOTED", 0)} demoted, '
          f'{counts.get("WITHDRAWN", 0)} withdrawn by a later control.** Several of the MET rows '
          'are met but underpowered, which the notes say individually.', '',
          '| registration | outcome | prediction | result |', '|---|---|---|---|']
    order = {k: i for i, k in enumerate(
        ['FAILED', 'NOT EVALUABLE', 'DEMOTED', 'WITHDRAWN', 'PENDING', 'MET'])}
    for r in sorted(REGISTRATIONS, key=lambda r: order.get(r['outcome'], 9)):
        L.append(f"| `{r['id']}` | **{r['outcome']}** | {r['prediction']} | {r['result']} |")
    L += ['', '## Notes, per registration', '']
    for r in sorted(REGISTRATIONS, key=lambda r: order.get(r['outcome'], 9)):
        L += [f"### `{r['id']}` - {r['outcome']}", '',
              f"- **Registered as:** {r['registered']}",
              f"- **Prediction:** {r['prediction']}",
              f"- **Result:** {r['result']}"]
        if r.get('note'):
            L.append(f"- **Reading:** {r['note']}")
        L += [f"- **Check it:** {r['verify']}", '']
    open(path, 'w').write('\n'.join(L) + '\n')
