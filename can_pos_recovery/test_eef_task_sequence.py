"""Adversarial physical sequences for the development acceptance rule."""
import unittest
import numpy as np
from eef_task_sequence import score_sequence


def complete_trace():
    n = 30
    r = np.zeros((n, 21)); c = np.zeros((n, 3), dtype=int)
    s = np.zeros((n, 6), dtype=bool); g = np.zeros((n, 7))
    r[:, 13] = .5; r[:, 15] = .2205; r[:, 16] = 1
    g[:, 0] = .6; g[:, 2] = .2205; g[:, 3] = 1
    s[3:, 0] = True; c[6:, 0] = 1
    c[:6, 1] = 1; c[12:23, 1] = 1
    r[12:23, 13] = np.linspace(.5, .534, 11); r[23:, 13] = .534
    c[22:, 2] = 1
    return dict(trajectory=r, contact_counts=c, stages=s, goal_pose=g)


class SequenceTest(unittest.TestCase):
    def test_release_then_supported_push(self):
        self.assertTrue(score_sequence(complete_trace())['complete'])

    def test_carried_contact_is_rejected(self):
        d=complete_trace();d['contact_counts'][:, 1]=1
        self.assertFalse(score_sequence(d)['complete'])

    def test_table_shove_without_pick_is_rejected(self):
        d=complete_trace();d['stages'][:, 0]=False
        self.assertFalse(score_sequence(d)['complete'])

    def test_airborne_second_carry_is_rejected(self):
        d=complete_trace();d['trajectory'][13:22, 15]+=.03
        self.assertFalse(score_sequence(d)['complete'])

    def test_passive_drift_is_not_robot_push(self):
        d=complete_trace();d['contact_counts'][6:, 1]=0
        self.assertFalse(score_sequence(d)['complete'])

    def test_short_of_contact_is_rejected(self):
        d=complete_trace();d['contact_counts'][:, 2]=0
        self.assertFalse(score_sequence(d)['complete'])

    def test_contact_then_far_separation_is_rejected(self):
        d=complete_trace();d['trajectory'][-1, 13]=.52
        self.assertFalse(score_sequence(d)['complete'])

    def test_deeply_interpenetrating_cans_are_rejected(self):
        d=complete_trace();d['trajectory'][-1, 13]=.56
        self.assertFalse(score_sequence(d)['complete'])


if __name__ == '__main__':
    unittest.main()
