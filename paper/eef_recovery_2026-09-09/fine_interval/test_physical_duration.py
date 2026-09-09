"""Retain the adversarial gate and reject short contacts at the finer cadence."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'can_pos_recovery'))
import test_eef_task_sequence as legacy
from fine_sequence import score_sequence


def repeat(data):
    return {k: np.repeat(v, 3, axis=0) for k, v in data.items()}


# All eight existing physical counterexamples run at the new cadence.
legacy.score_sequence = lambda data: score_sequence(repeat(data))
SequenceTest = legacy.SequenceTest


class DurationTest(unittest.TestCase):
    def test_thirty_ms_contact_does_not_meet_ninety_ms_gate(self):
        data = repeat(legacy.complete_trace())
        data['contact_counts'][:, 2] = 0
        data['contact_counts'][66:69, 2] = 1
        self.assertFalse(score_sequence(data)['complete'])

    def test_thirty_ms_release_does_not_meet_ninety_ms_gate(self):
        data = repeat(legacy.complete_trace())
        data['contact_counts'][:, 1] = 1
        data['contact_counts'][18:21, 1] = 0
        self.assertFalse(score_sequence(data)['complete'])


if __name__ == '__main__':
    unittest.main()
