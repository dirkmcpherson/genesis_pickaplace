#!/usr/bin/env python3
import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('episode_records', HERE / 'episode_records.py')
records = importlib.util.module_from_spec(spec)
spec.loader.exec_module(records)


def row(step=4, **hits):
    return {'step': step, 'episode/score': 0, 'episode/train_ep_record_valid': 1,
            **{'episode/train_ep_' + k: hits.get(k, 0) for k in records.STAGES}}


class Curves(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.path = self.root / 'metrics.jsonl'

    def tearDown(self):
        self.tmp.cleanup()

    def save(self, rows):
        self.path.write_text(''.join(json.dumps(r)+'\n' for r in rows))

    def test_unrewarded_place_and_identical_stages_are_valid(self):
        self.save([row(placed_v2=1), row(step=8, **dict.fromkeys(records.STAGES, 1))])
        self.assertEqual(records.read_records(self.path, 'placed_v2'), ([4., 8.], [1., 1.]))

    def test_missing_and_invalid_records_are_not_zeros_or_score(self):
        for r in [{'step': 4, 'episode/score': 7}, {**row(), 'episode/train_ep_record_valid': 0}]:
            self.save([r])
            with self.assertRaises(ValueError):
                records.read_records(self.path, 'picked')

    def test_true_subset_violation_and_partial_json_fail(self):
        self.save([row(contact=1)])
        with self.assertRaisesRegex(ValueError, 'contact without picked'):
            records.read_records(self.path, 'contact')
        self.path.write_text('{"step":')
        with self.assertRaisesRegex(ValueError, 'closed snapshot'):
            records.read_records(self.path, 'picked')

    def test_new_curves_have_place_and_label_slide_as_diagnostic(self):
        for arm in ('h0', 'm0'):
            d = self.root / arm; d.mkdir()
            (d/'metrics.jsonl').write_text(''.join(json.dumps(row(step=4*(i+1), picked=int(i>=2),
                placed_v2=int(i>=4), contact=int(i>=6), contact_push=int(i>=6),
                nested=int(i>=8), slide_success=int(i>=8)))+'\n' for i in range(20)))
        out = self.root / 'new.csv'
        cmd = [sys.executable, str(HERE/'learning_curves.py'), str(self.root), str(out), '4',
               '--episode-record', '--full-human-pattern', 'h%d', '--full-machine-pattern', 'm%d', '--seeds', '1']
        subprocess.run(cmd, check=True, capture_output=True)
        with out.open() as handle:
            rows = list(csv.DictReader(line for line in handle if not line.startswith('#')))
        stages = {r['stage'] for r in rows}
        self.assertIn('placed_v2', stages)
        self.assertIn('slide_within_episode_diagnostic', stages)
        self.assertIn('nested_proxy', stages)
        self.assertNotIn('nested_honest', stages)
        self.assertTrue(all(r['seed_ids']=='0' for r in rows))
        before = out.read_bytes()
        self.assertNotEqual(subprocess.run(cmd, capture_output=True).returncode, 0)
        self.assertEqual(before, out.read_bytes())

    def test_legacy_csv_is_byte_identical(self):
        original = subprocess.check_output(['git', 'show', 'HEAD:HRI_results/curves/learning_curves.py'], cwd=HERE)
        old = self.root / 'legacy.py'; old.write_bytes(original)
        for arm in ('dHfull_all', 'dDPfull'):
            d = self.root / ('full_r2d_state_' + arm + '_bnormclampS8ent5_s0'); d.mkdir()
            (d/'metrics.jsonl').write_text(''.join(json.dumps({'step':(i+1)*4,
                'episode/score':0 if i<2 else 1 if i<6 else 3 if i<10 else 7})+'\n' for i in range(20)))
        a, b = self.root/'old.csv', self.root/'new.csv'
        for script, out in [(old,a),(HERE/'learning_curves.py',b)]:
            subprocess.run([sys.executable,str(script),str(self.root),str(out),'4'], check=True,capture_output=True)
        self.assertEqual(a.read_bytes(), b.read_bytes())
        self.assertEqual(a.with_name('old_seeds.csv').read_bytes(), b.with_name('new_seeds.csv').read_bytes())


if __name__ == '__main__':
    unittest.main()
