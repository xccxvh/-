# -*- coding: utf-8 -*-
"""CLI契约测试：临时合成CSV/字节文件仅用于测试，绝非实物标定。"""
import csv
import json
from pathlib import Path
import tempfile
import unittest

from test_rail_calibration import binding, load_module


class CalibrateRailTests(unittest.TestCase):
    def setUp(self):
        self.m = load_module('calibrate_rail')
        self.api = load_module('rail_calibration')
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.sidecar = self.root/'binding.json'
        self.sidecar.write_text(json.dumps(binding()))
        self.video = self.root/'synthetic_video_bytes.bin'
        self.video.write_bytes(b'SYNTHETIC TEST ONLY - NOT A REAL CALIBRATION VIDEO')

    def collect(self, cm, start, output, predicted=True, mutate=None, missing=0):
        path = self.root/('source_%d.csv' % start)
        fields = ['frame', 'rail_s_px', 'pixel_valid', 'reference_valid', 'calibration_valid', 'cm_valid', 'rail_binding_id']
        if predicted: fields.append('predicted')
        rows = []
        for i in range(20):
            row = dict(frame=start+i, rail_s_px=500+20*cm, pixel_valid=1, reference_valid=1,
                       calibration_valid=0, cm_valid=0, rail_binding_id=self.api.binding_id(binding()))
            if predicted: row['predicted'] = 0
            if mutate: mutate(row, i)
            rows.append(row)
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows[missing:])
        args = ['collect', '--csv', str(path), '--video', str(self.video), '--binding', str(self.sidecar),
                '--known-position', '--cm', str(cm), '--start-frame', str(start), '--end-frame', str(start+19),
                '--output', str(self.root/output)]
        return self.m.main(args)

    def test_collect_keeps_invalid_and_missing_frames_in_denominator(self):
        def mutate(row, i):
            if i == 1: row['reference_valid'] = 0
            if i == 2: row['predicted'] = 1
            if i == 3: row['pixel_valid'] = 0
        data = self.collect(0, 0, 's.json', mutate=mutate, missing=1)
        info = self.api.session_values(data, binding())
        self.assertEqual(info['total_frames'], 20)
        self.assertEqual(info['samples'], 16)
        self.assertEqual(info['coverage'], .8)
        self.assertTrue(data['known_position'])
        self.assertEqual(data['source']['video_sha256'], self.m.sha256(self.video))

    def test_missing_predicted_uses_explicit_pixel_valid_contract(self):
        data = self.collect(-5, 0, 's.json', predicted=False)
        self.assertEqual(data['prediction_contract'], 'pixel_valid')
        self.assertEqual(self.api.session_values(data, binding())['samples'], 20)

    def test_output_exclusive_and_binding_mismatch_rejected(self):
        self.collect(0, 0, 's.json')
        original = (self.root/'s.json').read_bytes()
        with self.assertRaises(FileExistsError): self.collect(0, 0, 's.json')
        self.assertEqual((self.root/'s.json').read_bytes(), original)
        with self.assertRaises(ValueError):
            self.collect(0, 100, 'bad.json', mutate=lambda row, i: row.update(rail_binding_id='0'*64))
        self.assertFalse((self.root/'bad.json').exists())

    def test_end_to_end_fit_then_independent_verify_version_chain(self):
        for cm, start, name in [(-5, 0, 'minus.json'), (0, 100, 'zero.json'), (5, 200, 'plus.json'),
                                (-3, 400, 'verify_minus.json'), (3, 500, 'verify_plus.json')]:
            self.collect(cm, start, name)
        fit = self.m.main(['fit', '--binding', str(self.sidecar), '--sessions',
                          str(self.root/'minus.json'), str(self.root/'zero.json'), str(self.root/'plus.json'),
                          '--output', str(self.root/'model-v1.json')])
        self.assertFalse(fit['validated'])
        self.assertIsNone(self.api.RailCalibration(self.root/'model-v1.json', binding()).position(500))
        checked = self.m.main(['verify', '--binding', str(self.sidecar), '--model', str(self.root/'model-v1.json'),
                              '--sessions', str(self.root/'verify_minus.json'), str(self.root/'verify_plus.json'),
                              '--output', str(self.root/'model-v2.json')])
        self.assertTrue(checked['validated'])
        self.assertEqual(checked['parent']['sha256'], self.m.sha256(self.root/'model-v1.json'))
        self.assertAlmostEqual(self.api.RailCalibration(self.root/'model-v2.json', binding()).position(540), 2)
        self.assertFalse(json.loads((self.root/'model-v1.json').read_text())['validated'])

    def test_confirmation_and_frame_range_flags_are_required(self):
        args = ['collect', '--csv', 'unused', '--video', str(self.video), '--binding', str(self.sidecar),
                '--cm', '0', '--start-frame', '0', '--end-frame', '19', '--output', str(self.root/'s.json')]
        with self.assertRaises(SystemExit): self.m.main(args)
        args.append('--known-position'); args.remove('--end-frame'); args.remove('19')
        with self.assertRaises(SystemExit): self.m.main(args)

    def test_automatic_sidecar_and_explicit_binding_file_alias(self):
        self.collect(0, 0, 'first.json', predicted=False)
        auto = self.root/'source_0.binding.json'
        auto.write_text(json.dumps({'binding':binding(), 'rail_binding_id':self.api.binding_id(binding())}))
        args = ['collect', '--csv', str(self.root/'source_0.csv'), '--video', str(self.video),
                '--known-position', '--cm', '0', '--start-frame', '0', '--end-frame', '19',
                '--output', str(self.root/'auto.json')]
        result = self.m.main(args)
        self.assertEqual(result['source']['binding_path'], str(auto))
        args[args.index(str(self.root/'auto.json'))] = str(self.root/'explicit.json')
        args += ['--binding-file', str(auto)]
        self.assertEqual(self.m.main(args)['rail_binding_id'], result['rail_binding_id'])

    def test_sidecar_wrapper_id_checked(self):
        self.sidecar.write_text(json.dumps({'binding':binding(), 'rail_binding_id':'0'*64}))
        with self.assertRaises(ValueError): self.collect(0, 0, 'bad.json')


if __name__ == '__main__':
    unittest.main()
