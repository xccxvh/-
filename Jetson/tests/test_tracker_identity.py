# -*- coding: utf-8 -*-
"""身份门控回归：错误候选不能建立身份，也不能污染速度和参考。"""
import unittest

from ball_detector import BallCandidate
from ball_tracker import BallTracker


def candidate(x=400, width=40, height=12, appearance=40., **extra):
    metadata = dict(identity_valid=True, in_search_roi=True,
                    appearance=appearance, coordinate_version='relative-cap-v1',
                    measurement_y_px=320.)
    metadata.update(extra)
    return BallCandidate(x, width, height, .9, 'relative_cap', metadata)


class TrackerIdentityTests(unittest.TestCase):
    def tracker(self, **cfg):
        options = dict(identity_check=True, acquire_frames=1)
        options.update(cfg)
        return BallTracker(options, 1280, 720)

    def test_legacy_default_still_accepts_without_metadata(self):
        t = BallTracker({'acquire_frames': 1}, 1280, 720)
        self.assertTrue(t.update([BallCandidate(400, 40, 12, .9, 'geom')], 0)['valid'])

    def test_required_metadata_and_geometry_fail_closed(self):
        bad = []
        for key in candidate().extra:
            c = candidate()
            del c.extra[key]
            bad.append(c)
        for key, values in [('identity_valid', [False, 1, 'true']),
                            ('in_search_roi', [False, 1, 'true']),
                            ('coordinate_version', ['old']),
                            ('appearance', [float('nan'), float('inf'), None, [], '40']),
                            ('measurement_y_px', [float('nan'), -1, 720, None])]:
            bad.extend(candidate(**{key: value}) for value in values)
        for key in ['width', 'height']:
            for value in [0, -1, float('nan'), float('inf')]:
                bad.append(candidate(**{key: value}))
        c = candidate()
        c.method = 'geom'
        bad.append(c)
        for c in bad:
            with self.subTest(candidate=c.__dict__):
                out = self.tracker().update([c], 0)
                self.assertFalse(out['valid'])
                self.assertEqual(out.get('reason'), 'identity_rejected')

    def test_explicit_outside_roi_never_valid_even_legacy_mode(self):
        for enabled in [False, True]:
            out = self.tracker(identity_check=enabled).update(
                [candidate(in_search_roi=False)], 0)
            self.assertFalse(out['valid'])
            self.assertEqual(out.get('reason'), 'identity_rejected')

    def test_continuity_during_acquisition_and_tracking(self):
        for frames in [1, 3]:
            for bad in [candidate(width=100), candidate(width=10),
                        candidate(height=30), candidate(height=3),
                        candidate(appearance=100), candidate(appearance=-40)]:
                with self.subTest(frames=frames, bad=bad.__dict__):
                    t = self.tracker(acquire_frames=frames)
                    t.update([candidate()], 0)
                    out = t.update([bad], .03)
                    self.assertFalse(out['valid'])
                    self.assertEqual(out.get('reason'), 'identity_rejected')

    def test_compatible_candidate_wins_over_closer_wrong_identity(self):
        t = self.tracker()
        t.update([candidate()], 0)
        out = t.update([candidate(400, appearance=100), candidate(410)], .03)
        self.assertTrue(out['valid'])
        self.assertEqual(out['x_px'], 410)
        self.assertEqual(out.get('reason'), 'tracked')

    def test_rejection_preserves_velocity_and_identity_until_loss(self):
        t = self.tracker(velocity_alpha=1.)
        t.update([candidate()], 0)
        t.update([candidate(410)], .02)
        out = t.update([candidate(420, appearance=100)], .04)
        self.assertTrue(out['predicted'])
        self.assertFalse(out['valid'])
        self.assertAlmostEqual(t.v, 500.)
        out = t.update([candidate(430)], .06)
        self.assertTrue(out['valid'])
        self.assertAlmostEqual(t.v, 500.)

    def test_failed_acquisition_clears_identity(self):
        t = self.tracker(acquire_frames=2)
        t.update([candidate()], 0)
        self.assertEqual(t.update([candidate(appearance=100)], .03).get('reason'),
                         'identity_rejected')
        self.assertEqual(t.update([candidate(appearance=100)], .06).get('reason'), 'acquiring')
        self.assertTrue(t.update([candidate(appearance=100)], .09)['valid'])

    def test_gap_and_loss_clear_old_identity(self):
        for gap in [True, False]:
            t = self.tracker(uncertain_frames=1)
            t.update([candidate()], 0)
            if not gap:
                t.update([], .03)
                self.assertEqual(t.update([], .06)['state'], 'LOST')
            self.assertTrue(t.update([candidate(appearance=100)], 1 if gap else .09)['valid'])

    def test_reasons_cover_empty_ambiguity_acquisition_and_position(self):
        t = self.tracker(acquire_frames=2)
        self.assertEqual(t.update([], 0).get('reason'), 'no_candidates')
        self.assertEqual(t.update([candidate(400), candidate(600)], .03).get('reason'), 'ambiguous')
        self.assertEqual(t.update([candidate()], .06).get('reason'), 'acquiring')
        self.assertEqual(t.update([candidate()], .09).get('reason'), 'tracked')
        self.assertEqual(t.update([candidate(394), candidate(406)], .12).get('reason'), 'ambiguous')
        self.assertEqual(t.update([candidate(1000)], .15).get('reason'), 'position_rejected')
        self.assertEqual(t.update([], .18).get('reason'), 'no_candidates')

    def test_invalid_time_reason_and_identity_unchanged(self):
        for ts in [0, -1, float('nan'), float('inf')]:
            t = self.tracker()
            t.update([candidate()], 0)
            out = t.update([candidate(appearance=100)], ts)
            self.assertFalse(out['valid'])
            self.assertEqual(out.get('reason'), 'time')
            self.assertTrue(t.update([candidate()], .03)['valid'])

    def test_moderate_size_and_contrast_change_is_accepted(self):
        t = self.tracker(acquire_frames=2)
        t.update([candidate()], 0)
        self.assertTrue(t.update([candidate(width=48, height=14, appearance=45)], .03)['valid'])


if __name__ == '__main__':
    unittest.main()
