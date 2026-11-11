"""Research surface: physical limits, source support and held-out conditions."""
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from studies.ipex_force_surface import (
    DATA, IpexForceSurface, fit_surface, load_data, normalized_force, validate_surface,
)


class IpexSurfaceTests(unittest.TestCase):
    def setUp(self):
        self.rows, self.manifest = load_data()
        self.surface = IpexForceSurface(fit_surface(self.rows), self.manifest['source_envelope'])
        self.request = dict(depth_m=.016, feed_speed_m_s=.02, drum_width_m=.1989,
                            drum_diameter_m=.2375, rpm=8.5*.02/(math.pi*.2375)*60)

    def test_units_and_versioned_published_trend_provenance(self):
        self.assertEqual(self.rows.shape, (8, 3))
        self.assertEqual(self.manifest['data_kind'], 'digitized_published_trend')
        self.assertEqual(self.manifest['source']['ntrs_id'], '20210025846')
        self.assertEqual(self.manifest['validation']['independent_hardware_validation'], 'unavailable')
        # 30 mN/mm = 30 N/m, hence 6 N on a 200 mm-wide drum.
        self.assertAlmostEqual(30*.2, 30*200/1000)

    def test_offline_fit_recovers_independent_synthetic_coefficients(self):
        # Synthetic fixture only: exact pressure=1000 + 10000*v Pa.
        rows = np.array([[d, v, d*(1000+10000*v)] for d in (.01, .015, .02, .025) for v in (.01, .03)])
        np.testing.assert_allclose(fit_surface(rows), (1000, 10000), atol=1e-9)
        result = validate_surface(rows)
        self.assertLess(result['leave_one_depth_out']['max_absolute_error_N_m'], 1e-10)

    def test_zero_depth_limit_is_zero_but_outside_source_support(self):
        request = {**self.request, 'depth_m': 0.}
        with self.assertRaises(ValueError):
            self.surface.evaluate(**request)
        result = self.surface.evaluate(**request, allow_extrapolation=True)
        self.assertEqual(result.force_per_drum_N, 0.)
        self.assertEqual(result.regime, 'extrapolated')

    def test_one_drum_width_scaling_and_four_drum_magnitude_accounting(self):
        first = self.surface.evaluate(**self.request)
        second = self.surface.evaluate(**{**self.request, 'drum_width_m': 2*self.request['drum_width_m']},
                                       allow_extrapolation=True)
        self.assertAlmostEqual(second.force_per_drum_N, 2*first.force_per_drum_N)
        self.assertAlmostEqual(first.four_drum_force_magnitude_sum_N, 4*first.force_per_drum_N)
        self.assertAlmostEqual(first.force_per_drum_N, first.force_per_width_N_m*.1989)
        self.assertIn('no independent hardware validation', first.validation_status)

    def test_source_condition_and_unidentified_rpm_are_explicit(self):
        supported = self.surface.evaluate(**self.request)
        self.assertEqual(supported.source_envelope_violations, ())
        altered = {**self.request, 'rpm': self.request['rpm']*2}
        with self.assertRaisesRegex(ValueError, 'independent_RPM_effect_unidentified'):
            self.surface.evaluate(**altered)
        diagnostic = self.surface.evaluate(**altered, allow_extrapolation=True)
        self.assertEqual(diagnostic.force_per_drum_N, supported.force_per_drum_N)
        self.assertTrue(diagnostic.source_envelope_violations)

    def test_benchmark_is_not_clipped_or_silently_accepted(self):
        request = dict(depth_m=.025, feed_speed_m_s=.2, drum_width_m=.335, drum_diameter_m=.359, rpm=120.)
        with self.assertRaisesRegex(ValueError, 'feed_speed_m_s_above_source_range'):
            self.surface.evaluate(**request)
        result = self.surface.evaluate(**request, allow_extrapolation=True)
        self.assertEqual(result.regime, 'extrapolated')
        self.assertEqual(len(result.source_envelope_violations), 3)
        self.assertGreater(result.force_per_drum_N, 200.)
        self.assertAlmostEqual(result.tip_speed_to_feed_speed, 11.278317626387357)

    def test_invalid_inputs_and_unidentifiable_fits_raise(self):
        for changes in ({'rpm': 0.}, {'feed_speed_m_s': 0.}, {'depth_m': -.01},
                        {'drum_width_m': math.nan}, {'drum_diameter_m': math.inf}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.surface.evaluate(**{**self.request, **changes}, allow_extrapolation=True)
        for rows in ([[.01, .01, 2.], [.02, .01, 4.]], [[.01, .01, -2.], [.02, .03, 4.]]):
            with self.assertRaises(ValueError):
                fit_surface(rows)

    def test_whole_depth_series_are_held_out_and_errors_are_quantified(self):
        result = validate_surface(self.rows)
        self.assertEqual(len(result['folds']), 4)
        for fold in result['folds']:
            self.assertNotIn(fold['held_out_depth_m'], fold['training_depths_m'])
            self.assertEqual(len(fold['observed_N_m']), 2)
        self.assertAlmostEqual(result['leave_one_depth_out']['mae_N_m'], 3.657180625354632, places=7)
        self.assertGreater(result['leave_one_depth_out']['rmse_N_m'],
                           result['in_sample_curve_approximation']['rmse_N_m'])

    def test_smooth_surface_derivatives_and_monotone_depth_and_speed(self):
        a, b = self.surface.coefficients
        d, v, h = .017, .023, 1e-7
        f = lambda depth, speed: normalized_force(depth, speed, (a, b))
        self.assertAlmostEqual((f(d+h, v)-f(d-h, v))/(2*h), a+b*v, places=5)
        self.assertAlmostEqual((f(d, v+h)-f(d, v-h))/(2*h), d*b, places=5)
        self.assertGreater(f(d+.001, v), f(d, v))
        self.assertGreater(f(d, v+.001), f(d, v))

    def test_tampered_processed_data_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder)
            for name in ('manifest.json', 'figure13_trends.csv'):
                (destination/name).write_bytes((DATA/name).read_bytes())
            with (destination/'figure13_trends.csv').open('a') as stream:
                stream.write('\n')
            with self.assertRaisesRegex(ValueError, 'hash mismatch'):
                load_data(destination)


if __name__ == '__main__':
    unittest.main()
