"""Independent statics limits and display conversions for the report plot."""
import math
import unittest
import xml.etree.ElementTree as ET

from reporting.actuator_force_drawing import force_stroke_data, force_stroke_svg


class ActuatorForceReportTests(unittest.TestCase):
    def values(self):
        return dict(arm_length=1., pivot_x=0., bottom_pivot_height=0.,
                    actuator_pivot_x=0., top_pivot_height=1.,
                    lin_act_side_arm_offset=0., lin_act_top_arm_offset=0.,
                    travel_actuator_length=1.1, deposition_actuator_length=1.1,
                    actuator_retracted_length=1.1, actuator_extended_length=1.8,
                    actuator_catalog_retracted_length=1.,
                    drum_mass=4., payload_mass=8., arm_linear_mass=2., gravity=10.,
                    breakout_force=10., actuator_force_rating=200., actuator_safety_factor=2.)

    def test_per_actuator_force_from_independent_moment_balance(self):
        # For B=(0,0), A=(0,1), D=C and L=1, moment balance gives
        # F = actuator_length*(pair weight + half arm weight).
        data = force_stroke_data(self.values())
        self.assertEqual(len(data['points']), 81)
        for point in data['points']:
            self.assertAlmostEqual(point['empty_force_n'], 30*point['length_m'])
            self.assertAlmostEqual(point['loaded_force_n'], 70*point['length_m'])
            self.assertAlmostEqual(point['stroke_m'], point['length_m']-1.)
        self.assertAlmostEqual(max(p['loaded_force_n'] for p in data['points']), 126.)
        self.assertEqual(data['allowable_force_n'], 100.)  # Overload remains visible.

    def test_legacy_allowance_cannot_change_gravity_curves(self):
        values = self.values()
        expected = force_stroke_data(values)
        values['breakout_force'] = 1e12
        self.assertEqual(force_stroke_data(values), expected)
        svg = force_stroke_svg(expected)
        self.assertNotIn('data-load="breakout"', svg)
        self.assertIn('Quasi-static gravity loads only', svg)

    def test_unit_conversion_and_both_grid_directions(self):
        data = force_stroke_data(self.values())
        ns = '{http://www.w3.org/2000/svg}'
        metric, imperial = (ET.fromstring(force_stroke_svg(data, u)) for u in ('metric', 'imperial'))
        for root in (metric, imperial):
            lines = list(root.iter(ns+'line'))
            self.assertEqual(sum(n.get('data-grid') == 'x' for n in lines), 6)
            self.assertEqual(sum(n.get('data-grid') == 'y' for n in lines), 6)
        capacity = lambda root: next(float(n.get('data-capacity')) for n in root.iter() if n.get('data-load') == 'capacity')
        self.assertEqual(capacity(metric), 100.)
        self.assertAlmostEqual(capacity(imperial), 100/4.4482216152605)
        self.assertIn('Force per actuator (N)', ''.join(metric.itertext()))
        self.assertIn('Stroke from hardware minimum (in)', ''.join(imperial.itertext()))
        self.assertIn('39.370', ''.join(imperial.itertext()))  # One metre, independent inch conversion.

    def test_zero_load_limit_and_generic_stroke_reference(self):
        values = self.values()
        values.update(drum_mass=0., payload_mass=0., arm_linear_mass=0., breakout_force=0.)
        values.pop('actuator_catalog_retracted_length')
        data = force_stroke_data(values)
        self.assertFalse(data['hardware_origin'])
        self.assertEqual(data['origin_m'], 1.1)
        for point in data['points']:
            for name in ('empty', 'loaded'):
                self.assertEqual(point[name+'_force_n'], 0.)

    def test_guarded_poses_and_nonfinite_loads_do_not_become_force_curves(self):
        for changes in ({'actuator_extended_length': 2.1}, {'actuator_extended_length': 2.},
                        {'payload_mass': math.nan}, {'actuator_safety_factor': 0.}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                force_stroke_data({**self.values(), **changes})


if __name__ == '__main__':
    unittest.main()
