"""Physical incline balance, full-load accounting and strict failure reporting."""
import math
import unittest

from rover.alternatives.bekker_wong_reece import load_preset, NoDrivingSolution, solve_operating_point, wheel_at_slip
from rover.alternatives.slope_traversal import slope_response
from rover.config import ASSUMPTIONS
from rover.problem import build_problem
from reporting.report import snapshot


class SlopeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.soil, _ = load_preset()

    def test_high_slip_allowed_but_locked_wheel_fails(self):
        # Independently prescribe force at 99.5% slip, beyond the old 98% cap.
        point = wheel_at_slip(self.soil, .165, .125, 150., .995)
        solved = solve_operating_point(self.soil, .165, .125, 150., point['drawbar_pull_n'])
        self.assertAlmostEqual(solved['slip'], .995, places=8)
        locked = wheel_at_slip(self.soil, .165, .125, 150., 1.)
        with self.assertRaises(NoDrivingSolution):
            solve_operating_point(self.soil, .165, .125, 150., locked['drawbar_pull_n'])

    def test_independent_incline_force_balance_and_failure(self):
        mass, angle = 63., 5.
        result = slope_response(self.soil, .33, .125, mass, angle)
        normal = mass*9.80665*math.cos(math.radians(angle))/4
        expected = mass*9.80665*math.sin(math.radians(angle))/4
        point = wheel_at_slip(self.soil, .165, .125, normal, result['slip'])
        self.assertAlmostEqual(point['drawbar_pull_n'], expected, places=7)
        self.assertEqual(result['solution_valid'], 1.)
        failed = slope_response(self.soil, .33, .125, mass, 50.)
        self.assertEqual(failed['solution_valid'], 0.)
        self.assertEqual(failed['slip'], 1.)
        self.assertGreater(failed['force_shortfall'], 0.)
        self.assertLess(failed['traversal_margin'], 0.)

    def test_configurable_requirements_mass_accounting_and_strict_report(self):
        p = build_problem({'empty_slope_degrees': 5., 'loaded_slope_degrees': 5.})
        self.addCleanup(p.cleanup)
        p.run_model()
        for state, mass in [('empty', p.get_val('dry_mass').item()),
                            ('loaded', p.get_val('dry_mass').item()+p.get_val('payload_mass').item())]:
            expected = slope_response(self.soil, p.get_val('wheel_diameter').item(), p.get_val('wheel_width').item(), mass, 5.)
            self.assertAlmostEqual(p.get_val(state+'_slope_slip').item(), expected['slip'], places=10)
            self.assertTrue(snapshot(p)['constraints'][state+'_slope_traversal_margin']['passed'])
        # Reporting must reject the boundary even if a solver left a tiny reserve.
        p.set_val('empty_slope_solution_valid', 0.)
        p.set_val('empty_slope_slip', 1.)
        p.set_val('empty_slope_traversal_margin', -1e-12)
        self.assertFalse(snapshot(p)['constraints']['empty_slope_traversal_margin']['passed'])
        p.set_val('empty_slope_solution_valid', 1.)
        p.set_val('empty_slope_traversal_margin', 0.)
        self.assertFalse(snapshot(p)['constraints']['empty_slope_traversal_margin']['passed'])
        # At positive ground speed, arbitrarily high valid slip still passes.
        p.set_val('empty_slope_slip', 1.-1e-9)
        p.set_val('empty_slope_traversal_margin', 1e-9)
        self.assertTrue(snapshot(p)['constraints']['empty_slope_traversal_margin']['passed'])

    def test_default_slopes_follow_editable_configuration(self):
        p = build_problem()
        self.addCleanup(p.cleanup)
        p.run_model()
        for name in ('empty_slope_degrees', 'loaded_slope_degrees'):
            with self.subTest(name=name):
                self.assertEqual(p.get_val(name).item(), ASSUMPTIONS[name][0])

    def test_prescribed_slopes_reject_reference_geometry_and_invalid_settings(self):
        # Freeze this physical failure case independently of editable defaults.
        p = build_problem({'empty_slope_degrees': 25., 'loaded_slope_degrees': 20.})
        self.addCleanup(p.cleanup)
        p.run_model()
        self.assertEqual(p.get_val('empty_slope_degrees').item(), 25.)
        self.assertEqual(p.get_val('loaded_slope_degrees').item(), 20.)
        report = snapshot(p)
        for state in ('empty', 'loaded'):
            self.assertFalse(report['constraints'][state+'_slope_traversal_margin']['passed'])
        for name in ('empty_slope_degrees', 'loaded_slope_degrees'):
            for angle in (-1., 90., float('nan')):
                with self.subTest(name=name, angle=angle), self.assertRaises(ValueError):
                    build_problem({name: angle})


if __name__ == '__main__':
    unittest.main()
