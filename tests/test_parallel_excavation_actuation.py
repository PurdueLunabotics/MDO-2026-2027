"""Independent motion-duration/work limits and connected capacity checks."""
import math
import unittest

from openmdao.utils.assert_utils import assert_check_totals
from openmdao.visualization.n2_viewer.n2_viewer import _get_viewer_data

from rover.alternatives.parallel_excavation_actuation import excavation_motion
from rover.alternatives.pa04_actuator import INCH, RATED_FORCE
from rover.problem import build_problem
from reporting.report import snapshot


class ParallelExcavationTests(unittest.TestCase):
    def values(self, **changes):
        values = dict(travel_actuator_length=.4, front_dig_actuator_length=.5,
                      rear_dig_actuator_length=.55, actuator_speed=.01,
                      actuator_efficiency=1., travel_y=.5, front_dig_y=.2,
                      rear_dig_y=.1, drum_mass=8., payload_mass=20.,
                      arm_linear_mass=0., arm_length=.5, lin_act_side_arm_offset=0.,
                      gravity=10., breakout_force=0., cut_depth=.03,
                      actuator_idle_power=0., required_actuator_force=RATED_FORCE/2)
        values.update(changes)
        return values

    def test_parallel_time_and_independent_gravity_work(self):
        result = excavation_motion(self.values())
        self.assertAlmostEqual(result['actuation_time'], 2*15.)
        # Each arm lowers 40 N empty, then raises 140 N loaded, over its own height.
        self.assertAlmostEqual(result['arm_energy'], (40+140)*(.3+.4))
        self.assertAlmostEqual(result['lowering_duration'], 15.)
        self.assertNotAlmostEqual(result['actuation_time'], 2*(10+15))

    def test_pa04_sums_unequal_actuator_electrical_work(self):
        result = excavation_motion(self.values(), 'pa04-900')
        lower_speed, upper_speed, current = .39*INCH, .43*INCH, 9.
        self.assertAlmostEqual(result['actuation_time'], .15/lower_speed+.15/upper_speed)
        self.assertAlmostEqual(result['arm_energy'], 12*(14*.25/lower_speed+current*.25/upper_speed))
        equal = excavation_motion(self.values(front_dig_actuator_length=.55), 'pa04-900')
        self.assertAlmostEqual(equal['actuation_time'], result['actuation_time'])
        self.assertGreater(equal['arm_energy'], result['arm_energy'])

    def test_zero_motion_and_invalid_domain(self):
        for profile in ('reference', 'pa04-900'):
            zero = excavation_motion(self.values(front_dig_actuator_length=.4, rear_dig_actuator_length=.4), profile)
            self.assertEqual(set(zero.values()), {0.})
            for changes in ({'travel_actuator_length': 0}, {'front_dig_actuator_length': math.nan}):
                with self.assertRaises(ValueError):
                    excavation_motion(self.values(**changes), profile)
        for changes in ({'actuator_speed': 0}, {'actuator_efficiency': 1.1}, {'payload_mass': -1}):
            with self.assertRaises(ValueError):
                excavation_motion(self.values(**changes))
        with self.assertRaises(ValueError):
            excavation_motion(self.values(required_actuator_force=math.inf), 'pa04-900')

    def test_connected_contract_capacity_failures_and_derivatives(self):
        for profile in ('reference', 'pa04-900'):
            with self.subTest(profile=profile):
                p = build_problem(actuators=profile, traversal_mode='fixed')
                self.addCleanup(p.cleanup)
                p.run_model()
                v = lambda n: p.get_val(n).item()
                gravity_only = [v(n) for n in ('required_actuator_force', 'arm_energy', 'actuation_time')]
                p.set_val('breakout_force', 1e12)  # Old case files cannot add an allowance.
                p.run_model()
                self.assertEqual([v(n) for n in ('required_actuator_force', 'arm_energy', 'actuation_time')], gravity_only)
                self.assertAlmostEqual(v('actuation_time'), v('excavation_lowering_time')+v('excavation_raising_time'))
                self.assertIn('excavation_lowering_time', snapshot(p)['outputs'])
                conns = p.model._conn_global_abs_in2out
                self.assertEqual(conns['linkage_loads.rear_dig_actuator_length'], 'excavation.rear_dig_actuator_length')
                declared = _get_viewer_data(p)['declare_partials_list']
                self.assertIn('linkage_loads.actuation_time > linkage_loads.front_dig_actuator_length', declared)
                checks = p.check_totals(of=['actuation_time', 'arm_energy', 'excavation_lowering_time', 'excavation_raising_time'],
                                       wrt=['travel_actuator_length', 'inlet_depth'], method='fd', form='central',
                                       step=3e-5, step_calc='rel_avg', out_stream=None)
                assert_check_totals(checks, atol=1e-3, rtol=2e-3)
                if profile == 'pa04-900':
                    p.set_val('linkage_loads.actuator_leverage_max', 10000.)
                    p.model.linkage_loads.run_solve_nonlinear()
                    self.assertLess(v('actuator_curve_margin'), 0)
                    self.assertLess(v('actuator_force_margin'), 0)
                    self.assertGreater(v('arm_energy'), 0)


if __name__ == '__main__':
    unittest.main()
