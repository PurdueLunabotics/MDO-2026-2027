"""Analytical soil, staged-path, work and failure checks; no measured calibration."""
import math
import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_totals

from rover.alternatives.moving_excavation import MovingExcavation, moving_supply, operating_pose
from rover.linkage import drum_pose
from rover.motors import ReferenceMotorProvider


class DoubleElectrical:
    def __init__(self, motor):
        self.motor = motor

    def __getattr__(self, name):
        return getattr(self.motor, name)

    def evaluate(self, work, peak):
        return {k: 2*v for k, v in self.motor.evaluate(work, peak).items()}


class DoubleElectricalProvider(ReferenceMotorProvider):
    @classmethod
    def create(cls, get, role, **options):
        return DoubleElectrical(super().create(get, role, **options))


class SyntheticHalfCapture(MovingExcavation):
    """Synthetic response hook, not measured soil calibration."""
    def granular_response(self, inputs, froude):
        capture, lift = super().granular_response(inputs, froude)
        return capture/2, lift


class MovingExcavationTests(unittest.TestCase):
    def test_function_factory_keeps_moving_constraints_and_staged_motion(self):
        from rover.models import ModelSet
        from rover.problem import build_problem
        calls = []
        def factory():
            calls.append(1)
            return MovingExcavation()
        p = build_problem(models=ModelSet(excavation=factory))
        self.addCleanup(p.cleanup)
        p.run_model()
        self.assertEqual(len(calls), 1)
        self.assertIn("feed_margin", p.margin_specs)
        self.assertIn("axle_ground_margin", p.margin_specs)
        self.assertIn("feed_speed", p.design_specs)
        self.assertAlmostEqual(p.get_val("actuation_time").item(),
                               4*p.get_val("lowering_time").item())

    def component(self, provider=ReferenceMotorProvider, component=MovingExcavation, **changes):
        problem = om.Problem(reports=False)
        problem.model.add_subsystem("excavation", component(motor_model=provider), promotes=["*"])
        problem.setup()
        self.addCleanup(problem.cleanup)
        values = dict(dry_mass=60., drum_mass=8., extended_x=.6, extended_y=-.02,
                      travel_y=.7, lowering_time=4., feed_speed=.005,
                      lin_act_side_arm_offset=0., lin_act_top_arm_offset=0.)
        values.update(changes)
        for name, value in values.items():
            problem.set_val(name, value)
        problem.run_model()
        return problem

    def value(self, problem, name):
        return problem.get_val(name).item()

    def test_tandem_soil_conservation_and_independent_capacity(self):
        result = moving_supply(.15, .2, .03, .6, .8, 1200., 1500., .75,
                               .005, 20., 4.)
        expected_capacity = 4*math.pi*.12**2*.2*.6*.8
        self.assertAlmostEqual(result["storage_capacity"], expected_capacity)
        self.assertAlmostEqual(result["payload_mass"], expected_capacity*1200.)
        # Each of the two tracks has width L, depth 2*s and length X.
        independent_fresh = 2*.2*(2*.03)*result["feed_distance"]
        self.assertAlmostEqual(result["fresh_volume"], independent_fresh)
        self.assertAlmostEqual(result["payload_mass"], independent_fresh*1500.*.75)
        self.assertAlmostEqual(result["soil_supply_rate"], 2*.2*(2*.03)*.005)
        # The rear's top layer is already removed: do not count front s + rear 2s.
        self.assertNotAlmostEqual(result["fresh_volume"], 2*.2*(3*.03)*result["feed_distance"])

    def test_per_scoop_volume_boundary_and_raw_failure(self):
        boundary = .012*4*20/60
        p = self.component(feed_speed=boundary)
        self.assertAlmostEqual(self.value(p, "feed_margin"), 0.)
        self.assertAlmostEqual(self.value(p, "soil_supply_rate"), self.value(p, "scoop_throughput"))
        force = self.value(p, "cut_force")
        p.set_val("feed_speed", 2*boundary)
        p.run_model()
        self.assertLess(self.value(p, "feed_margin"), 0.)
        self.assertAlmostEqual(self.value(p, "cut_force"), 2*force)
        self.assertGreater(self.value(p, "soil_supply_rate"), self.value(p, "scoop_throughput"))

    def test_inverse_operating_pose_closes_eccentric_linkage(self):
        args = (.6, .13, .15, .15, .45)
        x, z, actuator, reserve = operating_pose(*args, .07, -.05, .04)
        actual = drum_pose(*args, actuator, -.05, .04)
        np.testing.assert_allclose(actual, [x, z], atol=1e-12)
        self.assertGreater(reserve, 0.)
        self.assertAlmostEqual(math.hypot(x-.13, z-.15), .6)

    def test_staged_pass_distance_and_acceleration_time(self):
        p = self.component()
        v = lambda n: self.value(p, n)
        lane = v("feed_distance")
        spacing = v("front_dig_x")-v("rear_dig_x")
        self.assertAlmostEqual(v("feed_drive_distance"), lane+spacing)
        self.assertGreater(v("feed_time"), v("feed_drive_distance")/v("feed_speed"))
        self.assertGreater(v("pair_cutting_time"), lane/v("feed_speed"))
        self.assertAlmostEqual(v("dig_overlap_time"), 0.)
        self.assertAlmostEqual(v("front_dig_depth"), v("inlet_depth"))
        self.assertAlmostEqual(v("rear_dig_depth"), 2*v("inlet_depth"))
        self.assertAlmostEqual(v("front_dig_y"), v("drum_diameter")/2-v("inlet_depth"))
        self.assertEqual(v("inlet_width"), v("drum_length"))
        self.assertEqual(v("cut_depth"), v("inlet_depth"))
        self.assertAlmostEqual(v("cut_force"), 4*v("drum_length")*v("cut_pressure")*v("feed_per_scoop"))

    def test_cutting_work_distance_limit_and_electrical_loss_once(self):
        p = self.component(internal_lift_factor=0., actuator_idle_power=0.)
        v = lambda n: self.value(p, n)
        radius = v("drum_diameter")/2
        omega = v("drum_rpm")*math.pi/30
        inertia = (v("drum_mass")+v("payload_mass"))*radius**2
        # Integrating pressure * bite * edge speed over path removes speed/rpm.
        cutting = v("cut_pressure")*4*v("drum_length")*radius
        cutting *= 2*math.pi/v("scoops_per_revolution")*v("feed_distance")
        expected = (cutting+inertia*omega**2)/v("drum_efficiency")
        self.assertAlmostEqual(v("dig_energy")-v("feed_drive_energy"), expected, places=8)
        p.set_val("feed_speed", .003)
        p.set_val("drum_rpm", 35.)
        p.run_model()
        new_omega = 35*math.pi/30
        expected = (cutting+inertia*new_omega**2)/v("drum_efficiency")
        self.assertAlmostEqual(v("dig_energy")-v("feed_drive_energy"), expected, places=8)

    def test_motor_provider_changes_electrical_accounting_without_mechanical_demands(self):
        normal = self.component(actuator_idle_power=0.)
        doubled = self.component(DoubleElectricalProvider, actuator_idle_power=0.)
        for name in ("payload_mass", "cut_force", "feed_time", "drum_torque_margin",
                     "feed_motor_margin", "feed_traction_margin"):
            self.assertEqual(self.value(normal, name), self.value(doubled, name))
        for name in ("dig_energy", "dig_power", "dump_energy", "dump_power",
                     "feed_drive_energy", "feed_drive_peak_power"):
            self.assertAlmostEqual(self.value(doubled, name), 2*self.value(normal, name))

    def test_granular_hook_capture_changes_fresh_supply_without_inventing_payload(self):
        normal = self.component()
        half = self.component(component=SyntheticHalfCapture)
        self.assertEqual(self.value(normal, "payload_mass"), self.value(half, "payload_mass"))
        self.assertAlmostEqual(self.value(half, "fresh_volume"), 2*self.value(normal, "fresh_volume"))
        self.assertAlmostEqual(self.value(half, "feed_distance"), 2*self.value(normal, "feed_distance"))

    def test_smooth_operating_derivatives(self):
        p = self.component()
        checked = p.check_totals(
            of=["feed_time", "dig_energy", "feed_margin", "operating_reach_margin", "payload_mass"],
            wrt=["feed_speed", "drum_rpm", "inlet_depth"],
            method="fd", form="central", step=1e-6, out_stream=None)
        assert_check_totals(checked, atol=2e-3, rtol=2e-3)

    def test_reach_and_domain_failures_remain_finite_and_signed(self):
        p = self.component(extended_y=.13)
        self.assertLess(self.value(p, "axle_ground_margin"), 0.)
        self.assertLess(self.value(p, "operating_reach_margin"), 0.)
        q = self.component(arm_length=.3, bottom_pivot_height=.8)
        self.assertLess(self.value(q, "excavation_domain_margin"), 0.)
        self.assertTrue(all(np.isfinite(q.get_val(n)).all() for n in q.model.excavation.OUTPUT_UNITS))
        singular = self.component(inlet_depth=.14)
        self.assertLess(self.value(singular, "excavation_domain_margin"), -1e-6)
        for kwargs in (dict(speed=0.), dict(capture=0.), dict(inlet_depth=.2), dict(inlet_depth=0.)):
            values = dict(radius=.15, length=.2, inlet_depth=.03,
                          fill=.6, usable=.8, stored_density=1200., soil_density=1500.,
                          capture=.75, speed=.005, rpm=20., scoops=4.)
            values.update(kwargs)
            with self.assertRaises(ValueError):
                moving_supply(**values)
        with self.assertRaises(ValueError):
            operating_pose(.3, .1, .8, .1, .9, 0.)

    def test_single_depth_contract_full_width_and_explicit_legacy_migration(self):
        from rover.problem import build_problem
        old = dict(cut_depth=.014, scoop_opening=.035,
                   scoop_depth_fraction=.5, cut_engagement=.15)
        p = build_problem(old)
        self.addCleanup(p.cleanup)
        p.run_model()
        self.assertIn("inlet_depth", p.design_specs)
        for name in old:
            self.assertNotIn(name, p.design_specs)
            self.assertNotIn(name, p.assumption_specs)
        self.assertEqual(p.get_val("inlet_depth").item(), .014)
        self.assertEqual(p.get_val("front_dig_depth").item(), .014)
        self.assertEqual(p.get_val("rear_dig_depth").item(), .028)
        self.assertEqual(p.get_val("inlet_width").item(), p.get_val("drum_length").item())
        self.assertEqual(p.model._conn_global_abs_in2out["linkage_loads.cut_depth"], "excavation.cut_depth")
        migration = p.model_metadata["excavation_operation"]["input_migration"]
        self.assertEqual(migration["retired_inputs"], old)
        with self.assertRaisesRegex(ValueError, "mix"):
            build_problem(dict(inlet_depth=.014, cut_depth=.012))


if __name__ == "__main__":
    unittest.main()
