import math
import unittest
from dataclasses import replace

import numpy as np
from scipy.integrate import quad

from rover.alternatives.bekker_wong_reece import (
    WheelDomainError, BekkerWongReeceMobility, contact_stress,
    load_preset, rover_on_slope,
    solve_operating_point, transfer_flags, wheel_at_slip, wheel_forces,
)


class RigidWheelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.soil, cls.evidence = load_preset()

    def test_source_units_and_independent_parameter_provenance(self):
        self.assertEqual(self.soil.kc, 221.26*1000)
        self.assertEqual(self.soil.kphi, 1957.51*1000)
        self.assertEqual(self.soil.cohesion_pa, .01*1000)
        self.assertAlmostEqual(math.degrees(self.soil.friction_angle_rad), 33.3)
        self.assertAlmostEqual(self.soil.shear_modulus_m, 2.55/100)
        self.assertEqual(self.evidence["source_conditions"]["relative_density_percent"], 14)
        self.assertIn("NOT measured", self.evidence["transcription"]["stress_shape"])

    def test_radial_stress_endpoints_peak_continuity_and_shear_bound(self):
        entry, slip = .4, .5
        peak = (self.soil.stress_c1+self.soil.stress_c2*slip)*entry
        theta = np.array([0., peak-1e-10, peak, peak+1e-10, entry])
        p, t, j = contact_stress(self.soil, .165, .125, entry, slip, theta)
        self.assertEqual(p[0], 0)
        self.assertEqual(p[-1], 0)
        self.assertAlmostEqual(p[1]/p[2], 1, places=8)
        self.assertAlmostEqual(p[3]/p[2], 1, places=8)
        self.assertEqual(j[-1], 0)
        self.assertTrue(np.all(j >= 0))
        self.assertTrue(np.all(t <= self.soil.cohesion_pa+p*math.tan(self.soil.friction_angle_rad)))

    def test_independent_zero_shear_compaction_and_normal_integral(self):
        soil = replace(self.soil, cohesion_pa=0., friction_angle_rad=0., n=1.)
        r, b, entry, slip = .165, .125, .4, .4
        point = wheel_forces(soil, r, b, entry, slip)
        peak = (soil.stress_c1+soil.stress_c2*slip)*entry
        # Independent scalar implementation and adaptive quadrature.
        def pressure(theta):
            mapped = theta if theta >= peak else entry-theta/peak*(entry-peak)
            return (soil.kc/b+soil.kphi)*r*(math.cos(mapped)-math.cos(entry))
        expected_n = b*r*quad(lambda th: pressure(th)*math.cos(th), 0, entry, points=[peak], epsabs=1e-10)[0]
        expected_r = b*r*quad(lambda th: pressure(th)*math.sin(th), 0, entry, points=[peak], epsabs=1e-10)[0]
        self.assertAlmostEqual(point["normal_force_n"], expected_n, places=8)
        self.assertAlmostEqual(point["drawbar_pull_n"], -expected_r, places=8)
        self.assertEqual(point["torque_nm"], 0.)
        with self.assertRaisesRegex(WheelDomainError, "No driving solution"):
            solve_operating_point(soil, r, b, 150., 0.)

    def test_load_equilibrium_and_power_conservation(self):
        for load in (50., 150., 300.):
            for slip in (.1, .4, .8):
                p = wheel_at_slip(self.soil, .165, .125, load, slip)
                self.assertLess(abs(p["normal_residual_n"]), 1e-7)
                omega = 1.7
                speed = .165*omega*(1-slip)
                dissipation = p["torque_nm"]*omega-p["drawbar_pull_n"]*speed
                self.assertGreater(dissipation, 0.)
                self.assertGreater(p["compaction_resistance_n"], 0.)

    def test_larger_load_changes_sinkage_and_slip(self):
        light = rover_on_slope(self.soil, .33, .125, 40, math.radians(5))
        heavy = rover_on_slope(self.soil, .33, .125, 80, math.radians(5))
        self.assertGreater(heavy["sinkage_m"], light["sinkage_m"])
        self.assertNotAlmostEqual(light["slip"], heavy["slip"], places=3)
        self.assertLess(abs(heavy["force_residual_n"]), 1e-7)

    def test_invalid_parameters_geometry_gravity_and_unsupported_load(self):
        for kwargs in ({"cohesion_pa": -1}, {"shear_modulus_m": 0}, {"n": 0},
                       {"kc": 0, "kphi": 0}, {"stress_c1": .9, "stress_c2": .2}):
            with self.assertRaises(WheelDomainError):
                wheel_at_slip(replace(self.soil, **kwargs), .165, .125, 150., .4)
        for radius, width, load, slip in [(0,.1,150,.2),(.1,0,150,.2),(.1,.1,0,.2),
                                          (.1,.1,150,1.001),(.1,.1,150,-.1),(.1,.1,float('nan'),.2),
                                          (.01,.01,1e8,.4)]:
            with self.assertRaises(WheelDomainError):
                wheel_at_slip(self.soil, radius, width, load, slip)
        for radius, width, entry, slip in [(0,.1,.4,.2),(.1,0,.4,.2),(.1,.1,0,.2),(.1,.1,float('nan'),.2)]:
            with self.assertRaises(WheelDomainError):
                wheel_forces(self.soil,radius,width,entry,slip)
        with self.assertRaisesRegex(WheelDomainError, "Earth"):
            rover_on_slope(self.soil,.33,.125,63,0.,gravity=1.62)
        with self.assertRaisesRegex(WheelDomainError, "No driving solution"):
            rover_on_slope(self.soil,.33,.125,63,math.radians(50))

    def test_solver_recovery_and_smooth_sensitivity(self):
        prescribed = wheel_at_slip(self.soil, .165, .125, 150., .6)
        recovered = solve_operating_point(self.soil, .165, .125, 150., prescribed["drawbar_pull_n"])
        self.assertAlmostEqual(recovered["slip"], .6, places=8)
        def derivative(step):
            return (rover_on_slope(self.soil,.33+step,.125,63,math.radians(5))["slip"]
                    -rover_on_slope(self.soil,.33-step,.125,63,math.radians(5))["slip"])/(2*step)
        self.assertAlmostEqual(derivative(1e-5), derivative(2e-5), places=5)

    def test_transfer_flags_report_extrapolation_instead_of_hiding_it(self):
        point = rover_on_slope(self.soil,.384,.064,63,math.radians(10))
        flags = transfer_flags(point)
        self.assertTrue(any("50 kPa" in flag for flag in flags))
        self.assertTrue(any("10 mm" in flag for flag in flags))

    def test_default_modelset_contract_energy_and_point_turn_invariance(self):
        from rover.models import ModelSet
        from rover.mobility import Mobility
        from rover.alternatives.calculated_traversal import CalculatedTraversalMobility
        from rover.problem import build_problem
        self.assertIs(ModelSet().mobility, CalculatedTraversalMobility)
        values = {"wheel_diameter": .33, "wheel_width": .125}
        # Compare soil laws at the same prescribed timing, independently of the
        # calculated timing selected by default in production.
        reference = build_problem(values, models=replace(ModelSet(), mobility=Mobility), traversal_mode="fixed")
        experiment = build_problem(values, traversal_mode="fixed")
        try:
            reference.run_model()
            experiment.run_model()
            for name in ("turn_time", "route_turn_time", "turn_peak_power", "turn_motor_margin", "turn_traction_margin", "turn_power_margin"):
                self.assertEqual(reference.get_val(name).item(), experiment.get_val(name).item())
            self.assertGreater(experiment.get_val("mobility_energy").item(), 0)
            self.assertTrue(experiment.model_metadata["evidence"]["mobility"]["exploratory"])
            self.assertFalse(any(n.startswith("_auto_ivc") for n in experiment.model._conn_global_abs_in2out.values()))
            v = lambda n: float(experiment.get_val(n).item())
            out = experiment.model.mobility.straight_response(v,mass=63.,drag=9999.,accel=.2,peak=.5,distance=3.)
            same = experiment.model.mobility.straight_response(v,mass=63.,drag=0.,accel=.2,peak=.5,distance=3.)
            self.assertEqual(out, same)  # no legacy compaction double-counting
            self.assertGreaterEqual(out["work"], .5*63.*.5**2)
            self.assertAlmostEqual(out["power"], out["force"]*out["peak_rim_speed"], places=8)
        finally:
            reference.cleanup()
            experiment.cleanup()


if __name__ == "__main__":
    unittest.main()
