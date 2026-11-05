"""Independent compression limits, static moments and integration contracts."""
from dataclasses import replace
import math
import unittest

import numpy as np
from openmdao.utils.assert_utils import assert_check_totals
from openmdao.visualization.n2_viewer.n2_viewer import _get_viewer_data

from rover.problem import build_problem
from rover.models import ModelSet, plate_frame_models, compliant_plate_frame_models
from rover.excavation import Excavation
from rover.alternatives.compliant_tipping import (
    COMPLIANCE_OUTPUTS, wheel_support_geometry, loaded_wheel_compliance,
    CompliantTraversalStability, CompliantPlateTraversalStability, CompliantObstacleStability,
)
from rover.alternatives.traversal_stability import TraversalStability, PlateTraversalStability, slope_support
from reporting.report import snapshot


def wheel_inputs(**changes):
    return {**dict(wheel_diameter=.359, grouser_height=.018,
                   tipping_radius_deflection_fraction=.2, tipping_reference_fill_fraction=.9,
                   dry_mass=40., payload_mass=40., fill_fraction=.4), **changes}


class CompressionPhysicsTests(unittest.TestCase):
    def test_zero_half_and_rated_total_load_and_grouser_accounting(self):
        # A 359 mm OVERALL wheel has a 323 mm tread diameter. At the rated
        # load its axle is 179.5 - 35.9 = 143.6 mm above the grouser-tip plane.
        for mass, compression in ((0., 0.), (75., .01795), (150., .0359)):
            result = wheel_support_geometry(.359, .018, .2, mass, 150.)
            self.assertAlmostEqual(result["tipping_wheel_tread_radius"], .1615)
            self.assertAlmostEqual(result["tipping_wheel_deflection"], compression)
            self.assertAlmostEqual(result["tipping_wheel_support_height"], .1795-compression)
        # Changing the rigid grouser length at FIXED overall diameter cannot
        # raise the axle again; it changes the body/grouser split only.
        for grouser in (0., .018, .025):
            result = wheel_support_geometry(.359, grouser, .2, 150., 150.)
            self.assertAlmostEqual(result["tipping_wheel_support_height"], .1436)
        rigid = wheel_support_geometry(.359, .018, 0., 150., 150.)
        self.assertAlmostEqual(rigid["tipping_wheel_support_height"], .1795)

    def test_fixed_reference_fill_and_rover_weight_are_included(self):
        # This design stores 100 kg per unit fill. Rated mass = 40+90=130 kg.
        low = loaded_wheel_compliance(wheel_inputs())
        full = loaded_wheel_compliance(wheel_inputs(fill_fraction=.9, payload_mass=90.))
        self.assertAlmostEqual(low["tipping_reference_mass"], 130.)
        self.assertEqual(low["tipping_reference_mass"], full["tipping_reference_mass"])
        self.assertAlmostEqual(low["tipping_wheel_deflection_ratio"], .2*80/130)
        self.assertAlmostEqual(full["tipping_wheel_deflection_ratio"], .2)
        empty = wheel_support_geometry(.359, .018, .2, 40., 130.)
        self.assertGreater(empty["tipping_wheel_deflection"], 0.)
        # A homogeneous scaling of both masses leaves the ratio unchanged.
        scaled = loaded_wheel_compliance(wheel_inputs(dry_mass=80., payload_mass=80.))
        self.assertAlmostEqual(low["tipping_wheel_deflection"], scaled["tipping_wheel_deflection"])

    def test_independent_gravity_moment_limit_after_compression(self):
        # At incipient longitudinal tip, mg*a*cos(theta) = mg*h*sin(theta).
        half_span, nominal_cg, delta, mass, gravity = .25, .45, .03, 120., 9.80665
        h = nominal_cg-delta
        angle = math.atan(half_span/h)
        restoring = mass*gravity*half_span*math.cos(angle)
        overturning = mass*gravity*h*math.sin(angle)
        self.assertAlmostEqual(restoring, overturning, places=10)
        result = slope_support(2*half_span, .7, 0., h, angle)
        self.assertAlmostEqual(result["slope_pitch_tip_margin"], 0., places=12)
        for theta in (.2, .4):
            rigid = slope_support(.5, .7, 0., nominal_cg, theta)
            compressed = slope_support(.5, .7, 0., h, theta)
            self.assertAlmostEqual(compressed["slope_tip_margin"]-rigid["slope_tip_margin"],
                                   delta*math.tan(theta))
        self.assertLess(slope_support(.5, .7, 0., h, angle+.01)["slope_tip_margin"], 0.)

    def test_failure_domains_reject_collapse_and_extrapolation(self):
        for args in ((0., .018, .2, 100., 150.), (.359, -.01, .2, 100., 150.),
                     (.359, .18, .2, 100., 150.), (.359, .15, .2, 100., 150.),
                     (.359, .018, -1., 100., 150.), (.359, .018, 1., 100., 150.),
                     (.359, .018, .2, -1., 150.), (.359, .018, .2, 151., 150.),
                     (.359, .018, .2, 100., 0.), (.359, .018, .2, math.nan, 150.)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                wheel_support_geometry(*args)
        for changes in ({"fill_fraction": 0.}, {"fill_fraction": .91}, {"dry_mass": 0.},
                        {"payload_mass": -1.}, {"tipping_reference_fill_fraction": 1.1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                loaded_wheel_compliance(wheel_inputs(**changes))


class CompressionIntegrationTests(unittest.TestCase):
    def problem(self, overrides=None, **options):
        p = build_problem(overrides, **options)
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def test_plate_factory_selection_and_original_reference_preserved(self):
        self.assertIs(ModelSet().traversal_stability, TraversalStability)
        self.assertIs(plate_frame_models().traversal_stability, PlateTraversalStability)
        self.assertIs(compliant_plate_frame_models().traversal_stability, CompliantPlateTraversalStability)
        p = self.problem({"fill_fraction": .4}, structure="plate-frame")
        rigid = self.problem({"fill_fraction": .4}, structure="plate-frame", models=plate_frame_models())
        delta = p.get_val("tipping_wheel_deflection").item()
        self.assertGreater(delta, 0.)
        for name in ("dry_mass", "payload_mass", "wheel_mass", "drum_mass", "loaded_cg_z",
                     "loaded_sprung_cg_z", "tip_stability_margin", "obstacle_tip_margin",
                     "obstacle_pose_margin", "rock_sweep_tip_margin", "rock_sweep_wheel_load_margin",
                     "mobility_stability_margin", "ground_clearance_margin", "cycle_energy", "cycle_time"):
            self.assertAlmostEqual(p.get_val(name).item(), rigid.get_val(name).item(), places=11, msg=name)
        for name in ("rock_sweep_cg_z", "obstacle_loaded_cg_z"):
            self.assertAlmostEqual(p.get_val(name).item(), rigid.get_val(name).item()-delta, places=12)
        gain = delta*math.tan(p.get_val("slope_angle").item())
        for name in ("slope_pitch_tip_margin", "slope_roll_tip_margin"):
            self.assertAlmostEqual(p.get_val(name).item()-rigid.get_val(name).item(), gain)

    def test_sources_recorded_reports_and_dependency_audit(self):
        # A function factory must retain the new outputs in recorded snapshots.
        models = replace(ModelSet(), traversal_stability=lambda: CompliantTraversalStability())
        p = self.problem({"fill_fraction": .4}, models=models)
        config = p.model_metadata["configuration"]
        self.assertTrue(config["static_wheel_compliance"])
        report = snapshot(p)
        recorded = snapshot(p, configuration=config)
        for name, units in COMPLIANCE_OUTPUTS.items():
            self.assertEqual(report["outputs"][name]["units"], units)
            self.assertEqual(recorded["outputs"][name], report["outputs"][name])
        for port, source in (("dry_mass", "geometry.dry_mass"), ("payload_mass", "excavation.payload_mass"),
                             ("loaded_cg_z", "mass_properties.loaded_cg_z"), ("fill_fraction", "inputs.fill_fraction")):
            self.assertEqual(p.model.get_source("traversal_stability."+port), source)
        declared = set(_get_viewer_data(p)["declare_partials_list"])
        self.assertIn("traversal_stability.slope_tip_margin > traversal_stability.dry_mass", declared)
        self.assertNotIn("traversal_stability.rock_sweep_tip_margin > traversal_stability.dry_mass", declared)
        comp = p.model.traversal_stability
        inputs = {n: comp._inputs[n].copy() for n in comp._var_rel_names["input"]}
        base = {n: np.zeros(1) for n in comp.OUTPUT_UNITS}
        comp.compute(inputs, base)
        self.assertEqual(set(comp.DEPENDENCIES), set(comp.OUTPUT_UNITS))
        for name in inputs:
            changed_inputs = {n: v.copy() for n, v in inputs.items()}
            changed_inputs[name] += 1e-4*max(abs(inputs[name].item()), .01)
            changed = {n: np.zeros(1) for n in comp.OUTPUT_UNITS}
            comp.compute(changed_inputs, changed)
            for out, dependencies in comp.DEPENDENCIES.items():
                if name not in dependencies:
                    np.testing.assert_allclose(changed[out], base[out], atol=1e-10, rtol=1e-10,
                                               err_msg=f"Missing {out} <- {name}")

    def test_smooth_derivatives_and_actual_fill_sensitivity(self):
        p = self.problem({"fill_fraction": .4, "wheelbase": .46}, structure="plate-frame")
        reference = p.get_val("tipping_reference_mass").item()
        partial_fill_ratio = p.get_val("tipping_wheel_deflection_ratio").item()
        checks = p.check_totals(of=["slope_tip_margin", "tipping_wheel_deflection", "tipping_reference_mass"],
                               wrt=["fill_fraction", "wheel_diameter", "drum_diameter", "battery_capacity"],
                               method="fd", form="central", step=3e-5, step_calc="rel_avg", out_stream=None)
        assert_check_totals(checks, atol=1e-3, rtol=2e-3)
        p.set_val("fill_fraction", .9)
        p.run_model()
        self.assertAlmostEqual(p.get_val("tipping_reference_mass").item(), reference)
        self.assertLess(partial_fill_ratio, .2)
        self.assertAlmostEqual(p.get_val("tipping_wheel_deflection_ratio").item(), .2)
        invalid = build_problem({"fill_fraction": .95}, structure="plate-frame")
        self.addCleanup(invalid.cleanup)
        with self.assertRaisesRegex(ValueError, "reference_fill_fraction"):
            invalid.run_model()

    def test_legacy_payload_contract_requires_explicit_rigid_models(self):
        with self.assertRaisesRegex(ValueError, "proportional"):
            build_problem(models=replace(compliant_plate_frame_models(), excavation=Excavation), structure="plate-frame")
        with self.assertRaisesRegex(ValueError, "proportional"):
            build_problem(models=ModelSet(excavation=Excavation, obstacle_stability=CompliantObstacleStability))
        # Explicit stationary/reference factories keep their original physics.
        p = self.problem(models=ModelSet(excavation=Excavation))
        self.assertFalse(p.model_metadata["configuration"]["static_wheel_compliance"])


if __name__ == "__main__":
    unittest.main()
