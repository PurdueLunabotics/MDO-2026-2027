"""Independent incline limits, reaction conservation and failure-domain checks."""
import math
import unittest
import numpy as np
from openmdao.utils.assert_utils import assert_check_totals

from rover.problem import build_problem
from reporting.report import snapshot
from rover.alternatives.obstacle_stability import raised_wheel_pose, projected_support_margin
from rover.alternatives.plate_frame import plate_frame_posed_cg
from rover.alternatives.traversal_stability import (
    slope_support, articulated_cg, wheel_reactions, rock_path_screen,
)


def rock_inputs(**changes):
    values = dict(wheelbase=.8, track_width=.7, wheel_diameter=.3, rocker_pivot_offset=.1,
                  obstacle_height=.4, loaded_sprung_mass=60., loaded_sprung_cg_x=0.,
                  loaded_sprung_cg_z=.3, wheel_mass=8., drive_mass=8., rocker_mass=4., gravity=9.80665)
    return {**values, **changes}


class TraversalStabilityTests(unittest.TestCase):
    def test_slope_exact_moment_limit_and_both_axes(self):
        # h=0.6, longitudinal half-span 0.4: mg moments balance at tan(a)=2/3.
        limit = math.atan(2/3)
        at = slope_support(.8, 1., 0., .6, limit)
        self.assertAlmostEqual(at["slope_pitch_tip_margin"], 0., places=12)
        self.assertAlmostEqual(at["slope_roll_tip_margin"], .1)
        self.assertAlmostEqual(at["slope_tip_angle"], limit)
        self.assertGreater(slope_support(.8, 1., 0., .6, limit-.01)["slope_tip_margin"], 0.)
        self.assertLess(slope_support(.8, 1., 0., .6, limit+.01)["slope_tip_margin"], 0.)
        self.assertEqual(slope_support(.8, 1., .05, .6, .3), slope_support(.8, 1., -.05, .6, .3))

    def test_cardinal_screens_bound_every_slope_heading(self):
        wb, track, x, z, angle = .9, .7, .03, .4, math.pi/6
        result = slope_support(wb, track, x, z, angle)
        for heading in np.linspace(0., 2*math.pi, 361):
            intercept = (x-z*math.tan(angle)*math.cos(heading), -z*math.tan(angle)*math.sin(heading))
            reserve = min(wb/2-abs(intercept[0]), track/2-abs(intercept[1]))
            self.assertGreaterEqual(reserve+1e-12, result["slope_tip_margin"])

    def test_zero_slope_is_level_support_and_invalid_domains(self):
        result = slope_support(.8, .7, .1, .4, 0.)
        self.assertAlmostEqual(result["slope_tip_margin"], .3)
        for args in ((0., .7, 0., .4, .3), (.8, .7, 0., 0., .3),
                     (.8, .7, 0., .4, math.pi/2), (.8, .7, 0., .4, -.1),
                     (.8, .7, math.nan, .4, .3)):
            with self.assertRaises(ValueError):
                slope_support(*args)
        for angle in (-.1, math.pi/2):
            with self.assertRaises(ValueError):
                build_problem({"slope_angle": angle})

    def test_flat_reactions_hand_solved_axle_loads(self):
        pose = raised_wheel_pose(.8, .7, .15, .1, 0.)
        mass, x, gravity = 80., .05, 9.80665
        forces, error = wheel_reactions(pose, (x, 0., .4), 60., 16., 4., gravity)
        expected = mass*gravity*np.array((.25+x/(2*.8), .25+x/(2*.8), .25-x/(2*.8), .25-x/(2*.8)))
        np.testing.assert_allclose(forces, expected, atol=1e-10)
        self.assertLess(error, 1e-10)
        self.assertAlmostEqual(sum(forces), mass*gravity)

    def test_articulated_force_moments_and_differential_balance(self):
        for fraction in (.5, 1.):
            pose = raised_wheel_pose(.8, .7, .15, .1, .4)
            cg = articulated_cg(pose, .15, .1, 60., .02, .3, 16., 4., fraction)
            forces, error = wheel_reactions(pose, cg, 60., 16., 4., 9.80665, fraction)
            contacts = np.array(pose["contacts"])
            np.testing.assert_allclose(contacts[:, :2].T @ forces, 80*9.80665*np.array(cg[:2]), atol=1e-10)
            side = np.array((1., -1., 1., -1.))
            expected = (16+fraction*4)/4*float(side @ contacts[:, 0])*9.80665
            self.assertAlmostEqual(float((side*contacts[:, 0]) @ forces), expected, places=10)
            self.assertLess(error, 1e-9)
            self.assertGreater(min(forces), 0.)

    def test_positive_global_hull_can_still_require_negative_wheel_load(self):
        pose = raised_wheel_pose(.46, .5, .09, .04, .4)
        cg = articulated_cg(pose, .09, .04, 80., 0., .5, 12., 2., .5)
        self.assertGreater(projected_support_margin(pose["contacts"], cg), 0.)
        forces, _ = wheel_reactions(pose, cg, 80., 12., 2., 9.80665, .5)
        self.assertLess(min(forces), 0.)

    def test_mass_placement_matches_plate_and_reference_zero_pose(self):
        pose = raised_wheel_pose(.8, .7, .15, .1, .2)
        cg = articulated_cg(pose, .15, .1, 60., 0., .3, 16., 4., .5)
        np.testing.assert_allclose(cg, plate_frame_posed_cg(pose, .15, .1, 60., 0., .3, 16., 4.))
        flat = raised_wheel_pose(.8, .7, .15, .1, 0.)
        for fraction in (.5, 1.):
            cg = articulated_cg(flat, .15, .1, 60., 0., .3, 16., 4., fraction)
            expected_z = (60*.3+16*.15+4*(.15+(1-fraction)*.1))/80
            self.assertAlmostEqual(cg[2], expected_z)

    def test_mirrored_raises_and_sweep_contains_endpoints(self):
        values = rock_inputs()
        sweep = rock_path_screen(values, rocker_centroid_fraction=.5)
        tips, loads = [], []
        for raised in range(4):
            pose = raised_wheel_pose(.8, .7, .15, .1, .4, raised)
            cg = articulated_cg(pose, .15, .1, 60., 0., .3, 16., 4., .5)
            tips.append(projected_support_margin(pose["contacts"], cg))
            loads.append(min(wheel_reactions(pose, cg, 60., 16., 4., 9.80665, .5)[0]))
        self.assertLess(max(tips)-min(tips), 1e-12)
        self.assertLess(max(loads)-min(loads), 1e-10)
        self.assertLessEqual(sweep["rock_sweep_tip_margin"], min(tips)+1e-12)
        self.assertLessEqual(sweep["rock_sweep_wheel_load_margin"], min(loads)+1e-10)
        self.assertEqual(sweep["rock_sweep_evaluated_height"], .4)
        self.assertEqual(sweep["rock_sweep_samples"], 41)
        dense = rock_path_screen(values, samples=401, rocker_centroid_fraction=.5)
        self.assertAlmostEqual(sweep["rock_sweep_tip_margin"], dense["rock_sweep_tip_margin"], places=5)
        self.assertAlmostEqual(sweep["rock_sweep_wheel_load_margin"], dense["rock_sweep_wheel_load_margin"], places=4)

    def test_unreachable_path_and_invalid_reaction_inputs_are_rejected(self):
        result = rock_path_screen(rock_inputs(wheelbase=.4))
        self.assertEqual(result["rock_sweep_pose_valid"], 0.)
        self.assertLess(result["rock_sweep_tip_margin"], 0.)
        self.assertLess(result["rock_sweep_wheel_load_margin"], 0.)
        for samples in (1, 2.5):
            with self.assertRaises(ValueError):
                rock_path_screen(rock_inputs(), samples=samples)
        pose = raised_wheel_pose(.8, .7, .15, .1, 0.)
        for mass, gravity, fraction in ((-1., 9.8, 1.), (16., 0., 1.), (16., 9.8, 2.)):
            with self.assertRaises(ValueError):
                wheel_reactions(pose, (0., 0., .3), 60., mass, 4., gravity, fraction)

    def test_active_sources_report_and_smooth_derivatives(self):
        p = build_problem({"wheelbase": .46})
        self.addCleanup(p.cleanup)
        p.run_model()
        report = snapshot(p)
        for name in ("slope_tip_margin", "rock_sweep_tip_margin", "rock_sweep_wheel_load_margin"):
            self.assertIn(name, report["constraints"])
            self.assertIn(name, p.model.get_constraints())
        self.assertEqual(p.model.get_source("traversal_stability.loaded_cg_z"), "mass_properties.loaded_cg_z")
        self.assertEqual(p.model.get_source("traversal_stability.rocker_mass"), "structure_geometry.rocker_mass")
        checks = p.check_totals(of=["slope_tip_margin", "rock_sweep_tip_margin", "rock_sweep_wheel_load_margin"],
                               wrt=["wheelbase", "track_width", "drum_diameter", "travel_actuator_length"],
                               method="fd", form="central", step=3e-5, step_calc="rel_avg", out_stream=None)
        assert_check_totals(checks, atol=1e-3, rtol=2e-3)

    def test_singular_reaction_geometry_does_not_create_feasible_loads(self):
        pose = raised_wheel_pose(.8, .7, .15, .1, 0.)
        pose["contacts"] = [(0., .35, 0.), (.4, -.35, 0.),
                            (0., .35, .4), (-.4, -.35, 0.)]
        with self.assertRaisesRegex(ValueError, "singular"):
            wheel_reactions(pose, (0., 0., .3), 60., 16., 4., 9.80665)


if __name__ == "__main__":
    unittest.main()
