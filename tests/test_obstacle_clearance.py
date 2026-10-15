import unittest
import math
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_totals

from rover.alternatives.obstacle_clearance import (
    chassis_point_in_world,
    obstacle_drum_wheel_gaps,
    minimum_sphere_gap,
    evaluate_obstacle_clearance,
    ObstacleClearance,
)
from rover.alternatives.obstacle_stability import raised_wheel_pose
from rover.problem import build_problem
from reporting.report import snapshot


def example_values():
    """Synthetic geometry from the student's worked example; not hardware data."""
    return {
        "wheelbase": .46, "track_width": .54, "wheel_diameter": .30,
        "wheel_width": .10, "rocker_pivot_offset": .10, "obstacle_height": .40,
        "drum_diameter": .28, "drum_length": .25, "drum_pair_gap": .06,
        "mechanism_clearance": .015, "travel_x": .4810305108377564,
        "travel_y": .6365979659441497,
    }

class ObstacleClearanceTests(unittest.TestCase):
    def test_level_pose_preserves_point(self):
        point = (0.48, 0.155, 0.637)

        pose = {
            "pitch": 0.0,
            "roll": 0.0,
            "shaft_origin": (0.0, 0.0, 0.25),
        }

        result = chassis_point_in_world(
            point, pose, level_shaft_height=0.25
        )

        for actual, expected in zip(result, point):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_unreachable_obstacle_is_rejected(self):
        level_drums = [
            (0.48, 0.155, 0.637),
            (0.48, -0.155, 0.637),
            (-0.48, 0.155, 0.637),
            (-0.48, -0.155, 0.637),
        ]

        with self.assertRaises(ValueError):
            obstacle_drum_wheel_gaps(
                level_drums=level_drums,
                wheelbase=0.40,
                track=0.54,
                wheel_radius=0.15,
                pivot_offset=0.10,
                height=0.40,
                drum_bound_radius=0.19,
                wheel_bound_radius=0.16,
            )

    def test_pitch_then_roll_and_translation(self):
        point = (1.0, 2.0, 3.25)
        pose = {
            "pitch": math.pi / 2,
            "roll": math.pi / 2,
            "shaft_origin": (4.0, 5.0, 6.0),
        }

        result = chassis_point_in_world(
            point, pose, level_shaft_height=0.25
        )

        expected = (1.0, 4.0, 8.0)
        for actual, target in zip(result, expected):
            self.assertAlmostEqual(actual, target, places=12)

    def test_minimum_gap_checks_cross_pairs(self):
        drum_centers = [
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
        ]
        wheel_centers = [
            (10.0, 0.0, 4.0),
            (3.0, 4.0, 0.0),
        ]

        gap, drum_index, wheel_index = minimum_sphere_gap(
            drum_centers,
            wheel_centers,
            drum_bound_radius=1.0,
            wheel_bound_radius=2.0,
        )

        self.assertAlmostEqual(gap, 1.0)
        self.assertEqual(drum_index, 1)
        self.assertEqual(wheel_index, 0)

    def test_rigid_transform_preserves_distances(self):
        points = [(1.2, -.3, .7), (-.4, .6, 1.1), (.1, .2, .3)]
        pose = {"pitch": .27, "roll": -.36, "shaft_origin": (3, -2, .8)}
        transformed = [chassis_point_in_world(point, pose, .25) for point in points]
        for i, j in ((0, 1), (0, 2), (1, 2)):
            self.assertAlmostEqual(math.dist(points[i], points[j]),
                                   math.dist(transformed[i], transformed[j]), places=12)

    def test_sphere_gap_touching_and_overlap(self):
        for distance, expected in ((4, 1), (3, 0), (2, -1)):
            with self.subTest(distance=distance):
                gap, _, _ = minimum_sphere_gap([(0, 0, 0)], [(distance, 0, 0)], 1, 2)
                self.assertAlmostEqual(gap, expected)

    def test_enclosing_sphere_overlap_is_not_collision_proof(self):
        # Parallel cylinders along y: radius .1, length 2, x separation 1.2.
        # Their radial envelopes have a known 1.0 m gap despite overlapping spheres.
        bound = math.hypot(.1, 1)
        sphere_gap, _, _ = minimum_sphere_gap([(0, 0, 0)], [(1.2, 0, 0)], bound, bound)
        self.assertLess(sphere_gap, 0)
        self.assertGreater(1.2 - 2*.1, 0)

    def test_invalid_points_radii_and_angles_are_rejected(self):
        for points in ([], [(0, 0)], [(0, 0, math.nan)], [(0, 0, math.inf)]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                minimum_sphere_gap(points, [(0, 0, 0)], 1, 1)
        for radius in (-1, math.nan, math.inf):
            with self.subTest(radius=radius), self.assertRaises(ValueError):
                minimum_sphere_gap([(0, 0, 0)], [(1, 0, 0)], radius, 1)
        pose = {"pitch": math.nan, "roll": 0, "shaft_origin": (0, 0, .25)}
        with self.assertRaises(ValueError):
            chassis_point_in_world((0, 0, 1), pose, .25)

    def test_mirrored_cases_and_required_clearance(self):
        values = example_values()
        result = evaluate_obstacle_clearance(values)
        self.assertAlmostEqual(result["obstacle_drum_wheel_gap_bound"], .08889977266591964)
        self.assertAlmostEqual(result["obstacle_drum_wheel_margin"], .07389977266591964)
        self.assertEqual(result["obstacle_clearance_pose_valid"], 1)
        self.assertEqual(result["obstacle_clearance_evaluated_height"], .4)
        changed = evaluate_obstacle_clearance({**values, "mechanism_clearance": .025})
        self.assertEqual(changed["obstacle_drum_wheel_gap_bound"], result["obstacle_drum_wheel_gap_bound"])
        self.assertAlmostEqual(changed["obstacle_drum_wheel_margin"], result["obstacle_drum_wheel_margin"]-.01)
        x, z = values["travel_x"], values["travel_y"]
        y = (values["drum_length"]+values["drum_pair_gap"])/2
        cases = obstacle_drum_wheel_gaps(
            [(sx*x, sy*y, z) for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))],
            .46, .54, .15, .10, .40, math.hypot(.14, .125), math.hypot(.15, .05),
        )
        self.assertEqual(len(cases), 4)
        for gap, _, _, _ in cases:
            self.assertAlmostEqual(gap, result["obstacle_drum_wheel_gap_bound"])

    def test_guarded_diagnostics_never_accept_unreachable_or_boundary_pose(self):
        values = example_values()
        boundary = raised_wheel_pose(.46, .54, .15, .10, 0)["reach_margin"]
        for changes in ({"wheelbase": .40}, {"obstacle_height": boundary}):
            with self.subTest(changes=changes):
                result = evaluate_obstacle_clearance({**values, **changes})
                self.assertEqual(result["obstacle_clearance_pose_valid"], 0)
                self.assertLess(result["obstacle_drum_wheel_margin"], -1e-6)
                self.assertLess(result["obstacle_clearance_evaluated_height"],
                                changes.get("obstacle_height", .4))
                self.assertTrue(all(math.isfinite(value) for value in result.values()))

    def test_invalid_component_geometry_is_rejected(self):
        for changes in ({"wheel_width": -1}, {"drum_length": 0},
                        {"drum_pair_gap": -1}, {"mechanism_clearance": -1},
                        {"travel_y": math.nan}, {"rocker_pivot_offset": .3},
                        {"obstacle_height": -1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                evaluate_obstacle_clearance({**example_values(), **changes})

    def test_openmdao_adapter_and_smooth_derivatives(self):
        p = om.Problem(reports=False)
        p.model.add_subsystem("clearance", ObstacleClearance(), promotes=["*"])
        p.setup()
        self.addCleanup(p.cleanup)
        values = {**example_values(), "obstacle_height": .2}
        for name, value in values.items():
            p.set_val(name, value)
        p.run_model()
        expected = evaluate_obstacle_clearance(values)
        for name, value in expected.items():
            self.assertAlmostEqual(p.get_val(name).item(), value)
        checks = p.check_totals(
            of=["obstacle_drum_wheel_gap_bound", "obstacle_drum_wheel_margin"],
            wrt=list(values), method="fd", form="central", step=3e-5, out_stream=None,
        )
        assert_check_totals(checks, atol=1e-5, rtol=2e-3)

    def test_active_constraint_sources_and_report(self):
        p = build_problem({"wheelbase": .46})
        self.addCleanup(p.cleanup)
        p.run_model()
        self.assertEqual(p.model.get_source("obstacle_clearance.travel_x"), "linkage.travel_x")
        self.assertEqual(p.model.get_source("obstacle_clearance.travel_y"), "linkage.travel_y")
        self.assertIn("obstacle_drum_wheel_margin", p.model.get_constraints())
        report = snapshot(p)
        self.assertTrue(report["constraints"]["obstacle_drum_wheel_margin"]["passed"])
        self.assertIn("obstacle_clearance", report["model_metadata"]["models"])
        self.assertEqual(report["outputs"]["obstacle_clearance_pose_valid"]["value"], 1)
        p.set_val("wheelbase", .40)
        p.run_model()
        report = snapshot(p)
        self.assertFalse(report["feasible"])
        self.assertFalse(report["constraints"]["obstacle_drum_wheel_margin"]["passed"])
        self.assertEqual(report["outputs"]["obstacle_clearance_pose_valid"]["value"], 0)
