"""Independent closure, virtual work, invalid-domain and report checks."""
import math
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

import numpy as np
from openmdao.utils.assert_utils import assert_check_totals
from openmdao.visualization.n2_viewer.n2_viewer import _get_viewer_data

from rover.linkage import attachment_point, drum_pose, sweep_poses
from rover.config import DESIGN, ASSUMPTIONS
from rover.problem import build_problem
from rover.report import snapshot
from rover.html_report import geometry_svg, values_from_snapshot


class ArmAttachmentTests(unittest.TestCase):
    def problem(self, **overrides):
        p = build_problem(overrides)
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def matched_poses(self, p, fraction):
        """Move the pin inward, retaining the same three drum positions."""
        v = lambda n: p.get_val(n).item()
        changes = {"actuator_attachment_fraction": fraction}
        for state, length in (("stowed", "actuator_retracted_length"),
                              ("travel", "travel_actuator_length"),
                              ("extended", "actuator_extended_length")):
            x, z = attachment_point(v("pivot_x"), v("bottom_pivot_height"),
                                    v(state+"_x"), v(state+"_y"), fraction)
            changes[length] = math.hypot(x-v("actuator_pivot_x"), z-v("top_pivot_height"))
        return changes

    def test_attachment_is_fixed_by_default_and_can_be_optimized(self):
        p = self.problem()
        self.assertNotIn("actuator_attachment_fraction", p.model.get_design_vars())
        self.assertEqual(p.get_val("actuator_attachment_fraction").item(), 1)
        assumptions = {n: s for n, s in ASSUMPTIONS.items() if n != "actuator_attachment_fraction"}
        with patch.dict(ASSUMPTIONS, assumptions, clear=True), patch.dict(
                DESIGN, {"actuator_attachment_fraction": (1.0, .001, 1.0, None)}):
            adjustable = self.problem()
            self.assertIn("actuator_attachment_fraction", adjustable.model.get_design_vars())

    def test_pin_and_drum_closure_at_all_states_and_sweep(self):
        p = self.problem()
        changes = self.matched_poses(p, .8)
        for n, value in changes.items():
            p.set_val(n, value)
        p.run_model()
        values = values_from_snapshot(snapshot(p))
        b = np.array([values["pivot_x"], values["bottom_pivot_height"]])
        a = np.array([values["actuator_pivot_x"], values["top_pivot_height"]])
        for state, length in (("stowed", "actuator_retracted_length"),
                              ("travel", "travel_actuator_length"),
                              ("extended", "actuator_extended_length")):
            c = np.array([values[state+"_x"], values[state+"_y"]])
            d = b+.8*(c-b)
            self.assertAlmostEqual(np.linalg.norm(c-b), values["arm_length"])
            self.assertAlmostEqual(np.linalg.norm(d-a), values[length])
            self.assertAlmostEqual(np.linalg.norm(d-b), values["actuator_attachment_distance"])
        for c in sweep_poses(values)[0]:
            d = b+.8*(np.array(c)-b)
            length = np.linalg.norm(d-a)
            reconstructed = drum_pose(values["arm_length"], *b, *a, length, .8)
            np.testing.assert_allclose(reconstructed, c, atol=1e-10)

    def test_virtual_work_and_gravity_energy_at_same_drum_poses(self):
        p = self.problem(actuator_idle_power=0)
        v = lambda n: p.get_val(n).item()
        reference_force, reference_energy = v("required_actuator_force"), v("arm_energy")
        changes = self.matched_poses(p, .8)
        for n, value in changes.items():
            p.set_val(n, value)
        p.run_model()
        # Same lifted mass and height imply the same gravity work, regardless of
        # pin placement; shorter pin radius changes stroke and force, not energy.
        self.assertAlmostEqual(v("arm_energy"), reference_energy)
        self.assertGreater(v("required_actuator_force"), reference_force)
        values = values_from_snapshot(snapshot(p))
        numeric = []
        for c in sweep_poses(values)[0]:
            d = attachment_point(v("pivot_x"), v("bottom_pivot_height"), *c, .8)
            length = math.hypot(d[0]-v("actuator_pivot_x"), d[1]-v("top_pivot_height"))
            # Independent work ratio: F/W = |dz_C/dl|.
            step = 1e-6
            args = (v("arm_length"), v("pivot_x"), v("bottom_pivot_height"),
                    v("actuator_pivot_x"), v("top_pivot_height"))
            z_plus = drum_pose(*args, length+step, .8)[1]
            z_minus = drum_pose(*args, length-step, .8)[1]
            numeric.append(abs((z_plus-z_minus)/(2*step)))
        self.assertAlmostEqual(v("actuator_leverage_max"), max(numeric), places=7)

    def test_invalid_attachment_and_unreachable_triangle_remain_failed(self):
        for fraction in (0, -.1, 1.01, float("nan")):
            with self.subTest(fraction=fraction), self.assertRaises(ValueError):
                build_problem({"actuator_attachment_fraction": fraction})
        p = self.problem(actuator_attachment_fraction=.25)
        report = snapshot(p)
        self.assertFalse(report["feasible"])
        self.assertFalse(report["constraints"]["triangle_extended_margin"]["passed"])
        self.assertTrue(all(math.isfinite(i["value"]) for i in report["outputs"].values()))
        # Guarded invalid triangles must not invent a longer arm, even near B.
        for fraction in (.001, .25):
            c = drum_pose(.6, .13, .15, .15, .45, .68, fraction)
            self.assertAlmostEqual(math.hypot(c[0]-.13, c[1]-.15), .6)
        with self.assertRaises(ValueError):
            drum_pose(.6, .13, .15, .15, .45, .38, 0)

    def test_attachment_derivatives_and_n2_connections(self):
        p = self.problem()
        for n, value in self.matched_poses(p, .8).items():
            p.set_val(n, value)
        p.run_model()
        checks = p.check_totals(
            of=["travel_x", "extended_y", "actuator_leverage_max", "actuator_attachment_distance",
                "required_actuator_force", "arm_energy", "loaded_cg_z", "loaded_yaw_inertia"],
            wrt=["actuator_attachment_fraction", "arm_length", "travel_actuator_length"],
            method="fd", form="central", step=3e-5, step_calc="rel_avg", out_stream=None)
        assert_check_totals(checks, atol=1e-3, rtol=2e-3)
        declared = set(_get_viewer_data(p)["declare_partials_list"])
        self.assertIn("linkage.actuator_leverage_max > linkage.actuator_attachment_fraction", declared)
        self.assertIn("mass_properties.loaded_cg_z > mass_properties.actuator_attachment_fraction", declared)

    def test_drawings_end_actuator_at_pin_not_drum(self):
        p = self.problem()
        for n, value in self.matched_poses(p, .8).items():
            p.set_val(n, value)
        p.run_model()
        values = values_from_snapshot(snapshot(p))
        for options in ({}, {"travel": True}, {"extended": True}):
            svg = geometry_svg(values, **options)
            root = ET.fromstring(svg)
            lines = root.findall(".//{http://www.w3.org/2000/svg}line")
            arm = next(x for x in lines if x.get("stroke") == "#246f99")
            actuator = next(x for x in lines if x.get("stroke") == "#d77422")
            for coord in ("x", "y"):
                expected = float(arm.get(coord+"1"))+.8*(float(arm.get(coord+"2"))-float(arm.get(coord+"1")))
                self.assertAlmostEqual(float(actuator.get(coord+"2")), expected, places=2)
            self.assertIn("80.0% of arm", svg)


if __name__ == "__main__":
    unittest.main()
