"""Exact user hardware choices and a positive deposition support reserve."""
import unittest
import json
import math
from pathlib import Path

from rover.config import configuration, PA04_STROKES
from rover.problem import build_problem
from reporting.report import snapshot


class GeometryRequirementTests(unittest.TestCase):
    def test_discrete_strokes_have_exact_command_endpoints(self):
        self.assertEqual(PA04_STROKES, {.1524: (.352298, .504698), .2032: (.403098, .606298),
                                      .2286: (.428498, .657098), .254: (.453898, .707898)})
        for stroke, ends in PA04_STROKES.items():
            design, assumptions = configuration("plate-frame", "x44-standard", "pa04-900", actuator_stroke=stroke)
            self.assertNotIn("actuator_catalog_stroke", design)
            self.assertEqual(assumptions["actuator_catalog_stroke"], (stroke, "m"))
            self.assertEqual(assumptions["actuator_installation_allowance"][0], 0.)
            for name in ("travel_actuator_length", "deposition_actuator_length",
                         "actuator_retracted_length", "actuator_extended_length"):
                self.assertEqual(design[name][1:3], ends)
                self.assertTrue(ends[0] <= design[name][0] <= ends[1])
        manifest = json.loads((Path(__file__).resolve().parents[1]/"data/actuators/installed_strokes_2026-11-07.json").read_text())
        self.assertEqual({row['stroke']: (row['minimum_length'], row['maximum_length'])
                          for row in manifest['choices']}, PA04_STROKES)

    def test_intermediate_stroke_and_outside_command_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "6-inch"):
            build_problem({"actuator_catalog_stroke": .17}, actuators="pa04-900")
        with self.assertRaisesRegex(ValueError, "positive bounds"):
            build_problem({"actuator_catalog_stroke": .2032, "deposition_actuator_length": .352298}, actuators="pa04-900")
        with self.assertRaisesRegex(ValueError, "already define installed"):
            build_problem({"actuator_installation_allowance": .1613408}, actuators="pa04-900")

    def test_deposition_threshold_keeps_raw_output_and_reports_signed_reserve(self):
        p = build_problem()
        self.addCleanup(p.cleanup)
        p.run_model()
        for distance, passed in ((.0249, False), (.025, True), (.026, True)):
            p.set_val("tip_stability_margin", distance)
            state = snapshot(p)
            self.assertEqual(state["outputs"]["tip_stability_margin"]["value"], distance)
            check = state["constraints"]["tip_stability_margin"]
            self.assertEqual(check["required_minimum"], .025)
            self.assertAlmostEqual(check["margin"], distance-.025)
            self.assertEqual(check["passed"], passed)
        self.assertAlmostEqual(float(p.model.get_constraints()["tip_stability_margin"]["lower"]), .0253)
        legacy = dict(p.model_metadata["configuration"])
        legacy.pop("margin_minimums")
        self.assertAlmostEqual(snapshot(p, configuration=legacy)["constraints"]["tip_stability_margin"]["margin"], .026)

    def test_study_adapters_match_full_model_and_pressure_is_conservative(self):
        from studies.geometry import GeometryEvaluator, GEOMETRY_MARGINS
        e = GeometryEvaluator({}, .1524, .359)
        _, values, _ = e.evaluate_values(e.values)
        p = build_problem(e.values, structure='plate-frame', motors='x44-standard', actuators='pa04-900')
        self.addCleanup(p.cleanup)
        p.run_model()
        for name in (*GEOMETRY_MARGINS, 'dry_mass', 'payload_mass', 'loaded_cg_z'):
            if name == 'ground_pressure_margin':
                continue
            self.assertTrue(math.isclose(values[name], p.get_val(name).item(), rel_tol=1e-8, abs_tol=1e-7), name)
        self.assertGreaterEqual(values['ground_pressure']+1e-7, p.get_val('ground_pressure').item())

    def test_study_rejects_nonfinite_inputs_and_changed_hardware(self):
        from studies.geometry import GeometryEvaluator
        e = GeometryEvaluator({}, .1524, .359)
        with self.assertRaisesRegex(ValueError, 'finite'):
            e.evaluate_values({**e.values, 'arm_length': math.nan})
        with self.assertRaisesRegex(ValueError, 'discrete actuator hardware'):
            e.evaluate_values({**e.values, 'actuator_catalog_stroke': .2032})


if __name__ == "__main__":
    unittest.main()
