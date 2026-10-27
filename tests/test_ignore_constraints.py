"""Relaxing the search must never hide a failed physical check."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from reporting.report import snapshot
from rover.problem import build_problem


class IgnoreConstraintTests(unittest.TestCase):
    def problem(self, **options):
        p = build_problem(**options)
        self.addCleanup(p.cleanup)
        return p

    def test_only_named_margin_is_removed_without_changing_physics_or_feasibility(self):
        # Deliberately low assumed friction guarantees a failed turn screen.
        ordinary = self.problem(overrides={"traction_coefficient": .01})
        relaxed = self.problem(overrides={"traction_coefficient": .01},
                               ignore_constraints="turn_traction_margin")
        ordinary.run_model()
        relaxed.run_model()
        before, after = snapshot(ordinary), snapshot(relaxed)
        self.assertEqual(set(relaxed.model.get_constraints()),
                         set(ordinary.model.get_constraints())-{"turn_traction_margin"})
        self.assertEqual(before["outputs"], after["outputs"])
        self.assertEqual(before["feasible"], after["feasible"])
        self.assertFalse(after["constraints"]["turn_traction_margin"]["passed"])
        self.assertFalse(after["constraints"]["turn_traction_margin"]["enforced_in_optimization"])
        self.assertFalse(after["feasible"])
        self.assertEqual(relaxed.model_metadata["configuration"]["ignored_optimizer_constraints"],
                         ["turn_traction_margin"])
        self.assertEqual(before["constraints"]["actuator_force_margin"],
                         after["constraints"]["actuator_force_margin"])

    def test_invalid_names_and_hard_limits_are_not_silently_ignored(self):
        for name in ("turn_fraction_margin", "dry_mass", "rover_width", "wheel_width_bounds"):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "active margin name"):
                build_problem(ignore_constraints=[name])
        with self.assertRaisesRegex(ValueError, "active margin name"):
            build_problem(structure="plate-frame", ignore_constraints=["chassis_clearance_margin"])
        p = self.problem(ignore_constraints=["empty_slope_traversal_margin", "loaded_slope_traversal_margin"])
        p.run_model()
        report = snapshot(p)
        for state in ("empty", "loaded"):
            name = state+"_slope_traversal_margin"
            self.assertNotIn(name, p.model.get_constraints())
            self.assertFalse(report["constraints"][name]["passed"])
        self.assertFalse(report["feasible"])

    def test_cli_and_html_keep_excluded_failures_visible(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"study.json"
            result = subprocess.run([
                sys.executable, "main.py", "--baseline", "--set", "traction_coefficient=0.01",
                "--ignore-constraint", "turn_traction_margin",
                "--ignore-constraint", "empty_slope_traversal_margin", "loaded_slope_traversal_margin",
                "--output", str(path),
            ], cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            record = json.loads(path.read_text(encoding="utf-8"))
            check = record["final"]["constraints"]["turn_traction_margin"]
            self.assertFalse(check["passed"])
            self.assertFalse(check["enforced_in_optimization"])
            self.assertFalse(record["final"]["feasible"])
            self.assertIn("Excluded from optimizer (still checked in full feasibility)", result.stdout)
            html = path.with_suffix(".html").read_text(encoding="utf-8")
            self.assertIn("Constraints excluded from optimizer: ", html)
            self.assertTrue("Turning: traction reserve (excluded from optimizer)" in html)
            self.assertIn("is not a feasible design", html)


if __name__ == "__main__":
    unittest.main()
