"""Bounded development runs retain physical checks and consistent final states."""
import json
import math
from dataclasses import replace
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from rover.config import configuration
from rover.development import BudgetedScipyDriver, RunBudgetExceeded, selected_freeze
from rover.problem import build_problem
from rover.models import ModelSet
from rover.alternatives.calculated_traversal import CalculatedTraversalMobility


ROOT = Path(__file__).resolve().parents[1]


class DevelopmentTests(unittest.TestCase):
    def test_vary_preserves_all_other_variables_as_fixed_and_rejects_mistakes(self):
        design, _ = configuration("plate-frame", "x44-standard", "pa04-900")
        self.assertEqual(set(selected_freeze(design, vary=["wheelbase", "arm_length"])),
                         set(design)-{"wheelbase", "arm_length"})
        for names in (["gravity"], ["typo"], []):
            with self.subTest(names=names), self.assertRaises(ValueError):
                selected_freeze(design, vary=names)
        with self.assertRaisesRegex(ValueError, "both"):
            selected_freeze(design, vary=["wheelbase"], freeze=["wheelbase"])

    def test_budget_guards_run_before_changing_the_model(self):
        d = BudgetedScipyDriver(optimizer="COBYLA", evaluation_limit=2, time_limit=5.)
        d.objective_evaluations = 2
        with self.assertRaisesRegex(RunBudgetExceeded, "evaluation"):
            d._objfunc(None)
        self.assertTrue(d.fail)
        d.objective_evaluations = 0
        d._deadline = 10.
        with patch("rover.development.time.perf_counter", return_value=11.):
            with self.assertRaisesRegex(RunBudgetExceeded, "time"):
                d._objfunc(None)

    def test_invalid_budgets_and_gradient_optimizers_are_rejected(self):
        for seconds in (0., -1., math.nan, math.inf):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                BudgetedScipyDriver(optimizer="COBYLA", evaluation_limit=2, time_limit=seconds)
        for evaluations in (0, -1, True, 1.5):
            with self.subTest(evaluations=evaluations), self.assertRaises(ValueError):
                BudgetedScipyDriver(optimizer="COBYLA", evaluation_limit=evaluations, time_limit=5.)
        with self.assertRaisesRegex(ValueError, "COBYLA"):
            BudgetedScipyDriver(optimizer="SLSQP", evaluation_limit=2, time_limit=5.)

    def test_calculated_route_reuses_explicit_force_solutions_without_changing_results(self):
        p = build_problem({"wheel_diameter": .33, "wheel_width": .125})
        self.addCleanup(p.cleanup)
        with patch.object(p.model.mobility, "straight_responder",
                          side_effect=AssertionError("Repeated soil solution")):
            p.run_model()
        outputs = {n: p.get_val(n).copy() for n in p.model.mobility.OUTPUT_UNITS}
        # Independently recompute force points using the retained fixed-route
        # responder, then account for exactly the same selected route.
        c = p.model.mobility
        v = lambda n: float(c._inputs[n].item())
        plan = c.route_plan(v)
        plan.pop("straight_response")
        reference = {}
        c.compute_route(c._inputs, reference, plan)
        for name, value in reference.items():
            self.assertAlmostEqual(float(outputs[name].item()), float(value), places=8, msg=name)

    def cli(self, *args):
        return subprocess.run([sys.executable, "-B", "main.py", *map(str, args)], cwd=ROOT,
                              capture_output=True, text=True, timeout=35)

    def test_custom_calculated_responder_is_not_bypassed(self):
        calls = []

        class CustomTraversal(CalculatedTraversalMobility):
            def straight_responder(self, values, segments):
                calls.append(True)
                return super().straight_responder(values, segments)

        p = build_problem(models=replace(ModelSet(), mobility=CustomTraversal))
        self.addCleanup(p.cleanup)
        p.run_model()
        self.assertEqual(calls, [True])

    def test_quick_run_honors_exact_budget_preserves_failures_and_skips_reports(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"quick.json"
            run = self.cli("--quick", "--vary", "battery_capacity", "--maxiter", "2",
                           "--time-limit", "20", "--output", output)
            self.assertEqual(run.returncode, 1, run.stdout+run.stderr)
            record = json.loads(output.read_text(encoding="utf-8"))
            self.assertFalse(record["optimizer_success"])
            self.assertEqual(record["status"], "failed_or_infeasible")
            self.assertEqual(record["performance"]["objective_evaluations"], 2)
            self.assertEqual(set(record["final"]["design"]), {"battery_capacity"})
            self.assertIn("wheel_diameter_bounds", record["final"]["constraints"])
            self.assertIn("tip_stability_margin", record["final"]["constraints"])
            self.assertEqual(list(Path(folder).iterdir()), [output])

    def test_profile_is_one_fixed_evaluation_with_numeric_output(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"profile.json"
            run = self.cli("--profile-model", "--output", output)
            self.assertEqual(run.returncode, 0, run.stdout+run.stderr)
            record = json.loads(output.read_text(encoding="utf-8"))
            self.assertIsNone(record["optimizer_success"])
            self.assertEqual(record["status"], "baseline")
            self.assertNotIn("driver_seconds", record["performance"]["timings"])
            self.assertGreater(record["performance"]["timings"]["baseline_seconds"], 0)
            self.assertTrue(output.with_name("profile_profile.txt").exists())
            self.assertFalse(output.with_suffix(".sql").exists())

    def test_time_budget_preserves_the_last_complete_design(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"timed.json"
            run = self.cli("--quick", "--vary", "battery_capacity", "--time-limit", "0.000001",
                           "--output", output)
            self.assertEqual(run.returncode, 1, run.stdout+run.stderr)
            record = json.loads(output.read_text(encoding="utf-8"))
            self.assertFalse(record["optimizer_success"])
            self.assertIn("time limit", record["optimizer_result"])
            self.assertEqual(record["performance"]["objective_evaluations"], 0)
            self.assertEqual(record["baseline"]["design"], record["final"]["design"])
            self.assertEqual(record["baseline"]["outputs"], record["final"]["outputs"])


if __name__ == "__main__":
    unittest.main()
