"""Fixed hardware must survive driver evaluations and report round trips."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import openmdao.api as om

from reporting.report import snapshot
from rover.config import configuration
from rover.problem import build_problem


HARDWARE = {
    "wheel_diameter": .33,
    "wheel_width": .18,
    "travel_actuator_length": .50,
    "actuator_retracted_length": .508,
    "actuator_extended_length": .70,
}


class FreezeDesignTests(unittest.TestCase):
    def problem(self, *args, **kwargs):
        p = build_problem(*args, **kwargs)
        self.addCleanup(p.cleanup)
        return p

    def test_freezing_changes_search_space_without_changing_physics(self):
        for structure, motors in (("reference", "reference"), ("plate-frame", "x44-standard")):
            with self.subTest(structure=structure):
                ordinary = self.problem(HARDWARE, structure=structure, motors=motors)
                frozen = self.problem(HARDWARE, structure=structure, motors=motors, freeze=HARDWARE)
                ordinary.run_model()
                frozen.run_model()
                expected, actual = snapshot(ordinary), snapshot(frozen)
                self.assertEqual(actual["outputs"], expected["outputs"])
                self.assertEqual(actual["constraints"], expected["constraints"])
                self.assertEqual(actual["feasible"], expected["feasible"])
                self.assertEqual(set(frozen.model.get_design_vars()), set(ordinary.design_specs)-set(HARDWARE))
                self.assertEqual(set(frozen.design_specs), set(ordinary.design_specs)-set(HARDWARE))
                for name, value in HARDWARE.items():
                    self.assertEqual(actual["assumptions"][name]["value"], value)
                    self.assertNotIn(name, actual["design"])
                    self.assertIn(name+"_bounds", actual["constraints"])
                self.assertFalse(any(source.startswith("_auto_ivc.")
                                     for source in frozen.model._conn_global_abs_in2out.values()))
        # Freezing is local to the problem, never a mutation of global defaults.
        self.assertTrue(set(HARDWARE) <= set(configuration()[0]))

    def test_driver_keeps_frozen_values_in_every_recorded_evaluation(self):
        design, _ = configuration()
        names = set(design)-{"battery_capacity"}
        p = self.problem(HARDWARE, optimizer="COBYLA", maxiter=4, cobyla_radius=.02, freeze=names)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"iterations.sql"
            p.driver.add_recorder(om.SqliteRecorder(str(path)))
            p.driver.recording_options["includes"] = ["*"]
            p.run_driver()
            final = snapshot(p)
            p.cleanup()
            reader = om.CaseReader(str(path))
            cases = list(reader.get_cases("driver", recurse=False))
            self.assertGreaterEqual(len(cases), 3)
            self.assertGreater(len({case.get_val("battery_capacity").item() for case in cases}), 1)
            for case in cases:
                state = snapshot(case, configuration=p.model_metadata["configuration"])
                for name in names:
                    self.assertEqual(case.get_val(name).item(), p.assumption_specs[name][0])
                    self.assertEqual(state["assumptions"][name], final["assumptions"][name])
        # Fixed variables still retain their original bound checks in reports.
        p.set_val("wheel_width", .01)
        self.assertFalse(snapshot(p)["constraints"]["wheel_width_bounds"]["passed"])

    def test_bad_freeze_requests_and_out_of_bounds_values_are_rejected(self):
        for name in ("wheel_size", "gravity"):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "unknown or already fixed"):
                build_problem(freeze=[name])
        with self.assertRaisesRegex(ValueError, "unknown or already fixed"):
            build_problem(structure="plate-frame", freeze=["frame_wall"])
        with self.assertRaisesRegex(ValueError, "positive bounds"):
            build_problem({"wheel_width": .01}, freeze=["wheel_width"])
        with self.assertRaisesRegex(ValueError, "--baseline"):
            build_problem(freeze=configuration()[0])
        p = self.problem(freeze="wheel_width")
        self.assertNotIn("wheel_width", p.design_specs)

    def test_current_actuator_length_bounds_are_enforced_when_frozen(self):
        for structure in ("reference", "plate-frame"):
            design, _ = configuration(structure)
            for name in ("travel_actuator_length", "actuator_retracted_length", "actuator_extended_length"):
                _, lower, upper, _ = design[name]
                self.assertEqual((lower, upper), (.453898, .707898))
                for value in (lower-1e-6, upper+1e-6):
                    with self.subTest(structure=structure, name=name, value=value):
                        with self.assertRaisesRegex(ValueError, "positive bounds"):
                            build_problem({name: value}, structure=structure, freeze=[name])

    def test_cli_uses_file_then_set_values_and_identifies_fixed_inputs_in_report(self):
        design, _ = configuration("plate-frame", "x44-standard")
        other_names = sorted(set(design)-set(HARDWARE)-{"battery_capacity"})
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder)/"hardware.json", Path(folder)/"study.json"
            source.write_text(json.dumps({"structure": "plate-frame", "motors": "x44-standard",
                                          "inputs": HARDWARE}), encoding="utf-8")
            before = source.read_bytes()
            result = subprocess.run([
                sys.executable, "main.py", "--inputs", str(source), "--optimize",
                "--freeze", *HARDWARE, "--freeze", *other_names,
                "--set", "wheel_diameter=0.34", "--maxiter", "4", "--output", str(output),
            ], cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
            self.assertIn(result.returncode, (0, 1), result.stderr)
            self.assertTrue(output.exists(), result.stdout+result.stderr)
            record = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(set(record["final"]["design"]), {"battery_capacity"})
            for state in (record["baseline"], record["final"]):
                for name, value in {**HARDWARE, "wheel_diameter": .34}.items():
                    self.assertEqual(state["assumptions"][name]["value"], value)
                    self.assertTrue(state["constraints"][name+"_bounds"]["passed"])
            self.assertEqual(record["model_metadata"]["configuration"]["frozen_design"]["wheel_diameter"][0], .34)
            self.assertFalse(record["final"]["feasible"])
            self.assertEqual(record["status"], "failed_or_infeasible")
            self.assertEqual(result.returncode, 1)
            self.assertIn("Frozen design variables (held fixed)", result.stdout)
            html = output.with_suffix(".html").read_text(encoding="utf-8")
            self.assertIn("Frozen design variables (held fixed)", html)
            self.assertIn("wheel_diameter (fixed)", html)
            self.assertEqual(source.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
