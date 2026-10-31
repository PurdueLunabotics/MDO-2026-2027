"""Independent timing limits, failure domains and optional-mode integration."""
from dataclasses import replace
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from openmdao.utils.assert_utils import assert_check_totals

from rover.config import ROUTE, configuration
from rover.models import ModelSet
from rover.mobility import Mobility
from rover.motors import Gearmotor, ReferenceMotor
from rover.problem import build_problem
from rover.alternatives.bekker_wong_reece import load_preset, WheelDomainError
from rover.alternatives.calculated_traversal import state_route, motor_rim_speed_limit
from reporting.report import snapshot


def inputs():
    design, assumptions = configuration()
    return {n:s[0] for n,s in {**design, **assumptions}.items()} | {
        "wheel_diameter": .33, "wheel_width": .125,
    }


class UnlimitedMotor:
    """Synthetic analytic limit only; not a physical motor or calibration."""
    def speed_limit(self, torque):
        return 1e10

    def peak_power(self):
        return 1e10


class CalculatedTraversalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.soil, _ = load_preset()

    def problem(self, *args, **kwargs):
        p = build_problem(*args, **kwargs)
        self.addCleanup(p.cleanup)
        return p

    def test_unlimited_motor_recovers_independent_rest_to_rest_route_limit(self):
        v = inputs()
        result = state_route(self.soil, v, UnlimitedMotor(), 63., 10.)
        expected = 0.
        for distance, yaw in ROUTE:
            for d, speed, acceleration in ((distance, v["travel_speed"], v["acceleration"]),
                    (abs(math.radians(yaw))*v["track_width"]/2,
                     v["travel_speed"]*v["turn_speed_fraction"],
                     v["acceleration"]*v["turn_accel_fraction"])):
                if d:
                    peak = min(speed, math.sqrt(d*acceleration))
                    expected += d/peak+peak/acceleration
        self.assertAlmostEqual(result["duration"], expected, places=10)
        self.assertEqual(result["acceleration"], v["acceleration"])
        self.assertLess(result["slip"], 1.)
        for _, distance, duration, peak, acceleration in result["profiles"]:
            self.assertAlmostEqual(distance, peak*(duration-peak/acceleration), places=10)

    def test_motor_limit_respects_torque_speed_and_power_with_slip(self):
        motor = Gearmotor(ReferenceMotor(100., 3000., False), 100., .8, .72)
        force, radius, factor = 120., .165, .7
        ground = motor_rim_speed_limit(motor, force, radius, factor)
        rpm = ground/factor/radius*30/math.pi
        self.assertGreaterEqual(4*motor.available_torque(rpm)/radius-force, .3-1e-8)
        self.assertGreaterEqual(4*motor.peak_power()-force*ground/factor, .4-1e-8)

    def test_weaker_motor_and_smaller_wheel_increase_time_for_same_mass(self):
        v = inputs() | {"travel_speed": 3.}
        strong = Gearmotor(ReferenceMotor(100., 3000., False), 100., .8, .72)
        weak = Gearmotor(ReferenceMotor(25., 3000., False), 100., .8, .72)
        a = state_route(self.soil, v, strong, 63., 10.)
        b = state_route(self.soil, v, weak, 63., 10.)
        small = state_route(self.soil, v | {"wheel_diameter": .18}, strong, 63., 10.)
        self.assertGreater(b["duration"], a["duration"])
        self.assertGreater(small["duration"], a["duration"])

    def test_failure_domains_do_not_fall_back_to_fixed_time(self):
        v = inputs()
        weak = Gearmotor(ReferenceMotor(.001, 3000., False), 100., .8, .72)
        with self.assertRaisesRegex(WheelDomainError, "cannot sustain"):
            state_route(self.soil, v, weak, 63., 10.)
        for change in ({"wheel_width": 0.}, {"acceleration": 0.}, {"travel_speed": float("nan")}):
            with self.subTest(change=change), self.assertRaises(WheelDomainError):
                state_route(self.soil, v | change, UnlimitedMotor(), 63., 10.)
        with self.assertRaises(WheelDomainError):
            state_route(self.soil, v, UnlimitedMotor(), 63., 10., route=((0., 0.),))

    def test_default_is_exact_calculated_mode_and_custom_models_are_preserved(self):
        a, b = self.problem(), self.problem(traversal_mode="calculated")
        a.run_model(); b.run_model()
        sa, sb = snapshot(a), snapshot(b)
        self.assertEqual(sa["outputs"], sb["outputs"])
        self.assertEqual(sa["constraints"], sb["constraints"])
        self.assertEqual(sa["whole_cycles"], sb["whole_cycles"])
        self.assertEqual(a.model_metadata["traversal_mode"], "calculated")
        self.assertNotEqual(a.get_val("travel_time").item(), 90.)
        fixed = self.problem(traversal_mode="fixed")
        fixed.run_model()
        self.assertEqual(fixed.get_val("travel_time").item(), 90.)
        custom = replace(ModelSet(), mobility=Mobility)
        with self.assertRaisesRegex(ValueError, "not replaced"):
            build_problem(models=custom)
        preserved = self.problem(models=custom, traversal_mode="fixed")
        self.assertIs(type(preserved.model.mobility), Mobility)
        with self.assertRaisesRegex(ValueError, "fixed or calculated"):
            build_problem(traversal_mode="unknown")

    def test_calculated_time_accounting_inactive_input_and_explicit_dependencies(self):
        p = self.problem(traversal_mode="calculated")
        p.run_model()
        before = snapshot(p)
        get = lambda n:p.get_val(n).item()
        self.assertAlmostEqual(get("travel_time"), get("outbound_travel_time")+get("return_travel_time"))
        self.assertAlmostEqual(get("travel_time"), get("route_translation_time")+get("route_turn_time"))
        self.assertNotEqual(get("outbound_travel_time"), get("return_travel_time"))
        self.assertFalse(any(source.startswith("_auto_ivc.") for source in p.model._conn_global_abs_in2out.values()))
        self.assertEqual(set(p.model.mobility.DEPENDENCIES), set(p.model.mobility.OUTPUT_UNITS))
        self.assertFalse(any("traversal_time_one_way" in deps for deps in p.model.mobility.DEPENDENCIES.values()))
        p.set_val("traversal_time_one_way", 123.)
        p.run_model()
        self.assertEqual(before["outputs"], snapshot(p)["outputs"])
        energy_before, time = get("mobility_energy"), get("travel_time")+get("turn_time")
        p.set_val("actuator_idle_power", get("actuator_idle_power")+5.)
        p.run_model()
        self.assertAlmostEqual(get("mobility_energy")-energy_before, 4*5*time, places=8)
        phases = sum(get(n) for n in ("dig_energy", "dump_energy", "relocation_energy", "mobility_energy", "arm_energy"))
        self.assertAlmostEqual(get("cycle_energy")*3600, phases+get("electronics_power")*get("cycle_time"), places=8)
        self.assertAlmostEqual(get("run_energy"), get("cycles")*get("cycle_energy"), places=10)

    def test_smooth_timing_and_energy_total_derivatives(self):
        p = self.problem({"wheel_diameter": .29, "wheel_width": .125}, traversal_mode="calculated")
        p.run_model()
        checks = p.check_totals(of=["travel_time", "mobility_energy"], wrt=["wheel_diameter", "battery_capacity"],
                               method="fd", form="central", step=3e-5, step_calc="rel_avg", out_stream=None)
        assert_check_totals(checks, atol=1e-3, rtol=2e-3)

    def test_case_mode_cli_override_and_report_actual_leg_times(self):
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder)/"case.json", Path(folder)/"calculated.json"
            source.write_text(json.dumps({"traversal_mode": "calculated", "inputs": {"traversal_time_one_way": 37.}}))
            before = source.read_bytes()
            common = [sys.executable, "main.py", "--inputs", str(source)]
            run = subprocess.run(common+["--output", str(output)], capture_output=True, text=True,
                                 cwd=Path(__file__).resolve().parents[1])
            self.assertEqual(run.returncode, 0, run.stderr)
            r = json.loads(output.read_text())
            self.assertEqual(r["model_metadata"]["traversal_mode"], "calculated")
            self.assertNotEqual(r["final"]["outputs"]["travel_time"]["value"], 74.)
            html = output.with_suffix(".html").read_text(encoding="utf-8")
            self.assertTrue("traversal_time_one_way is inactive" in html)
            self.assertFalse("37 s outbound + 37 s return" in html)
            fixed = Path(folder)/"fixed.json"
            run = subprocess.run(common+["--traversal-mode", "fixed", "--output", str(fixed)],
                                 capture_output=True, text=True, cwd=Path(__file__).resolve().parents[1])
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertEqual(json.loads(fixed.read_text())["final"]["outputs"]["travel_time"]["value"], 74.)
            self.assertEqual(source.read_bytes(), before)

    def test_cli_defaults_to_calculated_and_honors_explicit_fixed_file(self):
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder)/"case.json", Path(folder)/"result.json"
            for explicit_mode in (None, "fixed"):
                case = {"inputs": {"traversal_time_one_way": 37.}}
                if explicit_mode:
                    case["traversal_mode"] = explicit_mode
                source.write_text(json.dumps(case), encoding="utf-8")
                run = subprocess.run([sys.executable, "main.py", "--inputs", str(source),
                                      "--output", str(output)], capture_output=True, text=True,
                                     cwd=Path(__file__).resolve().parents[1])
                self.assertEqual(run.returncode, 0, run.stderr)
                record = json.loads(output.read_text(encoding="utf-8"))
                self.assertEqual(record["model_metadata"]["traversal_mode"], explicit_mode or "calculated")
                if explicit_mode == "fixed":
                    self.assertEqual(record["final"]["outputs"]["travel_time"]["value"], 74.)
                else:
                    self.assertNotEqual(record["final"]["outputs"]["travel_time"]["value"], 74.)


if __name__ == "__main__":
    unittest.main()
