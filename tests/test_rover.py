"""Independent formula checks, physical trends, and one complete optimizer run."""

import math
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_totals
from rover.components import Mission, travel_profile
from rover.config import DESIGN, ASSUMPTIONS
from rover.problem import build_problem, MARGINS
from rover.report import snapshot
from rover.dependencies import dependencies
from rover.mobility import route_profiles
from rover.html_report import geometry_svg, values_from_snapshot, write_report
from openmdao.visualization.n2_viewer.n2_viewer import _get_viewer_data


class RoverTests(unittest.TestCase):
    def problem(self, **overrides):
        p = build_problem(overrides)
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def test_travel_time(self):
        # 0->2 m/s at 1 m/s^2 takes 2 m; 6 m cruise; 2 m braking.
        self.assertEqual(travel_profile(10, 2, 1), (7, 2))
        # Too short to reach the requested speed: triangular profile.
        time, peak = travel_profile(1, 2, 1)
        self.assertEqual((time, peak), (2, 1))
        left = travel_profile(4 - 1e-7, 2, 1)[0]
        right = travel_profile(4 + 1e-7, 2, 1)[0]
        self.assertAlmostEqual(left, right, places=6)

    def test_guidebook_score_example(self):
        p = om.Problem(reports=False)
        p.model.add_subsystem("mission", Mission(), promotes=["*"])
        p.setup()
        self.addCleanup(p.cleanup)
        # Independently create the guidebook's 77551 cm^3, 66 kg, 36 Wh, 15 min.
        for name in ("dig_time", "dump_time", "travel_time", "turn_time"):
            p.set_val(name, 10, units="s")
        p.set_val("overhead_time", 10, units="s")  # 50 sec/cycle, 18 cycles
        p.set_val("actuation_time", 0, units="s")
        p.set_val("dry_mass", 66, units="kg")
        p.set_val("run_time", 900, units="s")
        p.set_val("delivery_efficiency", 1)
        p.set_val("berm_density", 1500, units="kg/m**3")
        p.set_val("payload_mass", 77551e-6 * 1500 / 18, units="kg")
        for name in ("dig_power", "dump_power", "electronics_power"):
            p.set_val(name, 0, units="W")
        p.set_val("arm_energy", 0, units="J")
        p.set_val("mobility_energy", 36 * 3600 / 18, units="J")
        p.run_model()
        self.assertAlmostEqual(p.get_val("mass_points").item(), 344.671111111, places=6)
        self.assertAlmostEqual(p.get_val("energy_points").item(), 215.419444444, places=6)
        self.assertAlmostEqual(p.get_val("construction_points").item(), 560.090555555, places=6)
        p.set_val("mass_score_weight", 0)
        p.run_model()
        self.assertAlmostEqual(p.get_val("construction_points").item(), 215.419444444, places=6)

    def test_four_drums_and_sequential_dump(self):
        p = self.problem()
        # Reachable soil, not nominal capacity, limits the stationary plunge.
        self.assertLess(p.get_val("payload_mass").item(), p.get_val("storage_capacity").item()*1250)
        # The original 0.40 m wheelbase no longer meets the new 0.40 m obstacle
        # pose requirement. Preserve that failure while checking drum behavior.
        report=snapshot(p)
        self.assertFalse(report["feasible"])
        self.assertFalse(report["constraints"]["obstacle_pose_margin"]["passed"])
        self.assertFalse(report["constraints"]["obstacle_drum_wheel_margin"]["passed"])
        self.assertTrue(all(c["passed"] for n,c in report["constraints"].items()
                            if n not in {"obstacle_pose_margin","obstacle_tip_margin",
                                         "obstacle_drum_wheel_margin"}))
        self.assertLessEqual(p.get_val("payload_mass").item(), p.get_val("fresh_volume").item()*1500)
        self.assertAlmostEqual(p.get_val("dump_time").item(), 2*(3*60/15+2))
        self.assertAlmostEqual(p.get_val("turn_time").item(), travel_profile(math.pi * p.get_val("track_width").item() / 2, .35, .2)[0])

    def test_gearing_tradeoff_and_power(self):
        p = self.problem(drive_gear_ratio=40)
        low_force = p.get_val("stall_drive_force").item()
        low_speed = p.get_val("no_load_speed").item()
        p.set_val("drive_gear_ratio", 80)
        p.run_model()
        self.assertAlmostEqual(p.get_val("stall_drive_force").item(), 2 * low_force)
        self.assertAlmostEqual(p.get_val("no_load_speed").item(), low_speed / 2)
        margin_before = p.get_val("straight_motor_margin").item()
        p.set_val("drive_motor_power", 200)
        p.run_model()
        self.assertGreater(p.get_val("straight_motor_margin").item(), margin_before)

    def test_shorter_mission_and_frozen_traversal(self):
        p = self.problem()
        first = snapshot(p)
        p.set_val("run_time", 450, units="s")
        p.run_model()
        self.assertAlmostEqual(p.get_val("run_energy").item(), first["outputs"]["run_energy"]["value"] / 2)
        self.assertAlmostEqual(p.get_val("berm_volume").item(), first["outputs"]["berm_volume"]["value"] / 2)
        before = p.get_val("travel_time").item()
        p.set_val("drive_motor_power", 300, units="W")
        p.set_val("drive_gear_ratio", 20)
        p.run_model()
        self.assertEqual(p.get_val("travel_time").item(), before)
        self.assertEqual(before, 90)
        self.assertEqual(p.get_val("route_distance").item(), 18)
        self.assertEqual(p.get_val("route_turn_angle").item(), 360)
        self.assertAlmostEqual(p.get_val("route_translation_time").item() + p.get_val("route_turn_time").item(), 90)
        self.assertNotIn("travel_speed", DESIGN)
        self.assertNotIn("acceleration", DESIGN)

    def test_infeasible_baseline_and_battery(self):
        p = self.problem(battery_capacity=10)
        report = snapshot(p)
        self.assertFalse(report["feasible"])
        self.assertFalse(report["constraints"]["battery_energy_margin"]["passed"])

    def test_route_profiles_preserve_distances(self):
        profiles = route_profiles(.52, 1, .4, .35, .5, 45)
        self.assertEqual(len(profiles), 7)
        self.assertAlmostEqual(sum(p[2] for p in profiles), 45)
        self.assertAlmostEqual(sum(p[1] for p in profiles if p[0] == "straight"), 9)
        self.assertAlmostEqual(sum(p[1] for p in profiles if p[0] == "turn"), math.pi * .52/2)
        for _, distance, duration, speed, accel in profiles:
            self.assertAlmostEqual(speed * (duration - speed/accel), distance)

    def test_linkage_closure_and_motion_time(self):
        p = self.problem()
        for state,length_name in (("stowed","actuator_retracted_length"),("extended","actuator_extended_length")):
            x,y = (p.get_val(state+"_"+coord).item() for coord in ("x","y"))
            px=p.get_val("pivot_x").item()
            self.assertAlmostEqual(math.hypot(x-px,y-p.get_val("bottom_pivot_height").item()),p.get_val("arm_length").item())
            self.assertAlmostEqual(math.hypot(x-p.get_val("actuator_pivot_x").item(),
                                              y-p.get_val("top_pivot_height").item()),p.get_val(length_name).item())
        self.assertGreaterEqual(p.get_val("dig_reach_margin").item(),0)
        before=p.get_val("actuation_time").item()
        p.set_val("actuator_speed",.05)
        p.run_model()
        self.assertAlmostEqual(p.get_val("actuation_time").item(),before/2)

    def test_infeasible_linkage_is_finite_and_rejected(self):
        p=self.problem(actuator_extended_length=1.05,arm_length=.30)
        result=snapshot(p)
        self.assertFalse(result["feasible"])
        self.assertFalse(result["constraints"]["triangle_extended_margin"]["passed"])
        self.assertTrue(all(math.isfinite(item["value"]) for item in result["outputs"].values()))

    def test_dependency_maps_and_n2(self):
        p=self.problem()
        n2=_get_viewer_data(p)
        declared=set(n2["declare_partials_list"])
        self.assertIn("geometry.battery_mass > geometry.battery_capacity",declared)
        self.assertNotIn("geometry.battery_mass > geometry.wheel_diameter",declared)
        self.assertIn("mobility.travel_time > mobility.traversal_time_one_way",declared)
        self.assertNotIn("mobility.travel_time > mobility.drive_motor_power",declared)
        self.assertNotIn("linkage.actuation_time > linkage.pivot_x",declared)
        # Numerically audit every OMITTED pair at two physically distinct designs.
        # This catches missing dependencies even though whole-model FD can optimize
        # successfully without using the declared component Jacobians.
        for alternate in (False,True):
            if alternate:
                for name,value in {"drum_length":.345,"wheel_width":.08,"wheel_diameter":.25,
                                   "actuator_retracted_length":.40,"travel_speed":.4}.items():
                    p.set_val(name,value)
                p.run_model()
            for comp in p.model.system_iter(recurse=True,typ=om.ExplicitComponent):
                if isinstance(comp,om.IndepVarComp):
                    continue
                mapping=getattr(comp,"DEPENDENCIES",None) or dependencies(type(comp).__name__)
                inputs={name:comp._inputs[name].copy() for name in comp._var_rel_names["input"]}
                out_names=comp._var_rel_names["output"]
                self.assertEqual(set(mapping),set(out_names))
                base={name:np.zeros(1) for name in out_names}
                comp.compute(inputs,base)
                for name in inputs:
                    step=1e-4*max(abs(inputs[name].item()),1e-2)
                    shifted={key:value.copy() for key,value in inputs.items()}
                    shifted[name]+=step
                    changed={key:np.zeros(1) for key in out_names}
                    comp.compute(shifted,changed)
                    for out in out_names:
                        if name not in mapping[out]:
                            delta=float(np.asarray(changed[out]-base[out]).item())
                            scale=max(1,abs(float(np.asarray(base[out]).item())))
                            self.assertLess(abs(delta)/scale,1e-9,f"Missing dependency {comp.name}.{out} <- {name}")

    def test_html_report_and_drawings(self):
        p=self.problem()
        snap=snapshot(p)
        values=values_from_snapshot(snap)
        for extended in (False,True):
            svg=geometry_svg(values,extended)
            self.assertIn('<svg',svg)
            self.assertIn('Ground level',svg)
            self.assertNotIn('nan',svg.lower())
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/"design.json"
            write_report({"final":snap,"baseline":snap,"status":"baseline","optimizer_success":None,"openmdao_version":"test"},output)
            html=output.with_suffix('.html').read_text(encoding='utf-8')
            self.assertIn('Design variables and bounds',html)
            self.assertIn('actuator_extended_length',html)
            self.assertIn('45 s outbound + 45 s return',html)
            self.assertIn('Guarded pose: requested height is unreachable',html)
            self.assertIn('Ditch requirement remains unverified',html)
            self.assertIn('Obstacle drum–wheel clearance',html)
            self.assertIn('Guarded clearance: requested pose is unreachable',html)
            self.assertTrue((output.parent/'design_extended.svg').exists())

    def test_invalid_config(self):
        for overrides in ({"battery_capacity": 0}, {"typo": 1}, {"drum_diameter": .36},
                          {"soil_density": float("nan")}, {"wheel_deflection_fraction": .6},
                          {"turn_angle": 1}, {"mass_score_weight": 0, "energy_score_weight": 0}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                build_problem(overrides)

    def test_freezing_a_design_variable(self):
        reduced = {name: spec for name, spec in DESIGN.items() if name != "wheel_diameter"}
        with patch.dict(DESIGN, reduced, clear=True), patch.dict(ASSUMPTIONS, {"wheel_diameter": (0.3, "m")}):
            p = self.problem()
            self.assertNotIn("wheel_diameter", p.model.get_design_vars())
            self.assertAlmostEqual(p.get_val("wheel_diameter").item(), .3)
            with self.assertRaises(ValueError):
                build_problem({"wheel_diameter": .36})

    def test_optimization_and_derivative_stability(self):
        # Verified feasible numerical start, with unchanged success/feasibility
        # assertions. The original cold-start failures remain documented.
        fixture=json.loads((Path(__file__).parent/"fixtures"/"obstacle_clearance_feasible_start.json").read_text())
        p = self.problem(**fixture["inputs"])
        self.assertTrue(snapshot(p)["feasible"])
        before = p.get_val("construction_points").item()
        checks = p.check_totals(of=["construction_points", *MARGINS], wrt=list(DESIGN),
                               method="fd", form="central", step=3e-5, step_calc="rel_avg", out_stream=None)
        assert_check_totals(checks, atol=1e-3, rtol=2e-3)
        p.model.approx_totals(method="fd", form="central", step=1e-5, step_calc="rel_avg")
        result = p.run_driver()
        p.run_model()
        report = snapshot(p)
        self.assertTrue(result.success)
        self.assertTrue(report["feasible"])
        self.assertGreater(p.get_val("construction_points").item(), before)
        for name in ("wheel_diameter", "wheel_width", "drum_diameter", "drum_length"):
            self.assertLess(p.get_val(name).item(), .36)
        self.assertLessEqual(report["whole_cycles"]["berm_volume_m3"], p.get_val("berm_volume").item())
        self.assertTrue(np.isfinite(p.get_val("construction_points")).all())


if __name__ == "__main__":
    unittest.main()
