"""Independent pose, load balance, energy, invalid-domain and connection checks."""
import math
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
from dataclasses import replace

from openmdao.utils.assert_utils import assert_check_totals
from openmdao.visualization.n2_viewer.n2_viewer import _get_viewer_data
from rover.problem import build_problem
from rover.config import DESIGN
from rover.models import ModelSet
from rover.mass_properties import mass_properties
from rover.alternatives.deposition_actuation import deposition_motion, DepositionActuation
from rover.alternatives.pa04_actuator import RATED_FORCE, INCH
from rover.excavation import Excavation
from rover.tip_stability import TipStability, DepositionTipStability, deposition_tip_margin
from reporting.report import snapshot
from reporting.html_report import write_report, values_from_snapshot, geometry_svg


class DepositionTests(unittest.TestCase):
    def problem(self, overrides=None, **options):
        p = build_problem(overrides, **options)
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def test_equal_poses_preserve_properties_and_add_zero_motion(self):
        for structure, actuators in (("reference", "reference"), ("plate-frame", "pa04-900")):
            with self.subTest(structure=structure):
                p = self.problem({"travel_actuator_length": .49}, structure=structure, actuators=actuators)
                self.assertEqual(p.get_val("deposition_actuator_length").item(), .49)
                for state in ("empty", "loaded", "half"):
                    for quantity in ("cg_x", "cg_z", "yaw_inertia"):
                        name = state+"_"+quantity
                        self.assertEqual(p.get_val(name).item(), p.get_val("deposition_"+name).item())
                for n in DepositionActuation.OUTPUT_UNITS:
                    self.assertEqual(p.get_val(n).item(), 0)
                self.assertIn("deposition_actuator_length", p.model.get_design_vars())
        # Deliberately distinct configured initial guesses remain independent.
        with patch.dict(DESIGN, {"deposition_actuator_length": (.48, .453898, .707898, "m")}):
            p = self.problem({"travel_actuator_length": .49})
            self.assertEqual(p.get_val("deposition_actuator_length").item(), .48)

    def test_deposition_changes_turn_state_and_leaves_travel_states_alone(self):
        for structure, motors in (("reference", "reference"), ("plate-frame", "x44-standard")):
            with self.subTest(structure=structure):
                p = self.problem({"travel_actuator_length": .52, "deposition_actuator_length": .49},
                                 structure=structure, motors=motors)
                unchanged = ("travel_x", "travel_y", "loaded_cg_x", "loaded_cg_z", "empty_yaw_inertia",
                    "loaded_yaw_inertia", "loaded_sprung_cg_z", "travel_time", "route_translation_time",
                    "route_turn_time", "slope_tip_margin", "obstacle_tip_margin", "dry_mass", "payload_mass")
                before = {n: p.get_val(n).item() for n in unchanged}
                x, z, inertia = (p.get_val("deposition_"+n).item() for n in
                                  ("x", "y", "half_yaw_inertia"))
                energy = p.get_val("mobility_energy").item()
                tip = p.get_val("tip_stability_margin").item()
                p.set_val("deposition_actuator_length", .46)
                p.run_model()
                for n, value in before.items():
                    self.assertAlmostEqual(p.get_val(n).item(), value, places=11, msg=n)
                self.assertLess(p.get_val("deposition_x").item(), x)
                self.assertGreater(p.get_val("deposition_y").item(), z)
                self.assertLess(p.get_val("deposition_half_yaw_inertia").item(), inertia)
                self.assertLess(p.get_val("mobility_energy").item(), energy)
                self.assertGreater(p.get_val("tip_stability_margin").item(), tip)
                dry, payload, x = (p.get_val(n).item() for n in ("dry_mass", "payload_mass", "deposition_x"))
                expected_cg = payload/2*x/(dry+payload/2)
                self.assertAlmostEqual(p.get_val("deposition_half_cg_x").item(), expected_cg)
                static_reserve = p.get_val("wheelbase").item()/2 - abs(expected_cg)
                self.assertLess(p.get_val("tip_stability_margin").item(), static_reserve)

    def test_turn_torque_reduces_rocker_pitch_reserve(self):
        values = dict(dry_mass=30., payload_mass=20., gravity=10., wheelbase=.5,
                      track_width=.6, wheel_diameter=.4, wheel_width=.1,
                      wheel_deflection_fraction=.1, rocker_load_factor=1.2,
                      rolling_pressure_scale=100_000., rolling_base=.05,
                      scrub_coefficient=.12, acceleration=.6, turn_accel_fraction=.5,
                      deposition_half_cg_x=.06, deposition_half_yaw_inertia=3.)
        # The contact chord is 0.24 m. Weight is 400 N, so the pressure
        # contribution is 0.05 in addition to the base coefficient of 0.05.
        # Rolling, scrub and acceleration require 12, 6 and 3 N m about yaw.
        # Each wheel delivers (21 / 0.3) * 0.2 / 4 = 3.5 N m at the shaft.
        # Balance the light wheel's normal force about its partner on one side.
        side_weight = 200.
        normal = (side_weight*(.25-.06) - 2*3.5) / .5
        self.assertAlmostEqual(deposition_tip_margin(values.__getitem__), normal*.5/side_weight)
        values["deposition_half_cg_x"] *= -1
        self.assertAlmostEqual(deposition_tip_margin(values.__getitem__), normal*.5/side_weight)
        # With negligible yaw inertia and scrub, the 52 N straight demand governs.
        values.update(deposition_half_yaw_inertia=.1, scrub_coefficient=0.)
        self.assertAlmostEqual(deposition_tip_margin(values.__getitem__), .25-.06-52*.2/400)
        values.update(rocker_load_factor=0., rolling_base=0., scrub_coefficient=0., acceleration=0.)
        self.assertAlmostEqual(deposition_tip_margin(values.__getitem__), .25-.06)
        values["rolling_base"] = 1.
        self.assertLess(deposition_tip_margin(values.__getitem__), 0.)

    def test_calculated_route_timing_does_not_use_deposition_pose(self):
        p = self.problem({"travel_actuator_length": .52, "deposition_actuator_length": .49},
                         traversal_mode="calculated")
        names = ("outbound_travel_time", "return_travel_time", "route_translation_time", "route_turn_time")
        before = {n: p.get_val(n).item() for n in names}
        p.set_val("deposition_actuator_length", .46)
        p.run_model()
        for n, value in before.items():
            self.assertEqual(p.get_val(n).item(), value)

    def test_reverse_and_forward_time_and_energy_enter_mission_once(self):
        # Both 0.5 m legs are triangular at a=0.25 m/s² and a 1 m/s cap.
        p = self.problem({"travel_speed": 2., "turn_speed_fraction": .5,
                          "acceleration": .5, "turn_accel_fraction": .5}, traversal_mode="fixed")
        g = lambda name: p.get_val(name).item()
        names = ("turn_time", "mobility_energy", "cycle_time", "cycle_energy", "travel_time", "route_distance")
        with_legs = {name: g(name) for name in names}
        self.assertEqual(p.model_metadata["deposition_operation"]["straight_leg_distance_m"], .5)
        p.model.mobility.options["deposition_translation"] = False
        p.run_model()
        extra_time = 4 * math.sqrt(.5 / .25)
        peak_sq = .5 * .25
        mass = g("dry_mass") + g("payload_mass") / 2
        weight = mass * g("gravity")
        diameter = g("wheel_diameter")
        deflection = diameter * g("wheel_deflection_fraction")
        area = 8 * g("wheel_width") * math.sqrt(deflection * (diameter - deflection))
        rolling = weight * (g("rolling_base") + g("rocker_load_factor") * weight / (area * g("rolling_pressure_scale")))
        extra_drive_j = (rolling * 1. + mass * peak_sq) / g("drive_efficiency")
        extra_drive_j += 4 * g("actuator_idle_power") * extra_time
        for name in ("travel_time", "route_distance"):
            self.assertEqual(with_legs[name], g(name))
        self.assertAlmostEqual(with_legs["turn_time"] - g("turn_time"), extra_time)
        self.assertAlmostEqual(with_legs["cycle_time"] - g("cycle_time"), extra_time)
        self.assertAlmostEqual(with_legs["mobility_energy"] - g("mobility_energy"), extra_drive_j)
        self.assertAlmostEqual((with_legs["cycle_energy"] - g("cycle_energy")) * 3600,
                               extra_drive_j + g("electronics_power") * extra_time)

    def test_height_alone_does_not_change_yaw_inertia(self):
        low = [(3., .4, .1, .2, .01), (2., -.2, -.1, .3, .02)]
        high = [(m,x,y,z+1.,i) for m,x,y,z,i in low]
        self.assertEqual(mass_properties(low)[3], mass_properties(high)[3])
        self.assertAlmostEqual(mass_properties(high)[2]-mass_properties(low)[2], 1.)

    def test_independent_work_and_one_time_mission_accounting(self):
        # Synthetic limiting case: massless arms, ideal lossless actuators, no idle.
        v = dict(travel_actuator_length=.5, deposition_actuator_length=.4,
            travel_y=.3, deposition_y=.6, drum_mass=12., payload_mass=40.,
            arm_linear_mass=0., arm_length=.6, lin_act_side_arm_offset=0.,
            gravity=10., actuator_speed=.02, actuator_efficiency=1.,
            actuator_idle_power=0., required_actuator_force=500.)
        result = deposition_motion(v)
        # All dry drums move in each direction; payload moves in the loaded leg.
        self.assertAlmostEqual(result["deposition_arm_energy"], (2*12+40)*10*.3)
        self.assertAlmostEqual(result["deposition_actuation_time"], 2*.1/.02)
        self.assertAlmostEqual(result["deposition_arm_peak_power"], 2*500*.02)
        p = self.problem({"travel_actuator_length": .52, "deposition_actuator_length": .46})
        g = lambda n: p.get_val(n).item()
        duration = sum(g(n) for n in ("dig_time", "dump_time", "travel_time", "turn_time",
            "actuation_time", "deposition_actuation_time", "relocation_time", "overhead_time"))-g("dig_overlap_time")
        joules = sum(g(n) for n in ("dig_energy", "dump_energy", "mobility_energy", "arm_energy",
            "deposition_arm_energy", "relocation_energy"))+g("electronics_power")*duration
        self.assertAlmostEqual(g("cycle_time"), duration)
        self.assertAlmostEqual(g("cycle_energy")*3600, joules)

    def test_pa04_electrical_accounting_has_no_second_loss_factor(self):
        v = dict(travel_actuator_length=.52, deposition_actuator_length=.46,
                 required_actuator_force=RATED_FORCE/2)
        result = deposition_motion(v, "pa04-900")
        raising = .06/(.43*INCH)
        lowering = .06/(.39*INCH)
        self.assertAlmostEqual(result["deposition_arm_energy"], 2*12*(9*raising+14*lowering))
        self.assertAlmostEqual(result["deposition_actuation_time"], raising+lowering)
        self.assertEqual(result["deposition_arm_peak_power"], 2*12*14)
        p = self.problem({"travel_actuator_length": .52, "deposition_actuator_length": .46},
                         actuators="pa04-900")
        before = p.get_val("deposition_arm_energy").item()
        p.set_val("actuator_efficiency", .3)
        p.set_val("actuator_idle_power", 100)
        p.run_model()
        self.assertEqual(p.get_val("deposition_arm_energy").item(), before)

    def test_invalid_deposition_is_finite_and_rejected(self):
        # Reachable command bounds do not establish triangle/pose feasibility.
        p = self.problem({"arm_length": .30, "deposition_actuator_length": .606298,
                          "actuator_extended_length": .60})
        state = snapshot(p)
        for n in ("deposition_reach_margin", "deposition_position_margin"):
            self.assertFalse(state["constraints"][n]["passed"])
        self.assertFalse(state["feasible"])
        self.assertTrue(all(math.isfinite(i["value"]) for i in state["outputs"].values()))
        p = self.problem({"deposition_actuator_length": .606298})
        self.assertFalse(snapshot(p)["constraints"]["deposition_clearance_margin"]["passed"])

    def test_connections_dependency_separation_and_smooth_derivatives(self):
        for structure, actuators in (("reference", "reference"), ("plate-frame", "pa04-900")):
            with self.subTest(structure=structure):
                p = self.problem({"travel_actuator_length": .52, "deposition_actuator_length": .46},
                                 structure=structure, actuators=actuators)
                conns = p.model._conn_global_abs_in2out
                self.assertEqual(conns["mobility.deposition_half_yaw_inertia"],
                                 "mass_properties.deposition_half_yaw_inertia")
                self.assertEqual(conns["tip_stability.deposition_half_yaw_inertia"],
                                 "mass_properties.deposition_half_yaw_inertia")
                self.assertEqual(conns["tip_stability.payload_mass"], "excavation.payload_mass")
                self.assertEqual(conns["mission.deposition_arm_energy"], "deposition_actuation.deposition_arm_energy")
                self.assertFalse(any(n.startswith("_auto_ivc") for n in conns.values()))
                declared = set(_get_viewer_data(p)["declare_partials_list"])
                self.assertIn("mass_properties.deposition_loaded_cg_z > mass_properties.deposition_y", declared)
                self.assertNotIn("mass_properties.loaded_cg_z > mass_properties.deposition_y", declared)
                self.assertIn("linkage.actuator_leverage_max > linkage.deposition_actuator_length", declared)
                self.assertIn("tip_stability.tip_stability_margin > tip_stability.acceleration", declared)
                checks = p.check_totals(of=["deposition_x", "deposition_loaded_cg_z", "deposition_half_yaw_inertia",
                    "deposition_arm_energy", "deposition_actuation_time", "tip_stability_margin"],
                    wrt=["deposition_actuator_length", "travel_actuator_length"],
                    method="fd", form="central", step=3e-5, step_calc="rel_avg", out_stream=None)
                assert_check_totals(checks, atol=1e-3, rtol=2e-3)

    def test_report_has_separate_views_and_motion_costs(self):
        p = self.problem({"travel_actuator_length": .52, "deposition_actuator_length": .46})
        state = snapshot(p)
        values = values_from_snapshot(state)
        self.assertNotEqual(geometry_svg(values, travel=True), geometry_svg(values, deposition=True))
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"states.json"
            write_report(dict(baseline=state, final=state, status="baseline",
                              model_metadata=p.model_metadata), output)
            html = output.with_suffix(".html").read_text(encoding="utf-8")
            self.assertIn('id="pose-deposition"', html)
            self.assertIn("Deposition turn · one pair full", html)
            self.assertIn("reverse 0.5 m, point turn, drive forward 0.5 m", html)
            self.assertIn("Added pose-change duration", html)
            self.assertTrue(output.with_name("states_deposition.svg").exists())

    def test_explicit_legacy_and_custom_factories_are_preserved(self):
        p = self.problem(models=ModelSet(excavation=Excavation))
        self.assertIsInstance(p.model.tip_stability, TipStability)
        class StudentTip(DepositionTipStability):
            pass
        p = self.problem(models=replace(ModelSet(), tip_stability=StudentTip))
        self.assertIsInstance(p.model.tip_stability, StudentTip)
