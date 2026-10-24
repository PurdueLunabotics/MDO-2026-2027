"""Independent equilibrium/energy/section checks for the optional frame profile."""
import math
import tempfile
from pathlib import Path
import unittest
import numpy as np
from openmdao.utils.assert_utils import assert_check_totals

from rover.config import configuration, DESIGN, ASSUMPTIONS
from rover.problem import build_problem
from reporting.report import snapshot
from reporting.html_report import write_report
from rover.alternatives.plate_frame import (
    PLATE_EDGES, arm_mount_loads, box_section, plate_nodes, plate_geometry,
    geometry_inventory, solve_plate, evaluate_frame_loads, plate_frame_posed_cg,
)
from rover.alternatives.obstacle_stability import raised_wheel_pose


def inputs(**changes):
    design, assumptions = configuration("plate-frame")
    values = {n: s[0] for n, s in {**design, **assumptions}.items()}
    values.update(dry_mass=50., payload_mass=20., drum_mass=8., wheel_mass=4., drive_mass=6.)
    values.update(changes)
    return values


class PlateFrameTests(unittest.TestCase):
    def problem(self, **changes):
        p = build_problem(changes, structure="plate-frame")
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def test_box_section_independent_rectangle_subtraction(self):
        area, ix, iz = box_section(.04, .06, .003)
        self.assertAlmostEqual(area, .04*.06-.034*.054)
        self.assertAlmostEqual(ix, (.04*.06**3-.034*.054**3)/12)
        self.assertAlmostEqual(iz, (.06*.04**3-.054*.034**3)/12)
        for args in ((.04, .06, .02), (0., .06, .001), (.04, math.nan, .002)):
            with self.assertRaises(ValueError):
                box_section(*args)

    def test_arm_force_and_moment_balance_known_pose(self):
        # Horizontal 1 m arm, vertical actuator at its tip: exact F_A=100 N.
        upper, lower, actuator = arm_mount_loads((0, 0), (1, 1), (1, 0), 100., 0., 1.)
        np.testing.assert_allclose(upper, (0, -100))
        np.testing.assert_allclose(lower, (0, 0))
        np.testing.assert_allclose(actuator, (0, 100))
        # General pose includes arm weight at half length; force/moment conservation.
        b, a, c = np.array((.1, .2)), np.array((.15, .6)), np.array((.65, .45))
        for fraction in (.6, 1.):
            upper, lower, actuator = arm_mount_loads(b, a, c, 120., 16., fraction)
            np.testing.assert_allclose(upper+lower, (0, -136), atol=1e-12)
            r = c-b
            d = fraction*r
            self.assertAlmostEqual(d[0]*actuator[1]-d[1]*actuator[0]-r[0]*(120+8), 0, places=10)
            line = a-b-fraction*r
            self.assertAlmostEqual(line[0]*actuator[1]-line[1]*actuator[0], 0, places=10)

    def test_toggle_and_bad_load_rejected(self):
        with self.assertRaises(ValueError):
            arm_mount_loads((0, 0), (2, 0), (1, 0), 100., 0., 1.)
        with self.assertRaises(ValueError):
            arm_mount_loads((0, 0), (1, 1), (1, 0), -1., 0., 1.)

    def test_truss_equilibrium_and_elastic_work(self):
        nodes = plate_nodes(inputs())
        forces = np.array(((0., 0.), (30., -100.), (-20., -80.), (-30., -100.), (20., -80.)))
        width, thickness, modulus = .03, .00635, 69e9
        axial, u, reaction = solve_plate(nodes, forces, width, thickness, modulus)
        np.testing.assert_allclose(reaction+forces.sum(axis=0), (0, 0), atol=1e-10)
        residual = forces.copy()
        residual[0] += reaction
        energy = 0.
        for n, (i, j) in zip(axial, PLATE_EDGES):
            d = nodes[j]-nodes[i]
            length = np.linalg.norm(d)
            residual[i] += n*d/length
            residual[j] -= n*d/length
            energy += n*n*length/(2*modulus*width*thickness)
        np.testing.assert_allclose(residual, 0., atol=1e-10)
        self.assertAlmostEqual(energy, float(np.sum(forces*u))/2, places=12)
        # Axial truss invariance under doubling a common area: same forces, half u.
        n2, u2, r2 = solve_plate(nodes, forces, 2*width, thickness, modulus)
        np.testing.assert_allclose(n2, axial, atol=1e-10)
        np.testing.assert_allclose(u2, u/2, atol=1e-12)
        np.testing.assert_allclose(r2, reaction, atol=1e-10)

    def test_pin_rejects_unbalanced_moment_and_extra_mechanism(self):
        nodes = plate_nodes(inputs())
        forces = np.zeros((5, 2))
        forces[1, 1] = -100.
        with self.assertRaisesRegex(ValueError, "unbalanced"):
            solve_plate(nodes, forces, .03, .00635, 69e9)
        flat = np.column_stack((np.arange(5), np.zeros(5)))
        with self.assertRaisesRegex(ValueError, "mechanism"):
            solve_plate(flat, np.zeros((5, 2)), .03, .00635, 69e9)

    def test_level_lower_boundary_matches_hand_solved_member_forces(self):
        # Two mirrored triangles, with the hub on the lower edge. Nodal
        # equilibrium alone gives these seven member forces (tension positive).
        a, b, h, weight = .2, .4, .3, 100.
        nodes = np.array(((0., 0.), (a, h), (b, 0.), (-a, h), (-b, 0.)))
        forces = np.zeros((5, 2))
        forces[2, 1] = forces[4, 1] = -weight
        axial, displacement, reaction = solve_plate(nodes, forces, .03, .00635, 69e9)
        upper = -weight*math.hypot(a, h)/h
        lower = weight*(a-b)/h
        outer = weight*math.hypot(a-b, h)/h
        np.testing.assert_allclose(axial, (upper, lower, upper, lower, outer, outer, weight*b/h), atol=1e-10)
        np.testing.assert_allclose(reaction, (0., 2*weight), atol=1e-10)
        self.assertTrue(np.isfinite(displacement).all())
        self.assertNotIn((2, 4), PLATE_EDGES)

    def test_hub_can_cross_lower_mount_height_without_topology_switch(self):
        # Physical pad separation is longitudinal here; no vertical spacing
        # requirement between S and B. No model fallback or spring is needed.
        for height in (.16, .18, .20, .25):
            values = inputs(rocker_pivot_offset=.03, bottom_pivot_height=height)
            g = plate_geometry(values)
            self.assertTrue(g["valid"], (height, g["geometry_margin"]))
            loads = evaluate_frame_loads(values)
            self.assertEqual(loads["frame_load_solution_valid"], 1.)
            self.assertLess(loads["frame_force_balance_error"], 1e-8)

    def test_geometry_tracks_mounts_axle_and_width(self):
        first = geometry_inventory(inputs())
        shifted = geometry_inventory(inputs(actuator_pivot_x=.17, bottom_pivot_height=.17, rocker_pivot_offset=.12))
        np.testing.assert_allclose(shifted["nodes"][0], (0, .27))
        np.testing.assert_allclose(shifted["nodes"][1], (.17, .45))
        np.testing.assert_allclose(shifted["nodes"][2], (.13, .17))
        self.assertFalse(np.allclose(first["lengths"], shifted["lengths"]))
        wider = geometry_inventory(inputs(plate_spoke_width=.05))
        self.assertGreater(wider["plate_mass_total"], first["plate_mass_total"])
        self.assertLess(wider["plate_geometry_margin"], first["plate_geometry_margin"])
        self.assertGreater(first["structure_mass"], first["rocker_mass"])
        self.assertAlmostEqual(first["cf_tube_outer_diameter"], .0254+2*.002)
        self.assertAlmostEqual(first["suspension_bearing_span"], .54-.10-2*.0127-.035)

    def test_invalid_shape_and_wheel_packing_remain_failed(self):
        bad = plate_geometry(inputs(cf_tube_span=.46))
        self.assertFalse(bad["valid"])
        self.assertLess(bad["packing_margin"], 0)
        # Above the upper mounts the cutout flips; geometry must stay failed.
        bad = plate_geometry(inputs(bottom_pivot_height=.50))
        self.assertFalse(bad["valid"])
        self.assertLess(bad["geometry_margin"], 0)
        p = self.problem(cf_tube_span=.46)
        report = snapshot(p)
        self.assertFalse(report["constraints"]["frame_packing_margin"]["passed"])
        self.assertFalse(report["feasible"])
        with self.assertRaises(ValueError):
            plate_geometry(inputs(plate_thickness=math.nan))
        for name in ('arm_tube_diameter', 'arm_wall'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                geometry_inventory(inputs(**{name: math.nan}))
        with self.assertRaises(ValueError):
            evaluate_frame_loads(inputs(dry_mass=1.))

    def test_spoke_width_strength_stiffness_and_buckling_trends(self):
        small = evaluate_frame_loads(inputs(plate_spoke_width=.02))
        large = evaluate_frame_loads(inputs(plate_spoke_width=.04))
        # Wider strips also add self-weight, so the full gravity stress does not
        # halve exactly. The isolated fixed-load area scaling is checked above.
        self.assertLess(large["plate_stress"], small["plate_stress"])
        self.assertGreater(large["plate_buckling_margin"], small["plate_buckling_margin"])
        self.assertLess(large["frame_force_balance_error"], 1e-8)
        failed = evaluate_frame_loads(inputs(plate_thickness=.0005))
        self.assertLess(failed["plate_buckling_margin"], 0)

    def test_cf_span_tradeoff_and_nominal_wheel_load(self):
        short = evaluate_frame_loads(inputs(cf_tube_span=.20))
        long = evaluate_frame_loads(inputs(cf_tube_span=.30))
        self.assertGreater(long["cf_bending_stress"], short["cf_bending_stress"])
        self.assertLess(long["axle_deflection"], short["axle_deflection"])
        self.assertAlmostEqual(short["frame_nominal_wheel_reaction"], (50+20)*9.80665/4)
        self.assertEqual(short["frame_load_solution_valid"], 1)

    def test_pose_mass_conservation_distinct_rocker_centroid(self):
        pose = raised_wheel_pose(.46, .54, .15, .10, 0.)
        cg = plate_frame_posed_cg(pose, .15, .1, 40., 0., .5, 8., 4.)
        self.assertAlmostEqual(cg[2], (40*.5+8*.15+4*.20)/52)
        # On an obstacle, rocker aggregate centroid lies halfway to shaft origin.
        pose = raised_wheel_pose(.46, .54, .15, .10, .2)
        cg = plate_frame_posed_cg(pose, .15, .1, 40., 0., .5, 8., 4.)
        self.assertTrue(all(math.isfinite(x) for x in cg))

    def test_profile_contract_mass_energy_and_reference_preservation(self):
        before_design, before_assumptions = dict(DESIGN), dict(ASSUMPTIONS)
        p = self.problem(wheelbase=.46)
        report = snapshot(p)
        self.assertIn("plate_spoke_width", report["design"])
        self.assertNotIn("plate_thickness", report["design"])
        self.assertNotIn("cf_tube_inner_diameter", report["design"])
        self.assertNotIn("frame_tube_diameter", report["design"])
        self.assertEqual(report["assumptions"]["plate_thickness"]["value"], .00635)
        self.assertEqual(report["assumptions"]["cf_tube_inner_diameter"]["value"], .0254)
        total = sum(p.get_val(n).item() for n in ("fixed_mass", "wheel_mass", "drum_mass", "drive_mass", "battery_mass", "structure_mass", "linkage_mass"))
        self.assertAlmostEqual(p.get_val("dry_mass").item(), total)
        self.assertAlmostEqual(p.get_val("loaded_sprung_mass").item()+p.get_val("wheel_mass").item()+p.get_val("drive_mass").item()+p.get_val("rocker_mass").item(), total+p.get_val("payload_mass").item())
        energy = sum(p.get_val(n).item() for n in ("dig_energy", "dump_energy", "relocation_energy", "mobility_energy", "arm_energy"))
        energy += p.get_val("electronics_power").item()*p.get_val("cycle_time").item()
        self.assertAlmostEqual(p.get_val("cycle_energy").item()*3600, energy)
        self.assertEqual(DESIGN, before_design)
        self.assertEqual(ASSUMPTIONS, before_assumptions)
        p.final_setup()
        conns = p.model._conn_global_abs_in2out
        self.assertEqual(conns["structure_loads.drum_mass"], "drum_mass.drum_mass")
        self.assertEqual(conns["mass_properties.structure_cg_z"], "structure_geometry.structure_cg_z")
        self.assertEqual(conns["structure_geometry.plate_spoke_width"], "inputs.plate_spoke_width")

    def test_removed_screens_are_diagnostics_not_feasibility_constraints(self):
        p = self.problem(drum_rpm=60., dump_rpm=60.)
        report = snapshot(p)
        for name in ("chassis_clearance_margin", "dig_froude_margin", "dump_froude_margin"):
            self.assertNotIn(name, p.model.get_constraints())
            self.assertNotIn(name, report["constraints"])
            self.assertIn(name, report["outputs"])
            self.assertIn(name, p.model_metadata["diagnostic_only_screens"])
        self.assertLess(p.get_val("dig_froude_margin").item(), 0.)
        self.assertLess(p.get_val("dump_froude_margin").item(), 0.)
        for name in ("wheel_clearance_margin", "obstacle_drum_wheel_margin",
                     "drum_torque_margin", "dump_torque_margin", "drum_power_margin"):
            self.assertIn(name, p.model.get_constraints())
        reference = build_problem()
        self.addCleanup(reference.cleanup)
        reference.run_model()
        self.assertIn("chassis_clearance_margin", reference.model.get_constraints())
        self.assertNotIn("dig_froude_margin", reference.model.get_constraints())
        self.assertNotIn("dump_froude_margin", reference.model.get_constraints())

    def test_total_derivatives_at_smooth_geometry(self):
        # Unequal box sides keep Euler's weak-axis selection at a smooth point.
        p = self.problem(wheelbase=.46, bottom_pivot_height=.16,
                         arm_box_width=.045, arm_box_height=.06)
        checks = p.check_totals(of=["construction_points", *p.margin_specs], wrt=list(p.design_specs),
                               method="fd", form="central", step=3e-5, step_calc="rel_avg", out_stream=None)
        assert_check_totals(checks, atol=1e-3, rtol=2e-3)

    def test_report_identifies_profile_materials_and_new_margins(self):
        p = self.problem()
        snap = snapshot(p)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"plate.json"
            write_report({"final": snap, "baseline": snap, "status": "baseline", "optimizer_success": None,
                          "openmdao_version": "test", "model_metadata": p.model_metadata}, output)
            html = output.with_suffix(".html").read_text(encoding="utf-8")
            self.assertIn("plate_spoke_width", html)
            self.assertIn("cf_bending_margin", html)
            self.assertIn("PROVISIONAL", html)
            self.assertIn("Removed screens: diagnostics only", html)
            self.assertTrue((Path(folder)/"plate_frame.svg").exists())

    def test_new_profile_failed_climbs_are_not_reported_feasible(self):
        p = self.problem()
        p.driver.options['maxiter'] = len(p.design_specs)+2  # Failed-run reporting, not convergence.
        before = p.get_val("construction_points").item()
        result = p.run_driver()
        p.run_model()
        self.assertFalse(result.success)
        self.assertFalse(snapshot(p)["feasible"])
        self.assertFalse(snapshot(p)["constraints"]["loaded_slope_traversal_margin"]["passed"])
        self.assertEqual(p.get_val("plate_thickness").item(), .00635)
        self.assertEqual(p.get_val("cf_tube_inner_diameter").item(), .0254)


if __name__ == "__main__":
    unittest.main()
