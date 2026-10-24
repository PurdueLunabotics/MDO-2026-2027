"""Independent box-arm section, beam-limit, mass and invalid-domain checks."""
import math
import unittest

from rover.alternatives.aluminum_box_arm import (
    box_arm_section, BoxArmPlateStructureGeometry, BoxArmPlateStructureLoads,
)
from rover.alternatives.plate_frame import arm_beam_screen, evaluate_frame_loads
from rover.config import configuration
from rover.models import plate_frame_models
from rover.problem import build_problem
from reporting.report import snapshot


class BoxArmTests(unittest.TestCase):
    def test_two_inch_section_and_independent_mass_inventory(self):
        # Prescribe the physical test case; future edits to a saved baseline
        # must not change this section-property or conservation check.
        d,a = configuration('plate-frame')
        v = {n:s[0] for n,s in {**d,**a}.items()}
        v.update(arm_box_width=2*.0254, arm_box_height=2*.0254, arm_wall=.0254/16)
        s = box_arm_section(v)
        outer, inner = .0508, .047625
        area = outer**2-inner**2
        inertia = (outer**4-inner**4)/12
        self.assertAlmostEqual(s['area'], area, places=14)
        self.assertAlmostEqual(s['bending_inertia'], inertia, places=16)
        self.assertAlmostEqual(s['buckling_inertia'], inertia, places=16)
        p = build_problem(v, structure='plate-frame')
        self.addCleanup(p.cleanup)
        p.run_model()
        self.assertAlmostEqual(p.get_val('arm_linear_mass').item(), area*v['tube_density'], places=12)
        # Change only arm wall: two arm masses, counted once by the linkage.
        before = {n:p.get_val(n).item() for n in ('dry_mass','linkage_mass','structure_mass')}
        p.set_val('arm_wall', .002)
        p.run_model()
        delta = 2*v['arm_length']*v['tube_density']*((outer**2-(outer-.004)**2)-area)
        self.assertAlmostEqual(p.get_val('dry_mass').item()-before['dry_mass'], delta, places=10)
        self.assertAlmostEqual(p.get_val('linkage_mass').item()-before['linkage_mass'], delta, places=10)
        self.assertEqual(p.get_val('structure_mass').item(), before['structure_mass'])

    def test_cantilever_point_load_matches_independent_beam_limit(self):
        values = dict(arm_box_width=.03, arm_box_height=.06, arm_wall=.002)
        s = box_arm_section(values)
        length, force, modulus = .7, 100., 69e9
        stress, tip, moment, compression = arm_beam_screen(
            length, 1., (length, 0.), (0., 0.), force, 0.,
            s['area'], s['bending_inertia'], s['depth'], modulus)
        self.assertAlmostEqual(moment, force*length)
        self.assertAlmostEqual(stress, force*length*.03/s['bending_inertia'])
        self.assertAlmostEqual(tip, force*length**3/(3*modulus*s['bending_inertia']))
        self.assertEqual(compression, 0.)

    def test_orientation_changes_flexure_but_euler_uses_weaker_axis(self):
        d, a = configuration('plate-frame')
        v = {n:s[0] for n,s in {**d,**a}.items()}
        v.update(dry_mass=50., payload_mass=20., drum_mass=8., wheel_mass=4., drive_mass=6.,
                 arm_box_width=.03, arm_box_height=.06)
        rotated = {**v, 'arm_box_width': .06, 'arm_box_height': .03}
        first, second = [box_arm_section(x) for x in (v, rotated)]
        self.assertEqual(first['area'], second['area'])
        self.assertEqual(first['buckling_inertia'], second['buckling_inertia'])
        self.assertLess(first['buckling_inertia'], first['bending_inertia'])
        r, rr = [evaluate_frame_loads(x, arm_section=box_arm_section) for x in (v, rotated)]
        self.assertAlmostEqual(r['arm_buckling_margin'], rr['arm_buckling_margin'])
        self.assertAlmostEqual(rr['arm_deflection']/r['arm_deflection'],
                               first['bending_inertia']/second['bending_inertia'])

    def test_invalid_sections_fail_and_legacy_case_needs_explicit_migration(self):
        for w,h,t in ((0,.05,.001), (.05,math.nan,.001), (.05,.05,0),
                       (.05,.05,.025), (.03,.06,.02), (.05,.05,math.inf)):
            with self.subTest(w=w,h=h,t=t), self.assertRaises(ValueError):
                box_arm_section(dict(arm_box_width=w,arm_box_height=h,arm_wall=t))
        with self.assertRaisesRegex(ValueError, 'arm_box_width and arm_box_height'):
            build_problem({'arm_tube_diameter': .0508}, structure='plate-frame')

    def test_slender_box_arm_has_a_negative_buckling_margin(self):
        d,a = configuration('plate-frame')
        v = {n:s[0] for n,s in {**d,**a}.items()}
        # An intentionally undersized physical limit, outside optimizer bounds.
        v.update(dry_mass=50., payload_mass=20., drum_mass=8., wheel_mass=4., drive_mass=6.,
                 arm_box_width=.005, arm_box_height=.005, arm_wall=.0001)
        result = evaluate_frame_loads(v, arm_section=box_arm_section)
        self.assertLess(result['arm_buckling_margin'], 0.)
        self.assertLess(result['structure_buckling_margin'], 0.)

    def test_model_selection_design_variables_and_connections(self):
        m = plate_frame_models()
        self.assertIs(m.structure_geometry, BoxArmPlateStructureGeometry)
        self.assertIs(m.structure_loads, BoxArmPlateStructureLoads)
        p = build_problem(structure='plate-frame')
        self.addCleanup(p.cleanup)
        p.run_model()
        for n in ('arm_box_width','arm_box_height','arm_wall'):
            self.assertIn(n, p.design_specs)
            self.assertEqual(p.model._conn_global_abs_in2out['structure_geometry.'+n], 'inputs.'+n)
            self.assertEqual(p.model._conn_global_abs_in2out['structure_loads.'+n], 'inputs.'+n)
        self.assertNotIn('arm_tube_diameter', p.design_specs)
        self.assertNotIn('arm_tube_diameter', p.model.structure_geometry.CONFIG_INPUTS)
        self.assertIn('arm_box_width', snapshot(p)['design'])
        d,_ = configuration()
        self.assertIn('arm_tube_diameter', d)
        self.assertNotIn('arm_box_width', d)


if __name__ == '__main__':
    unittest.main()
