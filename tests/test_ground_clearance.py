"""Independent inch limits, branch sensitivities, domains and model integration."""
import math
import unittest

from openmdao.utils.assert_utils import assert_check_totals
from rover.ground_clearance import axle_ground_clearance
from rover.problem import build_problem
from reporting.report import snapshot


class GroundClearanceTests(unittest.TestCase):
    def test_six_inch_boundary_from_independent_inch_geometry(self):
        # Independent inches: bottom=8.5-0.5-1-1=6;
        # rocker=5+4.75-0.75-1-1=7, including each distinct shaft radius.
        radii={'bottom_axle_radius':.5*.0254,'rocker_axle_radius':.75*.0254}
        out=axle_ground_clearance(10*.0254,8.5*.0254,4.75*.0254,**radii)
        self.assertAlmostEqual(out['bottom_pivot_ground_clearance']/.0254,6.)
        self.assertAlmostEqual(out['rocker_pivot_ground_clearance']/.0254,7.)
        self.assertAlmostEqual(out['ground_clearance_margin'],0.,places=14)
        # Now the rocker controls: 5+2.75-0.75-1-1=5 inches.
        out=axle_ground_clearance(10*.0254,10.5*.0254,2.75*.0254,**radii)
        self.assertAlmostEqual(out['ground_clearance']/.0254,5.)
        self.assertAlmostEqual(out['ground_clearance_margin']/.0254,-1.)

    def test_zero_deductions_and_rigid_translation(self):
        for bottom,offset in ((.12,.1),(.3,-.05)):
            out=axle_ground_clearance(.3,bottom,offset,0.,0.)
            self.assertEqual(out['ground_clearance'],min(bottom,.15+offset))
            a=axle_ground_clearance(.3,bottom,offset)
            b=axle_ground_clearance(.3,bottom+.07,offset+.07)
            self.assertAlmostEqual(b['ground_clearance']-a['ground_clearance'],.07)

    def test_each_radius_is_applied_before_selecting_the_lowest_surface(self):
        # The higher axle center can have the lower underside.
        out=axle_ground_clearance(.3,.20,.06,0.,0.,
                                   bottom_axle_radius=.005,rocker_axle_radius=.03)
        self.assertAlmostEqual(out['bottom_pivot_ground_clearance'],.195)
        self.assertAlmostEqual(out['rocker_pivot_ground_clearance'],.18)
        self.assertAlmostEqual(out['ground_clearance'],.18)
        for name in ('bottom_axle_radius','rocker_axle_radius'):
            for value in (-.001,math.nan):
                with self.subTest(name=name,value=value),self.assertRaises(ValueError):
                    axle_ground_clearance(.3,.2,.1,**{name:value})

    def test_negative_clearance_stays_failed_and_invalid_domains_raise(self):
        out=axle_ground_clearance(.3,.01,.1)
        self.assertLess(out['ground_clearance'],0.)
        self.assertLess(out['ground_clearance_margin'],0.)
        for args in ((0.,.2,.1),(.3,math.nan,.1),(.3,.2,.1,1.),(.3,.2,.1,-.1),
                     (.3,.2,.1,.2,-.01),(.3,.2,.1,.2,.0254,0.)):
            with self.subTest(args=args),self.assertRaises(ValueError):axle_ground_clearance(*args)

    def test_active_constraint_and_report_in_both_profiles(self):
        for structure in ('reference','plate-frame'):
            with self.subTest(structure=structure):
                p=build_problem(structure=structure)
                self.addCleanup(p.cleanup);p.run_model()
                state=snapshot(p)
                self.assertIn('ground_clearance_margin',p.margin_specs)
                self.assertTrue(any(n.rsplit('.',1)[-1]=='ground_clearance_margin' for n in p.model.get_constraints()))
                self.assertFalse(state['constraints']['ground_clearance_margin']['passed'])
                self.assertEqual(state['outputs']['ground_clearance']['units'],'m')
                self.assertAlmostEqual(p.get_val('bottom_pivot_axle_radius').item(),
                    p.get_val('cf_tube_inner_diameter').item()/2+p.get_val('cf_tube_wall').item())
                self.assertAlmostEqual(p.get_val('rocker_pivot_axle_radius').item(),
                    p.get_val('axle_outer_diameter').item()/2)
                self.assertTrue(all(not s.startswith('_auto_ivc') for s in p.model._conn_global_abs_in2out.values()))
                self.assertEqual(p.model._conn_global_abs_in2out['ground_clearance.bottom_pivot_height'],'inputs.bottom_pivot_height')

    def test_smooth_branch_derivatives_and_explicit_dependencies(self):
        for bottom in (.15,.29):
            p=build_problem({'bottom_pivot_height':bottom,'rocker_pivot_offset':.08},structure='plate-frame')
            self.addCleanup(p.cleanup);p.run_model()
            # Finite differences stay away from the min-branch equality.
            checks=p.check_totals(of=['ground_clearance','ground_clearance_margin'],
                wrt=['wheel_diameter','bottom_pivot_height','rocker_pivot_offset',
                     'cf_tube_inner_diameter','cf_tube_wall','axle_outer_diameter'],
                method='fd',form='central',step=3e-5,step_calc='rel_avg',out_stream=None)
            assert_check_totals(checks,atol=1e-8,rtol=1e-6)
            c=p.model.ground_clearance
            self.assertIn('bottom_pivot_height',c.DEPENDENCIES['ground_clearance_margin'])
            self.assertIn('rocker_pivot_offset',c.DEPENDENCIES['ground_clearance_margin'])
            self.assertIn('cf_tube_wall',c.DEPENDENCIES['ground_clearance_margin'])
            self.assertIn('axle_outer_diameter',c.DEPENDENCIES['ground_clearance_margin'])
            h=1e-5;args=[.3,bottom,.08]
            partials=[]
            for i in range(3):
                hi=args.copy();lo=args.copy();hi[i]+=h;lo[i]-=h
                partials.append((axle_ground_clearance(*hi)['ground_clearance']-axle_ground_clearance(*lo)['ground_clearance'])/(2*h))
            expected=(-.1,1.,0.) if bottom<.23 else (.4,0.,1.)
            for actual,want in zip(partials,expected):self.assertAlmostEqual(actual,want,places=8)


if __name__=='__main__':unittest.main()
