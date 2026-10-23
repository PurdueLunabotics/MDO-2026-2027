"""Independent geometric limits, common-rotation invariance and model contracts."""
import math
import unittest
import xml.etree.ElementTree as ET
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_totals
from rover.alternatives.parallel_cylinder_clearance import (
    parallel_cylinder_gap, unroll_point, cylinder_clearance_cases,
    evaluate_parallel_cylinder_clearance, ParallelCylinderClearance, chassis_point_in_world)
from rover.alternatives.obstacle_stability import raised_wheel_pose
from rover.models import ModelSet
from rover.problem import build_problem
from reporting.report import snapshot
from reporting.html_report import values_from_snapshot
from reporting.obstacle_drawing import obstacle_pose_svg


def example_values():
    """Synthetic geometry from the student's worked example; not hardware data."""
    return {'wheelbase':.46,'track_width':.54,'wheel_diameter':.30,
            'wheel_width':.10,'rocker_pivot_offset':.10,'obstacle_height':.40,
            'drum_diameter':.28,'drum_length':.25,'drum_pair_gap':.06,
            'mechanism_clearance':.015,'travel_x':.4810305108377564,
            'travel_y':.6365979659441497}


class ParallelCylinderTests(unittest.TestCase):
    def test_independent_radial_axial_corner_and_intersection_limits(self):
        # Radius sum .3, axial half-length sum .5. Hand-computable distances.
        for delta, expected in (((.6,0,0),.3), ((0,.9,0),.4),
                                ((.6,.9,0),.5), ((.3,0,0),0),
                                ((0,.5,0),0), ((.1,.1,0),-.2), ((0,0,0),-.3)):
            with self.subTest(delta=delta):
                self.assertAlmostEqual(parallel_cylinder_gap(delta,.1,.4,.2,.6)['gap'],expected)

    def test_long_cylinders_and_axially_separated_projected_overlap(self):
        gap = parallel_cylinder_gap((1.2,0,0),.1,2,.1,2)['gap']
        self.assertAlmostEqual(gap,1)
        # Coincident projected circles can also be separated along their axes.
        self.assertAlmostEqual(parallel_cylinder_gap((0,.8,0),.1,.2,.1,.2)['gap'],.6)

    def test_common_roll_translation_and_pitch_cannot_change_gap(self):
        geometry = cylinder_clearance_cases(example_values())
        for case in geometry['cases']:
            for angle in (-.8,.3,1.2):
                c,s=math.cos(angle),math.sin(angle)
                def move(p):
                    x,y,z=(a-b for a,b in zip(p,case['pose']['shaft_origin']))
                    return x+7,c*y-s*z-3,s*y+c*z+2
                posed={**case['pose'],'roll':case['pose']['roll']+angle,'shaft_origin':(7,-3,2)}
                drums=[unroll_point(move(p),posed) for p in case['drums']]
                wheels=[unroll_point(move(p),posed) for p in case['wheels']]
                for actual,expected in zip(drums+wheels,case['unrolled_drums']+case['unrolled_wheels']):
                    for a,b in zip(actual,expected):
                        self.assertAlmostEqual(a,b,places=12)
        delta=(.4,.2,.3)
        a=.51
        pitched=(math.cos(a)*delta[0]-math.sin(a)*delta[2],delta[1],
                 math.sin(a)*delta[0]+math.cos(a)*delta[2])
        self.assertAlmostEqual(parallel_cylinder_gap(delta,.1,.2,.2,.3)['gap'],
                               parallel_cylinder_gap(pitched,.1,.2,.2,.3)['gap'])

    def test_side_planes_and_all_pairs_are_retained(self):
        v=example_values()
        geometry=cylinder_clearance_cases(v)
        self.assertEqual(len(geometry['cases']),4)
        for case in geometry['cases']:
            self.assertEqual(len(case['pairs']),16)
            for i,p in enumerate(case['unrolled_wheels']):
                self.assertAlmostEqual(p[1],(1 if i in (0,2) else -1)*v['track_width']/2)
            self.assertAlmostEqual(case['worst']['gap'],geometry['cases'][0]['worst']['gap'])
        exact=evaluate_parallel_cylinder_clearance(v)
        self.assertAlmostEqual(exact['obstacle_drum_wheel_margin'],exact['obstacle_drum_wheel_gap_bound']-.015)
        self.assertGreater(evaluate_parallel_cylinder_clearance({**v,'travel_y':v['travel_y']+.1})['obstacle_drum_wheel_gap_bound'],
                           exact['obstacle_drum_wheel_gap_bound'])

    def test_invalid_domains_and_guarded_poses_remain_failed(self):
        for delta,args in (((0,0),(.1,.2,.1,.2)),((0,0,math.nan),(.1,.2,.1,.2)),
                           ((0,0,0),(-.1,.2,.1,.2)),((0,0,0),(.1,0,.1,.2))):
            with self.assertRaises(ValueError):
                parallel_cylinder_gap(delta,*args)
        for changes in ({'drum_length':0},{'wheel_width':-1},{'mechanism_clearance':-1},
                        {'drum_pair_gap':-1},{'obstacle_height':-1},{'travel_x':math.nan}):
            with self.assertRaises(ValueError):
                evaluate_parallel_cylinder_clearance({**example_values(),**changes})
        boundary=raised_wheel_pose(.46,.54,.15,.1,0)['reach_margin']
        for changes in ({'wheelbase':.4},{'obstacle_height':boundary}):
            r=evaluate_parallel_cylinder_clearance({**example_values(),**changes})
            self.assertEqual(r['obstacle_clearance_pose_valid'],0)
            self.assertLess(r['obstacle_drum_wheel_margin'],-1e-6)

    def test_adapter_smooth_derivatives(self):
        p=om.Problem(reports=False)
        p.model.add_subsystem('clearance',ParallelCylinderClearance(),promotes=['*'])
        p.setup()
        self.addCleanup(p.cleanup)
        values={**example_values(),'obstacle_height':.2}
        for n,v in values.items():
            p.set_val(n,v)
        p.run_model()
        checks=p.check_totals(of=['obstacle_drum_wheel_gap_bound','obstacle_drum_wheel_margin'],
                              wrt=list(values),method='fd',form='central',step=3e-5,out_stream=None)
        assert_check_totals(checks,atol=1e-5,rtol=2e-3)

    def test_cylinders_are_default_for_every_frame_and_motor_profile(self):
        self.assertIs(ModelSet().obstacle_clearance,ParallelCylinderClearance)
        for structure,motors in (('reference','reference'),('plate-frame','reference'),
                                 ('reference','x44-standard'),('plate-frame','x44-standard')):
            p=build_problem({'wheelbase':.46},structure=structure,motors=motors)
            self.addCleanup(p.cleanup)
            p.run_model()
            self.assertIsInstance(p.model.obstacle_clearance,ParallelCylinderClearance)
            self.assertTrue(p.model_metadata['models']['obstacle_clearance'].endswith('.ParallelCylinderClearance'))
            self.assertEqual(p.model.get_source('obstacle_clearance.travel_x'),'linkage.travel_x')
            self.assertEqual(p.model.get_source('obstacle_clearance.travel_y'),'linkage.travel_y')
            self.assertIn('obstacle_drum_wheel_margin',p.model.get_constraints())
            v=values_from_snapshot(snapshot(p))
            for name,expected in evaluate_parallel_cylinder_clearance(v).items():
                self.assertAlmostEqual(p.get_val(name).item(),expected)

    def test_mounted_point_level_and_pitch_roll_limits(self):
        point=(.48,.155,.637)
        p={'pitch':0.,'roll':0.,'shaft_origin':(0.,0.,.25)}
        self.assertEqual(chassis_point_in_world(point,p,.25),point)
        p={'pitch':math.pi/2,'roll':math.pi/2,'shaft_origin':(4.,5.,6.)}
        for actual,expected in zip(chassis_point_in_world((1.,2.,3.25),p,.25),(1.,4.,8.)):
            self.assertAlmostEqual(actual,expected)
        for bad in ((0,0),(0,0,math.nan)):
            with self.assertRaises(ValueError):
                chassis_point_in_world(bad,p,.25)

    def test_mounted_transform_preserves_distances(self):
        points=[(1.2,-.3,.7),(-.4,.6,1.1),(.1,.2,.3)]
        pose={'pitch':.27,'roll':-.36,'shaft_origin':(3,-2,.8)}
        transformed=[chassis_point_in_world(p,pose,.25) for p in points]
        for i,j in ((0,1),(0,2),(1,2)):
            self.assertAlmostEqual(math.dist(points[i],points[j]),math.dist(transformed[i],transformed[j]),places=12)

    def test_rock_figure_has_actual_pose_and_flagged_unreachable_example(self):
        p=build_problem({'wheelbase':.46})
        self.addCleanup(p.cleanup)
        p.run_model()
        values=values_from_snapshot(snapshot(p))
        svg=obstacle_pose_svg(values)
        root=ET.fromstring(svg)
        self.assertEqual(len([n for n in root.iter() if n.get('data-contact') is not None]),4)
        self.assertEqual(len([n for n in root.iter() if n.get('data-part')=='rock']),3)
        self.assertIn('Common roll removed',svg)
        self.assertIn('400.0 mm',svg)
        self.assertIn('UNREACHABLE',obstacle_pose_svg({**values,'wheelbase':.4}))
