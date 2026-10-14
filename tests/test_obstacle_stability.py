"""Independent gravity balance and invalid support-domain checks."""
import math
import unittest

from rover.alternatives.obstacle_stability import projected_support_margin, raised_wheel_pose, posed_loaded_cg
from rover.problem import build_problem
from rover.report import snapshot
from rover.config import DESIGN
from openmdao.utils.assert_utils import assert_check_totals


class ObstacleSupportTests(unittest.TestCase):
    def test_level_rectangle_limit(self):
        contacts = [(-.2, -.3, 0), (.2, -.3, 0), (.2, .3, 0), (-.2, .3, 0)]
        # Reproduces the independent level-ground rectangle expression.
        self.assertAlmostEqual(projected_support_margin(contacts, (.07, .02, .8)), .13)
        self.assertAlmostEqual(projected_support_margin(contacts, (.2, 0, .8)), 0)
        self.assertAlmostEqual(projected_support_margin(contacts, (.25, 0, .8)), -.05)

    def test_triangle_weighted_force_balance(self):
        # Explicit positive vertical reactions put CG inside a noncoplanar hull.
        contacts = [(0, 0, 0), (2, 0, .4), (0, 2, 0)]
        weights = [.2, .3, .5]
        cg = tuple(sum(w*p[axis] for w, p in zip(weights, contacts)) for axis in range(3))
        expected = .4 / math.sqrt(2)  # distance to x+y=2, smallest edge reserve
        self.assertAlmostEqual(projected_support_margin(contacts, cg), expected)
        self.assertLess(projected_support_margin(contacts, (1.2, 1.2, .5)), 0)

    def test_contact_heights_do_not_invent_chassis_tilt(self):
        flat = [(-.2, -.3, 0), (.2, -.3, 0), (.2, .3, 0), (-.2, .3, 0)]
        raised = list(flat)
        raised[2] = (.2, .3, .4)
        # Raising contact z alone is not a suspension/CG transformation.
        self.assertEqual(projected_support_margin(flat, (0, 0, .8)),
                         projected_support_margin(raised, (0, 0, .8)))

    def test_yaw_translation_order_and_redundant_contact_invariance(self):
        contacts = [(0, 0, 0), (2, 0, .4), (0, 2, 0)]
        cg = (.6, 1, .7)
        expected = projected_support_margin(contacts, cg)
        angle = .73

        def transform(p):
            return (4 + p[0]*math.cos(angle)-p[1]*math.sin(angle),
                    -3 + p[0]*math.sin(angle)+p[1]*math.cos(angle), p[2]+7)

        transformed = [transform(p) for p in reversed(contacts)]
        transformed += [transformed[0], transform((.5, .5, 0)), transform((1, 0, .2))]
        self.assertAlmostEqual(projected_support_margin(transformed, transform(cg)), expected)

    def test_invalid_or_degenerate_contacts_are_rejected(self):
        for contacts in ([], [(0, 0, 0), (1, 0, 0)],
                         [(0, 0, 0), (1, 0, .4), (2, 0, 0)],
                         [(0, 0, 0), (0, 0, .4), (1, 1, 0)],
                         [(0, 0, 0), (1, 0, 0), (0, 1, math.nan)],
                         [(0, 0), (1, 0), (0, 1)]):
            with self.subTest(contacts=contacts), self.assertRaises(ValueError):
                projected_support_margin(contacts, (0, 0, 1))

    def test_invalid_cg_is_rejected(self):
        contacts = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        for cg in ((0, 0), (0, 0, math.inf), (math.nan, 0, 1)):
            with self.subTest(cg=cg), self.assertRaises(ValueError):
                projected_support_margin(contacts, cg)

    def test_rigid_rockers_shaft_and_platform_heights(self):
        for offset in (-.05,0,.10):
            for raised in range(4):
                with self.subTest(offset=offset,raised=raised):
                    pose = raised_wheel_pose(.48,.57,.14,offset,.40,raised)
                    self.assertTrue(pose["valid"])
                    centers = pose["centers"]
                    for i,p in enumerate(centers):
                        self.assertAlmostEqual(p[2], .14+(.4 if i == raised else 0), places=10)
                    for front,rear in ((0,2),(1,3)):
                        self.assertAlmostEqual(math.dist(centers[front],centers[rear]),.48)
                    # Chassis longitudinal direction is midway between the two rockers.
                    self.assertAlmostEqual(abs(pose["pitch"]),math.asin(.4/(.48*math.cos(pose["roll"])))/2)
                    # Pivots lie on one rigid shaft and are offset normally from each rocker.
                    pivots=[]
                    for side,front,rear in ((1,0,2),(-1,1,3)):
                        r=pose["roll"]
                        pivot=(0,side*.57*math.cos(r)/2,pose["shaft_origin"][2]+side*.57*math.sin(r)/2)
                        pivots.append(pivot)
                        mid=tuple((a+b)/2 for a,b in zip(centers[front],centers[rear]))
                        self.assertAlmostEqual(math.dist(pivot,mid),abs(offset),places=10)
                    self.assertAlmostEqual(math.dist(*pivots),.57)

    def test_zero_height_cg_and_mass_boundary(self):
        # This checks the zero-articulation LEVEL-ground limit independently
        # of the separate, newly required 30-degree incline.
        p=build_problem({"obstacle_height":0, "slope_angle":0})
        self.addCleanup(p.cleanup)
        p.run_model()
        v=lambda n:p.get_val(n).item()
        mounted=v("loaded_sprung_mass")
        unsprung=v("wheel_mass")+v("drive_mass")+v("rocker_mass")
        self.assertAlmostEqual(mounted+unsprung,v("dry_mass")+v("payload_mass"))
        # Rocker mass is included in structure once; new posing places it at axles.
        self.assertLess(v("rocker_mass"),v("structure_mass"))
        expected_z=(mounted*v("loaded_sprung_cg_z")+unsprung*v("wheel_diameter")/2)/(mounted+unsprung)
        self.assertAlmostEqual(v("obstacle_loaded_cg_z"),expected_z)
        self.assertAlmostEqual(v("obstacle_tip_margin"),min(v("wheelbase"),v("track_width"))/2)
        self.assertTrue(snapshot(p)["feasible"])

    def test_invalid_reach_stays_failed_and_finite(self):
        p=build_problem()
        self.addCleanup(p.cleanup)
        p.run_model()
        report=snapshot(p)
        self.assertFalse(report["feasible"])
        self.assertLess(p.get_val("obstacle_pose_margin").item(),0)
        self.assertEqual(p.get_val("obstacle_pose_valid").item(),0)
        self.assertLess(p.get_val("obstacle_evaluated_height").item(),.4)
        self.assertTrue(all(math.isfinite(x["value"]) for x in report["outputs"].values()))
        for args in ((0,.5,.1,.1,.4),(.4,0,.1,.1,.4),(.4,.5,.1,.2,.4),(.4,.5,.1,.1,-.4)):
            with self.assertRaises(ValueError):
                raised_wheel_pose(*args)
        for changes in ({"rocker_pivot_offset":.2},{"obstacle_height":-1}):
            with self.assertRaises(ValueError):
                build_problem(changes)

    def test_centered_at_axle_reach_limit(self):
        L,T=.48,.57
        maximum=1/math.sqrt(1/L**2+1/(4*T**2))
        at_limit=raised_wheel_pose(L,T,.14,0,maximum)
        self.assertAlmostEqual(at_limit["reach_margin"],0)
        self.assertLess(raised_wheel_pose(L,T,.14,0,maximum+.001)["reach_margin"],0)
        self.assertGreater(raised_wheel_pose(L,T,.14,0,maximum-.001)["reach_margin"],0)

    def test_body_height_and_mirrored_cases(self):
        margins=[]
        for wheel in range(4):
            pose=raised_wheel_pose(.48,.57,.14,.1,.4,wheel)
            cg=posed_loaded_cg(pose,.14,.1,60,0,.4,20)
            margins.append(projected_support_margin(pose["contacts"],cg))
        self.assertLess(max(margins)-min(margins),1e-12)
        pose=raised_wheel_pose(.48,.57,.14,.1,.4)
        low=posed_loaded_cg(pose,.14,.1,60,0,.25,20)
        high=posed_loaded_cg(pose,.14,.1,60,0,1.5,20)
        self.assertGreater(projected_support_margin(pose["contacts"],low),0)
        self.assertLess(projected_support_margin(pose["contacts"],high),0)
        with self.assertRaises(ValueError):
            posed_loaded_cg(pose,.14,.1,0,0,.4,20)

    def test_below_axis_pivot_maximum_is_on_first_branch(self):
        L,T,d=.48,.57,-.05
        maximum=raised_wheel_pose(L,T,.14,d,0)["reach_margin"]
        # Independently sample feasible rigid-rocker angles. Maximum occurs
        # before vertical for a below-axis pivot; the endpoint alone is wrong.
        heights=[]
        for i in range(10001):
            t=i*math.pi/20000
            roll=math.atan((L/2*math.sin(t)+d*(math.cos(t)-1))/T)
            heights.append(L*math.cos(roll)*math.sin(t))
        self.assertAlmostEqual(maximum,max(heights),places=8)
        self.assertGreater(maximum,heights[-1])
        pose=raised_wheel_pose(L,T,.14,d,maximum-.001)
        self.assertTrue(pose["valid"])
        self.assertLess(pose["pitch"]*2,math.pi/2)

    def test_valid_pose_derivatives_and_n2(self):
        p=build_problem({"wheelbase":.46})
        self.addCleanup(p.cleanup)
        p.run_model()
        self.assertGreater(p.get_val("obstacle_pose_margin").item(),0)
        checks=p.check_totals(of=["obstacle_pose_margin","obstacle_tip_margin"],
                              wrt=list(DESIGN),method="fd",form="central",
                              step=3e-5,step_calc="rel_avg",out_stream=None)
        assert_check_totals(checks,atol=1e-3,rtol=2e-3)
        source=p.model.get_source("obstacle_stability.loaded_sprung_cg_z")
        self.assertEqual(source,"mass_properties.loaded_sprung_cg_z")
        self.assertEqual(p.model.get_source("obstacle_stability.rocker_mass"),"structure_geometry.rocker_mass")
