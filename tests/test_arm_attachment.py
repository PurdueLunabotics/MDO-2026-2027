"""Independent local coordinates, virtual work, eccentric balance and domains."""
import json
import math
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from openmdao.utils.assert_utils import assert_check_totals
from openmdao.visualization.n2_viewer.n2_viewer import _get_viewer_data
from rover.linkage import attachment_point, drum_pose, sweep_poses
from rover.alternatives.offset_attachment import offset_mount_loads, offset_beam_screen
from rover.problem import build_problem
from rover.config import DESIGN
from historical_fixtures import HISTORICAL_ACTUATOR_DESIGN
from reporting.report import snapshot
from reporting.html_report import geometry_svg, values_from_snapshot
from reporting.frame_members import chassis_members
from reporting.case_inputs import read_case


class ArmAttachmentTests(unittest.TestCase):
    def problem(self, **overrides):
        # Explicit original offset-domain fixture for zero/positive-pin limits.
        # Current hardware search bounds are preserved in production config.
        with patch.dict(DESIGN, {
            'lin_act_side_arm_offset': (0., -.15, .15, 'm'),
            'lin_act_top_arm_offset': (0., -.15, .15, 'm'),
        }):
            p = build_problem(overrides)
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def matched_poses(self, p, side=-.08, top=.04):
        v = lambda n: p.get_val(n).item()
        changes = dict(lin_act_side_arm_offset=side, lin_act_top_arm_offset=top)
        for state, length in (("stowed","actuator_retracted_length"),
                              ("travel","travel_actuator_length"),("extended","actuator_extended_length")):
            x,z = attachment_point(v("pivot_x"),v("bottom_pivot_height"),v(state+"_x"),v(state+"_y"),side,top)
            changes[length] = math.hypot(x-v("actuator_pivot_x"),z-v("top_pivot_height"))
        return changes

    def test_body_check_is_diagnostic_only_in_all_model_profiles(self):
        for structure in ('reference','plate-frame'):
            for actuators in ('reference','pa04-900'):
                with self.subTest(structure=structure,actuators=actuators):
                    p=build_problem(structure=structure,actuators=actuators)
                    self.addCleanup(p.cleanup);p.run_model()
                    report=snapshot(p)
                    self.assertNotIn('actuator_body_margin',p.margin_specs)
                    self.assertIn('actuator_body_margin',p.model.linkage.OUTPUT_UNITS)
                    self.assertFalse(any(n.rsplit('.',1)[-1]=='actuator_body_margin' for n in p.model.get_constraints()))
                    self.assertNotIn('actuator_body_margin',report['constraints'])
                    self.assertIn('actuator_body_margin',report['outputs'])
                    p.set_val('actuator_body_margin',-1000.)
                    diagnostic=snapshot(p)
                    self.assertEqual(diagnostic['outputs']['actuator_body_margin']['value'],-1000.)
                    self.assertEqual(diagnostic['constraints'],report['constraints'])
                    self.assertEqual(diagnostic['feasible'],report['feasible'])
                    self.assertEqual(p.model_metadata['schema_version'],7)

    def test_signed_local_coordinates_and_zero_offset_limit(self):
        for angle in (-1.1,0.,.8):
            u = np.array([math.cos(angle),math.sin(angle)])
            n = np.array([-u[1],u[0]])
            b = np.array([.13,.15]); c = b+.6*u
            for side,top in ((.05,.05),(-.08,.04),(.03,-.06),(0.,0.)):
                d = np.asarray(attachment_point(*b,*c,side,top))
                self.assertAlmostEqual(float((d-c)@u),side)
                self.assertAlmostEqual(float((d-c)@n),top)
        # Independently reconstruct the old tip circle intersection.
        p=self.problem();v=lambda n:p.get_val(n).item()
        self.assertNotIn("actuator_attachment_fraction",p.design_specs)
        for name in ("lin_act_side_arm_offset","lin_act_top_arm_offset"):
            self.assertIn(name,p.design_specs); self.assertEqual(v(name),0.)
        a=np.array([v("actuator_pivot_x"),v("top_pivot_height")]);b=np.array([v("pivot_x"),v("bottom_pivot_height")])
        gap=np.linalg.norm(a-b);q=(v("arm_length")**2+gap**2-v("travel_actuator_length")**2)/(2*gap)
        u=(a-b)/gap;expected=b+q*u+math.sqrt(v("arm_length")**2-q*q)*np.array([u[1],-u[0]])
        np.testing.assert_allclose(expected,[v("travel_x"),v("travel_y")],atol=1e-12)

    def test_pin_and_drum_closure_at_all_states_and_sweep(self):
        p=self.problem()
        for n,value in self.matched_poses(p).items():p.set_val(n,value)
        p.run_model();values=values_from_snapshot(snapshot(p))
        b=np.array([values["pivot_x"],values["bottom_pivot_height"]]);a=np.array([values["actuator_pivot_x"],values["top_pivot_height"]])
        for state,length in (("stowed","actuator_retracted_length"),("travel","travel_actuator_length"),("extended","actuator_extended_length")):
            c=np.array([values[state+"_x"],values[state+"_y"]]);d=np.array(attachment_point(*b,*c,-.08,.04))
            self.assertAlmostEqual(np.linalg.norm(c-b),values["arm_length"])
            self.assertAlmostEqual(np.linalg.norm(d-a),values[length])
            self.assertAlmostEqual(np.linalg.norm(d-b),values["actuator_attachment_distance"])
        for c in sweep_poses(values)[0]:
            d=np.array(attachment_point(*b,*c,-.08,.04));length=np.linalg.norm(d-a)
            np.testing.assert_allclose(drum_pose(values["arm_length"],*b,*a,length,-.08,.04),c,atol=1e-10)

    def test_virtual_work_and_gravity_energy_at_same_drum_poses(self):
        p=self.problem(actuator_idle_power=0);v=lambda n:p.get_val(n).item()
        energy=v("arm_energy")
        for n,value in self.matched_poses(p).items():p.set_val(n,value)
        p.run_model();self.assertAlmostEqual(v("arm_energy"),energy)
        numeric=[];values=values_from_snapshot(snapshot(p))
        for c in sweep_poses(values)[0]:
            d=attachment_point(v("pivot_x"),v("bottom_pivot_height"),*c,-.08,.04)
            length=math.hypot(d[0]-v("actuator_pivot_x"),d[1]-v("top_pivot_height"));h=1e-6
            args=(v("arm_length"),v("pivot_x"),v("bottom_pivot_height"),v("actuator_pivot_x"),v("top_pivot_height"))
            numeric.append(abs((drum_pose(*args,length+h,-.08,.04)[1]-drum_pose(*args,length-h,-.08,.04)[1])/(2*h)))
        self.assertAlmostEqual(v("actuator_leverage_max"),max(numeric),places=7)

    def test_eccentric_mount_independent_force_moment_and_beam_limits(self):
        L,s,t,W,Wa=1.,.1,.05,100.,20.
        upper,lower,f=offset_mount_loads((0,0),(0,1),(L,0),W,Wa,s,t)
        np.testing.assert_allclose(upper+lower,[0,-W-Wa],atol=1e-12)
        self.assertAlmostEqual((L+s)*f[1]-t*f[0]-W*L-Wa*(L+s)/2,0.,places=10)
        # Pure axial pin force at a nonzero top offset gives a known end couple.
        P,E,I,A,depth=70.,70e9,1e-6,1e-3,.05
        stress,delta,moment,compression=offset_beam_screen(L,0.,t,np.array([L,0.]),np.array([-P,0.]),0.,0.,A,I,depth,E)
        self.assertAlmostEqual(moment,t*P)
        self.assertAlmostEqual(stress,P/A+t*P*depth/(2*I))
        self.assertAlmostEqual(delta,t*P*L**2/(2*E*I),places=12)
        self.assertAlmostEqual(compression,P)
        with self.assertRaises(ValueError):offset_mount_loads((0,0),(2,0),(1,0),W,0.,0.,0.)

    @patch.dict(DESIGN, HISTORICAL_ACTUATOR_DESIGN)
    def test_invalid_offsets_and_unreachable_triangle_remain_failed(self):
        for overrides in ({"actuator_attachment_fraction":1.},{"lin_act_side_arm_offset":.151},
                          {"lin_act_top_arm_offset":float("nan")},{"lin_act_top_arm_offset":-.151}):
            with self.assertRaises(ValueError):build_problem(overrides)
        for side,top in ((-.6,0.),(float("nan"),0.),(0.,float("inf"))):
            with self.assertRaises(ValueError):drum_pose(.6,.13,.15,.15,.45,.38,side,top)
        p=self.problem(lin_act_side_arm_offset=-.15,actuator_extended_length=1.0)
        report=snapshot(p);self.assertFalse(report["feasible"])
        self.assertFalse(report["constraints"]["triangle_extended_margin"]["passed"])
        self.assertTrue(all(math.isfinite(i["value"]) for i in report["outputs"].values()))
        self.assertAlmostEqual(math.hypot(p.get_val("extended_x").item()-.13,p.get_val("extended_y").item()-.15),.6)

    def test_offset_derivatives_and_n2_connections(self):
        p=self.problem(lin_act_side_arm_offset=.05,lin_act_top_arm_offset=.05)
        checks=p.check_totals(of=["travel_x","extended_y","actuator_leverage_max","actuator_attachment_distance",
                                 "required_actuator_force","arm_energy","loaded_cg_z","loaded_yaw_inertia"],
                              wrt=["lin_act_side_arm_offset","lin_act_top_arm_offset","arm_length"],
                              method="fd",form="central",step=3e-5,step_calc="rel_avg",out_stream=None)
        assert_check_totals(checks,atol=1e-3,rtol=2e-3)
        declared=set(_get_viewer_data(p)["declare_partials_list"])
        for name in ("lin_act_side_arm_offset","lin_act_top_arm_offset"):
            self.assertIn("linkage.actuator_leverage_max > linkage."+name,declared)
            self.assertIn("mass_properties.loaded_cg_z > mass_properties."+name,declared)
        self.assertTrue(all(not source.startswith("_auto_ivc") for source in p.model._conn_global_abs_in2out.values()))

    def test_beam_envelope_is_continuous_through_zero_offsets(self):
        # The same conservative stress aggregation applies on both sides of zero.
        angle=.4; length=.6
        c=length*np.array([math.cos(angle),math.sin(angle)])
        for side in (-.1,0.,.1):
            force=offset_mount_loads((0,0),(0,.5),c,100.,10.,side,0.)[2]
            args=(c,force,100.,10.,.001,1e-6,.05,70e9)
            base=offset_beam_screen(length,side,0.,*args)
            for tiny in (-1e-10,1e-10):
                perturbed=offset_beam_screen(length,side,tiny,*args)
                np.testing.assert_allclose(perturbed,base,rtol=1e-8,atol=1e-3)

    def test_drawings_end_actuator_at_offset_pin_and_mirror_top_upward(self):
        p=self.problem(lin_act_side_arm_offset=.05,lin_act_top_arm_offset=.05)
        values=values_from_snapshot(snapshot(p))
        for opts in ({},{"travel":True},{"extended":True}):
            svg=geometry_svg(values,**opts);root=ET.fromstring(svg)
            self.assertIn("side +50.0 mm / top +50.0 mm",svg)
            lines=root.findall(".//{http://www.w3.org/2000/svg}line")
            actuator=next(x for x in lines if x.get("stroke")=="#d77422")
            bracket=next(x for x in lines if x.get("stroke")=="#246f99" and x.get("stroke-width")=="4")
            for coord in ("x2","y2"):self.assertEqual(actuator.get(coord),bracket.get(coord))
        members=chassis_members(values);pins=[m['end'] for m in members if m['kind']=='actuator']
        self.assertAlmostEqual(pins[0][0],-pins[1][0]);self.assertEqual(pins[0][2],pins[1][2])

    def test_old_case_conversion_is_explicit_and_mixed_coordinates_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/"case.json"
            p.write_text(json.dumps({"inputs":{"arm_length":.6,"actuator_attachment_fraction":.8}}))
            values,meta=read_case(p)
            self.assertAlmostEqual(values['lin_act_side_arm_offset'],-.12)
            self.assertEqual(values['lin_act_top_arm_offset'],0.)
            self.assertNotIn('actuator_attachment_fraction',values)
            self.assertIn('attachment_migration',meta['notes'])
            p.write_text(json.dumps({"inputs":{"arm_length":.6,"actuator_attachment_fraction":1.,"lin_act_top_arm_offset":.05}}))
            with self.assertRaises(ValueError):read_case(p)


if __name__ == "__main__":unittest.main()
