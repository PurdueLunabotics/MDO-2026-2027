"""Overhang equilibrium, beam limit, mass/energy accounting and failures."""
import math
import unittest
import numpy as np
from openmdao.utils.assert_utils import assert_check_totals
from rover.problem import build_problem
from rover.linkage import attachment_point
from rover.alternatives.extended_attachment import physical_length, extended_mount_loads,extended_beam_screen
from reporting.report import snapshot


class ExtendedAttachmentTests(unittest.TestCase):
    def problem(self,values=None,**options):
        p=build_problem(values,structure='plate-frame',**options)
        self.addCleanup(p.cleanup);p.run_model();return p

    def matched(self,p,fraction):
        get=lambda n:p.get_val(n).item()
        change={'actuator_attachment_fraction':fraction}
        for pose,name in [('travel','travel_actuator_length'),('stowed','actuator_retracted_length'),('extended','actuator_extended_length')]:
            x,z=attachment_point(get('pivot_x'),get('bottom_pivot_height'),get(pose+'_x'),get(pose+'_y'),fraction)
            change[name]=math.hypot(x-get('actuator_pivot_x'),z-get('top_pivot_height'))
        return change

    def test_independent_horizontal_overhang_moment_and_deflection(self):
        L,f,W,E,I,area,depth=1.,1.5,100.,70e9,1e-6,1e-3,.05
        upper,lower,actuator=extended_mount_loads((0,0),(0,1),(L,0),W,0.,f)
        np.testing.assert_allclose(upper+lower,[0.,-W],atol=1e-12)
        moment=f*L*actuator[1]-L*W
        self.assertAlmostEqual(float(moment),0.,places=10)
        stress,delta,M,compression=extended_beam_screen(L,f,np.array([L,0.]),actuator,W,0.,area,I,depth,E)
        self.assertAlmostEqual(M,W*L*(1-1/f),places=10)
        self.assertAlmostEqual(stress,abs(actuator[0])/area+M*depth/(2*I),places=6)
        expected=W*L**3*(2*f*f-3*f+1)/(6*E*I)
        self.assertAlmostEqual(delta,expected,places=12)
        self.assertAlmostEqual(compression,max(0.,-actuator[0]),places=10)

    def test_reference_numerics_unchanged_for_fraction_at_or_below_one(self):
        for fraction in (.8,1.):
            a=self.problem({'actuator_attachment_fraction':fraction})
            b=self.problem({'actuator_attachment_fraction':fraction},attachment_max=1.5)
            sa,sb=snapshot(a),snapshot(b)
            for n,item in sa['outputs'].items():self.assertEqual(item,sb['outputs'][n],n)
            self.assertEqual(sa['constraints'],sb['constraints'])
            self.assertEqual(b.get_val('arm_extension_mass').item(),0.)

    def test_mass_gravity_work_and_pin_geometry_include_extension(self):
        p=self.problem(attachment_max=1.5)
        get=lambda n:p.get_val(n).item()
        old_mass=get('linkage_mass');mu=get('arm_linear_mass');L=get('arm_length')
        changes=self.matched(p,1.2)
        for n,value in changes.items():p.set_val(n,value)
        p.run_model()
        self.assertAlmostEqual(get('arm_physical_length'),1.2*L)
        self.assertAlmostEqual(get('arm_extension_mass'),2*mu*.2*L)
        # Actuator-length mass changes are accounted separately from the tube.
        self.assertGreater(get('linkage_mass'),old_mass)
        v={n:i['value'] for group in ('design','assumptions','outputs') for n,i in snapshot(p)[group].items()}
        dry=sum(v[n] for n in ('fixed_mass','structure_mass','wheel_mass','drum_mass','drive_mass','battery_mass','linkage_mass'))
        self.assertAlmostEqual(dry,get('dry_mass'),places=10)
        for pose,name in [('travel','travel_actuator_length'),('stowed','actuator_retracted_length'),('extended','actuator_extended_length')]:
            x,z=attachment_point(v['pivot_x'],v['bottom_pivot_height'],v[pose+'_x'],v[pose+'_y'],1.2)
            self.assertAlmostEqual(math.hypot(x-v['actuator_pivot_x'],z-v['top_pivot_height']),v[name],places=10)
        empty=(get('drum_mass')/2+mu*(1.2*L)**2/(2*L))*get('gravity')
        loaded=empty+get('payload_mass')/2*get('gravity')
        expected=2*((loaded+empty)*max(get('travel_y')-get('extended_y'),0.)+get('breakout_force')*get('cut_depth'))/get('actuator_efficiency')+2*get('actuator_idle_power')*get('actuation_time')
        self.assertAlmostEqual(get('arm_energy'),expected,places=8)
        self.assertEqual(get('frame_load_solution_valid'),1.)
        # Subtract tails and independently add their centroid and yaw inertia.
        from rover.alternatives.extended_attachment import ExtendedPlateMassProperties
        from rover.alternatives.plate_frame import PlateMassProperties
        component=p.model.mass_properties
        inp={n:component._inputs[n].copy() for n in component._inputs}
        inp['linkage_mass']-=get('arm_extension_mass')
        core={};PlateMassProperties.compute(component,inp,core)
        each=mu*.2*L;tail_x=get('travel_x')+.1*(get('travel_x')-get('pivot_x'))
        local=each*(.2*(get('travel_x')-get('pivot_x')))**2/12
        self.assertAlmostEqual(get('half_yaw_inertia')-core['half_yaw_inertia'],2*(local+each*tail_x**2),places=10)

    def test_domain_failures_and_opt_in_bounds(self):
        for max_f in (.9,1.51,float('nan')):
            with self.assertRaises(ValueError):build_problem(attachment_max=max_f)
        with self.assertRaises(ValueError):build_problem(attachment_max=1.5)
        with self.assertRaises(ValueError):build_problem({'actuator_attachment_fraction':1.2},structure='plate-frame')
        p=self.problem({'actuator_attachment_fraction':1.2},attachment_max=1.5,optimize_attachment=True)
        self.assertEqual(p.design_specs['actuator_attachment_fraction'][2],1.5)
        for L,f in ((0.,1.2),(1.,0.),(float('nan'),1.2)):
            with self.assertRaises(ValueError):physical_length(L,f)

    def test_smooth_extension_mass_properties_derivatives(self):
        p=self.problem(attachment_max=1.5)
        for n,value in self.matched(p,1.2).items():p.set_val(n,value)
        p.set_val('fill_fraction',.4);p.run_model()
        checks=p.check_totals(of=['linkage_mass','arm_energy','required_actuator_force','empty_cg_z','half_yaw_inertia','loaded_sprung_cg_z'],
            wrt=['actuator_attachment_fraction','arm_length'],method='fd',form='central',step=3e-5,step_calc='rel_avg',out_stream=None)
        assert_check_totals(checks,atol=1e-3,rtol=2e-3)


if __name__=='__main__':unittest.main()
