"""Independent source reproduction, physical limits and integration contracts."""
import csv
import hashlib
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import tempfile
import numpy as np
from openmdao.utils.assert_utils import assert_check_totals
from dataclasses import replace
from rover.alternatives.x44_motor import CURVE
from rover.motors import Gearmotor
from rover.config import DESIGN, ASSUMPTIONS, configuration
from historical_fixtures import HISTORICAL_ACTUATOR_DESIGN
from rover.problem import build_problem
from reporting.report import snapshot
from reporting.html_report import write_report

ROOT = Path(__file__).resolve().parents[1]


class X44Tests(unittest.TestCase):
    def test_manifest_and_held_out_manufacturer_curve(self):
        path=ROOT/'data/motors/kraken_x44_standard_12v.csv'
        manifest=json.loads((path.parent/'manifest.json').read_text())
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),manifest['sha256'])
        with path.open(newline='') as f:
            rows=list(csv.DictReader(f))
        # Neither coefficient extraction nor test expectations fit the interior rows.
        for row in rows[1:-1]:
            rpm,torque,current,power,eff=[float(row[n]) for n in ('RPM','Torque','Current','Power','Efficiency')]
            self.assertAlmostEqual(CURVE.available_torque(rpm),torque,places=8)
            self.assertAlmostEqual(CURVE.free_current+torque/CURVE.kt,current,places=6)
            self.assertAlmostEqual(torque*rpm*math.pi/30,power,places=6)
            # CSV efficiency is full-duty evidence, not mission-wide efficiency.
            self.assertAlmostEqual(100*power/(12*current),eff,places=6)
        t=CURVE.kt*(41-CURVE.free_current)
        rpm=CURVE.free_rpm*(1-t/CURVE.stall_torque)
        power=t*rpm*math.pi/30
        self.assertLess(abs(power-393)/393,.01)
        self.assertLess(abs(power/(12*41)-.804),.002)

    def test_stall_free_and_peak_curve_limits(self):
        self.assertAlmostEqual(CURVE.available_torque(0),CURVE.stall_torque)
        self.assertEqual(CURVE.available_torque(CURVE.free_rpm),0.)
        self.assertAlmostEqual(CURVE.peak_power(),
                               CURVE.stall_torque*CURVE.free_rpm*math.pi/120)
        for rpm in (0,1000,4000,7500):
            mechanical=CURVE.available_torque(rpm)*rpm*math.pi/30
            self.assertLessEqual(mechanical,CURVE.peak_power())

    def test_failure_domain_and_current_limit(self):
        self.assertLess(CURVE.available_torque(CURVE.free_rpm+100),0.)
        self.assertLess(CURVE.available_torque(1000,20)-.5,0.)
        self.assertGreater(CURVE.available_torque(1000,20)-.1,0.)
        self.assertLess(CURVE.peak_power(20),CURVE.peak_power())
        limited=replace(CURVE,stator_current_limit=20)
        self.assertLess(limited.speed_limit(.5),0.)
        for rpm,limit in ((-1,0),(0,-1),(math.nan,0),(0,math.inf)):
            with self.assertRaises(ValueError): CURVE.available_torque(rpm,limit)
        for ratio,eta in ((0,.8),(100,0),(100,1.1)):
            with self.assertRaises(ValueError): Gearmotor(CURVE,ratio,eta,.72)

    def test_gear_conversion_and_single_combined_energy_loss(self):
        motor=Gearmotor(CURVE,100,.8,.72)
        self.assertAlmostEqual(motor.available_torque(30),CURVE.available_torque(3000)*80)
        self.assertAlmostEqual(motor.free_rpm,CURVE.free_rpm/100)
        self.assertAlmostEqual(motor.peak_power(),CURVE.peak_power()*.8)
        self.assertAlmostEqual(motor.speed_limit(8),CURVE.speed_limit(.1)/100)
        # Output work already includes gear loss through the combined efficiency.
        result=motor.evaluate(72,36)
        self.assertAlmostEqual(result['energy'],100)
        self.assertAlmostEqual(result['peak'],50)
        different_gears=Gearmotor(CURVE,100,.5,.72)
        self.assertEqual(result,different_gears.evaluate(72,36))
        self.assertLess(different_gears.available_torque(30),motor.available_torque(30))

    def test_work_and_startup_energy_match_independent_limits(self):
        motor=Gearmotor(CURVE,100,.8,.72)
        # Independent constant-force work plus kinetic energy, with no regen.
        force,distance,mass,speed=10.,1.25,20.,.5
        work=force*distance+.5*mass*speed**2
        result=motor.evaluate(work/4,2.)
        self.assertAlmostEqual(4*result['energy'],15./.72)
        # A drum ring starting unloaded: E = 1/2 I omega^2.
        inertia,omega=.2,4.
        result=motor.evaluate(.5*inertia*omega**2,inertia*omega**2)
        self.assertAlmostEqual(result['energy'],1.6/.72)
        self.assertGreater(result['energy'],1.6)

    def problem(self):
        p=build_problem({'wheelbase':.5,'drum_gear_ratio':65},structure='plate-frame',motors='x44-standard')
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def test_hardware_profile_mass_energy_sources_and_reference(self):
        d,a=dict(DESIGN),dict(ASSUMPTIONS)
        p=self.problem()
        s=snapshot(p)
        self.assertNotIn('drive_motor_power',s['design'])
        self.assertNotIn('drum_motor_power',s['design'])
        self.assertNotIn('drive_gear_ratio',s['design'])
        self.assertIn('drum_gear_ratio',s['design'])
        self.assertEqual(p.get_val('drive_gear_ratio').item(),100)
        self.assertAlmostEqual(p.get_val('drive_mass').item(),4*(.3401942775+.697))
        dm=.28; length=.25
        shell=4*5*(math.pi*dm*length+math.pi*dm*dm/2)
        self.assertAlmostEqual(p.get_val('drum_mass').item(),shell+2*(.3401942775+.753))
        total=sum(p.get_val(n).item() for n in ('fixed_mass','structure_mass','wheel_mass','drum_mass','drive_mass','battery_mass','linkage_mass'))
        self.assertAlmostEqual(total,p.get_val('dry_mass').item())
        energy=sum(p.get_val(n).item() for n in ('dig_energy','dump_energy','relocation_energy','mobility_energy','arm_energy'))
        energy+=p.get_val('electronics_power').item()*p.get_val('cycle_time').item()
        self.assertAlmostEqual(energy,p.get_val('cycle_energy').item()*3600)
        self.assertFalse(p.model_metadata['motor_performance']['continuous_operation_verified'])
        self.assertIn('undecided',p.model_metadata['motor_performance']['current_limits']['drive_stator_current_limit'])
        p.final_setup()
        sources=p.model._conn_global_abs_in2out
        self.assertFalse(any('_auto_ivc' in src for src in sources.values()))
        self.assertEqual(sources['mobility.drive_stator_current_limit'],'inputs.drive_stator_current_limit')
        self.assertEqual(DESIGN,d)
        self.assertEqual(ASSUMPTIONS,a)
        rd,ra=configuration()
        self.assertIn('drive_motor_power',rd)
        self.assertNotIn('x44_motor_mass',ra)

    def test_inactive_generic_motor_inputs_and_low_current_failure(self):
        p=self.problem()
        before=snapshot(p)
        for name in p.model_metadata['motor_performance']['inactive_reference_inputs']:
            p.set_val(name,p.get_val(name).item()*1.1)
        p.run_model()
        after=snapshot(p)
        for n,x in before['outputs'].items(): self.assertAlmostEqual(x['value'],after['outputs'][n]['value'],places=8,msg=n)
        p.set_val('drive_stator_current_limit',1)
        p.set_val('drum_stator_current_limit',1)
        p.run_model()
        self.assertLess(p.get_val('straight_motor_margin').item(),0)
        self.assertLess(p.get_val('drum_torque_margin').item(),0)
        self.assertFalse(snapshot(p)['feasible'])
        with self.assertRaises(ValueError): build_problem({'motor_bus_voltage':10},motors='x44-standard')
        with self.assertRaises(ValueError): build_problem({'drive_gear_ratio':90},motors='x44-standard')

    def test_combined_efficiencies_control_energy_not_torque(self):
        p=self.problem()
        get=lambda name:p.get_val(name).item()
        before={n:get(n) for n in ("dig_energy","dig_power","dump_energy","dump_power",
                                  "mobility_energy","drive_peak_power","relocation_energy",
                                  "relocation_peak_power","drum_torque_margin","straight_motor_margin")}
        dig_idle=6*get("actuator_idle_power")
        dump_idle=get("actuator_idle_power")
        drive_idle=4*get("actuator_idle_power")
        expected=[
            ("dig_energy",2*get("actuator_idle_power")*(get("pair_cutting_time")+2*get("drum_spinup_time"))+4*get("actuator_idle_power")*get("feed_time")),
            ("dig_power",dig_idle),
            ("dump_energy",dump_idle*get("dump_time")),
            ("dump_power",dump_idle),
            ("mobility_energy",drive_idle*(get("travel_time")+get("turn_time"))),
            ("drive_peak_power",drive_idle),
            ("relocation_energy",drive_idle*get("relocation_time")),
            ("relocation_peak_power",drive_idle)]
        p.set_val("drive_efficiency",get("drive_efficiency")/2)
        p.set_val("drum_efficiency",get("drum_efficiency")/2)
        p.run_model()
        for name,idle in expected:
            self.assertAlmostEqual(get(name)-idle,2*(before[name]-idle),msg=name)
        for name in ("drum_torque_margin","straight_motor_margin"):
            self.assertEqual(get(name),before[name])
        for n in ("drive_stator_current_rms","drum_required_voltage"):
            self.assertNotIn(n,snapshot(p)["outputs"])

    def test_drum_zero_resistance_limit_is_startup_kinetic_energy(self):
        p=self.problem()
        c=p.model.excavation
        inputs={n:c._inputs[n].copy() for n in c._var_rel_names["input"]}
        for name in ("cut_pressure","internal_lift_factor","actuator_idle_power"):
            inputs[name][:]=0.
        outputs={n:np.zeros(1) for n in c._var_rel_names["output"]}
        c.compute(inputs,outputs)
        value=lambda n:np.asarray(inputs[n]).item()
        radius=value("drum_diameter")/2
        inertia=(value("drum_mass")+np.asarray(outputs["payload_mass"]).item())*radius**2
        for energy,rpm in (("dig_energy","drum_rpm"),("dump_energy","dump_rpm")):
            omega=value(rpm)*math.pi/30
            expected=.5*inertia*omega**2/value("drum_efficiency")
            if energy == "dig_energy":
                expected *= 2  # Conservative two starts per pair during staged excavation.
                expected += np.asarray(outputs["feed_drive_energy"]).item()
            self.assertAlmostEqual(np.asarray(outputs[energy]).item(),expected,msg=energy)
        omega=value("drum_rpm")*math.pi/30
        motor=c.options["motor_model"].create(value,"drum")
        expected=2*motor.available_torque(value("drum_rpm")) - (
            value("drum_torque_service_factor")*inertia*omega/value("drum_spinup_time"))
        self.assertAlmostEqual(np.asarray(outputs["drum_torque_margin"]).item(),expected)

    def test_total_derivatives(self):
        p=self.problem()
        checks=p.check_totals(of=['construction_points',*p.margin_specs],wrt=list(p.design_specs),method='fd',form='central',step=3e-5,step_calc='rel_avg',out_stream=None)
        assert_check_totals(checks,atol=1e-3,rtol=2e-3)

    def test_optional_component_dependency_omissions(self):
        p=self.problem()
        for slot in ('geometry','drum_mass','mobility','excavation','relocation'):
            c=getattr(p.model,slot)
            inputs={n:c._inputs[n].copy() for n in c._var_rel_names['input']}
            out_names=c._var_rel_names['output']
            self.assertEqual(set(c.DEPENDENCIES),set(out_names))
            base={n:np.zeros(1) for n in out_names}
            c.compute(inputs,base)
            for n in inputs:
                shift={k:v.copy() for k,v in inputs.items()}
                shift[n]+=1e-4*max(.01,abs(inputs[n].item()))
                changed={k:np.zeros(1) for k in out_names}
                c.compute(shift,changed)
                for out in out_names:
                    if n not in c.DEPENDENCIES[out]:
                        error=abs(np.asarray(changed[out]-base[out]).item())/max(1.,abs(np.asarray(base[out]).item()))
                        self.assertLess(error,1e-9,f'{slot}.{out} <- {n}')

    @patch.dict(DESIGN, HISTORICAL_ACTUATOR_DESIGN)
    def test_historical_feasible_start_fails_new_bwr_climb_requirements(self):
        fixture=json.loads((ROOT/'tests/fixtures/x44_standard_feasible_start.json').read_text())
        inputs={n.replace('cf_tube_', 'cross_tube_'): v for n,v in fixture['inputs'].items()}
        p=build_problem(inputs,structure=fixture['structure'],motors=fixture['motors'],
                        optimizer=fixture['optimizer'],maxiter=2,cobyla_radius=fixture['cobyla_radius'])
        self.addCleanup(p.cleanup)
        p.run_model();before=snapshot(p)
        self.assertFalse(before['feasible'])
        self.assertFalse(before['constraints']['empty_slope_traversal_margin']['passed'])
        self.assertFalse(before['constraints']['loaded_slope_traversal_margin']['passed'])
        # This small-wheel historical start also exceeds the BWR straight-drive
        # motor envelope; unchanged turn/drum physics still passes its screens.
        self.assertFalse(before['constraints']['straight_motor_margin']['passed'])
        p.driver.options['maxiter'] = len(p.design_specs)+2
        result=p.run_driver();p.run_model();after=snapshot(p)
        self.assertFalse(result.success)
        self.assertFalse(after['feasible'])
        self.assertFalse(after['constraints']['straight_motor_margin']['passed'])
        for n in ("turn_motor_margin","drum_torque_margin","dump_torque_margin"):
            self.assertTrue(after["constraints"][n]["passed"],n)
        self.assertEqual(p.get_val("drive_gear_ratio").item(),100.)
        self.assertIn('wheelbase',p.design_specs)
        self.assertEqual(p.design_specs['wheelbase'][2],.65)
    def test_simplified_report_contract(self):
        p=self.problem()
        state=snapshot(p)
        with tempfile.TemporaryDirectory() as folder:
            out=Path(folder)/'motors.json'
            write_report({'final':state,'baseline':state,'status':'baseline','optimizer_success':None,
                          'openmdao_version':'test','model_metadata':p.model_metadata},out)
            html=out.with_suffix('.html').read_text(encoding='utf-8')
            self.assertIn('Standard X44 motors',html)
            self.assertIn('Continuous operation unverified',html)
            self.assertIn('Electrical energy = output-shaft mechanical work / combined efficiency',html)


if __name__=='__main__': unittest.main()
