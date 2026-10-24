"""Regression and replacement tests for the shared motor/gearbox interface."""
import json
from pathlib import Path
import unittest
from dataclasses import replace
import numpy as np

from rover.alternatives.x44_motor import CURVE, X44MotorProvider
from rover.motors import Gearmotor
from rover.problem import build_problem
from reporting.report import snapshot
from rover.models import ModelSet, plate_frame_models
from rover.alternatives.bekker_wong_reece import BekkerWongReeceMobility
from rover.excavation import Excavation
from rover.mobility import Mobility
from rover.relocation import SiteRelocation
from rover.drums import DrumMass
from rover.components import GeometryMass

ROOT = Path(__file__).resolve().parents[1]


class DoubleLossMotor:
    """Synthetic test backend: doubles electrical power, keeps mechanical limits."""
    def __init__(self, original):
        self.original=original

    def __getattr__(self,name):
        return getattr(self.original,name)

    def evaluate(self,work,peak):
        result=dict(self.original.evaluate(work,peak))
        result['energy']*=2
        result['peak']*=2
        return result


class DoubleLossProvider(X44MotorProvider):
    @classmethod
    def create(cls,get,role,**options):
        original=super().create(get,role,**options)
        return DoubleLossMotor(original)


class MotorInterfaceTests(unittest.TestCase):
    def problem(self,**options):
        p=build_problem({'wheelbase':.5,'drum_gear_ratio':65},**options)
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def test_reference_regression_and_x44_mechanical_invariants(self):
        fixture=json.loads((ROOT/'tests/fixtures/motor_interface_before.json').read_text())
        for c in fixture['cases']:
            with self.subTest(structure=c['structure'],motors=c['motors'],inputs=c['inputs']):
                # Freeze the legacy mobility equation for this historical motor
                # regression; default BWR is tested separately with both providers.
                models = plate_frame_models() if c['structure'] == 'plate-frame' else ModelSet()
                p=build_problem(c['inputs'],structure=c['structure'],motors=c['motors'],
                                models=replace(models,mobility=Mobility))
                try:
                    p.run_model();result=snapshot(p)
                    # This fixture predates the intentional X44 energy simplification.
                    # Keep all generic expectations; X44 mass/loads/envelope stay fixed.
                    # Fixture revision 2 updates only the user-replaced clearance
                    # margin; original mechanical expectations/tolerances remain.
                    changed_outputs = {
                        "construction_points", "cycle_energy", "run_energy", "peak_power",
                        "dig_energy", "dump_energy", "relocation_energy",
                        "dig_power", "dump_power", "relocation_peak_power",
                        "drive_peak_power", "turn_peak_power", "mobility_energy",
                    }
                    retired_outputs = {n for n in c["outputs"]
                                       if "stator_current" in n or "required_voltage" in n}
                    if c["motors"] == "reference":
                        old_checks = [v['passed'] for n,v in result['constraints'].items()
                                      if n not in ('empty_slope_traversal_margin','loaded_slope_traversal_margin')]
                        self.assertEqual(all(old_checks),c['feasible'])
                    for n,expected in c['outputs'].items():
                        if c["motors"] == "x44-standard" and n in changed_outputs | retired_outputs:
                            continue
                        actual=p.get_val(n).item()
                        self.assertLessEqual(abs(actual-expected),1e-11*max(1.,abs(expected)),n)
                    for n,expected in c['constraint_margins'].items():
                        if c["motors"] == "x44-standard" and n in {"battery_energy_margin", "battery_power_margin"}:
                            continue  # intentionally changed energy model; tested independently below
                        actual=result['constraints'][n]['margin']
                        self.assertLessEqual(abs(actual-expected),1e-11*max(1.,abs(expected)),n)
                finally:
                    p.cleanup()

    def test_hardware_profile_keeps_the_same_discipline_classes(self):
        p=self.problem(motors='x44-standard')
        for slot,cls in [('excavation',Excavation),('mobility',BekkerWongReeceMobility),('relocation',SiteRelocation),
                         ('drum_mass',DrumMass),('geometry',GeometryMass)]:
            self.assertIs(type(getattr(p.model,slot)),cls)
        self.assertIs(p.model.excavation.options['motor_model'],X44MotorProvider)
        self.assertIs(p.model.mobility.options['motor_model'],X44MotorProvider)
        self.assertIs(p.model.relocation.options['motor_model'],X44MotorProvider)

    def test_one_motor_replacement_reaches_every_consumer(self):
        # Set only provider options, never replace any robot discipline.
        p=self.problem(motors='x44-standard')
        for slot,energy,peak,multiplicity,time in [
            ('excavation','dig_energy','dig_power',2,'dig_time'),
            ('relocation','relocation_energy','relocation_peak_power',4,'relocation_time'),
            ('mobility','mobility_energy','drive_peak_power',4,None)]:
            c=getattr(p.model,slot)
            inp={n:c._inputs[n].copy() for n in c._var_rel_names['input']}
            base={n:np.zeros(1) for n in c._var_rel_names['output']}
            c.compute(inp,base)
            c.options['motor_model']=DoubleLossProvider
            changed={n:np.zeros(1) for n in base}
            c.compute(inp,changed)
            scalar=lambda result,n:np.asarray(result[n]).item()
            idle=multiplicity*p.get_val('actuator_idle_power').item()
            duration=(2*p.get_val('traversal_time_one_way').item()+scalar(base,'turn_time')
                      if time is None else scalar(base,time))
            self.assertAlmostEqual(scalar(changed,energy)-idle*duration,
                                   2*(scalar(base,energy)-idle*duration))
            self.assertAlmostEqual(scalar(changed,peak)-idle,2*(scalar(base,peak)-idle))
            if slot=='excavation':
                self.assertEqual(changed['payload_mass'],base['payload_mass'])
                self.assertEqual(changed['cut_force'],base['cut_force'])
                self.assertEqual(changed['drum_torque_margin'],base['drum_torque_margin'])
            if slot=='mobility':
                self.assertEqual(changed['travel_time'],base['travel_time'])
                self.assertEqual(changed['turn_traction_margin'],base['turn_traction_margin'])

    def test_student_excavation_improvement_survives_motor_selection(self):
        class LowerLift(Excavation):
            def granular_response(self,inputs,froude):
                capture,lift=super().granular_response(inputs,froude)
                return capture,lift/2
        for motors in ('reference','x44-standard'):
            base=self.problem(motors=motors)
            custom=self.problem(motors=motors,models=ModelSet(excavation=LowerLift))
            self.assertIs(type(custom.model.excavation),LowerLift)
            self.assertEqual(base.get_val('payload_mass').item(),custom.get_val('payload_mass').item())
            self.assertLess(custom.get_val('dig_energy').item(),base.get_val('dig_energy').item())
            self.assertGreater(custom.get_val('drum_torque_margin').item(),base.get_val('drum_torque_margin').item())

    def test_reference_dependencies_still_cover_both_motoring_branches(self):
        p=self.problem()
        for slot in ('geometry','drum_mass','excavation','mobility','relocation'):
            c=getattr(p.model,slot)
            inputs={n:c._inputs[n].copy() for n in c._var_rel_names['input']}
            output_names=c._var_rel_names['output']
            self.assertEqual(set(c.DEPENDENCIES),set(output_names))
            original={n:np.zeros(1) for n in output_names};c.compute(inputs,original)
            for name in inputs:
                changed_inputs={k:v.copy() for k,v in inputs.items()}
                changed_inputs[name]+=1e-4*max(.01,abs(inputs[name].item()))
                changed={n:np.zeros(1) for n in output_names};c.compute(changed_inputs,changed)
                for output in output_names:
                    if name not in c.DEPENDENCIES[output]:
                        err=abs(np.asarray(changed[output]-original[output]).item())
                        scale=max(1.,abs(np.asarray(original[output]).item()))
                        self.assertLess(err/scale,1e-9,f'{slot}.{output} <- {name}')

    def test_invalid_work_and_efficiency_are_rejected(self):
        motor=Gearmotor(CURVE,100,.8,.72)
        for work,peak in ((-1,0),(0,-1),(float('nan'),1),(1,float('inf'))):
            with self.assertRaises(ValueError): motor.evaluate(work,peak)
        for eta in (0,-.1,1.1,float('nan'),float('inf')):
            with self.assertRaises(ValueError): Gearmotor(CURVE,100,.8,eta)
        self.assertEqual(motor.evaluate(0.,0.),{"energy":0.,"peak":0.})


if __name__=='__main__': unittest.main()
