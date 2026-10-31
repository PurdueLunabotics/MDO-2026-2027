"""Regression and replacement tests for the shared motor/gearbox interface."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
from dataclasses import replace
import numpy as np

from rover.alternatives.x44_motor import CURVE, X44MotorProvider
from rover.motors import Gearmotor
from rover.problem import build_problem
from rover.config import DESIGN
from historical_fixtures import HISTORICAL_ACTUATOR_DESIGN
from reporting.report import snapshot
from rover.models import ModelSet, plate_frame_models
from rover.alternatives.plate_frame import PlateStructureGeometry, PlateStructureLoads, arm_beam_screen
from rover.alternatives.offset_attachment import OffsetArmMechanics
from rover.alternatives.calculated_traversal import CalculatedTraversalMobility
from rover.excavation import Excavation
from rover.alternatives.moving_excavation import MovingExcavation
from rover.mobility import Mobility
from rover.relocation import SiteRelocation
from rover.drums import DrumMass
from rover.components import GeometryMass

ROOT = Path(__file__).resolve().parents[1]


def current_frame_name(name):
    """Read unchanged historical fixture names through the cross-tube contract."""
    return name.replace('cf_tube_', 'cross_tube_').replace('cf_bending_', 'cross_tube_bending_')


class HistoricalAxlePinMechanics(OffsetArmMechanics):
    """Freeze the fixture's original beam envelope, with the pin at the axle."""
    @staticmethod
    def beam_screen(length, side, top, *args):
        if side != 0 or top != 0:
            raise ValueError('Historical motor fixture supports zero offsets only')
        return arm_beam_screen(length, 1., *args)


class HistoricalRoundPlateLoads(PlateStructureLoads):
    ARM_MECHANICS = HistoricalAxlePinMechanics


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

    @patch.dict(DESIGN, {**HISTORICAL_ACTUATOR_DESIGN,
                         'lin_act_side_arm_offset': (0., -.15, .15, 'm')})
    def test_reference_regression_and_x44_mechanical_invariants(self):
        # Freeze the fixture's zero-offset geometry independently of live bounds.
        fixture=json.loads((ROOT/'tests/fixtures/motor_interface_before.json').read_text())
        for c in fixture['cases']:
            with self.subTest(structure=c['structure'],motors=c['motors'],inputs=c['inputs']):
                # Freeze the legacy mobility equation for this historical motor
                # regression; default BWR is tested separately with both providers.
                models = plate_frame_models() if c['structure'] == 'plate-frame' else ModelSet()
                inputs = {current_frame_name(n): v for n,v in c['inputs'].items()}
                inputs.setdefault('obstacle_height', .40)  # Historical fixture's rock height.
                inputs.setdefault('actuator_safety_factor', 1.5)  # Historical acceptance threshold.
                if c['structure'] == 'plate-frame':
                    # Freeze the original round-aluminum-arm model as well as
                    # mobility for this historical motor-interface regression.
                    models = replace(models, structure_geometry=PlateStructureGeometry,
                                     structure_loads=HistoricalRoundPlateLoads)
                    inputs.setdefault('arm_wall', .002)
                    # This motor-only regression retains the fixture's historical
                    # CF material, not the new default aluminum cross-tubes.
                    inputs.update(cross_tube_density=1600., cross_tube_youngs_modulus=70e9,
                                  cross_tube_yield_strength=350e6)
                p=build_problem(inputs,structure=c['structure'],motors=c['motors'],
                                models=replace(models,mobility=Mobility,excavation=Excavation), traversal_mode="fixed")
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
                        # Historical acceptance predates climbing and ground-clearance
                        # requirements; current requirements have independent tests.
                        old_checks = [v['passed'] for n,v in result['constraints'].items()
                                      if n not in ('empty_slope_traversal_margin','loaded_slope_traversal_margin',
                                                   'ground_clearance_margin')]
                        self.assertEqual(all(old_checks),c['feasible'])
                    for n,expected in c['outputs'].items():
                        if c["motors"] == "x44-standard" and n in changed_outputs | retired_outputs:
                            continue
                        actual=p.get_val(current_frame_name(n)).item()
                        self.assertLessEqual(abs(actual-expected),1e-11*max(1.,abs(expected)),n)
                    for n,expected in c['constraint_margins'].items():
                        if n == 'actuator_body_margin':
                            continue  # User retired this criterion; historical fixture data are unchanged.
                        if c["motors"] == "x44-standard" and n in {"battery_energy_margin", "battery_power_margin"}:
                            continue  # intentionally changed energy model; tested independently below
                        actual=result['constraints'][current_frame_name(n)]['margin']
                        self.assertLessEqual(abs(actual-expected),1e-11*max(1.,abs(expected)),n)
                finally:
                    p.cleanup()

    def test_hardware_profile_keeps_the_same_discipline_classes(self):
        p=self.problem(motors='x44-standard')
        for slot,cls in [('excavation',MovingExcavation),('mobility',CalculatedTraversalMobility),('relocation',SiteRelocation),
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
            duration=(scalar(base,'travel_time')+scalar(base,'turn_time')
                      if time is None else scalar(base,time))
            idle_energy = idle*duration
            idle_peak = idle
            if slot == 'excavation':
                idle_energy = idle*(scalar(base,'pair_cutting_time')+2*p.get_val('drum_spinup_time').item())
                idle_energy += 4*p.get_val('actuator_idle_power').item()*scalar(base,'feed_time')
                idle_peak += 4*p.get_val('actuator_idle_power').item()
            self.assertAlmostEqual(scalar(changed,energy)-idle_energy,
                                   2*(scalar(base,energy)-idle_energy))
            self.assertAlmostEqual(scalar(changed,peak)-idle_peak,2*(scalar(base,peak)-idle_peak))
            if slot=='excavation':
                self.assertEqual(changed['payload_mass'],base['payload_mass'])
                self.assertEqual(changed['cut_force'],base['cut_force'])
                self.assertEqual(changed['drum_torque_margin'],base['drum_torque_margin'])
            if slot=='mobility':
                self.assertEqual(changed['travel_time'],base['travel_time'])
                self.assertEqual(changed['turn_traction_margin'],base['turn_traction_margin'])

    def test_student_excavation_improvement_survives_motor_selection(self):
        class LowerLift(MovingExcavation):
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
