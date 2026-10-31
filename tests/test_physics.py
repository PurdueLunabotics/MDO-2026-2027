"""Independent limits, conservation, failure domains, and replaceability checks."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import openmdao.api as om
from rover.problem import build_problem, MARGINS
from rover.config import DESIGN
from rover.models import ModelSet
from rover.wheels import WheelMass
from rover.drums import DrumMass
from rover.linkage import LinkageGeometry
from rover.structure import StructureGeometry
from rover.components import GeometryMass
from rover.excavation import Excavation, segment_area, motor_torque
from rover.structure import tube_section
from rover.tip_stability import extended_pair_tip_margin
from rover.calibration import ResponseTable
from examples.empirical_excavation import demonstration_model
from historical_fixtures import HISTORICAL_ACTUATOR_DESIGN


class PhysicsTests(unittest.TestCase):
    def problem(self, **kwargs):
        # Historical limits below use the original tip-mounted actuator fixture.
        # Keep production attachment bounds and the assertions unchanged.
        with patch.dict(DESIGN, {
            **HISTORICAL_ACTUATOR_DESIGN,
            'lin_act_side_arm_offset': (0., -.15, .15, 'm'),
            'lin_act_top_arm_offset': (0., -.15, .15, 'm'),
        }):
            p=build_problem(traversal_mode="fixed", **kwargs)
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def evaluate(self, comp, **changes):
        inputs={n:comp._inputs[n].copy() for n in comp._var_rel_names['input']}
        inputs.update({n:np.array([value]) for n,value in changes.items()})
        outputs={n:0.0 for n in comp._var_rel_names['output']}
        comp.compute(inputs,outputs)
        return outputs

    def test_segment_geometry(self):
        r=.2
        self.assertEqual(segment_area(r,0),0)
        self.assertAlmostEqual(segment_area(r,r),math.pi*r*r/2)
        self.assertAlmostEqual(segment_area(r,3*r),math.pi*r*r)
        # Independent midpoint integration of horizontal strips.
        h=.037
        z=np.linspace(-r,-r+h,10001)
        mid=(z[:-1]+z[1:])/2
        numeric=np.sum(2*np.sqrt(r*r-mid*mid))*(h/10000)
        self.assertAlmostEqual(segment_area(r,h),numeric,places=8)

    def test_reachable_mass_and_no_soil_no_payload(self):
        # Preserve independent regression coverage of the inactive plunge reference.
        p=self.problem(models=ModelSet(excavation=Excavation))
        comp=p.model.excavation
        above=self.evaluate(comp,extended_y=.20)
        self.assertEqual(above['payload_mass'],0)
        self.assertLess(above['dig_reach_margin'],0)
        self.assertTrue(all(math.isfinite(x) for x in above.values()))
        small=self.evaluate(comp,extended_y=.13)
        large=self.evaluate(comp,extended_y=.10)
        self.assertGreater(large['payload_mass'],small['payload_mass'])
        for result in (small,large):
            self.assertLessEqual(result['payload_mass'],result['fresh_volume']*1500)
            self.assertLessEqual(result['payload_mass'],result['storage_capacity']*1250)
        storage=self.evaluate(comp,fill_fraction=.9)
        self.assertAlmostEqual(storage['payload_mass'],p.get_val('payload_mass').item())
        nearly_full=self.evaluate(comp,fill_fraction=.001,extended_y=.10)
        self.assertLess(nearly_full['payload_mass'],large['payload_mass'])
        self.assertAlmostEqual(nearly_full['dig_time'],large['dig_time'])
        # More revolutions at the same fully extended position create no new soil.
        faster=self.evaluate(comp,drum_rpm=60,extended_y=.10)
        self.assertAlmostEqual(faster['payload_mass'],large['payload_mass'])

    def test_storage_cap_allows_partial_fill(self):
        p=self.problem(models=ModelSet(excavation=Excavation))
        comp=p.model.excavation
        shallow=self.evaluate(comp,extended_y=.13)
        deep=self.evaluate(comp,extended_y=.02)
        for result in (shallow,deep):
            expected=4*p.get_val('drum_length').item()*segment_area(
                p.get_val('drum_diameter').item()/2,result['trench_depth'])*p.get_val('soil_density').item()*p.get_val('capture_fraction').item()
            self.assertAlmostEqual(result['reachable_soil_mass'],expected)
        self.assertNotIn('full_fill_margin',MARGINS)
        self.assertLess(shallow['payload_mass'],shallow['target_payload_mass'])
        self.assertAlmostEqual(deep['payload_mass'],deep['target_payload_mass'])

    def test_start_travel_dig_positions_and_short_wheelbase(self):
        p=self.problem()
        v=lambda n:p.get_val(n).item()
        self.assertIn('wheelbase',p.model.get_design_vars())
        self.assertIn('track_width',p.model.get_design_vars())
        self.assertLessEqual(v('stowed_length'),1.5)
        self.assertLessEqual(v('stowed_height'),.75)
        self.assertGreater(v('travel_y'),v('stowed_y'))
        self.assertGreater(v('stowed_y'),v('extended_y'))
        self.assertAlmostEqual(v('wheel_first_margin'),v('travel_y')-v('drum_diameter')/2-v('wheel_diameter'))
        self.assertGreater(v('wheel_first_margin'),0)
        self.assertEqual(v('dig_overlap_time'),0)
        self.assertAlmostEqual(v('actuation_time'),4*v('lowering_time'))
        p.set_val('travel_actuator_length',.60);p.run_model()
        self.assertLess(p.get_val('wheel_first_margin').item(),0)
        p.set_val('travel_actuator_length',.38);p.run_model()
        before=v('wheel_clearance_margin')
        p.set_val('wheelbase',.5);p.run_model()
        self.assertLess(p.get_val('wheel_clearance_margin').item(),before)

    def test_track_width_changes_envelope_and_turn_path(self):
        p=self.problem()
        p.set_val('drum_length',.12)
        p.set_val('track_width',.52)
        p.run_model()
        narrow_width=p.get_val('rover_width').item()
        narrow_turn=p.get_val('turn_time').item()
        p.set_val('track_width',.62)
        p.run_model()
        self.assertAlmostEqual(p.get_val('rover_width').item()-narrow_width,.10)
        self.assertGreater(p.get_val('turn_time').item(),narrow_turn)

    def test_wheel_diameter_screen_matches_trench_depth(self):
        p=self.problem()
        self.assertIn('trench_traverse_margin',MARGINS)
        comp=p.model.trench_traversal
        for wheel,expected in ((.18,-.06),(.24,0),(.30,.06)):
            result=self.evaluate(comp,wheel_diameter=wheel,trench_depth=.24)
            self.assertAlmostEqual(result['trench_traverse_margin'],expected)

    def test_trench_screen_can_be_replaced_independently(self):
        from rover.trench import TrenchTraversal
        class ConservativeTrench(TrenchTraversal):
            def compute(self, inputs, outputs):
                super().compute(inputs,outputs)
                outputs['trench_traverse_margin']-=.02
        reference=self.problem()
        replacement=self.problem(models=ModelSet(trench_traversal=ConservativeTrench))
        self.assertAlmostEqual(reference.get_val('trench_traverse_margin').item()
                               -replacement.get_val('trench_traverse_margin').item(),.02)
        self.assertAlmostEqual(reference.get_val('payload_mass').item(),replacement.get_val('payload_mass').item())

    def test_team_mass_replacements_compose(self):
        class HeavierWheel(WheelMass):
            def compute(self, inputs, outputs):
                super().compute(inputs, outputs)
                outputs['wheel_mass'] += .25

        class HeavierDrum(DrumMass):
            def compute(self, inputs, outputs):
                super().compute(inputs, outputs)
                outputs['drum_mass'] += .50

        class HeavierFrame(StructureGeometry):
            def compute(self, inputs, outputs):
                super().compute(inputs, outputs)
                outputs['structure_mass'] += .30

        class HeavierLinkage(LinkageGeometry):
            def compute(self, inputs, outputs):
                super().compute(inputs, outputs)
                outputs['linkage_mass'] += .20

        class HeavierDrive(GeometryMass):
            def compute(self, inputs, outputs):
                super().compute(inputs, outputs)
                outputs['drive_mass'] += .40
                outputs['dry_mass'] += .40

        reference=self.problem()
        combined=self.problem(models=ModelSet(
            wheel_mass=HeavierWheel, drum_mass=HeavierDrum,
            structure_geometry=HeavierFrame, linkage=HeavierLinkage,
            geometry=HeavierDrive))
        self.assertAlmostEqual(combined.get_val('wheel_mass').item()-reference.get_val('wheel_mass').item(),.25)
        self.assertAlmostEqual(combined.get_val('drum_mass').item()-reference.get_val('drum_mass').item(),.50)
        self.assertAlmostEqual(combined.get_val('structure_mass').item()-reference.get_val('structure_mass').item(),.30)
        self.assertAlmostEqual(combined.get_val('linkage_mass').item()-reference.get_val('linkage_mass').item(),.20)
        self.assertAlmostEqual(combined.get_val('drive_mass').item()-reference.get_val('drive_mass').item(),.40)
        self.assertAlmostEqual(combined.get_val('dry_mass').item()-reference.get_val('dry_mass').item(),1.65)
        self.assertAlmostEqual(combined.get_val('payload_mass').item(),reference.get_val('payload_mass').item())

    def test_speed_and_motor_limits(self):
        p=self.problem()
        fast=self.evaluate(p.model.excavation,drum_rpm=1000)
        self.assertLess(fast['dig_froude_margin'],0)
        self.assertLess(fast['drum_torque_margin'],0)
        rated=100/(3000*math.pi/30)*100*.8
        self.assertAlmostEqual(motor_torque(100,3000,100,.8,30),rated)
        self.assertAlmostEqual(motor_torque(100,3000,100,.8,60),0)
        self.assertLess(motor_torque(100,3000,100,.8,70),0)
        self.assertIn('inlet_depth',p.model.get_design_vars())

    def test_yaw_parallel_axis_and_half_loaded_cg(self):
        p=self.problem()
        comp=p.model.mass_properties
        mass=40.0
        result=self.evaluate(comp,payload_mass=mass)
        x=p.get_val('travel_x').item()
        y=(.25+.06)/2
        local=(3*.14**2+.25**2)/12
        delta=result['loaded_yaw_inertia']-result['empty_yaw_inertia']
        self.assertAlmostEqual(delta,mass*(x*x+y*y+local))
        self.assertAlmostEqual(result['half_yaw_inertia']-result['empty_yaw_inertia'],delta/2)
        self.assertAlmostEqual(result['half_cg_x'],mass/2*x/(p.get_val('dry_mass').item()+mass/2))
        self.assertAlmostEqual(result['loaded_cg_x'],0)

    def test_extended_asymmetric_pair_tip_margin(self):
        dry, payload, wb, track, x = 40., 40., .4, .52, .5
        # Two loaded drums hold half the total payload; mirrored dry hardware
        # cancels when both pairs are at the same extension.
        expected_cg = (payload/2*x)/(dry+payload/2)
        expected = min(wb/2-expected_cg,track/2)
        self.assertAlmostEqual(extended_pair_tip_margin(dry,payload,wb,track,x),expected)
        self.assertAlmostEqual(extended_pair_tip_margin(dry,payload,wb,track,-x),expected)
        self.assertAlmostEqual(extended_pair_tip_margin(dry,0,wb,track,x),wb/2)
        self.assertLess(extended_pair_tip_margin(dry,160,.2,track,.7),0)
        p=self.problem()
        self.assertAlmostEqual(p.get_val('tip_stability_margin').item(),extended_pair_tip_margin(
            p.get_val('dry_mass').item(),p.get_val('payload_mass').item(),
            p.get_val('wheelbase').item(),p.get_val('track_width').item(),
            p.get_val('deposition_x').item()))
        self.assertEqual(p.get_val('tip_worst_mask').item(),3)

    def test_actuator_pivot_moves_independently(self):
        p=self.problem()
        self.assertNotAlmostEqual(p.get_val('actuator_pivot_x').item(),p.get_val('pivot_x').item())
        before=p.get_val('extended_x').item()
        p.set_val('actuator_pivot_x',.18)
        p.run_model()
        self.assertNotAlmostEqual(p.get_val('extended_x').item(),before)
        self.assertAlmostEqual(math.hypot(p.get_val('extended_x').item()-.18,
                                          p.get_val('extended_y').item()-p.get_val('top_pivot_height').item()),
                               p.get_val('actuator_extended_length').item())

    def test_relocation_has_no_trench_input(self):
        p=self.problem()
        self.assertNotIn('trench_depth',p.model.relocation._var_rel_names['input'])
        self.assertGreater(p.get_val('relocation_time').item(),0)
        self.assertGreater(p.get_val('relocation_energy').item(),0)

    def test_tube_limits_and_load_scaling(self):
        area,inertia,polar=tube_section(.05,.002)
        self.assertAlmostEqual(area,math.pi*(.05**2-.046**2)/4)
        self.assertAlmostEqual(polar,2*inertia)
        self.assertGreater(tube_section(.06,.002)[1],inertia)
        p=self.problem()
        low=self.evaluate(p.model.structure_loads,payload_mass=0)
        high=self.evaluate(p.model.structure_loads,payload_mass=100)
        self.assertGreater(high['structure_stress'],low['structure_stress'])
        self.assertGreater(high['structure_deflection'],low['structure_deflection'])

    def test_phase_energy_and_overlap(self):
        p=self.problem()
        v=lambda n:p.get_val(n).item()
        cycle=sum(v(n) for n in ('dig_time','dump_time','travel_time','turn_time','actuation_time','relocation_time','overhead_time'))-v('dig_overlap_time')
        self.assertAlmostEqual(cycle,v('cycle_time'))
        energy=sum(v(n) for n in ('dig_energy','dump_energy','mobility_energy','arm_energy','relocation_energy'))+v('electronics_power')*cycle
        self.assertAlmostEqual(energy/3600,v('cycle_energy'))
        self.assertGreaterEqual(v('peak_power'),v('dig_power')+v('arm_peak_power')+v('electronics_power'))

    def test_table_domains_and_injection(self):
        path=Path(__file__).parents[1]/'examples/data/synthetic_response.json'
        with self.assertRaisesRegex(ValueError,'Synthetic'):
            ResponseTable(path)
        table=ResponseTable(path,allow_synthetic=True)
        conditions={n:s['value'] for n,s in table.data['conditions'].items()}
        self.assertEqual(table.evaluate(.1,conditions),(.59,.16))
        with self.assertRaisesRegex(ValueError,'outside validated'):
            table.evaluate(.4,conditions)
        with self.assertRaisesRegex(ValueError,'fixed-condition'):
            table.evaluate(.1,{**conditions,'drum_diameter':.30})
        malformed=json.loads(path.read_text())
        malformed['rows'][1][0]=0
        with tempfile.TemporaryDirectory() as tmp:
            bad=Path(tmp)/'bad.json'
            bad.write_text(json.dumps(malformed))
            with self.assertRaisesRegex(ValueError,'strictly increasing'):
                ResponseTable(bad,allow_synthetic=True)
        p=self.problem(models=ModelSet(excavation=demonstration_model))
        self.assertGreater(p.get_val('payload_mass').item(),0)
        self.assertEqual(p.model_metadata['evidence']['excavation']['data_kind'],'synthetic')
        with self.assertRaisesRegex(ValueError,'fixed-condition'):
            table.evaluate(.1,{**conditions,'drum_diameter':float('nan')})

    def test_replacement_contract_rejects_missing_outputs_and_sources(self):
        class MissingOutputs(om.ExplicitComponent):
            def setup(self):
                self.add_output('payload_mass',units='kg')
        with self.assertRaisesRegex(ValueError,'missing required output'):
            build_problem(models=ModelSet(excavation=MissingOutputs))
        class MissingSource(Excavation):
            def setup(self):
                super().setup()
                self.add_input('unmeasured_soil_coefficient')
        with self.assertRaisesRegex(ValueError,'No explicit source'):
            build_problem(models=ModelSet(excavation=MissingSource))

    def test_replacement_contract_rejects_unit_scale_change(self):
        class WrongUnits(Excavation):
            OUTPUT_UNITS={**Excavation.OUTPUT_UNITS,'payload_mass':'g'}
        with self.assertRaisesRegex(ValueError,'native units'):
            build_problem(models=ModelSet(excavation=WrongUnits))


if __name__=='__main__':
    unittest.main()
