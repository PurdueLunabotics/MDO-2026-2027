"""Catalog reproduction, electrical/work limits, failure domains and integration."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_totals
from openmdao.visualization.n2_viewer.n2_viewer import _get_viewer_data
from rover.alternatives.pa04_actuator import (
    INCH, LB, RATED_FORCE, MASS_INTERCEPT_LB, MASS_SLOPE_LB_PER_IN,
    catalog_geometry, load_performance, cycle_performance, verify_source_data,
)
from rover.config import configuration, DESIGN, ASSUMPTIONS
from rover.problem import build_problem
from reporting.case_inputs import read_case
from reporting.report import snapshot
from reporting.html_report import write_report
from studies.traversal_fill import load_study_inputs

ROOT = Path(__file__).resolve().parents[1]


class PA04Tests(unittest.TestCase):
    def problem(self, changes=None, **kwargs):
        p = build_problem(changes, actuators="pa04-900", **kwargs)
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def test_length_table_and_added_allowance(self):
        with (ROOT/'data/actuators/pa04_lengths.csv').open(newline='') as stream:
            for row in csv.DictReader(stream):
                stroke = float(row['stroke_in'])*INCH
                if 2*INCH <= stroke <= 40*INCH:
                    lo, hi, _ = catalog_geometry(stroke)
                    self.assertAlmostEqual(lo, float(row['retracted_in'])*INCH+.1613408)
                    self.assertAlmostEqual(hi, float(row['extended_in'])*INCH+.1613408)
                    self.assertAlmostEqual(hi-lo, stroke)
        # Fractional, unavailable catalog size remains continuous for the solver.
        a, b, _ = catalog_geometry(7.123*INCH, .20)
        self.assertAlmostEqual(a, (7.123+7.87)*INCH+.20)
        c, d, _ = catalog_geometry(7.123*INCH, 0.)
        self.assertAlmostEqual(a-c, .20)
        self.assertAlmostEqual(b-d, .20)

    def test_versioned_fit_and_held_out_catalog_errors(self):
        verify_source_data()
        spec = importlib.util.spec_from_file_location("pa04_fit", ROOT/'data/actuators/fit_pa04.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        result = module.fit()
        self.assertAlmostEqual(result['intercept_lb'], MASS_INTERCEPT_LB)
        self.assertAlmostEqual(result['slope_lb_per_in'], MASS_SLOPE_LB_PER_IN)
        self.assertEqual(result['errors']['validation']['count'], 3)
        self.assertLess(result['errors']['validation']['max_abs_lb'], .16)
        self.assertAlmostEqual(catalog_geometry(10*INCH)[2]/LB, MASS_INTERCEPT_LB+10*MASS_SLOPE_LB_PER_IN)

    def test_load_curve_endpoints_and_independent_work_limit(self):
        self.assertEqual(load_performance(0), (.47*INCH, 4.))
        self.assertAlmostEqual(load_performance(RATED_FORCE)[0], .39*INCH)
        self.assertEqual(load_performance(RATED_FORCE)[1], 14.)
        # Interior point from the straight chart, independently interpolated.
        speed, current = load_performance(RATED_FORCE/2)
        self.assertAlmostEqual(speed, .43*INCH)
        self.assertEqual(current, 9.)
        for force in np.linspace(0, RATED_FORCE, 25):
            speed, current = load_performance(force)
            self.assertGreater(12*current, force*speed)
        motion = .21
        result = cycle_performance(RATED_FORCE/2, motion, motion/(.39*INCH))
        electrical = 2*12*(9*motion/(.43*INCH)+14*motion/(.39*INCH))
        self.assertAlmostEqual(result['arm_energy'], electrical)
        self.assertGreater(result['arm_energy'], 2*(RATED_FORCE/2)*motion)
        self.assertEqual(result['arm_peak_power'], 336.)
        zero = cycle_performance(0, 0, 0)
        self.assertEqual(zero['actuation_time'], 0)
        self.assertEqual(zero['arm_energy'], 0)

    def test_strict_helpers_and_fixed_hardware(self):
        for stroke in (.01, 1.1, math.nan):
            with self.assertRaises(ValueError):
                catalog_geometry(stroke)
        for allowance in (-.1, math.inf):
            with self.assertRaises(ValueError):
                catalog_geometry(.2, allowance)
        for force in (-1., RATED_FORCE+1., math.nan):
            with self.assertRaises(ValueError):
                load_performance(force)
        for changes in ({'actuator_force_rating': 3500}, {'actuator_speed': .066}):
            with self.assertRaisesRegex(ValueError, 'freezes'):
                build_problem(changes, actuators='pa04-900')
        design, assumptions = configuration(actuators='pa04-900')
        self.assertNotIn('actuator_force_rating', design)
        self.assertNotIn('actuator_speed', design)
        self.assertIn('actuator_catalog_stroke', design)
        self.assertEqual(assumptions['actuator_force_rating'][0], RATED_FORCE)
        self.assertEqual(DESIGN['actuator_speed'][0], .025)
        self.assertNotIn('actuator_installation_allowance', ASSUMPTIONS)

    def test_mass_energy_timing_connections_and_inactive_reference_coefficients(self):
        p = self.problem(structure='plate-frame', motors='x44-standard')
        v = lambda n: p.get_val(n).item()
        self.assertAlmostEqual(v('linkage_mass'), 2*(v('actuator_unit_mass')+v('arm_linear_mass')*v('arm_length')))
        self.assertAlmostEqual(v('dry_mass'), sum(v(n) for n in ('fixed_mass', 'structure_mass', 'wheel_mass', 'drive_mass', 'drum_mass', 'battery_mass', 'linkage_mass')))
        self.assertAlmostEqual(v('actuation_time'), v('lowering_time')+v('actuator_raising_time'))
        self.assertAlmostEqual(v('dig_overlap_time'), v('lowering_time'))
        self.assertAlmostEqual(v('cycle_time'), sum(v(n) for n in ('dig_time', 'dump_time', 'travel_time', 'turn_time', 'actuation_time', 'relocation_time', 'overhead_time'))-v('lowering_time'))
        energy = sum(v(n) for n in ('dig_energy', 'dump_energy', 'mobility_energy', 'relocation_energy', 'arm_energy'))
        self.assertAlmostEqual(v('cycle_energy')*3600, energy+v('electronics_power')*v('cycle_time'))
        before = [v(n) for n in ('linkage_mass', 'arm_energy', 'arm_peak_power', 'actuation_time')]
        for name in ('actuator_efficiency', 'actuator_fixed_mass', 'actuator_mass_per_force', 'actuator_mass_per_power', 'actuator_mass_per_length'):
            p.set_val(name, v(name)*1.1)
        p.run_model()
        np.testing.assert_allclose([v(n) for n in ('linkage_mass', 'arm_energy', 'arm_peak_power', 'actuation_time')], before)
        conns = p.model._conn_global_abs_in2out
        self.assertEqual(conns['excavation.lowering_time'], 'linkage.lowering_time')
        self.assertEqual(conns['mission.actuation_time'], 'linkage_loads.actuation_time')
        declared = _get_viewer_data(p)['declare_partials_list']
        self.assertIn('linkage_loads.actuation_time > linkage_loads.payload_mass', declared)

    def test_overload_domains_stay_failed_and_packaging_is_diagnostic(self):
        p = self.problem({'breakout_force': 10000.})
        report = snapshot(p)
        self.assertFalse(report['constraints']['actuator_curve_margin']['passed'])
        self.assertEqual(p.get_val('actuator_curve_valid').item(), 0)
        self.assertFalse(report['feasible'])
        self.assertTrue(all(math.isfinite(x['value']) for x in report['outputs'].values()))
        # Even a domain excursion smaller than normal report tolerances fails.
        v = lambda n: p.get_val(n).item()
        weight = (v('drum_mass')/2+v('payload_mass')/2+v('arm_linear_mass')*v('arm_length')/2)*v('gravity')
        p.set_val('breakout_force', (RATED_FORCE+1e-4)/v('actuator_leverage_max')-weight)
        p.run_model()
        self.assertFalse(snapshot(p)['constraints']['actuator_curve_margin']['passed'])
        p.set_val('actuator_catalog_stroke', 40*INCH+1e-8)
        p.run_model()
        self.assertFalse(snapshot(p)['constraints']['actuator_stroke_domain_margin']['passed'])
        p.set_val('actuator_catalog_stroke', .01)
        p.run_model()
        self.assertLess(p.get_val('actuator_stroke_domain_margin').item(), 0)
        values, meta = read_case(ROOT/'design_baseline.json')
        baseline = build_problem(values, structure=meta['structure'], motors=meta['motors'], actuators=meta['actuators'])
        self.addCleanup(baseline.cleanup)
        baseline.run_model()
        self.assertNotIn('actuator_body_margin', snapshot(baseline)['constraints'])
        self.assertIn('actuator_body_margin', baseline.model.linkage.OUTPUT_UNITS)
        self.assertFalse(snapshot(baseline)['feasible'])
        with self.assertRaisesRegex(ValueError, 'hardware profiles'):
            load_study_inputs(ROOT/'design_baseline.json', ROOT/'design_optimized.json')

    def test_smooth_total_derivatives_and_dependency_audit(self):
        p = self.problem({'actuator_catalog_stroke': .2032})
        checked = p.check_totals(of=['actuation_time', 'arm_energy', 'linkage_mass', 'actuator_body_margin', 'actuator_catalog_retracted_length', 'actuator_catalog_extended_length'],
                                 wrt=['actuator_catalog_stroke', 'arm_length', 'travel_actuator_length'],
                                 method='fd', form='central', step=3e-5, step_calc='rel_avg', out_stream=None)
        assert_check_totals(checked, atol=1e-3, rtol=2e-3)
        for comp in (p.model.linkage, p.model.linkage_loads):
            inputs = {n: comp._inputs[n].copy() for n in comp._var_rel_names['input']}
            base = {}
            comp.compute(inputs, base)
            for name in inputs:
                varied = dict(inputs)
                varied[name] = inputs[name]+1e-4*max(1, abs(inputs[name].item()))
                trial = {}
                comp.compute(varied, trial)
                for output in base:
                    if name not in comp.DEPENDENCIES[output]:
                        self.assertAlmostEqual(float(np.asarray(base[output]).item()), float(np.asarray(trial[output]).item()), places=9, msg=f'{comp.pathname}.{output} omits {name}')

    def test_report_keeps_profile_and_domain_notes(self):
        p = self.problem()
        state = snapshot(p)
        record = {'baseline': state, 'final': state, 'status': 'baseline',
                  'optimizer_success': None, 'model_metadata': p.model_metadata}
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/'actuator.json'
            write_report(record, output)
            html = output.with_suffix('.html').read_text(encoding='utf-8')
            self.assertIn('900 lbf / 12 V', html)
            self.assertIn('gravity-assisted lowering', html)
            self.assertIn('actuator_catalog_retracted_length', html)
            self.assertIn('Actuator packaging is not enforced', html)

    def test_combined_cli_freeze_calculated_timing_and_ignored_constraint(self):
        frozen = ['wheel_diameter', 'wheel_width', 'travel_actuator_length',
                  'actuator_retracted_length', 'actuator_extended_length', 'actuator_catalog_stroke']
        source = ROOT/'design_baseline.json'
        before = source.read_bytes()
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/'combined.json'
            result = subprocess.run([
                sys.executable, str(ROOT/'main.py'), '--inputs', str(source), '--optimize',
                '--freeze', *frozen, '--set', 'empty_slope_degrees=0', '--set', 'loaded_slope_degrees=0',
                '--ignore-constraint', 'turn_traction_margin', '--traversal-mode', 'calculated',
                '--maxiter', '1', '--output', str(output)], cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(result.returncode, 1, result.stdout+result.stderr)
            record = json.loads(output.read_text(encoding='utf-8'))
            self.assertEqual(record['model_metadata']['actuator_profile'], 'pa04-900')
            self.assertEqual(record['model_metadata']['traversal_mode'], 'calculated')
            self.assertEqual(set(record['model_metadata']['configuration']['frozen_design']), set(frozen))
            self.assertFalse(record['final']['constraints']['turn_traction_margin']['enforced_in_optimization'])
            self.assertNotIn('actuator_body_margin', record['final']['constraints'])
            self.assertIn('actuator_body_margin', record['final']['outputs'])
            for name in frozen:
                self.assertEqual(record['baseline']['assumptions'][name], record['final']['assumptions'][name])
            # Recorded cases must retain PA-04 outputs when stroke is an assumption.
            cases = list(om.CaseReader(str(output.with_suffix('.sql'))).get_cases('driver', recurse=False))
            self.assertTrue(cases)
            state = snapshot(cases[0], configuration=record['model_metadata']['configuration'])
            self.assertIn('actuator_unit_mass', state['outputs'])
            self.assertIn('outbound_travel_time', state['outputs'])
            self.assertIn('PA-04 actuators', output.with_suffix('.html').read_text(encoding='utf-8'))
        self.assertEqual(before, source.read_bytes())

    def test_catalog_actuators_compose_with_physical_arm_extension(self):
        ordinary = self.problem(structure='plate-frame')
        extended = self.problem(structure='plate-frame', attachment_max=1.5)
        a, b = snapshot(ordinary), snapshot(extended)
        for name, value in a['outputs'].items():
            self.assertEqual(value, b['outputs'][name], name)
        v = lambda n: extended.get_val(n).item()
        # Preserve the drum poses while moving the actuator pin to f=1.2.
        fraction = 1.2
        from rover.linkage import attachment_point
        for pose, length in (('travel', 'travel_actuator_length'), ('stowed', 'actuator_retracted_length'), ('extended', 'actuator_extended_length')):
            x, z = attachment_point(v('pivot_x'), v('bottom_pivot_height'), v(pose+'_x'), v(pose+'_y'), (fraction-1)*v('arm_length'))
            extended.set_val(length, math.hypot(x-v('actuator_pivot_x'), z-v('top_pivot_height')))
        extended.set_val('lin_act_side_arm_offset', (fraction-1)*v('arm_length'))
        extended.run_model()
        expected_mass = 2*(v('actuator_unit_mass')+fraction*v('arm_length')*v('arm_linear_mass'))
        expected_force = ((v('drum_mass')+v('payload_mass'))/2
                          +v('arm_linear_mass')*fraction**2*v('arm_length')/2)*v('gravity')
        expected_force = (expected_force+v('breakout_force'))*v('actuator_leverage_max')
        self.assertAlmostEqual(v('linkage_mass'), expected_mass)
        self.assertAlmostEqual(v('required_actuator_force'), expected_force)
        self.assertAlmostEqual(v('actuation_time'), v('lowering_time')+v('actuator_raising_time'))
        # A PA-04 seed retains the frozen force even for an inward pin.
        from studies.linkage_multistart import geometry_seed, make_problem
        seed = geometry_seed({'final': a, 'model_metadata': ordinary.model_metadata}, .9, 'matched', 0.)
        self.assertEqual(seed['actuator_force_rating'], RATED_FORCE)
        study = make_problem(seed, actuators='pa04-900', maxiter=1)
        self.addCleanup(study.cleanup)
        study.run_model()
        self.assertEqual(study.get_val('actuator_force_rating').item(), RATED_FORCE)


if __name__ == '__main__':
    unittest.main()
