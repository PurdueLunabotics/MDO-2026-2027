"""Sweep coordinates, physical invariants and truthful candidate selection."""
import copy
import csv
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from rover.mobility import route_profiles
from rover.problem import build_problem
from rover.config import configuration
from rover.alternatives.x44_motor import CURVE
from reporting.report import snapshot
from studies.traversal_fill import (validate_grid, select_attempt, run_cell,
                                    load_study_inputs, resolve_output, DEFAULT_TIMES, DEFAULT_FILLS)
from studies.traversal_fill_view import write_explorer


class TraversalFillStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.values, cls.meta, cls.structure, cls.motors, cls.starts = load_study_inputs(
            Path('design_baseline.json'), Path('design_optimized.json'))

    def evaluate(self, seconds, fill):
        p = build_problem({**self.values, 'traversal_time_one_way': seconds, 'fill_fraction': fill},
                          structure=self.structure, motors=self.motors)
        self.addCleanup(p.cleanup)
        p.run_model()
        self.assertNotIn('traversal_time_one_way', p.design_specs)
        self.assertNotIn('fill_fraction', p.design_specs)
        return snapshot(p)

    def test_complete_grid_endpoints_and_invalid_domains(self):
        times, fills = validate_grid(DEFAULT_TIMES, DEFAULT_FILLS)
        self.assertEqual((times[0], times[-1], fills[0], fills[-1], len(times)*len(fills)), (15, 180, .2, .9, 96))
        for times, fills in (([], [.4]), ([15,15], [.4]), ([14], [.4]), ([181], [.4]),
                             ([45], [.1]), ([45], [1]), ([math.nan], [.4]), ([45], [math.inf]), ([True],[.4])):
            with self.subTest(times=times, fills=fills), self.assertRaises(ValueError):
                validate_grid(times, fills)

    def test_default_output_resumes_matching_run_and_preserves_legacy_results(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            legacy = root/'study.json'
            legacy.write_text(json.dumps({'fingerprint':'legacy'}), encoding='utf-8')
            before = legacy.read_bytes()
            with patch('studies.traversal_fill.DEFAULT_OUTPUT_ROOT', root):
                first = resolve_output(None, 'a'*64)
                first.mkdir(parents=True)
                (first/'study.json').write_text(json.dumps({'fingerprint':'a'*64}), encoding='utf-8')
                self.assertEqual(resolve_output(None, 'a'*64), first)
                changed = resolve_output(None, 'b'*64)
                self.assertNotEqual(changed, first)
                self.assertFalse(changed.exists())
                self.assertEqual(first.parent, root/'runs')
                self.assertEqual(legacy.read_bytes(), before)

    def test_explicit_output_rejects_stale_manifest_and_unrequested_cells(self):
        # Check the entire folder, even when a requested grid would not overlap
        # the old cases, so a new manifest cannot hide mixed-model results.
        for record in ('study.json', 'cases/t15_f0p2/cell.json'):
            with self.subTest(record=record), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                stale = root/record
                stale.parent.mkdir(parents=True, exist_ok=True)
                stale.write_text(json.dumps({'fingerprint':'old'}), encoding='utf-8')
                before = stale.read_bytes()
                with self.assertRaisesRegex(ValueError, 'Omit --output'):
                    resolve_output(root, 'new')
                self.assertEqual(stale.read_bytes(), before)
                self.assertEqual(resolve_output(root, 'old'), root)
                self.assertEqual(resolve_output(root/'fresh', 'new'), root/'fresh')

    def test_default_fingerprint_prefix_collision_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch('studies.traversal_fill.DEFAULT_OUTPUT_ROOT', Path(folder)):
                first = resolve_output(None, 'a'*16+'b'*48)
                first.mkdir(parents=True)
                (first/'study.json').write_text(json.dumps({'fingerprint':'a'*16+'b'*48}), encoding='utf-8')
                with self.assertRaisesRegex(ValueError, 'Stale study/cache'):
                    resolve_output(None, 'a'*16+'c'*48)

    def test_route_distance_and_inverse_time_scaling_are_preserved(self):
        v = self.values
        a, b = [route_profiles(v['track_width'], v['travel_speed'], v['acceleration'],
                              v['turn_speed_fraction'], v['turn_accel_fraction'], t) for t in (15,180)]
        self.assertAlmostEqual(sum(p[2] for p in a), 15)
        self.assertAlmostEqual(sum(p[2] for p in b), 180)
        self.assertAlmostEqual(sum(p[1] for p in a if p[0]=='straight'), 9)
        for fast, slow in zip(a,b):
            self.assertEqual(fast[:2], slow[:2])
            self.assertAlmostEqual(slow[2], 12*fast[2])
            self.assertAlmostEqual(fast[3], 12*slow[3])
            self.assertAlmostEqual(fast[4], 144*slow[4])
        for t in (15,180):
            outputs = self.evaluate(t,.4)['outputs']
            self.assertAlmostEqual(outputs['travel_time']['value'], 2*t)
            self.assertAlmostEqual(outputs['route_translation_time']['value']+outputs['route_turn_time']['value'], 2*t)

    def test_fill_is_a_four_drum_capacity_ceiling_not_forced_payload(self):
        v = self.values
        capacity = 4*math.pi*(v['drum_diameter']/2-v['cut_depth'])**2*v['drum_length']*v['usable_volume_fraction']
        for fill in (.2,.9):
            out = self.evaluate(45,fill)['outputs']
            target = capacity*fill*v['stored_soil_density']
            self.assertAlmostEqual(out['target_payload_mass']['value'], target)
            self.assertAlmostEqual(out['payload_mass']['value'], min(target,out['reachable_soil_mass']['value']))

    def test_fifteen_second_profile_exceeds_even_no_load_hardware_speed_cap(self):
        design, _ = configuration(self.structure,self.motors)
        v = self.values
        # The narrowest permitted track minimizes time spent in route turns.
        profiles = route_profiles(design['track_width'][1], v['travel_speed'], v['acceleration'],
                                  v['turn_speed_fraction'], v['turn_accel_fraction'], 15)
        required = max(p[3] for p in profiles if p[0]=='straight')
        maximum_free_speed = CURVE.free_rpm/v['drive_gear_ratio']*math.pi*design['wheel_diameter'][2]/60
        self.assertGreater(required,maximum_free_speed)
        self.assertGreater(15*required/maximum_free_speed,17)

    def record(self, success, feasible, score, margin=1):
        return {'optimizer_success':success, 'status':'converged_feasible' if success and feasible else 'failed_or_infeasible',
                'model_metadata':{'configuration':{'margins':{'test_margin':(1,'m')}}},
                'final':{'feasible':feasible, 'outputs':{'construction_points':{'value':score}},
                         'constraints':{'test_margin':{'margin':margin,'passed':feasible,'units':'m'}}}}

    def test_failed_or_infeasible_high_scores_never_win(self):
        low = self.record(True,True,10)
        high = self.record(True,True,20)
        failed = self.record(False,True,1000)
        infeasible = self.record(True,False,2000,-.1)
        self.assertIs(select_attempt([failed,low,infeasible,high]), high)
        self.assertIs(select_attempt([infeasible,failed]), failed)
        self.assertFalse(failed['optimizer_success'])
        self.assertIsNone(select_attempt([{'optimizer_success':False,'status':'error'}]))

    def test_exhausted_solver_is_saved_as_failure_without_touching_source(self):
        before = {p:p.read_bytes() for p in (Path('design_baseline.json'),Path('design_optimized.json'))}
        values = copy.deepcopy(self.values)
        with tempfile.TemporaryDirectory() as folder:
            job = {'seconds':15., 'fill':.9, 'output':folder, 'source_inputs':values,
                   'source_name':'design_baseline.json', 'source_revision':'test', 'source_sha256':self.meta['sha256'],
                   'structure':self.structure, 'motors':self.motors, 'maxiter':1,
                   'starts':[('baseline',{})], 'fingerprint':'test'}
            row = run_cell(job)
            self.assertEqual(row['status'],'failed_or_infeasible')
            self.assertFalse(row['optimizer_success'])
            record = json.loads((Path(folder)/row['analysis']).read_text())
            self.assertFalse(record['optimizer_success'])
            self.assertEqual(record['final']['assumptions']['traversal_time_one_way']['value'],15.)
            self.assertEqual(record['final']['assumptions']['fill_fraction']['value'],.9)
            self.assertTrue((Path(folder)/row['report']).exists())
        self.assertEqual(values,self.values)
        for p, raw in before.items():
            self.assertEqual(p.read_bytes(),raw)

    def test_fixed_baseline_never_runs_optimizer_and_retains_failed_checks_and_scores(self):
        before = {p:p.read_bytes() for p in (Path('design_baseline.json'),Path('design_optimized.json'))}
        expected = self.evaluate(45, .9)
        with tempfile.TemporaryDirectory() as folder:
            job = {'seconds':45., 'fill':.9, 'output':folder, 'source_inputs':copy.deepcopy(self.values),
                   'source_name':'design_baseline.json', 'source_revision':'test', 'source_sha256':self.meta['sha256'],
                   'structure':self.structure, 'motors':self.motors, 'maxiter':2000,
                   'starts':[('baseline',{})], 'fingerprint':'test-fixed', 'mode':'fixed_design'}
            with patch('openmdao.core.problem.Problem.run_driver', side_effect=AssertionError('Optimizer must not run')) as driver:
                row = run_cell(job)
                driver.assert_not_called()
            self.assertEqual(row['status'], 'baseline')
            self.assertIsNone(row['optimizer_success'])
            self.assertFalse(row['feasible'])
            self.assertIn('empty_slope_traversal_margin', row['failed_checks'])
            self.assertIn('loaded_slope_traversal_margin', row['failed_checks'])
            self.assertEqual(row['outputs']['construction_points'], expected['outputs']['construction_points']['value'])
            self.assertEqual(row['design'], {n:v['value'] for n,v in expected['design'].items()})
            record = json.loads((Path(folder)/row['analysis']).read_text())
            self.assertEqual(record['report_context']['mode'], 'fixed_design')
            self.assertEqual(record['baseline'], record['final'])
            self.assertTrue((Path(folder)/row['report']).exists())
            study = {'mode':'fixed_design', 'rows':[row], 'source_inputs':self.values, 'fingerprint':'test-fixed'}
            write_explorer(study, folder)
            with (Path(folder)/'summary.csv').open(newline='') as stream:
                saved = next(csv.DictReader(stream))
            self.assertEqual(saved['mode'], 'fixed_design')
            self.assertEqual(saved['feasible'], 'False')
            self.assertEqual(saved['optimizer_success'], '')
            self.assertEqual(float(saved['construction_points']), row['outputs']['construction_points'])
        for p, raw in before.items():
            self.assertEqual(p.read_bytes(), raw)

    def test_fixed_baseline_exception_does_not_invent_a_score_or_geometry(self):
        with tempfile.TemporaryDirectory() as folder:
            job = {'seconds':45., 'fill':.4, 'output':folder, 'source_inputs':self.values,
                   'source_name':'design_baseline.json', 'source_revision':'test', 'source_sha256':self.meta['sha256'],
                   'structure':self.structure, 'motors':self.motors, 'maxiter':2000,
                   'starts':[('baseline',{})], 'fingerprint':'test-fixed-error', 'mode':'fixed_design'}
            with patch('rover.problem.build_problem', side_effect=ValueError('Actual invalid input')):
                row = run_cell(job)
            self.assertEqual(row['status'], 'error')
            self.assertNotIn('outputs', row)
            self.assertIsNone(row['analysis'])
            write_explorer({'mode':'fixed_design', 'rows':[row], 'source_inputs':self.values}, folder)
            report = (Path(folder)/row['report']).read_text(encoding='utf-8')
            self.assertIn('No usable baseline evaluation', report)
            self.assertIn('Actual invalid input', report)
            self.assertNotIn('<svg', report)

    def test_missing_final_state_gets_diagnostics_without_invented_geometry(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            (output/'cases/t15_f0p7').mkdir(parents=True)
            row = {'id':'t15_f0p7','status':'error','traversal_time_s':15,'fill_fraction':.7,
                   'analysis':None,'report':None,'attempts':[{'start':'baseline','status':'error',
                                                            'construction_points':None}],
                   'errors':['Actual solver exception: <invalid state>']}
            study = {'rows':[row],'source_inputs':self.values,'fingerprint':'test'}
            write_explorer(study,output)
            html=(output/row['report']).read_text(encoding='utf-8')
            self.assertIn('No usable optimized design',html)
            self.assertIn('&lt;invalid state&gt;',html)
            self.assertNotIn('<svg',html)
            self.assertIsNone(row['analysis'])
            self.assertEqual(row['status'],'error')
            self.assertTrue((output/'summary.csv').exists())
            self.assertEqual(json.loads((output/'study.json').read_text())['rows'][0]['report'],row['report'])
