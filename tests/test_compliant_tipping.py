"""Independent static equilibrium, spring/stop limits and invalid-domain tests."""
from dataclasses import replace
import math
import unittest

import numpy as np
from openmdao.utils.assert_utils import assert_check_totals
from openmdao.visualization.n2_viewer.n2_viewer import _get_viewer_data

from rover.alternatives.compliant_tipping import (
    StaticWheelModel, spring_displacement, axis_reserves, coupled_slope_screen,
)
from rover.alternatives.traversal_stability import TraversalStability, slope_support, articulated_cg, wheel_reactions, rock_path_screen
from rover.alternatives.obstacle_stability import raised_wheel_pose
from rover.alternatives.compliant_tipping import CompliantPlateTraversalStability, CompliantObstacleStability, EXTRA_OUTPUTS
from rover.models import ModelSet, plate_frame_models, compliant_plate_frame_models
from rover.problem import build_problem
from rover.excavation import Excavation
from reporting.report import snapshot


def wheel_values(**changes):
    """Synthetic inventory and prescribed wheel law, not measured data."""
    values = dict(wheelbase=.5, track_width=.57, wheel_diameter=.3, rocker_pivot_offset=.08,
                loaded_sprung_mass=60., loaded_sprung_cg_x=0., loaded_sprung_cg_z=.43,
                wheel_mass=12., drive_mass=0., rocker_mass=4., gravity=9.80665,
                dry_mass=36., payload_mass=40., fill_fraction=.9, tipping_reference_fill_fraction=.9,
                tipping_radius_deflection_fraction=.2, wheel_compression_hard_stop=.064,
                grouser_height=.018)
    return {**values, **changes}


def model(**changes):
    return replace(StaticWheelModel.from_values(wheel_values()), **changes)


def rock_screen(m, height, **options):
    values = dict(wheel_diameter=2*m.radius, rocker_pivot_offset=m.pivot_offset,
                  wheelbase=m.wheelbase, track_width=m.track, obstacle_height=height,
                  wheel_mass=m.wheel_drive_mass, drive_mass=0.)
    return rock_path_screen(values, wheel_model=m, **options)


class StaticWheelPhysicsTests(unittest.TestCase):
    def test_drive_bracket_height_survives_compressed_pose_and_conserves_mass(self):
        base=StaticWheelModel.from_values(wheel_values(wheel_mass=6.,drive_mass=6.))
        raised=replace(base,drive_cg_offset=.05842)
        a,b=base.solve(),raised.solve()
        self.assertEqual(base.mass,raised.mass)
        self.assertAlmostEqual(b['cg'][2]-a['cg'][2],6*.05842/base.mass)
        np.testing.assert_allclose(b['normal'],raised.mass*raised.gravity/4,atol=1e-9)
        tilted=raised.solve(angle=math.radians(20),heading=.3)
        self.assertLess(tilted['force_error'],1e-8)
        self.assertTrue(tilted['equation_valid'])

    def test_linear_spring_then_hard_stop_preserves_extra_force(self):
        k, stop = 11000., .064
        for load, expected in ((0., 0.), (330., .03), (704., stop), (1000., stop)):
            delta, reaction = spring_displacement(load, k, stop)
            self.assertAlmostEqual(delta, expected)
            self.assertAlmostEqual(k*delta+reaction, load)
        self.assertAlmostEqual(spring_displacement(1000., k, stop)[1], 296.)
        self.assertEqual(spring_displacement(1000., math.inf, stop), (0., 0.))

    def test_flat_limit_exact_quarter_load_and_reference_deflection(self):
        m = model()
        p = m.solve()
        np.testing.assert_allclose(p['normal'], m.mass*m.gravity/4, atol=1e-10)
        np.testing.assert_allclose(p['compression'], .2*m.radius, atol=1e-12)
        self.assertAlmostEqual(p['pitch'], 0.)
        self.assertAlmostEqual(p['roll'], 0.)
        self.assertTrue(p['contact_valid'])
        # Whole inventory's original CG, with each body/suspension group once.
        nominal = (60*.43+12*.15+4*(.15+.08/2))/76
        self.assertAlmostEqual(p['cg'][2], nominal-.03)

    def test_rigid_limit_matches_original_slope_and_rock_geometry(self):
        m = model(stiffness=math.inf)
        flat = m.solve()
        angle = math.radians(20)
        expected = slope_support(m.wheelbase, m.track, 0., flat['cg'][2], angle)
        pitch = axis_reserves(m.solve(angle, 0.))[0]
        roll = axis_reserves(m.solve(angle, math.pi/2))[1]
        self.assertAlmostEqual(pitch, expected['slope_pitch_tip_margin'], places=11)
        self.assertAlmostEqual(roll, expected['slope_roll_tip_margin'], places=11)
        for height in (0., .1, .25):
            old = raised_wheel_pose(m.wheelbase, m.track, m.radius, m.pivot_offset, height)
            old_cg = articulated_cg(old, m.radius, m.pivot_offset, 60., 0., .43, 12., 4., .5)
            old_loads, _ = wheel_reactions(old, old_cg, 60., 12., 4., m.gravity, .5)
            current = m.solve(terrain=np.array((height, 0., 0., 0.)))
            np.testing.assert_allclose(current['centers'], old['centers'], atol=1e-10)
            np.testing.assert_allclose(current['cg'], old_cg, atol=1e-10)
            np.testing.assert_allclose(current['normal'], old_loads, atol=1e-8)

    def test_downhill_compression_changes_body_angle_and_cg(self):
        m = model()
        angle = math.radians(20)
        pitch = m.solve(angle, 0.)
        roll = m.solve(angle, math.pi/2)
        self.assertGreater(pitch['compression'][0], pitch['compression'][2])
        self.assertLess(pitch['pitch'], 0.)
        self.assertGreater(pitch['cg'][0], 0.)
        self.assertAlmostEqual(m.wheelbase*math.sin(pitch['pitch']),
                               pitch['compression'][2]-pitch['compression'][0])
        self.assertGreater(roll['compression'][0], roll['compression'][1])
        self.assertLess(roll['roll'], 0.)
        self.assertGreater(roll['cg'][1], 0.)
        self.assertAlmostEqual(m.track*math.sin(roll['roll']),
                               roll['compression'][1]-roll['compression'][0])
        # Equal-heave-only benefit is not the coupled slope result.
        uniform_height = m.solve()['cg'][2]
        uniform = m.wheelbase/2-uniform_height*math.tan(angle)
        self.assertLess(axis_reserves(pitch)[0], uniform)

    def test_independent_full_force_moment_and_differential_balance(self):
        m = model()
        for angle, heading, terrain in ((.3, .7, np.zeros(4)),
                                        (0., 0., np.array((.25, 0., 0., 0.)))):
            p = m.solve(angle, heading, terrain)
            direction = -p['gravity']/(-p['gravity'][2])
            forces = p['normal'][:, None]*direction
            np.testing.assert_allclose(forces.sum(axis=0)+m.mass*p['gravity'], 0., atol=1e-8)
            np.testing.assert_allclose(np.cross(p['contacts']-p['cg'], forces).sum(axis=0), 0., atol=1e-8)
            axis = np.array((0., math.cos(p['roll']), math.sin(p['roll'])))
            torques = []
            for sy, indices in ((1, [0, 2]), (-1, [1, 3])):
                pivot = np.array(p['shaft_origin'])+sy*m.track/2*axis
                wheel_center = p['centers'][indices]
                rocker_cg = (1-m.rocker_fraction)*pivot+m.rocker_fraction*wheel_center.mean(axis=0)
                torque = np.cross(p['contacts'][indices]-pivot, forces[indices]).sum(axis=0)
                torque += np.cross(wheel_center-pivot, m.wheel_drive_mass/4*p['gravity']).sum(axis=0)
                torque += np.cross(rocker_cg-pivot, m.rocker_mass/2*p['gravity'])
                torques.append(float(torque @ axis))
            self.assertAlmostEqual(*torques, places=8)
            self.assertLess(p['pose_error'], 1e-9)

    def test_stops_engage_without_turning_overload_into_force_clipping(self):
        # Reduced travel makes the stop active at a fully physical contact pose.
        m = model(hard_stop=.04)
        p = m.solve(math.radians(20), math.pi/4)
        self.assertTrue(p['contact_valid'])
        self.assertGreater(max(p['stop_reaction']), 0.)
        for force, delta, stop_force in zip(p['normal'], p['compression'], p['stop_reaction']):
            self.assertLessEqual(delta, .04+1e-12)
            self.assertAlmostEqual(force, m.stiffness*delta+stop_force, places=8)
        self.assertAlmostEqual(sum(p['normal']), m.mass*m.gravity*math.cos(math.radians(20)))

    def test_unloading_is_separate_from_global_tip_and_stays_failed(self):
        m = model()
        tip = m.limit(math.pi/6, 'tip')
        load = m.limit(math.pi/6, 'load')
        self.assertTrue(tip['valid'] and load['valid'])
        self.assertLess(load['angle'], tip['angle'])
        p = m.solve((tip['angle']+load['angle'])/2, math.pi/6)
        self.assertGreater(p['tip_margin'], 0.)
        self.assertLess(min(p['normal']), 0.)
        self.assertFalse(p['contact_valid'])
        self.assertTrue(p['guarded'])
        # No negative spring compression and no invented tensile wheel support.
        self.assertEqual(p['compression'][int(np.argmin(p['normal']))], 0.)

    def test_regular_branch_rejects_tall_soft_unstable_flat_root(self):
        m = model(sprung_z=3., stiffness=6000.)
        p = m.solve()
        self.assertGreater(p['tip_margin'], 0.)
        self.assertGreater(min(p['normal']), 0.)
        self.assertLess(p['branch_margin'], 0.)
        self.assertLess(p['equilibrium_margin'], 0.)
        self.assertFalse(p['equation_valid'])

    def test_near_zero_angle_seed_does_not_break_height_compatibility(self):
        m = model(radius=.09, wheelbase=.65, track=.699)
        initial = m.solve()['q']
        initial[1:] = (1e-23, -1e-22, 1e-24)
        for height in (.0075, .15, .3):
            p = m.solve(terrain=np.array((height, 0., 0., 0.)), initial=initial)
            self.assertTrue(p['equation_valid'])
            self.assertLess(p['pose_error'], 1e-9)
            np.testing.assert_allclose(p['centers'][:, 2]+p['compression'],
                                       np.array((height, 0., 0., 0.))+m.radius, atol=1e-9)

    def test_mirrored_raises_and_dense_heading_search(self):
        m = model()
        cases = []
        for wheel in range(4):
            terrain = np.zeros(4); terrain[wheel] = .25
            cases.append(m.solve(terrain=terrain))
        self.assertLess(np.ptp([p['tip_margin'] for p in cases]), 1e-9)
        self.assertLess(np.ptp([min(p['normal']) for p in cases]), 1e-8)
        sweep = rock_screen(m, .25, samples=9)
        self.assertLessEqual(sweep['rock_sweep_tip_margin'], cases[0]['tip_margin']+1e-9)
        search = coupled_slope_screen(m, .3, limits=False)
        dense = [m.solve(.3, h, check_branch=False) for h in np.linspace(0., 2*math.pi, 721)]
        self.assertAlmostEqual(search['slope_tip_margin'], min(p['tip_margin'] for p in dense), delta=1e-6)
        self.assertAlmostEqual(search['slope_wheel_load_margin'], min(min(p['normal']) for p in dense), delta=.03)

    def test_reference_fill_grousers_and_rover_weight(self):
        low = StaticWheelModel.from_values(wheel_values(dry_mass=40., fill_fraction=.4,
                                                       loaded_sprung_mass=64.))
        full = StaticWheelModel.from_values(wheel_values(dry_mass=40., payload_mass=90.,
                                                        loaded_sprung_mass=114.))
        self.assertEqual(low.reference_mass, 130.)
        self.assertEqual(low.stiffness, full.stiffness)
        np.testing.assert_allclose(low.solve()['compression'], .2*.15*80/130)
        np.testing.assert_allclose(full.solve()['compression'], .2*.15)
        empty = replace(full, sprung_mass=24.)
        self.assertGreater(min(empty.solve()['compression']), 0.)
        self.assertEqual(spring_displacement(0., full.stiffness, .064)[0], 0.)
        for height in (0., .018, .025):
            changed = replace(full, grouser_height=height).solve()
            np.testing.assert_allclose(changed['centers'], full.solve()['centers'])
        for changes in ({'fill_fraction': 0.}, {'dry_mass': 0.}, {'payload_mass': -1.},
                        {'tipping_radius_deflection_fraction': 1.},
                        {'wheel_compression_hard_stop': .02}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                StaticWheelModel.from_values(wheel_values(**changes))

    def test_physical_domains_and_unreachable_requests(self):
        for args in ((-1., 1000., .064), (1., 0., .064), (1., math.nan, .064), (1., 1000., 0.)):
            with self.assertRaises(ValueError):
                spring_displacement(*args)
        for change in ({'hard_stop': .64}, {'hard_stop': -.1}, {'stiffness': 0.},
                       {'radius': .01}, {'gravity': math.nan}, {'wheel_drive_mass': -1.}):
            with self.assertRaises(ValueError):
                model(**change)
        m = model()
        for angle, terrain in ((math.pi/2, None), (-.1, None), (0., [0., 0.]), (0., [0., 0., 0., -.1])):
            with self.assertRaises(ValueError):
                m.solve(angle, terrain=terrain)
        unreachable = rock_screen(m, .8, samples=3)
        self.assertLess(unreachable['rock_equilibrium_margin'], 0.)
        self.assertLess(unreachable['rock_sweep_tip_margin'], 0.)
        self.assertLess(unreachable['rock_sweep_wheel_load_margin'], 0.)


class StaticWheelIntegrationTests(unittest.TestCase):
    def problem(self, overrides=None, **options):
        p = build_problem(overrides, **options)
        self.addCleanup(p.cleanup)
        p.run_model()
        return p

    def test_default_selection_new_margins_sources_and_records(self):
        self.assertIs(compliant_plate_frame_models().traversal_stability, CompliantPlateTraversalStability)
        p = self.problem({'slope_angle': .15, 'obstacle_height': .15}, structure='plate-frame')
        s = snapshot(p)
        self.assertEqual(p.model_metadata['schema_version'], 12)
        self.assertEqual(p.get_val('wheel_compression_hard_stop').item(), .064)
        self.assertTrue(p.model_metadata['configuration']['static_wheel_load_transfer'])
        for name in ('slope_wheel_load_margin', 'wheel_equilibrium_margin'):
            self.assertIn(name, s['constraints'])
            self.assertIn(name, p.model.get_constraints())
        for name in EXTRA_OUTPUTS:
            self.assertIn(name, s['outputs'])
        self.assertEqual(snapshot(p, configuration=p.model_metadata['configuration'])['outputs'], s['outputs'])
        self.assertFalse(any(source.startswith('_auto_ivc') for source in p.model._conn_global_abs_in2out.values()))
        self.assertEqual(p.model.get_source('traversal_stability.wheel_compression_hard_stop'), 'inputs.wheel_compression_hard_stop')
        declared = set(_get_viewer_data(p)['declare_partials_list'])
        self.assertIn('traversal_stability.slope_tip_margin > traversal_stability.wheel_compression_hard_stop', declared)
        # This valid inventory previously sent an unconstrained limit step past 90 degrees.
        regression = self.problem({'wheel_diameter': .33, 'wheel_width': .18, 'travel_actuator_length': .50,
                                   'actuator_retracted_length': .508, 'actuator_extended_length': .60},
                                  structure='plate-frame', motors='x44-standard')
        self.assertGreater(regression.get_val('wheel_equilibrium_margin').item(), 0.)
        self.assertGreater(regression.get_val('slope_tip_angle').item(), 0.)

    def test_mass_energy_and_nominal_ports_preserved(self):
        options = dict(structure='plate-frame', traversal_mode='fixed')
        p = self.problem(**options)
        rigid = self.problem(models=plate_frame_models(), **options)
        for name in ('dry_mass', 'payload_mass', 'loaded_cg_z', 'cycle_time', 'cycle_energy',
                     'construction_points', 'tip_stability_margin', 'ground_clearance_margin'):
            self.assertAlmostEqual(p.get_val(name).item(), rigid.get_val(name).item(), places=10, msg=name)
        self.assertAlmostEqual(sum(p.get_val(n).item() for n in ('loaded_sprung_mass', 'wheel_mass', 'drive_mass', 'rocker_mass')),
                               p.get_val('dry_mass').item()+p.get_val('payload_mass').item())
        self.assertIs(ModelSet().traversal_stability, TraversalStability)

    def test_smooth_total_derivatives_with_load_transfer(self):
        p = self.problem({'slope_angle': .15, 'obstacle_height': .15, 'wheelbase': .46},
                         structure='plate-frame', traversal_mode='fixed')
        checks = p.check_totals(of=['slope_pitch_tip_margin', 'slope_roll_tip_margin',
                                    'slope_wheel_load_margin', 'rock_sweep_tip_margin', 'rock_sweep_wheel_load_margin'],
                               wrt=['wheel_diameter', 'wheelbase', 'track_width', 'battery_capacity'],
                               method='fd', form='central', step=3e-5, step_calc='rel_avg', out_stream=None)
        assert_check_totals(checks, atol=2e-3, rtol=3e-3)

    def test_legacy_payload_contract_requires_explicit_rigid_models(self):
        with self.assertRaisesRegex(ValueError, 'proportional'):
            build_problem(models=replace(compliant_plate_frame_models(), excavation=Excavation),
                          structure='plate-frame')
        with self.assertRaisesRegex(ValueError, 'proportional'):
            build_problem(models=ModelSet(excavation=Excavation, obstacle_stability=CompliantObstacleStability))
        p = self.problem(models=ModelSet(excavation=Excavation))
        self.assertFalse(p.model_metadata['configuration']['static_wheel_compliance'])

    def test_component_dependency_omissions_include_new_and_inactive_paths(self):
        p = self.problem({'slope_angle': .15, 'obstacle_height': .15},
                         structure='plate-frame', traversal_mode='fixed')
        for component in (p.model.traversal_stability, p.model.obstacle_stability):
            inputs = {n: component._inputs[n].copy() for n in component._var_rel_names['input']}
            original = {n: np.zeros(1) for n in component.OUTPUT_UNITS}
            component.compute(inputs, original)
            self.assertEqual(set(component.DEPENDENCIES), set(component.OUTPUT_UNITS))
            for name in inputs:
                changed_inputs = {n: value.copy() for n, value in inputs.items()}
                changed_inputs[name] += 1e-4*max(abs(inputs[name].item()), .01)
                changed = {n: np.zeros(1) for n in component.OUTPUT_UNITS}
                component.compute(changed_inputs, changed)
                for out, dependencies in component.DEPENDENCIES.items():
                    if name not in dependencies:
                        np.testing.assert_allclose(changed[out], original[out], rtol=1e-10, atol=1e-10,
                                                   err_msg=f'Missing dependency {component.name}.{out} <- {name}')


if __name__ == '__main__':
    unittest.main()
