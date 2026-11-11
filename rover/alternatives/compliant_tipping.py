"""Static center-contact tipping with load-dependent wheel compression.

Student assumptions: 20% of overall radius at the same design's 0.9-fill
quarter load; linear spring to a 64 mm hard stop. No FEA or soil calibration.
Reuses the existing rocker geometry, mass placement, reactions and rock sweep.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import root, minimize_scalar, least_squares

from ..components import scalar
from .obstacle_stability import ObstacleStability, projected_support_margin, rocker_pose
from .traversal_stability import TraversalStability, articulated_cg, wheel_reactions, rock_path_screen


COMPLIANCE_CONFIG = ("grouser_height", "tipping_radius_deflection_fraction",
                     "tipping_reference_fill_fraction", "fill_fraction", "wheel_compression_hard_stop")
COMPLIANCE_MASSES = {"dry_mass": "kg", "payload_mass": "kg"}
REFERENCE_MASS_INPUTS = {"dry_mass", "payload_mass", "fill_fraction", "tipping_reference_fill_fraction"}
RATIO_INPUTS = REFERENCE_MASS_INPUTS | {"tipping_radius_deflection_fraction"}
DEFLECTION_INPUTS = RATIO_INPUTS | {"wheel_diameter"}
COMPLIANCE_OUTPUTS = {
    "tipping_reference_mass": "kg", "tipping_wheel_outer_radius": "m",
    "tipping_wheel_tread_radius": "m", "tipping_wheel_deflection": "m",
    "tipping_wheel_deflection_ratio": None, "tipping_wheel_support_height": "m",
    "tipping_loaded_cg_z": "m",
}


POSE_TOL = 1e-7  # m; numerical equilibrium tolerance, not a physical allowance


def _absolute_jacobian(function, state):
    """Central physical steps remain resolvable at near-zero symmetric angles."""
    step = 1e-6  # m for height, rad for pose/terrain angles
    return np.column_stack([(function(state+step*axis)-function(state-step*axis))/(2*step)
                            for axis in np.eye(len(state))])


def spring_displacement(force, stiffness, hard_stop):
    """Compression and additional stop reaction, never a capped force [SI]."""
    force = np.asarray(force)
    if (not np.isfinite(force).all() or np.any(force < 0) or not 0 < stiffness <= math.inf
            or not math.isfinite(hard_stop) or hard_stop <= 0):
        raise ValueError("Wheel law requires nonnegative force, positive stiffness and stop travel.")
    return np.minimum(force/stiffness, hard_stop), np.maximum(0., force-stiffness*hard_stop)


@dataclass(frozen=True)
class StaticWheelModel:
    wheelbase: float
    track: float
    radius: float
    pivot_offset: float
    sprung_mass: float
    sprung_x: float
    sprung_z: float
    wheel_drive_mass: float
    rocker_mass: float
    rocker_fraction: float
    stiffness: float
    hard_stop: float
    gravity: float
    reference_mass: float
    grouser_height: float = .018
    drive_mass: float = 0.
    drive_cg_offset: float = 0.

    def __post_init__(self):
        numbers = tuple(v for n, v in vars(self).items() if n != 'stiffness')
        if (not all(math.isfinite(x) for x in numbers)
                or min(self.wheelbase, self.track, self.radius, self.sprung_mass,
                       self.hard_stop, self.gravity, self.reference_mass) <= 0
                or min(self.wheel_drive_mass, self.rocker_mass, self.grouser_height) < 0
                or not 0 <= self.drive_mass <= self.wheel_drive_mass or self.drive_cg_offset < 0
                or not 0 <= self.rocker_fraction <= 1
                or not 0 < self.stiffness <= math.inf
                or abs(self.pivot_offset) >= self.wheelbase/2
                or self.hard_stop >= self.radius-self.grouser_height):
            raise ValueError("Invalid static wheel geometry, masses, stiffness or hard-stop travel.")

    @property
    def mass(self):
        return self.sprung_mass+self.wheel_drive_mass+self.rocker_mass

    @classmethod
    def from_values(cls, values, rocker_fraction=.5):
        v = values.__getitem__
        fill, reference_fill = v('fill_fraction'), v('tipping_reference_fill_fraction')
        radius, ratio = v('wheel_diameter')/2, v('tipping_radius_deflection_fraction')
        if (not 0 < fill <= 1 or not 0 < reference_fill <= 1 or not 0 <= ratio < 1
                or not math.isfinite(fill+reference_fill+ratio)
                or v('dry_mass') <= 0 or v('payload_mass') < 0):
            raise ValueError("Physical fill, mass and reference radial compression are required.")
        reference_mass = v('dry_mass')+v('payload_mass')*reference_fill/fill
        stop = v('wheel_compression_hard_stop')
        if ratio*radius > stop:
            raise ValueError("Reference compression cannot exceed the specified hard stop.")
        stiffness = math.inf if ratio == 0 else reference_mass*v('gravity')/(4*ratio*radius)
        return cls(v('wheelbase'), v('track_width'), radius, v('rocker_pivot_offset'),
                   v('loaded_sprung_mass'), v('loaded_sprung_cg_x'), v('loaded_sprung_cg_z'),
                   v('wheel_mass')+v('drive_mass'), v('rocker_mass'), rocker_fraction,
                   stiffness, stop, v('gravity'), reference_mass, v('grouser_height'),
                   v('drive_mass'), values.get('drive_motor_cg_offset',0.))

    def geometry(self, q, terrain):
        pose = rocker_pose(self.wheelbase, self.track, self.pivot_offset, *q)
        pose['centers'] = np.asarray(pose['centers'])
        pose['contacts'] = np.column_stack((pose['centers'][:, :2], terrain))
        pose['cg'] = np.asarray(articulated_cg(
            pose, self.radius, self.pivot_offset, self.sprung_mass, self.sprung_x,
            self.sprung_z, self.wheel_drive_mass, self.rocker_mass, self.rocker_fraction,
            drive_mass=self.drive_mass, drive_cg_offset=self.drive_cg_offset))
        return pose

    def reactions(self, q, terrain, gravity):
        pose = self.geometry(q, terrain)
        normal, error = wheel_reactions(pose, pose['cg'], self.sprung_mass,
                                       self.wheel_drive_mass, self.rocker_mass,
                                       gravity, self.rocker_fraction,
                                       drive_mass=self.drive_mass, drive_cg_offset=self.drive_cg_offset)
        return pose, normal, error

    def _residual(self, q, terrain, gravity):
        pose, normal, _ = self.reactions(q, terrain, gravity)
        compression, _ = spring_displacement(np.maximum(normal, 0.), self.stiffness, self.hard_stop)
        return (pose['centers'][:, 2]-terrain-self.radius+compression)/self.radius

    def solve(self, angle=0., heading=0., terrain=None, initial=None, *, check_branch=True):
        """Solve the four-contact branch; explicitly flag unloading/invalid roots.

        Negative reactions may be needed beyond wheel unloading. A zero-
        compression diagnostic continuation keeps arithmetic finite there;
        it is flagged as an impossible four-contact state, never made feasible.
        Hard stops are different: compression stops but reaction can increase.
        """
        terrain = np.zeros(4) if terrain is None else np.asarray(terrain, dtype=float)
        if (not math.isfinite(angle+heading) or not 0 <= angle < math.pi/2
                or terrain.shape != (4,) or not np.isfinite(terrain).all() or min(terrain) < 0):
            raise ValueError("Finite planar slope/heading and four nonnegative platform heights required.")
        g = self.gravity*np.array((math.sin(angle)*math.cos(heading),
                                   math.sin(angle)*math.sin(heading), -math.cos(angle)))
        level_delta = min(self.mass*(-g[2])/(4*self.stiffness), self.hard_stop)
        if initial is None:
            initial = np.array((self.radius+self.pivot_offset+float(np.mean(terrain))-level_delta,
                                (terrain[0]+terrain[2]-terrain[1]-terrain[3])/(2*self.track),
                                (terrain[0]-terrain[2])/self.wheelbase,
                                (terrain[1]-terrain[3])/self.wheelbase))
        else:
            initial = np.asarray(initial, dtype=float)
        if initial.shape != (4,) or not np.isfinite(initial).all():
            raise ValueError("Initial pose must contain four finite coordinates.")

        def residual(q):
            return self._residual(q, terrain, g)

        jacobian = lambda state: _absolute_jacobian(residual, state)
        try:
            solution = root(residual, initial, jac=jacobian, method='hybr',
                            options={'xtol': 1e-10, 'maxfev': 100})
            q = solution.x
            bad_root = (not np.isfinite(q).all() or max(abs(q[1:])) >= math.pi/2-1e-4
                        or np.max(np.abs(residual(q)))*self.radius > POSE_TOL)
        except (np.linalg.LinAlgError, ValueError, OverflowError):
            q, bad_root = initial, True
        if bad_root:
            # A second numerical method solves the SAME equations. It never
            # substitutes rigid/default physics, clips reactions, or bypasses
            # the unchanged residual/branch/contact acceptance screens.
            angular_limit = math.pi/2-1e-4
            lower = np.array((min(terrain)+self.radius-self.hard_stop-abs(self.pivot_offset)-1e-6,
                              -angular_limit, -angular_limit, -angular_limit))
            upper = np.array((max(terrain)+self.radius+abs(self.pivot_offset)+1e-6,
                              angular_limit, angular_limit, angular_limit))
            seed = np.clip(initial, lower+1e-8, upper-1e-8)
            solution = least_squares(residual, seed, jac=jacobian, bounds=(lower, upper),
                                     xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=120)
            q = solution.x
        pose, normal, force_error = self.reactions(q, terrain, g)
        error = float(np.max(np.abs(residual(q))))*self.radius
        compression, stop_reaction = spring_displacement(np.maximum(normal, 0.), self.stiffness, self.hard_stop)
        # Follow the regular branch connected to the rigid geometry. A zero or
        # negative incremental-compliance eigenvalue rejects an overturned/fold
        # root even if its algebraic force balance happens to close.
        branch = 1.
        if check_branch:
            geometric = _absolute_jacobian(lambda state: self.geometry(state, terrain)['centers'][:, 2]/self.radius, q)
            eigenvalues = np.linalg.eigvals(np.linalg.solve(geometric, _absolute_jacobian(residual, q)))
            branch = float(np.min(eigenvalues.real))
        geometry_valid = bool(np.isfinite(q).all() and max(abs(q[1:])) < math.pi/2-1e-4)
        equation_valid = bool(geometry_valid and error <= POSE_TOL and branch > 0)
        intercept = pose['cg'][:2]-g[:2]/g[2]*pose['cg'][2]
        tip = projected_support_margin(pose['contacts'], (*intercept, 0.))
        return {**pose, 'q': q, 'normal': normal, 'compression': compression,
                'stop_reaction': stop_reaction, 'gravity': g, 'intercept': intercept,
                'tip_margin': tip, 'pose_error': error,
                'force_error': force_error,
                'branch_margin': branch, 'equation_valid': equation_valid,
                'contact_valid': bool(equation_valid and min(normal) >= -1e-8),
                'guarded': bool(not equation_valid or min(normal) < -1e-8),
                'equilibrium_margin': branch if equation_valid else -max(1., error/POSE_TOL)}

    def limit(self, heading, kind='tip'):
        """Solve pose and limiting inclination together, without nested bisection.

        Tip means zero GLOBAL hull reserve on the four-contact continuation.
        Load means first zero normal reaction. These are deliberately separate:
        one-wheel unloading need not be a complete rollover. Post-lift poses
        are guarded diagnostics, not predictions of a three-contact trajectory.
        """
        if kind not in ('tip', 'load') or not math.isfinite(heading):
            raise ValueError("Limit must be tip or load, at a finite heading.")
        terrain = np.zeros(4)
        zero = self.solve(check_branch=False)
        distance = min(self.wheelbase/(2*max(abs(math.cos(heading)), 1e-12)),
                       self.track/(2*max(abs(math.sin(heading)), 1e-12)))
        guess = min(math.radians(75), math.atan2(distance, max(zero['cg'][2], .01)))
        start = self.solve(guess, heading, check_branch=False)

        def residual(state):
            q, angle = state[:4], state[4]
            g = self.gravity*np.array((math.sin(angle)*math.cos(heading),
                                       math.sin(angle)*math.sin(heading), -math.cos(angle)))
            pose, normal, _ = self.reactions(q, terrain, g)
            compression, _ = spring_displacement(np.maximum(normal, 0.), self.stiffness, self.hard_stop)
            heights = (pose['centers'][:, 2]-self.radius+compression)/self.radius
            if kind == 'load':
                boundary = min(normal)/(self.mass*self.gravity)
            else:
                point = pose['cg'][:2]-g[:2]/g[2]*pose['cg'][2]
                boundary = projected_support_margin(pose['contacts'], (*point, 0.))/self.radius
            return np.r_[heights, boundary]

        seed = np.r_[start['q'], guess]
        try:
            result = root(residual, seed,
                          jac=lambda state: _absolute_jacobian(residual, state), method='hybr',
                          options={'xtol': 1e-10, 'maxfev': 120})
            state = result.x
            error = float(np.max(np.abs(residual(state))))
        except (np.linalg.LinAlgError, ValueError, OverflowError):
            state, error = seed, math.inf
        angular_limit = math.pi/2-1e-4
        if (error >= 1e-6 or not 0 < state[4] < angular_limit
                or max(abs(state[1:4])) >= angular_limit):
            # Same five equations, with physical angle bounds on solver steps.
            lower = np.array((-math.inf, -angular_limit, -angular_limit, -angular_limit, 1e-6))
            upper = np.array((math.inf, angular_limit, angular_limit, angular_limit, angular_limit))
            result = least_squares(residual, np.clip(seed, lower+1e-8, upper-1e-8),
                                   jac=lambda q: _absolute_jacobian(residual, q), bounds=(lower, upper),
                                   xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=120)
            state = result.x
            error = float(np.max(np.abs(residual(state))))
        angle = float(state[4])
        valid = bool(np.isfinite(state).all() and 0 < angle < math.pi/2
                     and error < 1e-6 and max(abs(state[1:4])) < angular_limit)
        if not valid:
            return {'angle': 0., 'valid': False, 'contact_valid': False, 'error': error}
        pose = self.solve(angle, heading, initial=state[:4], check_branch=False)
        return {'angle': angle, 'valid': bool(pose['equation_valid']),
                'contact_valid': bool(min(pose['normal']) >= -1e-5), 'error': error,
                'pose': pose}


def axis_reserves(pose):
    """Signed front/rear and left/right edge-normal reserves in terrain metres."""
    xy = pose['contacts'][:, :2]
    center = np.mean(xy, axis=0)
    point = pose['intercept']
    reserves = []
    for i, j in ((0, 1), (2, 3), (0, 2), (1, 3)):
        delta = xy[j]-xy[i]
        normal = np.array((-delta[1], delta[0]))
        normal /= np.linalg.norm(normal)
        if np.dot(center-xy[i], normal) < 0:
            normal = -normal
        reserves.append(float(np.dot(point-xy[i], normal)))
    return min(reserves[:2]), min(reserves[2:])


def _heading_search(model, evaluator, *, step_degrees=5., polish=True):
    """Deterministic full-circle search, using exact mirrored-load symmetries.

    A finite grid plus local bounded refinement is a numerical search, not an
    analytical proof over the continuum. Dense independent checks are required.
    """
    if not math.isfinite(step_degrees) or not 0 < step_degrees <= 15:
        raise ValueError("Heading grid step must be in (0, 15] degrees.")
    symmetric = abs(model.sprung_x) < 1e-14
    maximum = math.pi/2 if symmetric else 2*math.pi
    count = int(math.ceil(math.degrees(maximum)/step_degrees))
    headings = np.linspace(0., maximum, count+1)
    values = [evaluator(float(heading)) for heading in headings]
    candidates = list(zip(headings, values))
    if polish:
        for i in range(1, count):
            if (values[i][0] <= values[i-1][0] and values[i][0] <= values[i+1][0]
                    and values[i][0] < max(values[i-1][0], values[i+1][0])-1e-10):
                result = minimize_scalar(lambda h: evaluator(h)[0],
                                         bounds=(headings[i-1], headings[i+1]), method='bounded',
                                         options={'xatol': 2e-6, 'maxiter': 30})
                candidates.append((float(result.x), evaluator(float(result.x))))
        if not symmetric and min(values[0][0], values[-1][0]) <= min(values[1][0], values[-2][0]):
            spacing = headings[1]-headings[0]
            result = minimize_scalar(lambda h: evaluator(h % (2*math.pi))[0],
                                     bounds=(-spacing, spacing), method='bounded',
                                     options={'xatol': 2e-6, 'maxiter': 30})
            h = float(result.x) % (2*math.pi)
            candidates.append((h, evaluator(h)))
    heading, result = min(candidates, key=lambda item: item[1][0])
    return float(heading), result, values


def slope_cardinals(model, angle):
    """Actual coupled pitch/roll curves for the existing two-curve plot."""
    front = model.solve(angle, 0.)
    left = model.solve(angle, math.pi/2)
    if abs(model.sprung_x) < 1e-14:
        return axis_reserves(front)[0], axis_reserves(left)[1], (front, left)
    rear = model.solve(angle, math.pi)
    right = model.solve(angle, 3*math.pi/2)
    return min(axis_reserves(p)[0] for p in (front, rear)), min(axis_reserves(p)[1] for p in (left, right)), (front, left, rear, right)


def coupled_slope_screen(model, angle, *, step_degrees=5., polish=True, limits=True):
    """Requested-slope global support, four-contact reactions and pose diagnostics."""
    # Reuse only this invocation's explicitly evaluated heading results. There
    # is no persistent cache, stochastic search or hidden state across computes.
    poses = {}

    def evaluate(heading):
        pose = model.solve(angle, heading)
        poses[heading] = pose
        return pose

    tip_heading, (_, tip_pose), _ = _heading_search(
        model, lambda h: ((p := evaluate(h))['tip_margin'], p),
        step_degrees=step_degrees, polish=polish)

    def load_evaluation(heading):
        pose = poses.get(heading)
        if pose is None:
            pose = evaluate(heading)
        return float(min(pose['normal'])), pose

    load_heading, (_, load_pose), _ = _heading_search(
        model, load_evaluation, step_degrees=step_degrees, polish=polish)
    pitch, roll, cardinal = slope_cardinals(model, angle)
    evaluated = list(poses.values())+list(cardinal)
    result = {'slope_pitch_tip_margin': pitch, 'slope_roll_tip_margin': roll,
              'slope_tip_margin': tip_pose['tip_margin'],
              'slope_wheel_load_margin': float(min(load_pose['normal'])),
              'slope_worst_heading': tip_heading, 'slope_load_worst_heading': load_heading,
              'slope_equilibrium_valid': float(all(p['contact_valid'] for p in evaluated)),
              'slope_equilibrium_margin': min(p['equilibrium_margin'] for p in evaluated),
              'slope_pose_error': max(p['pose_error'] for p in evaluated)}
    for i, wheel in enumerate(('fl', 'fr', 'rl', 'rr')):
        for field, values in (('normal_load', load_pose['normal']),
                              ('compression', load_pose['compression']),
                              ('stop_reaction', load_pose['stop_reaction'])):
            result[f'slope_{wheel}_{field}'] = float(values[i])
    if limits:
        _, (_, tip), _ = _heading_search(
            model, lambda h: ((p := model.limit(h, 'tip'))['angle'], p),
            step_degrees=step_degrees, polish=polish)
        _, (_, load), _ = _heading_search(
            model, lambda h: ((p := model.limit(h, 'load'))['angle'], p),
            step_degrees=step_degrees, polish=polish)
        result.update(slope_tip_angle=tip['angle'], slope_contact_limit_angle=load['angle'])
        if not tip['valid'] or not load['valid']:
            result['slope_equilibrium_margin'] = min(result['slope_equilibrium_margin'], -1.)
    return result


EXTRA_OUTPUTS = {
    'wheel_spring_compliance': 'm/N',
    'slope_wheel_load_margin': 'N', 'wheel_equilibrium_margin': None,
    'slope_worst_heading': 'rad', 'slope_load_worst_heading': 'rad',
    'slope_contact_limit_angle': 'rad', 'slope_equilibrium_valid': None,
    'slope_pose_error': 'm', 'rock_equilibrium_valid': None, 'rock_pose_error': 'm',
}
for wheel in ('fl', 'fr', 'rl', 'rr'):
    EXTRA_OUTPUTS.update({f'slope_{wheel}_normal_load': 'N', f'slope_{wheel}_compression': 'm',
                          f'slope_{wheel}_stop_reaction': 'N'})

EVIDENCE = {
    'revision': 2,
    'status': 'Provisional analytical static model; student linear law and 64 mm stop, no FEA or hardware calibration',
    'stiffness': 'k = reference_mass*g/(4*0.20*overall_radius); reference mass is same design at 0.9 fill',
    'stop': 'Linear normal-force/compression relation to 0.064 m; rigid stop carries additional reaction without increasing compression',
    'suspension': 'Four wheel normals and shaft height/roll plus both rocker pitches solved together; chassis pitch is their average',
    'holding': 'Contact reactions allocated parallel to opposite gravity, equivalent to the same tangential/normal ratio at every contact; adequate friction and holding torque assumed',
    'slope': '30-degree requirement unchanged; 5-degree full-heading grid with local bounded refinement, exact symmetry reductions for mirrored mass inventories',
    'limits': 'Global hull zero and first wheel unloading reported separately; post-lift continuation is guarded, not a three-contact prediction',
    'grousers': 'Rigid 18 mm radial height inside overall diameter; point contacts at tips, no finite-patch or soil-sinkage credit',
    'omissions': 'Finite footprints, camber contact corrections, lateral wheel compliance, soil deformation, hard-stop strength, joint limits, motor holding capability and dynamics',
}


class CompliantTraversalStability(TraversalStability):
    STATIC_WHEEL_COMPLIANCE = True
    STATIC_WHEEL_LOAD_TRANSFER = True
    REQUIRES_LINEAR_FILL_PAYLOAD = True
    CONFIG_INPUTS = TraversalStability.CONFIG_INPUTS+COMPLIANCE_CONFIG
    COUPLED_INPUTS = {**TraversalStability.COUPLED_INPUTS, **COMPLIANCE_MASSES}
    REQUIRED_OUTPUT_UNITS = TraversalStability.OUTPUT_UNITS
    OUTPUT_UNITS = {**TraversalStability.OUTPUT_UNITS, **COMPLIANCE_OUTPUTS, **EXTRA_OUTPUTS}
    used_inputs = (set(CONFIG_INPUTS) | set(COUPLED_INPUTS))-{'loaded_cg_x', 'loaded_cg_z'}
    slope_inputs = used_inputs-{'obstacle_height'}
    rock_inputs = used_inputs-{'slope_angle'}
    DEPENDENCIES = dict.fromkeys(OUTPUT_UNITS, used_inputs)
    for name in OUTPUT_UNITS:
        if name.startswith('slope_'):
            DEPENDENCIES[name] = slope_inputs
        if name.startswith('rock_'):
            DEPENDENCIES[name] = rock_inputs
    DEPENDENCIES.update({
        'tipping_reference_mass': REFERENCE_MASS_INPUTS,
        'wheel_spring_compliance': REFERENCE_MASS_INPUTS | {'wheel_diameter', 'tipping_radius_deflection_fraction', 'gravity'},
        'tipping_wheel_outer_radius': {'wheel_diameter'},
        'tipping_wheel_tread_radius': {'wheel_diameter', 'grouser_height'},
        'tipping_wheel_deflection': DEFLECTION_INPUTS | {'wheel_compression_hard_stop', 'loaded_sprung_mass', 'wheel_mass', 'drive_mass', 'rocker_mass'},
        'tipping_wheel_deflection_ratio': DEFLECTION_INPUTS | {'wheel_compression_hard_stop', 'loaded_sprung_mass', 'wheel_mass', 'drive_mass', 'rocker_mass'},
        'tipping_wheel_support_height': DEFLECTION_INPUTS | {'wheel_compression_hard_stop', 'loaded_sprung_mass', 'wheel_mass', 'drive_mass', 'rocker_mass'},
        'tipping_loaded_cg_z': used_inputs-{'slope_angle', 'obstacle_height'},
        'rock_sweep_samples': set(),
    })
    rocker_centroid_fraction = 1.
    model_evidence = EVIDENCE

    def compute(self, inputs, outputs):
        values = {n: scalar(inputs, n) for n in inputs}
        model = StaticWheelModel.from_values(values, self.rocker_centroid_fraction)
        flat = model.solve()
        slope = coupled_slope_screen(model, values['slope_angle'])
        rock = rock_path_screen(values, wheel_model=model)
        delta = min(model.mass*model.gravity/(4*model.stiffness), model.hard_stop)
        result = {**slope, **rock,
                  'wheel_spring_compliance': 1/model.stiffness,
                  'tipping_reference_mass': model.reference_mass,
                  'tipping_wheel_outer_radius': model.radius,
                  'tipping_wheel_tread_radius': model.radius-model.grouser_height,
                  'tipping_wheel_deflection': delta, 'tipping_wheel_deflection_ratio': delta/model.radius,
                  'tipping_wheel_support_height': model.radius-delta,
                  'tipping_loaded_cg_z': flat['cg'][2],
                  'wheel_equilibrium_margin': min(flat['equilibrium_margin'], slope['slope_equilibrium_margin'], rock['rock_equilibrium_margin'])}
        for name in self.OUTPUT_UNITS:
            outputs[name] = result[name]


class CompliantPlateTraversalStability(CompliantTraversalStability):
    rocker_centroid_fraction = 0.


class CompliantObstacleStability(ObstacleStability):
    REQUIRES_LINEAR_FILL_PAYLOAD = True
    CONFIG_INPUTS = tuple(n for n in CompliantTraversalStability.CONFIG_INPUTS if n != 'slope_angle')
    COUPLED_INPUTS = {n: u for n, u in CompliantTraversalStability.COUPLED_INPUTS.items()
                      if n not in ('loaded_cg_x', 'loaded_cg_z')}
    OUTPUT_UNITS = {**ObstacleStability.OUTPUT_UNITS, 'obstacle_equilibrium_valid': None}
    DEPENDENCIES = dict.fromkeys(OUTPUT_UNITS, set(CONFIG_INPUTS) | set(COUPLED_INPUTS))
    pose_inputs = {'wheelbase', 'track_width', 'wheel_diameter', 'rocker_pivot_offset', 'obstacle_height'}
    for name in ('obstacle_pose_margin', 'obstacle_evaluated_height'):
        DEPENDENCIES[name] = pose_inputs
    rocker_centroid_fraction = 1.
    model_evidence = EVIDENCE

    def compute(self, inputs, outputs):
        values = {n: scalar(inputs, n) for n in inputs}
        model = StaticWheelModel.from_values(values, self.rocker_centroid_fraction)
        result = rock_path_screen(values, wheel_model=model, samples=2)
        pose, reach = result['endpoint'], result['reach']
        outputs['obstacle_pose_margin'] = reach['reach_margin']
        outputs['obstacle_pose_valid'] = float(reach['valid'])
        outputs['obstacle_equilibrium_valid'] = float(reach['valid'] and pose['contact_valid'])
        outputs['obstacle_evaluated_height'] = reach['used_height']
        outputs['obstacle_worst_wheel'] = result['endpoint_wheel']
        outputs['obstacle_tip_margin'] = pose['tip_margin']
        outputs['obstacle_chassis_pitch'], outputs['obstacle_chassis_roll'] = pose['pitch'], pose['roll']
        for axis, value in zip('xyz', pose['cg']):
            outputs['obstacle_loaded_cg_'+axis] = value


class CompliantPlateObstacleStability(CompliantObstacleStability):
    rocker_centroid_fraction = 0.
