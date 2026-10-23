"""Default exact clearance for the current parallel wheel/drum cylinders.

All axes share chassis roll. Undo that common rigid rotation, then combine
the circular x/z gap with the finite axial y gap. This is not valid for
independent camber, steering, flexible envelopes or arbitrary CAD shapes.
"""
import math
from .obstacle_stability import raised_wheel_pose
from ..components import scalar
from ..discipline import Discipline

# Numerical rejection offset at invalid poses, not a physical allowance.
INVALID_POSE_OFFSET = 1e-4


def chassis_point_in_world(point, pose, level_shaft_height):
    """Apply chassis pitch, shared roll and translation to a mounted point."""
    if len(point) != 3 or len(pose['shaft_origin']) != 3 or not all(
            math.isfinite(v) for v in (*point, *pose['shaft_origin'],
                                       pose['pitch'], pose['roll'], level_shaft_height)):
        raise ValueError('Mounted transform needs finite 3-D points, origin and angles.')
    x,y,z=point
    z-=level_shaft_height
    c,s=math.cos(pose['pitch']),math.sin(pose['pitch'])
    xp,zp=c*x-s*z,s*x+c*z
    c,s=math.cos(pose['roll']),math.sin(pose['roll'])
    ox,oy,oz=pose['shaft_origin']
    return ox+xp,oy+c*y-s*zp,oz+s*y+c*zp


def unroll_point(point, pose):
    """Shaft-relative coordinates after removing the shared roll rotation."""
    if len(point) != 3 or len(pose['shaft_origin']) != 3 or not all(
            math.isfinite(v) for v in (*point, *pose['shaft_origin'], pose['roll'])):
        raise ValueError('Unrolling needs finite 3-D points, origin and roll.')
    x, y, z = (p-o for p, o in zip(point, pose['shaft_origin']))
    c, s = math.cos(pose['roll']), math.sin(pose['roll'])
    return x, c*y+s*z, -s*y+c*z


def parallel_cylinder_gap(delta, radius_a, length_a, radius_b, length_b):
    """Signed Euclidean gap of finite cylinders with common y axes [m].

    Positive is exact surface separation. Negative is minus the smallest
    relative translation needed to resolve intersection of the ideal solids.
    """
    if len(delta) != 3 or not all(math.isfinite(v) for v in
            (*delta, radius_a, length_a, radius_b, length_b)):
        raise ValueError('Cylinder geometry must be finite and three-dimensional.')
    if min(radius_a, length_a, radius_b, length_b) <= 0:
        raise ValueError('Cylinder radii and lengths must be positive.')
    radial = math.hypot(delta[0], delta[2])-radius_a-radius_b
    axial = abs(delta[1])-(length_a+length_b)/2
    gap = math.hypot(max(radial, 0), max(axial, 0))+min(max(radial, axial), 0)
    return {'gap': gap, 'radial_gap': radial, 'axial_gap': axial}


def cylinder_clearance_cases(values):
    """All 64 pairs in four endpoint poses, with world and unrolled geometry.

    Invalid requested poses retain flagged guarded geometry for diagnostics.
    The separate evaluator enforces their failing requested-height margin.
    """
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError('Clearance inputs must be finite.')
    for name in ('wheel_diameter', 'wheel_width', 'drum_diameter', 'drum_length'):
        if values[name] <= 0:
            raise ValueError(f'{name} must be positive.')
    for name in ('drum_pair_gap', 'mechanism_clearance'):
        if values[name] < 0:
            raise ValueError(f'{name} must be nonnegative.')
    v = values.__getitem__
    rw, rd = v('wheel_diameter')/2, v('drum_diameter')/2
    yd = (v('drum_length')+v('drum_pair_gap'))/2
    level_drums = [(sx*v('travel_x'), sy*yd, v('travel_y'))
                   for sx, sy in ((1,1),(1,-1),(-1,1),(-1,-1))]
    requested = raised_wheel_pose(v('wheelbase'),v('track_width'),rw,
                                  v('rocker_pivot_offset'),v('obstacle_height'))
    cases = []
    for raised in range(4):
        pose = raised_wheel_pose(v('wheelbase'),v('track_width'),rw,
                                 v('rocker_pivot_offset'),requested['used_height'],raised)
        drums = [chassis_point_in_world(p,pose,rw+v('rocker_pivot_offset')) for p in level_drums]
        local_drums = [unroll_point(p,pose) for p in drums]
        local_wheels = [unroll_point(p,pose) for p in pose['centers']]
        pairs = []
        for di, d in enumerate(local_drums):
            for wi, w in enumerate(local_wheels):
                gap = parallel_cylinder_gap(tuple(a-b for a,b in zip(d,w)),
                                            rd,v('drum_length'),rw,v('wheel_width'))
                pairs.append({**gap,'drum':di,'wheel':wi})
        cases.append({'raised':raised,'pose':pose,'drums':drums,'wheels':pose['centers'],
                      'unrolled_drums':local_drums,'unrolled_wheels':local_wheels,
                      'pairs':pairs,'worst':min(pairs,key=lambda p:p['gap'])})
    return {'requested_pose':requested,'cases':cases}


def evaluate_parallel_cylinder_clearance(values):
    geometry = cylinder_clearance_cases(values)
    requested = geometry['requested_pose']
    gap = min(c['worst']['gap'] for c in geometry['cases'])
    margin = gap-values['mechanism_clearance']
    if not requested['valid']:
        margin = min(margin, requested['reach_margin']-INVALID_POSE_OFFSET)
    return {'obstacle_drum_wheel_gap_bound':gap, 'obstacle_drum_wheel_margin':margin,
            'obstacle_clearance_pose_valid':float(requested['valid']),
            'obstacle_clearance_evaluated_height':requested['used_height']}


class ParallelCylinderClearance(Discipline):
    """Default ModelSet.obstacle_clearance discipline; native contract retained."""
    CONFIG_INPUTS = ('wheelbase','track_width','wheel_diameter','wheel_width',
                     'rocker_pivot_offset','obstacle_height','drum_diameter',
                     'drum_length','drum_pair_gap','mechanism_clearance')
    COUPLED_INPUTS = {'travel_x':'m','travel_y':'m'}
    OUTPUT_UNITS = {'obstacle_drum_wheel_gap_bound':'m','obstacle_drum_wheel_margin':'m',
                    'obstacle_clearance_pose_valid':None,'obstacle_clearance_evaluated_height':'m'}
    gap_inputs = (set(CONFIG_INPUTS)-{'mechanism_clearance'}) | set(COUPLED_INPUTS)
    pose_inputs = {'wheelbase','track_width','rocker_pivot_offset','obstacle_height'}
    DEPENDENCIES = {'obstacle_drum_wheel_gap_bound':gap_inputs,
                    'obstacle_drum_wheel_margin':gap_inputs | {'mechanism_clearance'},
                    'obstacle_clearance_pose_valid':pose_inputs,
                    'obstacle_clearance_evaluated_height':pose_inputs}
    model_evidence = {
        'revision':2, 'status':'default analytical ideal-cylinder geometry; no hardware validation',
        'geometry':'parallel finite cylinders; common roll factored out',
        'poses':'four single-wheel-raised endpoints; fixed travel arm position',
        'clearance':'exact signed cylinder gap minus mechanism_clearance [m]; legacy gap_bound port retained',
        'domain':'rigid right circular cylinders with parallel transverse axes and shared roll',
        'guard':'unreachable poses retain flagged guarded diagnostics and failing margin',
        'omissions':'intermediate crossing poses, independent camber/steering, terrain, deflection, grousers and other hardware',
    }

    def compute(self, inputs, outputs):
        for name, value in evaluate_parallel_cylinder_clearance(
                {n:scalar(inputs,n) for n in inputs}).items():
            outputs[name] = value
