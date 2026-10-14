"""Conservative drum/wheel clearance in four prescribed obstacle poses.

Rigid cylindrical envelopes are enclosed by spheres. Positive gap bounds prove
separation of those envelopes; negative bounds do not prove actual collision.
The climbing trajectory, structural/wheel deflection and other hardware are
outside this analytical screen. See docs/OBSTACLE_TRAVERSAL.md.
"""
import math
from .obstacle_stability import raised_wheel_pose
from ..discipline import Discipline
from ..components import scalar

# Numerical rejection offset at invalid poses, not a physical clearance allowance.
INVALID_POSE_OFFSET = 1e-4  # m; larger than the report's scaled numerical tolerance

class ObstacleClearance(Discipline):
    CONFIG_INPUTS = (
        "wheelbase", "track_width", "wheel_diameter", "wheel_width",
        "rocker_pivot_offset", "obstacle_height",
        "drum_diameter", "drum_length", "drum_pair_gap",
        "mechanism_clearance",
    )

    COUPLED_INPUTS = {
        "travel_x": "m",
        "travel_y": "m",
    }

    OUTPUT_UNITS = {
        "obstacle_drum_wheel_gap_bound": "m",
        "obstacle_drum_wheel_margin": "m",
        "obstacle_clearance_pose_valid": None,
        "obstacle_clearance_evaluated_height": "m",
    }

    pose_inputs = {
        "wheelbase", "track_width",
        "rocker_pivot_offset", "obstacle_height",
    }
    gap_inputs = (
        set(CONFIG_INPUTS) - {"mechanism_clearance"}
    ) | set(COUPLED_INPUTS)

    DEPENDENCIES = {
        "obstacle_drum_wheel_gap_bound": gap_inputs,
        "obstacle_drum_wheel_margin": (
            gap_inputs | {"mechanism_clearance"}
        ),
        "obstacle_clearance_pose_valid": pose_inputs,
        "obstacle_clearance_evaluated_height": pose_inputs,
    }

    model_evidence = {
        "revision": 1,
        "status": "analytical conservative envelope screen; no hardware validation",
        "geometry": "spheres enclosing rigid wheel and outer cutting-drum cylinders",
        "poses": "four single-wheel-raised cases; arms fixed in travel position",
        "clearance": "gap lower bound minus mechanism_clearance, in metres",
        "negative_bound": "inconclusive about cylinder collision; conservatively rejected",
        "guard": "unreachable poses use labeled guarded height and a failing margin",
        "invalid_pose_offset_m": INVALID_POSE_OFFSET,
        "omissions": "intermediate crossing poses, terrain, deflection, hubs/shafts/arms/frame interference",
    }

    def compute(self, inputs, outputs):
        values = {name: scalar(inputs, name) for name in inputs}
        for name, value in evaluate_obstacle_clearance(values).items():
            outputs[name] = value


def _finite_points(points, label):
    try:
        result = [tuple(float(value) for value in point) for point in points]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain finite three-dimensional points.") from exc
    if not result or any(len(point) != 3 or not all(math.isfinite(v) for v in point)
                         for point in result):
        raise ValueError(f"{label} must contain finite three-dimensional points.")
    return result


def _validate_bound_radii(drum_bound_radius, wheel_bound_radius):
    if any(not math.isfinite(radius) or radius < 0
           for radius in (drum_bound_radius, wheel_bound_radius)):
        raise ValueError("Enclosing-sphere radii must be finite and nonnegative.")


def chassis_point_in_world(point, pose, level_shaft_height):
    """Move a chassis-mounted point from level-ground to obstacle coordinates."""
    x, y, z = _finite_points([point], "Mounted point")[0]
    shaft_x, shaft_y, shaft_z = _finite_points([pose["shaft_origin"]], "Shaft origin")[0]
    if not all(math.isfinite(value) for value in
               (pose["pitch"], pose["roll"], level_shaft_height)):
        raise ValueError("Pose angles and original shaft height must be finite.")

    # Express height relative to the suspension shaft.
    z_relative = z - level_shaft_height

    # Apply chassis pitch.
    pitch = pose["pitch"]
    pitched_x = x * math.cos(pitch) - z_relative * math.sin(pitch)
    pitched_z = x * math.sin(pitch) + z_relative * math.cos(pitch)

    # Apply chassis roll.
    roll = pose["roll"]
    rolled_y = y * math.cos(roll) - pitched_z * math.sin(roll)
    rolled_z = y * math.sin(roll) + pitched_z * math.cos(roll)

    # Translate from shaft-relative coordinates to world coordinates.
    return (
        shaft_x + pitched_x,
        shaft_y + rolled_y,
        shaft_z + rolled_z,
    )

def minimum_sphere_gap(drum_centers, wheel_centers,
                       drum_bound_radius, wheel_bound_radius):
    """Return the smallest sphere gap and its drum/wheel indices.

    Positive guarantees separation of the enclosed shapes.
    Negative is inconclusive about actual shape collision.
    """
    drum_centers = _finite_points(drum_centers, "Drum centers")
    wheel_centers = _finite_points(wheel_centers, "Wheel centers")
    _validate_bound_radii(drum_bound_radius, wheel_bound_radius)
    pair_gaps = [
        (
            math.dist(drum, wheel) - drum_bound_radius - wheel_bound_radius,
            drum_index,
            wheel_index,
        )
        for drum_index, drum in enumerate(drum_centers)
        for wheel_index, wheel in enumerate(wheel_centers)
    ]
    return min(pair_gaps)

def obstacle_drum_wheel_gaps(
    level_drums, wheelbase, track, wheel_radius, pivot_offset,
    height, drum_bound_radius, wheel_bound_radius,
):
    """Return (gap, raised wheel, drum, wheel) for each obstacle case.

    Unreachable poses raise an error rather than reporting guarded clearance.
    """
    level_drums = _finite_points(level_drums, "Level drum centers")
    _validate_bound_radii(drum_bound_radius, wheel_bound_radius)
    level_shaft_height = wheel_radius + pivot_offset
    cases = []

    for raised in range(4):
        case_pose = raised_wheel_pose(
            wheelbase=wheelbase, track=track, radius=wheel_radius,
            pivot_offset=pivot_offset, height=height, raised=raised,
        )

        if not case_pose["valid"]:
            raise ValueError("Requested obstacle pose is unreachable.")

        case_drums = [
            chassis_point_in_world(point, case_pose, level_shaft_height)
            for point in level_drums
        ]

        gap, drum_index, wheel_index = minimum_sphere_gap(
            case_drums, case_pose["centers"],
            drum_bound_radius, wheel_bound_radius,
        )
        cases.append((gap, raised, drum_index, wheel_index))

    return cases


def evaluate_obstacle_clearance(values):
    """Evaluate the component contract from ordinary SI scalar inputs.

    At reachable poses the margin is the sphere gap bound minus the required
    mechanism clearance. Guarded diagnostics at unreachable poses never make
    a design feasible, including exactly at the guarded reach boundary.
    """
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("Obstacle clearance inputs must be finite.")
    for name in ("wheel_diameter", "wheel_width", "drum_diameter", "drum_length"):
        if values[name] <= 0:
            raise ValueError(f"{name} must be positive.")
    for name in ("drum_pair_gap", "mechanism_clearance"):
        if values[name] < 0:
            raise ValueError(f"{name} must be nonnegative.")

    wheel_radius = values["wheel_diameter"] / 2
    drum_radius = values["drum_diameter"] / 2
    drum_x, drum_z = values["travel_x"], values["travel_y"]  # linkage y is vertical
    drum_y = (values["drum_length"] + values["drum_pair_gap"]) / 2
    level_drums = [(sx * drum_x, sy * drum_y, drum_z)
                   for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
    wheel_bound_radius = math.hypot(wheel_radius, values["wheel_width"] / 2)
    drum_bound_radius = math.hypot(drum_radius, values["drum_length"] / 2)

    requested_pose = raised_wheel_pose(
        wheelbase=values["wheelbase"], track=values["track_width"], radius=wheel_radius,
        pivot_offset=values["rocker_pivot_offset"], height=values["obstacle_height"],
    )
    evaluated_height = requested_pose["used_height"]
    cases = obstacle_drum_wheel_gaps(
        level_drums=level_drums, wheelbase=values["wheelbase"], track=values["track_width"],
        wheel_radius=wheel_radius, pivot_offset=values["rocker_pivot_offset"],
        height=evaluated_height, drum_bound_radius=drum_bound_radius,
        wheel_bound_radius=wheel_bound_radius,
    )
    gap_bound = min(cases)[0]
    margin = gap_bound - values["mechanism_clearance"]
    if not requested_pose["valid"]:
        margin = min(margin, requested_pose["reach_margin"] - INVALID_POSE_OFFSET)
    return {
        "obstacle_drum_wheel_gap_bound": gap_bound,
        "obstacle_drum_wheel_margin": margin,
        "obstacle_clearance_pose_valid": float(requested_pose["valid"]),
        "obstacle_clearance_evaluated_height": evaluated_height,
    }
