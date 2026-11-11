"""Loaded travel-pose slope and single-wheel articulation stability screens.

Pure gravity calculations, not traction/climbing or terrain-contact simulation.
The rock screen retains the user's horizontal-platform, one-wheel case. Ideal
average-angle differential torque balance adds a wheel-unloading check to the
global support hull. See docs/TRAVERSAL_STABILITY.md for derivation and limits.
"""
import math
import numpy as np

from ..components import scalar
from ..discipline import Discipline
from .obstacle_stability import raised_wheel_pose, posed_loaded_cg, projected_support_margin, drive_centers

ROCK_SAMPLES = 41  # includes level ground and the requested endpoint


def slope_support(wheelbase, track, cg_x, cg_z, angle):
    """Worst gravity-line reserve on a planar incline, over every heading [m].

    Coordinates refer to the slope-attached frame (z normal to the plane).
    Paired drums make lateral CG zero. Gravity intersects the plane at a
    displacement cg_z*tan(angle) downhill. Checking both rectangular half-spans
    bounds every heading, including uphill/downhill and either cross-slope.
    Adequate friction/holding torque is assumed, not established by this test.
    """
    if (not all(math.isfinite(v) for v in (wheelbase, track, cg_x, cg_z, angle))
            or wheelbase <= 0 or track <= 0 or cg_z <= 0 or not 0 <= angle < math.pi/2):
        raise ValueError("Slope screen needs positive spans/CG height and angle in [0, pi/2).")
    downhill = cg_z*math.tan(angle)
    pitch = wheelbase/2-abs(cg_x)-downhill
    roll = track/2-downhill
    limit = math.atan2(min(wheelbase/2-abs(cg_x), track/2), cg_z)
    return {"slope_pitch_tip_margin": pitch, "slope_roll_tip_margin": roll,
            "slope_tip_margin": min(pitch, roll), "slope_tip_angle": limit}


def articulated_cg(pose, radius, offset, sprung_mass, sprung_x, sprung_z,
                   wheel_drive_mass, rocker_mass, rocker_centroid_fraction, *, drive_mass=0., drive_cg_offset=0.):
    """Conserve existing mass groups; interpolate uniform rocker centroid only.

    Fraction 1 retains reference wheel-midpoint placement; fraction 0.5 is the
    historical V rocker; fraction 0 puts a straight rocker at its shaft bearing.
    """
    if (not all(math.isfinite(v) for v in (wheel_drive_mass, rocker_mass, rocker_centroid_fraction))
            or min(wheel_drive_mass, rocker_mass) < 0 or not 0 <= rocker_centroid_fraction <= 1):
        raise ValueError("Invalid suspension mass/centroid fraction.")
    body = np.asarray(posed_loaded_cg(pose, radius, offset, sprung_mass, sprung_x, sprung_z, 0.))
    wheel = np.mean(pose["centers"], axis=0)
    rocker = (1-rocker_centroid_fraction)*np.asarray(pose["shaft_origin"])+rocker_centroid_fraction*wheel
    total = sprung_mass+wheel_drive_mass+rocker_mass
    if not math.isfinite(drive_mass) or not 0 <= drive_mass <= wheel_drive_mass:
        raise ValueError("Drive mass must be a nonnegative subset of wheel/drive mass.")
    motor_offset = np.mean(drive_centers(pose, drive_cg_offset),axis=0)-wheel
    return tuple((sprung_mass*body+wheel_drive_mass*wheel+rocker_mass*rocker+drive_mass*motor_offset)/total)


def wheel_reactions(pose, cg, sprung_mass, wheel_drive_mass, rocker_mass,
                    gravity, rocker_centroid_fraction=1., *, drive_mass=0., drive_cg_offset=0.):
    """Force/moment and ideal differential balance; signed normals [N].

    Gravity may be a positive magnitude (horizontal platforms) or a vector in
    the terrain frame. Holding forces are parallel to opposite gravity, with
    the same tangential/normal ratio at each contact. Adequate friction and
    motor holding torque are assumed. Negative reactions remain failures.
    """
    contacts, cg = np.asarray(pose["contacts"], float), np.asarray(cg, float)
    g = np.array((0., 0., -gravity)) if np.ndim(gravity) == 0 else np.asarray(gravity, float)
    if (contacts.shape != (4, 3) or cg.shape != (3,) or g.shape != (3,)
            or not all(np.isfinite(a).all() for a in (contacts, cg, g)) or g[2] >= 0
            or not all(math.isfinite(v) for v in (sprung_mass, wheel_drive_mass, rocker_mass, rocker_centroid_fraction))
            or sprung_mass <= 0 or min(wheel_drive_mass, rocker_mass) < 0
            or not 0 <= rocker_centroid_fraction <= 1):
        raise ValueError("Four finite contacts and valid mass/gravity data required.")
    origin = np.asarray(pose["shaft_origin"])
    x, y, z = (contacts-origin).T
    cx, cy, cz = (np.asarray(pose["centers"])-origin).T
    span_x, span_y = np.ptp(x), np.ptp(y)
    if min(span_x, span_y) <= 1e-9:
        raise ValueError("Degenerate contact support.")
    cr, sr = math.cos(pose["roll"]), math.sin(pose["roll"])
    side = np.array((1., -1., 1., -1.))
    direction = -g/(-g[2])  # contact force per unit normal force
    torque = side*(direction[0]*(cr*z-sr*y)+x*(sr*direction[1]-cr))
    own_moment = (wheel_drive_mass+rocker_centroid_fraction*rocker_mass)/4*float(
        side @ (g[0]*(cr*cz-sr*cy)+cx*(sr*g[1]-cr*g[2])))
    if not math.isfinite(drive_mass) or not 0 <= drive_mass <= wheel_drive_mass:
        raise ValueError("Drive mass must be a nonnegative subset of wheel/drive mass.")
    dx,dy,dz = (np.asarray(drive_centers(pose,drive_cg_offset))-np.asarray(pose["centers"])).T
    own_moment += drive_mass/4*float(side @ (g[0]*(cr*dz-sr*dy)+dx*(sr*g[1]-cr*g[2])))
    matrix = np.array((np.ones(4), (x-direction[0]*z)/span_x,
                       (y-direction[1]*z)/span_y, torque/span_x))
    if np.linalg.cond(matrix) > 1e10:
        raise ValueError("Differential reaction balance is singular.")
    mass, c = sprung_mass+wheel_drive_mass+rocker_mass, cg-origin
    target = np.array((mass*(-g[2]), mass*(-g[2])*(c[0]-direction[0]*c[2])/span_x,
                       mass*(-g[2])*(c[1]-direction[1]*c[2])/span_y, -own_moment/span_x))
    forces = np.linalg.solve(matrix, target)
    return forces, float(np.max(np.abs(matrix @ forces-target)))


def rock_path_screen(values, *, samples=ROCK_SAMPLES, rocker_centroid_fraction=1., include_trace=False, wheel_model=None):
    """Sample all four one-wheel raises from zero to obstacle_height.

    Finite sampling is explicit; denser independent re-evaluation is required
    when assessing an optimized design. No claim of a real rock approach path.
    Unreachable endpoints retain the separate failing reach constraint and
    force both path margins negative; guarded diagnostics are labeled.
    """
    if not isinstance(samples, int) or samples < 2:
        raise ValueError("Rock sampling requires an integer count >= 2.")
    v = values.__getitem__
    radius, offset = v("wheel_diameter")/2, v("rocker_pivot_offset")
    end = raised_wheel_pose(v("wheelbase"), v("track_width"), radius, offset, v("obstacle_height"))
    wd = v("wheel_mass")+v("drive_mass")
    tips, loads = [], []
    error, poses, previous = 0., [], {}
    # Mirrored masses give identical left/right (and, at x=0, fore/aft) cases.
    choices = range(4) if wheel_model is None else ((0,) if abs(wheel_model.sprung_x) < 1e-14 else (0, 2))
    for height in np.linspace(0., end["used_height"], samples):
        for raised in choices:
            if wheel_model is None:
                pose = raised_wheel_pose(v("wheelbase"), v("track_width"), radius, offset, float(height), raised)
                cg = articulated_cg(pose, radius, offset, v("loaded_sprung_mass"),
                                     v("loaded_sprung_cg_x"), v("loaded_sprung_cg_z"),
                                     wd, v("rocker_mass"), rocker_centroid_fraction,
                                     drive_mass=v("drive_mass"), drive_cg_offset=values.get("drive_motor_cg_offset",0.))
                forces, residual = wheel_reactions(pose, cg, v("loaded_sprung_mass"), wd,
                                                   v("rocker_mass"), v("gravity"), rocker_centroid_fraction,
                                                   drive_mass=v("drive_mass"), drive_cg_offset=values.get("drive_motor_cg_offset",0.))
            else:
                terrain = np.zeros(4)
                terrain[raised] = height
                pose = wheel_model.solve(terrain=terrain, initial=previous.get(raised))
                cg, forces, residual = pose["cg"], pose["normal"], pose["force_error"]
                if pose["equation_valid"]:
                    previous[raised] = pose["q"]
            poses.append(pose)
            error = max(error, residual)
            tips.append((projected_support_margin(pose["contacts"], cg), float(height), raised, pose, cg))
            unloaded = int(np.argmin(forces))
            loads.append((float(forces[unloaded]), float(height), raised, unloaded))
    tip, height, raised, pose, cg = min(tips, key=lambda c: c[0])
    load, load_height, load_raised, unloaded = min(loads, key=lambda c: c[0])
    if not end["valid"]:
        tip = min(tip, end["reach_margin"]-1e-4)
        load = min(load, -1.)
    result = {"rock_sweep_tip_margin": tip, "rock_sweep_worst_height": height,
            "rock_sweep_raised_wheel": float(raised),
            "rock_sweep_pitch": pose["pitch"], "rock_sweep_roll": pose["roll"],
            "rock_sweep_cg_x": cg[0], "rock_sweep_cg_y": cg[1], "rock_sweep_cg_z": cg[2],
            "rock_sweep_wheel_load_margin": load, "rock_sweep_load_height": load_height,
            "rock_sweep_load_raised_wheel": float(load_raised), "rock_sweep_unloaded_wheel": float(unloaded),
            "rock_sweep_pose_valid": float(end["valid"]), "rock_sweep_evaluated_height": end["used_height"],
            "rock_sweep_samples": float(samples), "rock_sweep_force_balance_error": error}
    if wheel_model is not None:
        endpoint = min(tips[-len(choices):], key=lambda c: c[0])
        result.update(rock_equilibrium_margin=min(p["equilibrium_margin"] for p in poses),
                      rock_equilibrium_valid=float(end["valid"] and all(p["contact_valid"] for p in poses)),
                      rock_pose_error=max(p["pose_error"] for p in poses),
                      endpoint=endpoint[3], endpoint_wheel=endpoint[2], reach=end)
        if not end["valid"]:
            result["rock_equilibrium_margin"] = min(result["rock_equilibrium_margin"], -1.)
    if include_trace:
        result["trace"] = [
            {"height": tips[i][1], "tip": min(c[0] for c in tips[i:i+len(choices)]),
             "load": min(c[0] for c in loads[i:i+len(choices)])}
            for i in range(0, len(tips), len(choices))
        ]
    return result


class TraversalStability(Discipline):
    CONFIG_INPUTS = ("wheelbase", "track_width", "wheel_diameter", "rocker_pivot_offset",
                     "obstacle_height", "slope_angle", "gravity", "drive_motor_cg_offset")
    COUPLED_INPUTS = {"loaded_cg_x": "m", "loaded_cg_z": "m", "loaded_sprung_mass": "kg",
                      "loaded_sprung_cg_x": "m", "loaded_sprung_cg_z": "m",
                      "wheel_mass": "kg", "drive_mass": "kg", "rocker_mass": "kg"}
    OUTPUT_UNITS = {"slope_pitch_tip_margin": "m", "slope_roll_tip_margin": "m",
                    "slope_tip_margin": "m", "slope_tip_angle": "rad",
                    "rock_sweep_tip_margin": "m", "rock_sweep_worst_height": "m",
                    "rock_sweep_raised_wheel": None, "rock_sweep_pitch": "rad", "rock_sweep_roll": "rad",
                    "rock_sweep_cg_x": "m", "rock_sweep_cg_y": "m", "rock_sweep_cg_z": "m",
                    "rock_sweep_wheel_load_margin": "N", "rock_sweep_load_height": "m",
                    "rock_sweep_load_raised_wheel": None, "rock_sweep_unloaded_wheel": None,
                    "rock_sweep_pose_valid": None, "rock_sweep_evaluated_height": "m",
                    "rock_sweep_samples": None, "rock_sweep_force_balance_error": "N"}
    slope_inputs = {"wheelbase", "track_width", "loaded_cg_x", "loaded_cg_z", "slope_angle"}
    rock_inputs = (set(CONFIG_INPUTS) | set(COUPLED_INPUTS))-{"slope_angle", "loaded_cg_x", "loaded_cg_z"}
    DEPENDENCIES = dict.fromkeys(OUTPUT_UNITS, rock_inputs)
    for n in OUTPUT_UNITS:
        if n.startswith("slope_"):
            DEPENDENCIES[n] = slope_inputs
    DEPENDENCIES["rock_sweep_samples"] = set()
    rocker_centroid_fraction = 1.
    model_evidence = {"revision": 1, "status": "analytical quasi-static screen; no experimental validation",
                      "loading": "all four drums loaded; fixed travel arms; slope and rock tested separately",
                      "slope": "user requirement 30 degrees, planar rigid contact, every heading; friction/holding torque assumed",
                      "rock": "user retained one raised wheel; 41 heights from zero to obstacle_height (default 0.30 m), each wheel in turn",
                      "reaction": "ideal frictionless average-angle differential gives equal left/right pitch moments; reject negative platform reaction",
                      "omissions": "real rock profile, combined slope/rock, post-lift contact sequence, joint stops/strength, sliding, motor torque, impact and moving payload"}

    def compute(self, inputs, outputs):
        values = {n: scalar(inputs, n) for n in inputs}
        result = slope_support(values["wheelbase"], values["track_width"], values["loaded_cg_x"],
                               values["loaded_cg_z"], values["slope_angle"])
        result.update(rock_path_screen(values, rocker_centroid_fraction=self.rocker_centroid_fraction))
        for name in self.OUTPUT_UNITS:
            outputs[name] = result[name]


class PlateTraversalStability(TraversalStability):
    rocker_centroid_fraction = 0.
    model_evidence = {**TraversalStability.model_evidence,
                      "rocker_mass": "one straight tube per side, centered on its shaft bearing; rigid massless end brackets"}
