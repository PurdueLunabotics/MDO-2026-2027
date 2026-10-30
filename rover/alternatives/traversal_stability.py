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
from .obstacle_stability import raised_wheel_pose, posed_loaded_cg, projected_support_margin

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
                   wheel_drive_mass, rocker_mass, rocker_centroid_fraction):
    """Conserve existing mass groups; interpolate uniform rocker centroid only.

    Fraction 1 reproduces the reference wheel-midpoint rocker placement;
    fraction 0.5 reproduces the plate-profile sloping half-arm placement.
    """
    if (not all(math.isfinite(v) for v in (wheel_drive_mass, rocker_mass, rocker_centroid_fraction))
            or min(wheel_drive_mass, rocker_mass) < 0 or not 0 <= rocker_centroid_fraction <= 1):
        raise ValueError("Invalid suspension mass/centroid fraction.")
    body = np.asarray(posed_loaded_cg(pose, radius, offset, sprung_mass, sprung_x, sprung_z, 0.))
    wheel = np.mean(pose["centers"], axis=0)
    rocker = (1-rocker_centroid_fraction)*np.asarray(pose["shaft_origin"])+rocker_centroid_fraction*wheel
    total = sprung_mass+wheel_drive_mass+rocker_mass
    return tuple((sprung_mass*body+wheel_drive_mass*wheel+rocker_mass*rocker)/total)


def wheel_reactions(pose, cg, sprung_mass, wheel_drive_mass, rocker_mass,
                    gravity, rocker_centroid_fraction=1.):
    """Unique vertical platform reactions under ideal differential equilibrium.

    FL,FR,RL,RR. Three equations balance whole-rover weight and moments.
    The fourth equates the left/right external rocker pitch moments after
    subtracting each side's wheel/drive and rocker gravity moment. Equal pitch
    moments follow ideal virtual work for theta_body=(theta_L+theta_R)/2.
    Negative forces are returned unchanged: the prescribed four-contact pose
    would require a wheel to pull on the ground and cannot be held statically.
    """
    contacts = np.asarray(pose["contacts"], dtype=float)
    cg = np.asarray(cg, dtype=float)
    if (contacts.shape != (4, 3) or cg.shape != (3,)
            or not np.isfinite(contacts).all() or not np.isfinite(cg).all()
            or not all(math.isfinite(v) for v in (sprung_mass, wheel_drive_mass, rocker_mass, gravity, rocker_centroid_fraction))
            or sprung_mass <= 0 or gravity <= 0 or min(wheel_drive_mass, rocker_mass) < 0
            or not 0 <= rocker_centroid_fraction <= 1):
        raise ValueError("Four finite contacts and valid mass/gravity data required.")
    x, y = contacts[:, 0], contacts[:, 1]
    # Translate to the shaft origin; conditioning scales do not alter physics.
    x = x-float(pose["shaft_origin"][0])
    y = y-float(pose["shaft_origin"][1])
    span_x, span_y = np.ptp(x), np.ptp(y)
    if min(span_x, span_y) <= 1e-9:
        raise ValueError("Degenerate contact support.")
    side = np.array((1., -1., 1., -1.))
    matrix = np.array((np.ones(4), x/span_x, y/span_y, side*x/span_x))
    if np.linalg.cond(matrix) > 1e10:
        raise ValueError("Differential reaction balance is singular.")
    mass = sprung_mass+wheel_drive_mass+rocker_mass
    difference = (wheel_drive_mass+rocker_centroid_fraction*rocker_mass)/4*float(side @ x)
    target = gravity*np.array((mass, mass*(cg[0]-pose["shaft_origin"][0])/span_x,
                               mass*(cg[1]-pose["shaft_origin"][1])/span_y, difference/span_x))
    forces = np.linalg.solve(matrix, target)
    residual = float(np.max(np.abs(matrix @ forces-target)))
    return forces, residual


def rock_path_screen(values, *, samples=ROCK_SAMPLES, rocker_centroid_fraction=1., include_trace=False):
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
    error = 0.
    for height in np.linspace(0., end["used_height"], samples):
        for raised in range(4):
            pose = raised_wheel_pose(v("wheelbase"), v("track_width"), radius, offset, float(height), raised)
            cg = articulated_cg(pose, radius, offset, v("loaded_sprung_mass"),
                                 v("loaded_sprung_cg_x"), v("loaded_sprung_cg_z"),
                                 wd, v("rocker_mass"), rocker_centroid_fraction)
            forces, residual = wheel_reactions(pose, cg, v("loaded_sprung_mass"), wd,
                                               v("rocker_mass"), v("gravity"), rocker_centroid_fraction)
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
    if include_trace:
        result["trace"] = [
            {"height": tips[i][1], "tip": min(c[0] for c in tips[i:i+4]),
             "load": min(c[0] for c in loads[i:i+4])}
            for i in range(0, len(tips), 4)
        ]
    return result


class TraversalStability(Discipline):
    CONFIG_INPUTS = ("wheelbase", "track_width", "wheel_diameter", "rocker_pivot_offset",
                     "obstacle_height", "slope_angle", "gravity")
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
    rocker_centroid_fraction = .5
    model_evidence = {**TraversalStability.model_evidence,
                      "rocker_mass": "plate-profile uniform sloping half-arms; centroid midway between shaft and wheel midpoint"}
