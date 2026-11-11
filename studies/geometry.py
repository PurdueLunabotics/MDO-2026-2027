"""Geometry search using existing ModelSet adapters, followed by full-model audit.

The inexpensive search checks selected exact equilibrium poses. Only a complete
existing compliant slope/rock screen can certify a saved geometry candidate.
No mission feasibility or global optimality is inferred from this study.
"""
from dataclasses import replace
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.optimize import minimize

from rover.config import configuration, REQUIREMENTS, MINIMUM_MARGINS
from rover.models import compliant_plate_frame_models
from rover.problem import MARGINS, PLATE_FRAME_MARGINS, MOVING_EXCAVATION_MARGINS
from rover.alternatives.pa04_actuator import PA04LinkageGeometry, PA04LinkageLoads
from rover.alternatives.parallel_excavation_actuation import parallel_load_factory
from rover.alternatives.x44_motor import X44MotorProvider
from rover.motor_inventory import CatalogAssemblyMass
from rover.alternatives.offset_attachment import attachment_point
from rover.alternatives.compliant_tipping import StaticWheelModel, coupled_slope_screen
from rover.alternatives.obstacle_stability import raised_wheel_pose
from rover.alternatives.traversal_stability import rock_path_screen
from rover.mobility import wheel_contact_area, peak_ground_pressure
from rover.alternatives.parallel_cylinder_clearance import chassis_point_in_world, unroll_point, parallel_cylinder_gap


GEOMETRY_MARGINS = tuple("""
ground_clearance_margin ground_pressure_margin pit_depth_margin transport_clearance_margin wheel_first_margin
deposition_reach_margin deposition_position_margin deposition_clearance_margin
startup_position_margin dig_position_margin pivot_spacing_margin pivot_mount_margin
pivot_longitudinal_margin actuator_pivot_longitudinal_margin bottom_mount_margin
stroke_margin triangle_retracted_margin triangle_extended_margin toggle_margin
actuator_toggle_margin wheel_clearance_margin deployed_height_margin
actuator_force_margin actuator_curve_margin actuator_stroke_domain_margin
cut_depth_margin frame_section_margin arm_section_margin structure_stress_margin
structure_deflection_margin structure_buckling_margin tip_stability_margin
plate_geometry_margin frame_packing_margin plate_stress_margin plate_buckling_margin
cross_tube_bending_margin operating_reach_margin axle_ground_margin
excavation_domain_margin obstacle_drum_wheel_margin
""".split())

ANGLE_COMMANDS = {
    "travel_angle": "travel_actuator_length",
    "deposition_angle": "deposition_actuator_length",
    "stowed_angle": "actuator_retracted_length",
    "extended_angle": "actuator_extended_length",
}
VARIABLES = ("wheelbase", "wheel_width", "track_width", "drum_length", "arm_length", "bottom_pivot_height",
             "pivot_x", "actuator_pivot_x", "top_pivot_height", "lin_act_side_arm_offset",
             "lin_act_top_arm_offset", "arm_box_height", "arm_wall", "plate_spoke_width",
             "axle_wall", *ANGLE_COMMANDS)
ANGLE_BOUNDS = {"travel_angle": (15., 85.), "deposition_angle": (15., 85.),
                "stowed_angle": (5., 85.), "extended_angle": (-70., -1.)}


class StudyBudget(RuntimeError):
    pass


class GeometryEvaluator:
    """Offline adapter execution; fresh explicit inputs on every evaluation.

    No physics is copied. These are the same factories/providers used by the
    plate-frame/X44/PA-04 model. Fast results require subsequent full screening.
    """
    def __init__(self, source, stroke, wheel_diameter, *, plunge_depth=None, vary_wheel=False):
        design, assumptions = configuration("plate-frame", "x44-standard", "pa04-900",
                                             actuator_stroke=stroke)
        self.design = design
        self.values = {n: s[0] for n, s in {**design, **assumptions}.items()}
        self.values.update(source)
        self.values.update(wheel_diameter=wheel_diameter, drum_diameter=.359,
                           actuator_catalog_stroke=stroke, actuator_installation_allowance=0.)
        self.plunge_depth = plunge_depth
        self.variables = VARIABLES+(("wheel_diameter",) if vary_wheel else ())
        models = replace(compliant_plate_frame_models(), linkage=PA04LinkageGeometry,
                         linkage_loads=parallel_load_factory(PA04LinkageLoads))
        self.components = []
        for slot in ("structure_geometry", "linkage", "wheel_mass", "drum_mass", "geometry",
                     "excavation", "linkage_loads", "mass_properties", "structure_loads",
                     "ground_clearance", "tip_stability", "obstacle_clearance"):
            component = getattr(models, slot)()
            if slot == "excavation":
                component.options["motor_model"] = X44MotorProvider
                component.prepare_force_model()
            if slot in ("geometry", "drum_mass"):
                component.options["assembly_mass_model"] = CatalogAssemblyMass
            self.components.append((slot, component))
        self.bounds = np.asarray([ANGLE_BOUNDS[n] if n in ANGLE_BOUNDS
                                  else design[n][1:3] for n in self.variables], float)
        self.scales = {**MARGINS, **PLATE_FRAME_MARGINS, **MOVING_EXCAVATION_MARGINS,
                       "actuator_stroke_domain_margin": (.5, "m"), "actuator_curve_margin": (4000., "N")}

    def encode(self, values):
        x = np.array([values[n] for n in self.variables], float)
        return np.clip((x-self.bounds[:, 0])/np.diff(self.bounds, axis=1)[:, 0], 0., 1.)

    def decode(self, x):
        physical = self.bounds[:, 0]+x*np.diff(self.bounds, axis=1)[:, 0]
        variables = dict(zip(self.variables, physical))
        values = {**self.values, **{n: v for n, v in variables.items() if n not in ANGLE_COMMANDS}}
        for name, command in ANGLE_COMMANDS.items():
            angle = math.radians(variables[name])
            cx = values["pivot_x"]+values["arm_length"]*math.cos(angle)
            cz = values["bottom_pivot_height"]+values["arm_length"]*math.sin(angle)
            pin = attachment_point(values["pivot_x"], values["bottom_pivot_height"], cx, cz,
                                   values["lin_act_side_arm_offset"], values["lin_act_top_arm_offset"])
            values[command] = math.hypot(pin[0]-values["actuator_pivot_x"],
                                         pin[1]-values["top_pivot_height"])
        return values

    def evaluate(self, x, angle=30.1, headings=(0., math.pi/4, math.pi/2), rocks=(.3,)):
        values = self.decode(x)
        return self.evaluate_values(values, angle, headings, rocks)

    def evaluate_values(self, values, angle=30.1, headings=(0., math.pi/4, math.pi/2), rocks=(.3,)):
        values = dict(values)
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError("Geometry study inputs must be finite")
        if (not math.isclose(values["actuator_catalog_stroke"], self.values["actuator_catalog_stroke"], rel_tol=0., abs_tol=1e-12)
                or values["actuator_installation_allowance"] != 0.):
            raise ValueError("Geometry candidate must retain the selected discrete actuator hardware")
        original = dict(values)
        for slot, component in self.components:
            # Only existing declared inputs are passed when available. Older
            # SimpleComponents intentionally accept the complete scalar map.
            declared = set(getattr(component, "CONFIG_INPUTS", ())) | set(getattr(component, "COUPLED_INPUTS", {}))
            supplied = {n: np.asarray([v]) for n, v in values.items() if not declared or n in declared}
            # Provider-specific inputs are selected by setup in OpenMDAO. The
            # standalone mass/excavation adapters need the full explicit map.
            if slot in ("drum_mass", "geometry", "excavation", "linkage_loads"):
                supplied = {n: np.asarray([v]) for n, v in values.items()}
            outputs = {}
            component.compute(supplied, outputs)
            values.update({n: float(v) for n, v in outputs.items()})
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError("Geometry adapter returned a nonfinite prediction")
        pressure_factors = []
        deposition_acceleration = values["acceleration"] * values["turn_accel_fraction"]
        for state, mass, acceleration in (("loaded", values["dry_mass"]+values["payload_mass"], values["acceleration"]),
                                          ("empty", values["dry_mass"], values["acceleration"]),
                                          ("deposition_half", values["dry_mass"]+values["payload_mass"]/2, deposition_acceleration)):
            factor = 1+2*(abs(values[state+"_cg_x"])+values[state+"_cg_z"]*acceleration/values["gravity"])/values["wheelbase"]
            pressure_factors.append((mass, factor))
        area = wheel_contact_area(values["wheel_diameter"], values["wheel_width"], values["wheel_deflection_fraction"])
        values["ground_pressure"] = peak_ground_pressure(values["rocker_load_factor"], values["gravity"], area, pressure_factors)
        values["ground_pressure_margin"] = values["allowable_ground_pressure"]-values["ground_pressure"]
        constraints = {}
        for name, (upper, _) in REQUIREMENTS.items():
            constraints[name] = (upper*.999-values[name])/upper
        for name in GEOMETRY_MARGINS:
            scale = self.scales[name][0]
            minimum = MINIMUM_MARGINS.get(name, (0., None))[0]
            constraints[name] = (values[name]-minimum-1e-3*scale)/scale
        for name in self.design:
            _, low, high, _ = self.design[name]
            buffer = 1e-5 if name in ANGLE_COMMANDS.values() else 0.
            constraints[name+"_bounds"] = (min(values[name]-low, high-values[name])-buffer)/(high-low)
        model = StaticWheelModel.from_values(values, 0.)
        flat = model.solve()
        constraints["flat_equilibrium"] = flat["equilibrium_margin"]
        constraints["flat_contact"] = float(min(flat["normal"]))/500.
        constraints["obstacle_pose_margin"] = raised_wheel_pose(
            values["wheelbase"], values["track_width"], values["wheel_diameter"]/2,
            values["rocker_pivot_offset"], values["obstacle_height"])["reach_margin"]/.4
        for i, heading in enumerate(headings):
            pose = model.solve(math.radians(angle), heading)
            constraints[f"slope_{i}_tip"] = (pose["tip_margin"]-.0003)/.3
            constraints[f"slope_{i}_load"] = (float(min(pose["normal"]))-.5)/500.
            constraints[f"slope_{i}_equilibrium"] = pose["equilibrium_margin"]
        for i, height in enumerate(rocks):
            pose = model.solve(terrain=np.array((height, 0., 0., 0.)))
            constraints[f"rock_{i}_tip"] = (pose["tip_margin"]-.0004)/.4
            constraints[f"rock_{i}_load"] = (float(min(pose["normal"]))-.5)/500.
            constraints[f"rock_{i}_equilibrium"] = pose["equilibrium_margin"]
        if self.plunge_depth is not None:
            constraints["plunge_depth"] = (-values["extended_y"]+values["drum_diameter"]/2-self.plunge_depth)/.45
        return original, values, constraints


def search(evaluator, seed, *, seconds=45., angle=30.1, headings=(0., math.pi/4, math.pi/2), rocks=(.3,), maximize=False):
    """Bounded local search; preserve actual solver status and best screening point."""
    start = time.perf_counter()
    deadline = start+seconds
    initial = evaluator.encode(seed)
    max_angle = maximize is True or maximize == "angle"
    max_wheel = maximize == "wheel"
    if max_angle:
        initial = np.r_[initial, (angle-30.1)/29.9]
    last, best, calls = [None, None], [None, math.inf], 0

    def at(x):
        nonlocal calls
        if time.perf_counter() >= deadline:
            raise StudyBudget("Geometry search time budget reached")
        if last[0] is not None and np.array_equal(x, last[0]):
            return last[1]
        target = 30.1+29.9*x[-1] if max_angle else angle
        result = evaluator.evaluate(x[:-1] if max_angle else x, target, headings, rocks)
        c = np.asarray(list(result[2].values()))
        violation = float(np.sum(np.minimum(c, 0.)**2))
        quality = violation if violation > 1e-12 else (-result[0]['wheel_diameter'] if max_wheel else -target)
        if quality < best[1]:
            best[:] = [x.copy(), quality]
        last[:] = [x.copy(), (result, c)]
        calls += 1
        return last[1]

    success, message = False, "Not run"
    try:
        result = minimize(lambda x: -x[-1] if (max_angle or max_wheel) else .0001*float(np.sum((x-initial)**2)),
                          initial, method="SLSQP", bounds=[(1e-9, 1.-1e-9)]*len(initial),
                          constraints=[{"type": "ineq", "fun": lambda x: at(x)[1]}],
                          options={"maxiter": 100, "ftol": 1e-8, "eps": 2e-5})
        success, message = bool(result.success), str(result.message)
        last_iterate = result.x.tolist()
    except StudyBudget as exc:
        message, last_iterate = str(exc), last[0].tolist() if last[0] is not None else None
    chosen = best[0] if best[0] is not None else initial
    target = 30.1+29.9*chosen[-1] if max_angle else angle
    inputs, outputs, constraints = evaluator.evaluate(chosen[:-1] if max_angle else chosen, target, headings, rocks)
    return {"inputs": inputs, "outputs": outputs, "screen_constraints": constraints,
            "screen_passed": min(constraints.values()) >= -1e-6,
            "target_angle_degrees": target, "optimizer_success": success,
            "optimizer_message": message, "last_optimizer_iterate": last_iterate,
            "selected_normalized_point": chosen.tolist(), "evaluations": calls,
            "elapsed_seconds": time.perf_counter()-start,
            "notice": "Candidate screening only; requires complete compliant slope/rock and full-system audit"}


def verify_geometry(evaluator, candidate):
    """Full existing heading/limit and 41-height rock checks, with cardinal angles."""
    inputs, values, _ = evaluator.evaluate_values(candidate["inputs"])
    # Verify physical requirements, not the extra inward numerical reserves
    # used by the optimizer. Never clip outputs or replace failed predictions.
    constraints = {n: upper-values[n] for n, (upper, _) in REQUIREMENTS.items()}
    constraints.update({n: values[n]-MINIMUM_MARGINS.get(n, (0., None))[0]
                        for n in GEOMETRY_MARGINS})
    for n, (_, low, high, _) in evaluator.design.items():
        constraints[n+"_bounds"] = min(inputs[n]-low, high-inputs[n])
    model = StaticWheelModel.from_values(values, 0.)
    slope = coupled_slope_screen(model, math.pi/6)
    rock = rock_path_screen(values, wheel_model=model)
    collision, collision_valid = math.inf, True
    yd = (values["drum_length"]+values["drum_pair_gap"])/2
    drums = [(sx*values["travel_x"], sy*yd, values["travel_y"])
             for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
    previous = {}
    for height in np.linspace(0., values["obstacle_height"], 41):
        for raised in range(4):
            terrain = np.zeros(4)
            terrain[raised] = height
            pose = model.solve(terrain=terrain, initial=previous.get(raised))
            if pose["equation_valid"]:
                previous[raised] = pose["q"]
            collision_valid &= bool(pose["contact_valid"])
            local_drums = [unroll_point(chassis_point_in_world(d, pose, model.radius+model.pivot_offset), pose)
                           for d in drums]
            wheels = [unroll_point(w, pose) for w in pose["centers"]]
            for drum in local_drums:
                for wheel in wheels:
                    gap = parallel_cylinder_gap(tuple(a-b for a, b in zip(drum, wheel)),
                                                values["drum_diameter"]/2, values["drum_length"],
                                                model.radius, values["wheel_width"])["gap"]
                    collision = min(collision, gap-values["mechanism_clearance"])
    constraints["compliant_rock_path_collision_margin"] = collision
    constraints["collision_pose_validity"] = 1. if collision_valid else -1.
    angles = {}
    for axis, heading in (("pitch", 0.), ("roll", math.pi/2)):
        for kind in ("tip", "load"):
            limit = model.limit(heading, kind)
            angles[f"{axis}_{kind}_limit_degrees"] = math.degrees(limit["angle"])
            constraints[f"{axis}_{kind}_above_30"] = limit["angle"]-math.pi/6 if limit["valid"] else -1.
    for name in ("slope_tip_margin", "slope_wheel_load_margin", "slope_equilibrium_margin",
                 "rock_sweep_tip_margin", "rock_sweep_wheel_load_margin", "rock_equilibrium_margin"):
        val = slope[name] if name.startswith("slope") else rock[name]
        constraints["full_"+name] = val
    constraints["slope_limit_above_30"] = slope["slope_contact_limit_angle"]-math.pi/6
    constraints["slope_numerical_validity"] = slope["slope_equilibrium_valid"]-.5
    constraints["rock_numerical_validity"] = rock["rock_equilibrium_valid"]-.5
    failures = {n: v for n, v in constraints.items() if v < 0}
    candidate.update(outputs=values, geometry_verified=not failures, verification_failures=failures,
                     verification_angles=angles,
                     full_slope_screen=slope,
                     full_rock_screen={n: v for n, v in rock.items() if n not in ("endpoint", "reach")},
                     compliant_rock_path_collision_margin=collision)
    return candidate


def main():
    import argparse
    from rover.linkage import drum_pose
    from reporting.case_inputs import read_case

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, default=Path("design_baseline.json"))
    parser.add_argument("--stroke", type=int, choices=(6, 8), required=True)
    parser.add_argument("--wheel-diameter", type=float, default=.359)
    parser.add_argument("--vary-wheel", action="store_true")
    parser.add_argument("--seconds", type=float, default=40.)
    parser.add_argument("--maximize", choices=("feasibility", "angle", "wheel"), default="feasibility")
    parser.add_argument("--seed", type=Path, help="Previous geometry study JSON, or a scalar seed including arm angles")
    parser.add_argument("--output", type=Path, default=Path("results/geometry_study.json"))
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be positive and finite")
    if not .18 <= args.wheel_diameter <= .359:
        parser.error("--wheel-diameter must lie in [0.18, 0.359] metres")
    if args.maximize == "wheel" and not args.vary_wheel:
        parser.error("--maximize wheel requires --vary-wheel")
    source, metadata = read_case(args.inputs)
    evaluator = GeometryEvaluator(source, args.stroke*.0254, args.wheel_diameter, vary_wheel=args.vary_wheel)
    previous = json.loads(args.seed.read_text(encoding="utf-8")) if args.seed else {}
    seed = {**evaluator.values, **previous.get("inputs", previous)}
    pose_names = {"travel_angle": ("travel_x", "travel_y"), "deposition_angle": ("deposition_x", "deposition_y"),
                  "stowed_angle": ("stowed_x", "stowed_y"), "extended_angle": ("extended_x", "extended_y")}
    for name, command in ANGLE_COMMANDS.items():
        if "outputs" in previous:
            x, z = (previous["outputs"][n] for n in pose_names[name])
        else:
            x, z = drum_pose(seed["arm_length"], seed["pivot_x"], seed["bottom_pivot_height"],
                             seed["actuator_pivot_x"], seed["top_pivot_height"], seed[command],
                             seed["lin_act_side_arm_offset"], seed["lin_act_top_arm_offset"])
        seed.setdefault(name, math.degrees(math.atan2(z-seed["bottom_pivot_height"], x-seed["pivot_x"])))
    result = search(evaluator, seed, seconds=args.seconds, maximize=args.maximize)
    result = verify_geometry(evaluator, result)
    result["source"] = metadata
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False, default=lambda x: x.item())+"\n", encoding="utf-8")
    print(f"Geometry verified: {result['geometry_verified']}; optimizer success: {result['optimizer_success']}")
    print(result["verification_angles"])
    print(f"Saved {args.output.resolve()}")
    return 0 if result["geometry_verified"] and result["optimizer_success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
