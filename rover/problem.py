"""The complete optimization formulation lives here, separate from the physics."""

import math
import hashlib
from pathlib import Path
import openmdao.api as om
from .config import DESIGN, ASSUMPTIONS, REQUIREMENTS, ROUTE, configuration
from .components import Mission
from .models import ModelSet, plate_frame_models
from .contracts import validate_interfaces

# Every margin must be >= 0. Values below are scales, not additional limits.
# Climb acceptance is strict: a zero reserve or failed solution does not pass.
STRICT_SLOPE_MARGINS = {"empty_slope_traversal_margin": "empty", "loaded_slope_traversal_margin": "loaded"}
MARGINS = {
    "empty_slope_traversal_margin": (1.0, None),
    "loaded_slope_traversal_margin": (1.0, None),
    "drum_power_margin": (200.0, "W"),
    "penetration_margin": (500.0, "N"),
    "dig_traction_margin": (300.0, "N"),
    "cut_depth_margin": (0.15, "m"),
    "ground_pressure_margin": (25000.0, "Pa"),
    "straight_motor_margin": (300.0, "N"),
    "turn_motor_margin": (300.0, "N"),
    "straight_traction_margin": (300.0, "N"),
    "turn_traction_margin": (300.0, "N"),
    "straight_power_margin": (400.0, "W"),
    "turn_power_margin": (400.0, "W"),
    "battery_energy_margin": (100.0, "W*h"),
    "battery_power_margin": (1000.0, "W"),
    "complete_cycle_margin": (900.0, "s"),
    "dig_reach_margin": (0.30, "m"),
    "pit_depth_margin": (0.45, "m"),
    "transport_clearance_margin": (0.30, "m"),
    "wheel_first_margin": (0.359, "m"),
    "startup_position_margin": (0.10, "m"),
    "dig_position_margin": (0.10, "m"),
    "pivot_spacing_margin": (0.40, "m"),
    "pivot_mount_margin": (0.75, "m"),
    "pivot_longitudinal_margin": (0.30, "m"),
    "actuator_pivot_longitudinal_margin": (0.30, "m"),
    "bottom_mount_margin": (0.30, "m"),
    "stroke_margin": (0.50, "m"),
    "actuator_body_margin": (0.50, "m"),
    "triangle_retracted_margin": (0.50, "m"),
    "triangle_extended_margin": (0.50, "m"),
    "toggle_margin": (0.50, "m"),
    "actuator_toggle_margin": (0.50, "m"),
    "wheel_clearance_margin": (0.30, "m"),
    "chassis_clearance_margin": (0.30, "m"),
    "deployed_height_margin": (2.50, "m"),
    "actuator_force_margin": (4000.0, "N"),
    "drum_torque_margin": (50.0, "N*m"), "dump_torque_margin": (25.0, "N*m"),
    "payload_margin": (5.0, "kg"), "trench_traverse_margin": (0.359, "m"),
    "fresh_site_margin": (0.5, "m"),
    "frame_section_margin": (0.02, "m"), "arm_section_margin": (0.02, "m"),
    "structure_stress_margin": (1e8, "Pa"), "structure_deflection_margin": (0.005, "m"),
    "structure_buckling_margin": (10000.0, "N"), "mobility_stability_margin": (0.3, "m"),
    "tip_stability_margin": (0.3, "m"),
    "obstacle_pose_margin": (0.4, "m"), "obstacle_tip_margin": (0.4, "m"),
    "obstacle_drum_wheel_margin": (0.4, "m"),
    "slope_tip_margin": (0.3, "m"),
    "rock_sweep_tip_margin": (0.4, "m"), "rock_sweep_wheel_load_margin": (500., "N"),
    "excavation_area_margin": (12.0, "m**2"),
    "site_drive_margin": (300.0, "N"),
}


PLATE_FRAME_MARGINS = {"plate_geometry_margin": (0.1, "m"), "frame_packing_margin": (0.1, "m"),
                      "plate_stress_margin": (1e8, "Pa"), "plate_buckling_margin": (10000., "N"),
                      "cf_bending_margin": (1e8, "Pa")}


def validate(values, design=None, assumptions=None):
    """Catch invalid inputs before they become division errors or NaNs in a driver."""
    design = DESIGN if design is None else design
    assumptions = ASSUMPTIONS if assumptions is None else assumptions
    zero_allowed = {"electronics_power", "actuator_idle_power", "rolling_base",
                    "scrub_coefficient", "internal_lift_factor", "vertical_cut_fraction",
                    "force_imbalance", "mass_score_weight", "energy_score_weight",
                    "obstacle_height", "rocker_pivot_offset", "slope_angle",
                    "drive_stator_current_limit", "drum_stator_current_limit",
                    "empty_slope_degrees", "loaded_slope_degrees"}
    fractions = {name for name in assumptions if "fraction" in name or "efficiency" in name}
    fractions |= {"cut_engagement", "delivery_efficiency", "force_imbalance"}
    for name, value in values.items():
        if not math.isfinite(value) or (value < 0 and name != "rocker_pivot_offset") or (value == 0 and name not in zero_allowed):
            raise ValueError(f"{name} must be finite and {'nonnegative' if name in zero_allowed else 'positive'}.")
        if name in fractions and value > 1:
            raise ValueError(f"{name} must be <= 1.")
    minimum_wheelbase = design["wheelbase"][1] if "wheelbase" in design else values["wheelbase"]
    if abs(values["rocker_pivot_offset"]) >= minimum_wheelbase/2:
        raise ValueError("Absolute rocker_pivot_offset must be smaller than half the minimum wheelbase.")
    if not 0 < values["wheel_deflection_fraction"] < 0.5:
        raise ValueError("wheel_deflection_fraction must be between 0 and 0.5.")
    if not 0 <= values["slope_angle"] < math.pi/2:
        raise ValueError("slope_angle must be in [0, pi/2) radians.")
    for name in ("empty_slope_degrees", "loaded_slope_degrees"):
        if not 0 <= values[name] < 90:
            raise ValueError(f"{name} must be in [0, 90) degrees.")
    if values["scoops_per_revolution"] != int(values["scoops_per_revolution"]):
        raise ValueError("scoops_per_revolution must be an integer.")
    if values["mass_score_weight"] + values["energy_score_weight"] == 0:
        raise ValueError("At least one score weight must be positive.")
    for name, (initial, lower, upper, _) in design.items():
        if not 0 < lower < upper or not lower <= values[name] <= upper:
            raise ValueError(f"{name} must start within its positive bounds [{lower}, {upper}].")
    for name in ("wheel_diameter", "wheel_width", "drum_diameter", "drum_length"):
        if values[name] >= 0.36 or (name in design and design[name][2] >= 0.36):
            raise ValueError(f"{name}: user requires a bound strictly below 0.360 m.")
    if values["turn_angle"] < math.pi:
        raise ValueError("turn_angle must include at least the required 180-degree deposition turn.")
    if not ROUTE or any(not math.isfinite(d + a) or d < 0 for d, a in ROUTE) or sum(d for d, _ in ROUTE) <= 0:
        raise ValueError("ROUTE needs finite nonnegative distances and nonzero total travel.")


def build_problem(overrides=None, *, optimizer=None, maxiter=None, display=False, models=None, structure="reference", cobyla_radius=None, motors="reference", solver_tol=None):
    # Plate envelopes contain min/max/contact branches. Use a small-step
    # derivative-free solve for that profile; keep explicit SLSQP available.
    optimizer = optimizer or ("COBYLA" if structure == "plate-frame" else "SLSQP")
    if optimizer not in ("SLSQP", "COBYLA"):
        raise ValueError("optimizer must be SLSQP or COBYLA.")
    if maxiter is None:
        maxiter = 2000 if structure == "plate-frame" and optimizer == "COBYLA" else 200
    if not isinstance(maxiter, int) or isinstance(maxiter, bool) or maxiter < 1:
        raise ValueError("maxiter must be a positive integer.")
    if solver_tol is None:
        solver_tol = 1e-5 if structure == "plate-frame" or motors == "x44-standard" else 1e-3
    if not math.isfinite(solver_tol) or solver_tol <= 0:
        raise ValueError("solver_tol must be positive and finite.")
    if cobyla_radius is None and structure == "plate-frame" and optimizer == "COBYLA":
        cobyla_radius = .02
    if cobyla_radius is not None and (optimizer != "COBYLA" or not math.isfinite(cobyla_radius) or cobyla_radius <= 0):
        raise ValueError("cobyla_radius needs a positive finite value and optimizer='COBYLA'.")
    design, assumptions = configuration(structure, motors)
    margins = {**MARGINS, **(PLATE_FRAME_MARGINS if structure == "plate-frame" else {})}
    diagnostic_screens = {
        "dig_froude_margin": "User removed the uncalibrated speed cap; constant capture/lift response assumes no additional high-speed loss.",
        "dump_froude_margin": "User removed the uncalibrated speed cap; actual dumping/retention speed response remains unverified.",
    }
    if structure == "plate-frame":
        margins.pop("chassis_clearance_margin")
        diagnostic_screens["chassis_clearance_margin"] = (
            "Legacy filled rectangular envelope does not represent the open plate frame; actual frame/equipment interference remains unverified.")
    models = models or (plate_frame_models() if structure == "plate-frame" else ModelSet())
    if motors == "x44-standard":
        from .alternatives.x44_motor import X44MotorProvider, verify_source_data
        from .motor_inventory import CatalogAssemblyMass
        verify_source_data()
    values = {name: spec[0] for name, spec in {**design, **assumptions}.items()}
    for name, value in (overrides or {}).items():
        if name not in values:
            raise ValueError(f"Unknown input {name!r}. See rover/config.py.")
        values[name] = float(value)
    validate(values, design, assumptions)
    if motors == "x44-standard" and (values["motor_bus_voltage"] != 12 or values["drive_gear_ratio"] != 100):
        raise ValueError("The selected hardware profile requires 12 V and a fixed 100:1 drive ratio.")
    problem = om.Problem(reports=False)
    problem.design_specs, problem.assumption_specs, problem.margin_specs = design, assumptions, margins
    problem.model_metadata = {"schema_version": 2, "structure_profile": structure, "motor_profile": motors,
                              "configuration": {"design": design, "assumptions": assumptions, "margins": margins},
                              "optimizer_settings": {"optimizer": optimizer, "maxiter": maxiter, "tol": solver_tol, "cobyla_radius": cobyla_radius},
                              "diagnostic_only_screens": diagnostic_screens,
                              "models": models.metadata(), "calibration": "uncalibrated; see model cards",
                              "source_files_sha256": {p.relative_to(Path(__file__).parent).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                                       for p in sorted(Path(__file__).parent.rglob("*.py"))}}
    if motors == "x44-standard":
        problem.model_metadata["motor_performance"] = {
            "curve": "data/motors/kraken_x44_standard_12v.csv",
            "manifest": "data/motors/manifest.json",
            "curve_sha256": hashlib.sha256((Path(__file__).parent.parent/"data/motors/kraken_x44_standard_12v.csv").read_bytes()).hexdigest(),
            "mode": "standard trapezoidal, 12 V", "drive_ratio": 100,
            "implementation": "shared Gearmotor; output work/peak divided by provisional combined efficiency",
            "energy_model_revision": 2,
            "combined_efficiencies": {role: values[role+"_efficiency"] for role in ("drive", "drum")},
            "continuous_operation_verified": False,
            "current_limits": {n: values[n] or "undecided; voltage envelope only" for n in ("drive_stator_current_limit", "drum_stator_current_limit")},
            "limitations": "Constant combined efficiencies are uncalibrated; no current/RMS/partial-duty electrical predictions, battery sag or thermal limits. Drum ratio is continuous exploratory; cartridge availability/ratings and motor adapter require verification.",
            "inactive_reference_inputs": ["drive_motor_power", "drum_motor_power", "motor_rated_rpm", "motor_mass_per_watt", "gear_mass_per_torque", "drive_fixed_mass", "drum_fixed_mass"],
            "power_margin_semantics": "Necessary instantaneous manufacturer peak-curve screen, not a continuous rating; torque-speed/current envelope also enforced."}
    problem.model_metadata["traversal_requirements"] = {
        "obstacle_height_m": values["obstacle_height"],
        "obstacle_status": "posed gravity support, geometric reach and conservative drum/wheel envelope screens only",
        "slope_angle_rad": values["slope_angle"],
        "slope_status": "fully loaded static planar slope support; separate from BWR climbing requirements",
        "empty_climb_degrees": values["empty_slope_degrees"],
        "loaded_climb_degrees": values["loaded_slope_degrees"],
        "climb_acceptance": "BWR driving solution with slip strictly less than 1, empty=dry and full=dry+payload; equal mean wheel loads, no motor/speed acceptance test",
        "rock_path_status": "one wheel at a time, 41 sampled heights; ideal differential reaction balance; real crossing path unverified",
        "ditch_depth_m": 0.50, "ditch_profile": "user: circle-like",
        "ditch_status": "unverified: width/radius and crossing contact sequence unknown"}
    model = problem.model
    def add_discipline(slot):
        component = getattr(models,slot)()
        # Preserve the selected discipline (including student replacements).
        # Motor hardware changes only its declared provider/inventory options.
        if motors == "x44-standard":
            option = ("motor_model" if slot in ("excavation","mobility","relocation")
                      else "assembly_mass_model" if slot in ("geometry","drum_mass") else None)
            if option:
                if option not in component.options:
                    raise ValueError(f"{slot} must expose the {option!r} option to use the X44 hardware profile; the selected discipline was not replaced.")
                component.options[option] = X44MotorProvider if option == "motor_model" else CatalogAssemblyMass
        model.add_subsystem(slot,component,promotes=["*"])
    inputs = om.IndepVarComp()
    for name, value in values.items():
        spec = design.get(name, assumptions.get(name))
        inputs.add_output(name, val=value, units=spec[-1])
    model.add_subsystem("inputs", inputs, promotes=["*"])
    # Kinematics/mass -> geometry -> excavation -> linkage loads and mobility -> mission.
    # Battery capacity is a design variable, avoiding a hidden mass/energy sizing loop.
    for slot in ("structure_geometry","linkage","wheel_mass","drum_mass","geometry",
                 "excavation","linkage_loads","mass_properties","structure_loads",
                 "relocation","trench_traversal","tip_stability","obstacle_stability",
                 "obstacle_clearance","traversal_stability","mobility","slope_traversal"):
        add_discipline(slot)
    model.add_subsystem("mission", Mission(), promotes=["*"])
    for name, (_, lower, upper, _) in design.items():
        model.add_design_var(name, lower=lower, upper=upper, ref0=lower, ref=upper)
    # Small inward reserves stop a numerically successful solve from ending just
    # outside a hard requirement when many constraints become active together.
    for name, (upper, _) in REQUIREMENTS.items():
        model.add_constraint(name, upper=upper*(1-1e-3), ref=upper)
    for name, (scale, _) in margins.items():
        model.add_constraint(name, lower=0. if name in STRICT_SLOPE_MARGINS else 1e-3*scale, ref=scale)
    # V2 finite-soil scores are typically tens to hundreds, not the old thousands.
    model.add_objective("construction_points", scaler=-1 / 100.0)  # maximize
    # Deliberately recompute the full model for perturbations: easy to replace models.
    # Avoid complex-step assumptions, derivative coloring, or hand-maintained Jacobians.
    model.approx_totals(method="fd", form="central", step=1e-5, step_calc="rel_avg")
    problem.driver = om.ScipyOptimizeDriver(optimizer=optimizer, tol=solver_tol)
    problem.driver.options["maxiter"] = maxiter
    problem.driver.options["disp"] = display
    if cobyla_radius is not None:
        problem.driver.opt_settings["rhobeg"] = cobyla_radius
    try:
        problem.setup()
        validate_interfaces(problem)
    except Exception:
        problem.cleanup()
        raise
    problem.model_metadata["evidence"] = {
        slot: getattr(getattr(model, slot), "model_evidence", {"status": "uncalibrated reference model"})
        for slot in vars(models)
    }
    problem.model_metadata["motor_options"] = {}
    for slot in ("excavation","mobility","relocation","geometry","drum_mass"):
        component = getattr(model,slot)
        problem.model_metadata["motor_options"][slot] = {
            name: f"{getattr(component.options[name], '__module__', type(component.options[name]).__module__)}.{getattr(component.options[name], '__qualname__', type(component.options[name]).__qualname__)}"
            for name in ("motor_model","assembly_mass_model") if name in component.options}
    return problem
