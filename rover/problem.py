"""The complete optimization formulation lives here, separate from the physics."""

import math
import hashlib
from pathlib import Path
import openmdao.api as om
from .config import DESIGN, ASSUMPTIONS, REQUIREMENTS, ROUTE
from .components import Mission
from .models import ModelSet
from .contracts import validate_interfaces

# Every margin must be >= 0. Values below are scales, not additional limits.
MARGINS = {
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
    "dig_froude_margin": (0.3, None), "dump_froude_margin": (0.2, None),
    "payload_margin": (5.0, "kg"), "trench_traverse_margin": (0.359, "m"),
    "fresh_site_margin": (0.5, "m"),
    "frame_section_margin": (0.02, "m"), "arm_section_margin": (0.02, "m"),
    "structure_stress_margin": (1e8, "Pa"), "structure_deflection_margin": (0.005, "m"),
    "structure_buckling_margin": (10000.0, "N"), "mobility_stability_margin": (0.3, "m"),
    "tip_stability_margin": (0.3, "m"),
    "obstacle_pose_margin": (0.4, "m"), "obstacle_tip_margin": (0.4, "m"),
    "excavation_area_margin": (12.0, "m**2"),
    "site_drive_margin": (300.0, "N"),
}


def validate(values):
    """Catch invalid inputs before they become division errors or NaNs in a driver."""
    zero_allowed = {"electronics_power", "actuator_idle_power", "rolling_base",
                    "scrub_coefficient", "internal_lift_factor", "vertical_cut_fraction",
                    "force_imbalance", "mass_score_weight", "energy_score_weight",
                    "obstacle_height", "rocker_pivot_offset"}
    fractions = {name for name in ASSUMPTIONS if "fraction" in name or "efficiency" in name}
    fractions |= {"cut_engagement", "delivery_efficiency", "force_imbalance"}
    for name, value in values.items():
        if not math.isfinite(value) or (value < 0 and name != "rocker_pivot_offset") or (value == 0 and name not in zero_allowed):
            raise ValueError(f"{name} must be finite and {'nonnegative' if name in zero_allowed else 'positive'}.")
        if name in fractions and value > 1:
            raise ValueError(f"{name} must be <= 1.")
    minimum_wheelbase = DESIGN["wheelbase"][1] if "wheelbase" in DESIGN else values["wheelbase"]
    if abs(values["rocker_pivot_offset"]) >= minimum_wheelbase/2:
        raise ValueError("Absolute rocker_pivot_offset must be smaller than half the minimum wheelbase.")
    if not 0 < values["wheel_deflection_fraction"] < 0.5:
        raise ValueError("wheel_deflection_fraction must be between 0 and 0.5.")
    if values["scoops_per_revolution"] != int(values["scoops_per_revolution"]):
        raise ValueError("scoops_per_revolution must be an integer.")
    if values["mass_score_weight"] + values["energy_score_weight"] == 0:
        raise ValueError("At least one score weight must be positive.")
    for name, (initial, lower, upper, _) in DESIGN.items():
        if not 0 < lower < upper or not lower <= values[name] <= upper:
            raise ValueError(f"{name} must start within its positive bounds [{lower}, {upper}].")
    for name in ("wheel_diameter", "wheel_width", "drum_diameter", "drum_length"):
        if values[name] >= 0.36 or (name in DESIGN and DESIGN[name][2] >= 0.36):
            raise ValueError(f"{name}: user requires a bound strictly below 0.360 m.")
    if values["turn_angle"] < math.pi:
        raise ValueError("turn_angle must include at least the required 180-degree deposition turn.")
    if not ROUTE or any(not math.isfinite(d + a) or d < 0 for d, a in ROUTE) or sum(d for d, _ in ROUTE) <= 0:
        raise ValueError("ROUTE needs finite nonnegative distances and nonzero total travel.")


def build_problem(overrides=None, *, optimizer="SLSQP", maxiter=200, display=False, models=None):
    models = models or ModelSet()
    values = {name: spec[0] for name, spec in {**DESIGN, **ASSUMPTIONS}.items()}
    for name, value in (overrides or {}).items():
        if name not in values:
            raise ValueError(f"Unknown input {name!r}. See rover/config.py.")
        values[name] = float(value)
    validate(values)
    problem = om.Problem(reports=False)
    problem.model_metadata = {"schema_version": 2,
                              "models": models.metadata(), "calibration": "uncalibrated; see model cards",
                              "source_files_sha256": {p.relative_to(Path(__file__).parent).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                                       for p in sorted(Path(__file__).parent.rglob("*.py"))}}
    problem.model_metadata["traversal_requirements"] = {
        "obstacle_height_m": values["obstacle_height"],
        "obstacle_status": "posed gravity support and geometric reach screens only",
        "ditch_depth_m": 0.50, "ditch_profile": "user: circle-like",
        "ditch_status": "unverified: width/radius and crossing contact sequence unknown"}
    model = problem.model
    inputs = om.IndepVarComp()
    for name, value in values.items():
        spec = DESIGN.get(name, ASSUMPTIONS.get(name))
        inputs.add_output(name, val=value, units=spec[-1])
    model.add_subsystem("inputs", inputs, promotes=["*"])
    # Kinematics/mass -> geometry -> excavation -> linkage loads and mobility -> mission.
    # Battery capacity is a design variable, avoiding a hidden mass/energy sizing loop.
    model.add_subsystem("structure_geometry", models.structure_geometry(), promotes=["*"])
    model.add_subsystem("linkage", models.linkage(), promotes=["*"])
    model.add_subsystem("wheel_mass", models.wheel_mass(), promotes=["*"])
    model.add_subsystem("drum_mass", models.drum_mass(), promotes=["*"])
    model.add_subsystem("geometry", models.geometry(), promotes=["*"])
    model.add_subsystem("excavation", models.excavation(), promotes=["*"])
    model.add_subsystem("linkage_loads", models.linkage_loads(), promotes=["*"])
    model.add_subsystem("mass_properties", models.mass_properties(), promotes=["*"])
    model.add_subsystem("structure_loads", models.structure_loads(), promotes=["*"])
    model.add_subsystem("relocation", models.relocation(), promotes=["*"])
    model.add_subsystem("trench_traversal", models.trench_traversal(), promotes=["*"])
    model.add_subsystem("tip_stability", models.tip_stability(), promotes=["*"])
    model.add_subsystem("obstacle_stability", models.obstacle_stability(), promotes=["*"])
    model.add_subsystem("mobility", models.mobility(), promotes=["*"])
    model.add_subsystem("mission", Mission(), promotes=["*"])
    for name, (_, lower, upper, _) in DESIGN.items():
        model.add_design_var(name, lower=lower, upper=upper, ref0=lower, ref=upper)
    # Small inward reserves stop a numerically successful solve from ending just
    # outside a hard requirement when many constraints become active together.
    for name, (upper, _) in REQUIREMENTS.items():
        model.add_constraint(name, upper=upper*(1-1e-3), ref=upper)
    for name, (scale, _) in MARGINS.items():
        model.add_constraint(name, lower=1e-3*scale, ref=scale)
    # V2 finite-soil scores are typically tens to hundreds, not the old thousands.
    model.add_objective("construction_points", scaler=-1 / 100.0)  # maximize
    # Deliberately recompute the full model for perturbations: easy to replace models.
    # Avoid complex-step assumptions, derivative coloring, or hand-maintained Jacobians.
    model.approx_totals(method="fd", form="central", step=1e-5, step_calc="rel_avg")
    problem.driver = om.ScipyOptimizeDriver(optimizer=optimizer, tol=1e-3)
    problem.driver.options["maxiter"] = maxiter
    problem.driver.options["disp"] = display
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
    return problem
