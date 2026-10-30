"""The complete optimization formulation lives here, separate from the physics."""

import math
import hashlib
from pathlib import Path
from dataclasses import replace
import openmdao.api as om
from .config import DESIGN, ASSUMPTIONS, REQUIREMENTS, ROUTE, configuration, migrate_inlet_inputs
from .components import Mission
from .models import ModelSet, plate_frame_models, traversal_models, extended_attachment_models
from .contracts import validate_interfaces

# Every margin must be >= 0. Values below are scales, not additional limits.
# Climb acceptance is strict: a zero reserve or failed solution does not pass.
STRICT_SLOPE_MARGINS = {"empty_slope_traversal_margin": "empty", "loaded_slope_traversal_margin": "loaded"}
MARGINS = {
    "ground_clearance_margin": (0.2032, "m"),
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
                      "cross_tube_bending_margin": (1e8, "Pa")}

MOVING_EXCAVATION_MARGINS = {
    "feed_margin": (0.012, "m"),
    "operating_reach_margin": (0.1, "m"),
    "axle_ground_margin": (0.15, "m"),
    "excavation_domain_margin": (0.1, "m"),
    "feed_motor_margin": (300., "N"),
    "feed_traction_margin": (300., "N"),
    "feed_power_margin": (400., "W"),
}


def validate(values, design=None, assumptions=None, attachment_max=1.):
    """Catch invalid inputs before they become division errors or NaNs in a driver."""
    design = DESIGN if design is None else design
    assumptions = ASSUMPTIONS if assumptions is None else assumptions
    zero_allowed = {"electronics_power", "actuator_idle_power", "rolling_base",
                    "scrub_coefficient", "internal_lift_factor", "vertical_cut_fraction",
                    "force_imbalance", "mass_score_weight", "energy_score_weight",
                    "obstacle_height", "rocker_pivot_offset", "slope_angle",
                    "drive_stator_current_limit", "drum_stator_current_limit",
                    "empty_slope_degrees", "loaded_slope_degrees", "actuator_installation_allowance",
                    "lin_act_side_arm_offset", "lin_act_top_arm_offset",
                    "clearance_radius_deflection_fraction", "differential_bar_clearance_allowance"}
    fractions = {name for name in assumptions if "fraction" in name or "efficiency" in name}
    fractions |= {"cut_engagement", "delivery_efficiency", "force_imbalance"}
    for name, value in values.items():
        if not math.isfinite(value) or (value < 0 and name not in {"rocker_pivot_offset", "lin_act_side_arm_offset", "lin_act_top_arm_offset"}) or (value == 0 and name not in zero_allowed):
            raise ValueError(f"{name} must be finite and {'nonnegative' if name in zero_allowed else 'positive'}.")
        if name in fractions and value > 1:
            raise ValueError(f"{name} must be <= 1.")
    minimum_wheelbase = design["wheelbase"][1] if "wheelbase" in design else values["wheelbase"]
    if abs(values["rocker_pivot_offset"]) >= minimum_wheelbase/2:
        raise ValueError("Absolute rocker_pivot_offset must be smaller than half the minimum wheelbase.")
    if not 0 < values["wheel_deflection_fraction"] < 0.5:
        raise ValueError("wheel_deflection_fraction must be between 0 and 0.5.")
    if not 0 <= values["clearance_radius_deflection_fraction"] < 1:
        raise ValueError("clearance_radius_deflection_fraction must be in [0, 1).")
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
        if not lower < upper or (name not in {"lin_act_side_arm_offset", "lin_act_top_arm_offset"} and lower <= 0) or not lower <= values[name] <= upper:
            raise ValueError(f"{name} must start within its positive bounds [{lower}, {upper}].")
    for name in ("wheel_diameter", "wheel_width", "drum_diameter", "drum_length"):
        if values[name] >= 0.36 or (name in design and design[name][2] >= 0.36):
            raise ValueError(f"{name}: user requires a bound strictly below 0.360 m.")
    if values["turn_angle"] < math.pi:
        raise ValueError("turn_angle must include at least the required 180-degree deposition turn.")
    if not ROUTE or any(not math.isfinite(d + a) or d < 0 for d, a in ROUTE) or sum(d for d, _ in ROUTE) <= 0:
        raise ValueError("ROUTE needs finite nonnegative distances and nonzero total travel.")


def build_problem(overrides=None, *, optimizer=None, maxiter=None, display=False, models=None, structure="reference", cobyla_radius=None, motors="reference", actuators="reference", solver_tol=None, freeze=None, ignore_constraints=None, traversal_mode="fixed", optimize_attachment=False, attachment_max=1.):
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
    models = models or (plate_frame_models() if structure == "plate-frame" else ModelSet())
    models = traversal_models(models, traversal_mode)
    excavation_component = models.excavation()
    moving_excavation = getattr(excavation_component, "MOVING_EXCAVATION", False)
    design, assumptions = configuration(structure, motors, actuators,
        excavation_mode="moving" if moving_excavation else "legacy")
    retired_excavation_inputs = {}
    if moving_excavation:
        overrides, retired_excavation_inputs = migrate_inlet_inputs(overrides or {})
    if isinstance(attachment_max,bool) or not math.isfinite(attachment_max) or not 1 <= attachment_max <= 1.5:
        raise ValueError('attachment_max must be finite and between 1 and 1.5')
    if attachment_max>1 and structure!='plate-frame':
        raise ValueError('attachment_max > 1 currently requires the plate-frame box-arm profile')
    if not isinstance(optimize_attachment, bool):
        raise ValueError("optimize_attachment must be a boolean")
    # Both signed offsets are ordinary design variables; legacy opt-in flag is redundant.
    frozen_names = set([freeze] if isinstance(freeze, str) else freeze or ())
    unknown = frozen_names - set(design)
    if unknown:
        raise ValueError("--freeze requires active design-variable names for this profile; "
                         f"unknown or already fixed: {', '.join(sorted(unknown))}")
    if frozen_names == set(design):
        raise ValueError("Cannot freeze every design variable; use --baseline for a fixed evaluation.")
    margins = {**MARGINS, **(PLATE_FRAME_MARGINS if structure == "plate-frame" else {})}
    diagnostic_screens = {
        "actuator_body_margin": "User removed this constraint; the reserve is diagnostic only and does not affect optimization or feasibility. Actual actuator packaging remains unverified.",
        "dig_froude_margin": "User removed the uncalibrated speed cap; constant capture/lift response assumes no additional high-speed loss.",
        "dump_froude_margin": "User removed the uncalibrated speed cap; actual dumping/retention speed response remains unverified.",
    }
    if structure == "plate-frame":
        margins.pop("chassis_clearance_margin")
        diagnostic_screens["chassis_clearance_margin"] = (
            "Legacy filled rectangular envelope does not represent the open plate frame; actual frame/equipment interference remains unverified.")
    if actuators == "pa04-900":
        margins.update(actuator_stroke_domain_margin=(.5, "m"), actuator_curve_margin=(4000., "N"))
    ignored_constraints = set([ignore_constraints] if isinstance(ignore_constraints, str) else ignore_constraints or ())
    if moving_excavation:
        margins.pop("dig_reach_margin")
        margins.update(MOVING_EXCAVATION_MARGINS)
        diagnostic_screens["dig_reach_margin"] = (
            "Compatibility alias of operating_reach_margin; use the active operating_reach_margin and axle_ground_margin checks.")
    else:
        # Explicit legacy replacements do not acquire an inactive optimizer variable.
        entry = design.pop("feed_speed")
        assumptions["feed_speed"] = (entry[0], entry[-1])
    unknown = ignored_constraints - set(margins)
    if unknown:
        raise ValueError("--ignore-constraint requires an active margin name; "
                         f"unknown or unavailable: {', '.join(sorted(unknown))}. "
                         "Mass/dimension limits and design bounds cannot be excluded.")
    if actuators == "pa04-900":
        from .linkage import LinkageGeometry, LinkageLoads
        from .alternatives.pa04_actuator import PA04LinkageGeometry, PA04LinkageLoads, verify_source_data
        verify_source_data()
        if models.linkage not in (LinkageGeometry, PA04LinkageGeometry) or models.linkage_loads not in (LinkageLoads, PA04LinkageLoads):
            raise ValueError("PA-04 profile needs PA04LinkageGeometry/Loads; a custom linkage was not replaced.")
        models = replace(models, linkage=PA04LinkageGeometry, linkage_loads=PA04LinkageLoads)
    if moving_excavation:
        from .alternatives.staged_linkage_loads import staged_load_factory
        models = replace(models, linkage_loads=staged_load_factory(models.linkage_loads))
    # A round outside diameter is not a box width/height specification. Require
    # explicit case migration rather than silently ignoring old arm geometry.
    if (structure == 'plate-frame' and 'arm_tube_diameter' in (overrides or {})
            and not {'arm_box_width', 'arm_box_height'} <= set(overrides)
            and 'arm_tube_diameter' not in getattr(models.structure_geometry, 'CONFIG_INPUTS', ())):
        raise ValueError("arm_tube_diameter is inactive for aluminum box arms. Replace it with arm_box_width and arm_box_height (metres); keep arm_wall in metres. Historical round-arm cases require reevaluation.")
    if motors == "x44-standard":
        from .alternatives.x44_motor import X44MotorProvider, verify_source_data
        from .motor_inventory import CatalogAssemblyMass
        verify_source_data()
    values = {name: spec[0] for name, spec in {**design, **assumptions}.items()}
    for name, value in (overrides or {}).items():
        if name not in values:
            raise ValueError(f"Unknown input {name!r}. See rover/config.py.")
        values[name] = float(value)
    validate(values, design, assumptions, attachment_max=attachment_max)
    # Validate against the original search bounds before converting selected
    # variables into fixed inputs. Preserve those bounds for report auditing.
    frozen_design = {}
    for name in sorted(frozen_names):
        _, lower, upper, units = design.pop(name)
        frozen_design[name] = (values[name], lower, upper, units)
        assumptions[name] = (values[name], units)
    if motors == "x44-standard" and (values["motor_bus_voltage"] != 12 or values["drive_gear_ratio"] != 100):
        raise ValueError("The selected hardware profile requires 12 V and a fixed 100:1 drive ratio.")
    if actuators == "pa04-900":
        for name in ("actuator_force_rating", "actuator_speed"):
            if not math.isclose(values[name], assumptions[name][0], rel_tol=1e-12, abs_tol=0.):
                raise ValueError("PA-04 900 lbf profile freezes force rating and nominal lowering speed; remove reference force/speed overrides.")
            values[name] = assumptions[name][0]
    if values["arm_length"]+values["lin_act_side_arm_offset"] <= 0:
        raise ValueError("Actuator pin axial station must be beyond the chassis pivot")
    problem = om.Problem(reports=False)
    problem.design_specs, problem.assumption_specs, problem.margin_specs = design, assumptions, margins
    problem.model_metadata = {"schema_version": 7, "structure_profile": structure, "motor_profile": motors, "actuator_profile": actuators,
                              "excavation_mode": "moving" if moving_excavation else "custom_or_legacy",
                              "traversal_mode": traversal_mode,
                              "optimize_attachment": optimize_attachment,
                              "attachment_max": attachment_max,
                              "configuration": {"design": design, "assumptions": assumptions, "margins": margins,
                                                "frozen_design": frozen_design,
                                                "ignored_optimizer_constraints": sorted(ignored_constraints),
                                                "excavation_mode": "moving" if moving_excavation else "custom_or_legacy",
                                                "traversal_mode": traversal_mode, "attachment_max": attachment_max},
                              "optimizer_settings": {"optimizer": optimizer, "maxiter": maxiter, "tol": solver_tol, "cobyla_radius": cobyla_radius},
                              "diagnostic_only_screens": diagnostic_screens,
                              "models": models.metadata(), "calibration": "uncalibrated; see model cards",
                              "source_files_sha256": {p.relative_to(Path(__file__).parent).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                                       for p in sorted(Path(__file__).parent.rglob("*.py"))}}
    if moving_excavation:
        problem.model_metadata["excavation_operation"] = {
            "front_depth": "inlet_depth", "rear_depth": "2*inlet_depth",
            "inlet_width": "drum_length; full axial width of each drum engaged",
            "fresh_layer_per_pair": "inlet_depth; rear follows front in the same tracks",
            "storage_radius": "drum_diameter/2 - inlet_depth; inlet protrusion excluded",
            "feed_screen": "60*feed_speed/(scoops_per_revolution*drum_rpm) <= inlet_depth",
            "input_migration": {"retired_inputs": retired_excavation_inputs,
                "policy": "Legacy cut_depth becomes inlet_depth. Separate scoop_opening, scoop_depth_fraction and partial-width cut_engagement are retired; front/rear depths follow inlet_depth and full width is enforced."},
            "entry_exit": "Staged front/rear lowering and raising; both cut the same lane, travel includes station separation. No plunge/endcap soil credit.",
            "arm_accounting": "Full capability-sweep loads/work retained as conservative envelopes; sum both arms' motion durations for staged positioning; no lowering/cutting overlap credit.",
            "recovery": "Maximum-extension axle reaches ground; actual driving on drums and motor-body clearance unverified.",
            "calibration": "No fitted data added; existing capture, cutting, rolling and efficiency assumptions remain uncalibrated.",
        }
    if actuators == "pa04-900":
        problem.model_metadata["actuator_performance"] = {
            "manifest": "data/actuators/manifest.json", "voltage_V": 12,
            "manifest_sha256": hashlib.sha256((Path(__file__).parent.parent/"data/actuators/manifest.json").read_bytes()).hexdigest(),
            "rated_force_N": values["actuator_force_rating"],
            "installation_allowance_m": values["actuator_installation_allowance"],
            "stroke_domain_m": [.0508, 1.016], "load_domain_N": [0., values["actuator_force_rating"]],
            "lowering_assumption": "Rated-load speed/current throughout lowering; gravity-assisted operation unmeasured.",
            "raising_assumption": "Constant maximum sampled extraction force throughout raising, including breakout allowance.",
            "energy_accounting": "Two actuators: 12 V * phase current * phase duration; no additional reference efficiency or idle loss.",
            "validation": "Held-out catalog mass rows only; no independent hardware validation or thermal/duty-cycle qualification.",
            "inactive_reference_inputs": ["actuator_efficiency", "actuator_fixed_mass", "actuator_mass_per_force", "actuator_mass_per_power", "actuator_mass_per_length", "actuator_housing_allowance"],
        }
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
    problem.model_metadata["actuator_packaging"] = {
        "status": "Not enforced: actuator packaging reserve is diagnostic only at user request",
        "catalog_endpoints": "Diagnostics only; commands are not checked against these endpoints",
        "reference_allowance": "actuator_housing_allowance affects the generic diagnostic only; inactive for PA-04",
    }
    problem.model_metadata["attachment_coordinates"] = {
        "revision": 2, "equation": "D=C+side*u+top*(-u_z,u_x)",
        "side_positive": "past drum axle away from chassis pivot",
        "top_positive": "above arm in local side elevation; rear mirrored",
        "arm": "uniform tube continues to max(L,L+side); extension mass and gravity included",
        "limitations": "Rigid normal bracket assumed massless; bracket mass, strength, actuator-body and member collisions require CAD/hardware data.",
    }
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
        component = excavation_component if slot == "excavation" else getattr(models,slot)()
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
                 "obstacle_clearance","traversal_stability","mobility","slope_traversal","ground_clearance"):
        add_discipline(slot)
    model.add_subsystem("mission", Mission(), promotes=["*"])
    for name, (_, lower, upper, _) in design.items():
        model.add_design_var(name, lower=lower, upper=upper, ref0=lower, ref=upper)
    # Small inward reserves stop a numerically successful solve from ending just
    # outside a hard requirement when many constraints become active together.
    for name, (upper, _) in REQUIREMENTS.items():
        model.add_constraint(name, upper=upper*(1-1e-3), ref=upper)
    for name, (scale, _) in margins.items():
        if name not in ignored_constraints:
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
