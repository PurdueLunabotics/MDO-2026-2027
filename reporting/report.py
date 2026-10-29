"""Physical-unit results, feasibility checks, and a separate whole-cycle estimate."""

import math
from rover.config import DESIGN, ASSUMPTIONS, REQUIREMENTS
from rover.problem import MARGINS, STRICT_SLOPE_MARGINS

OUTPUTS = {
    "dry_mass": "kg", "payload_mass": "kg", "wheel_mass": "kg", "drum_mass": "kg",
    "drive_mass": "kg", "battery_mass": "kg", "rover_length": "m", "rover_width": "m",
    "rover_height": "m", "ground_pressure": "Pa", "actual_travel_speed": "m/s",
    "dig_time": "s", "travel_time": "s", "dump_time": "s", "turn_time": "s",
    "cycle_time": "s", "cycle_energy": "W*h", "cycles": None, "berm_volume": "m**3",
    "run_energy": "W*h", "average_power": "W", "peak_power": "W",
    "mass_points": None, "energy_points": None, "construction_points": None,
    "no_load_speed": "m/s", "loaded_motor_speed_limit": "m/s",
    "loaded_power_speed_limit": "m/s", "stall_drive_force": "N",
    "actuation_time": "s", "lowering_time": "s", "arm_energy": "J", "arm_peak_power": "W",
    "required_actuator_force": "N", "linkage_mass": "kg", "stroke": "m",
    "stowed_x": "m", "stowed_y": "m", "travel_x": "m", "travel_y": "m", "extended_x": "m", "extended_y": "m",
    "sweep_x_min": "m", "sweep_x_max": "m", "actuator_leverage_max": None,
    "actuator_attachment_distance": "m", "arm_physical_length": "m", "arm_extension_mass": "kg",
    "actuator_body_margin": "m",  # diagnostic only; excluded from feasibility/optimizer
    "chassis_clearance_margin": "m",  # legacy diagnostic; inactive in plate-frame
    "stowed_length": "m", "stowed_height": "m", "deployed_length": "m", "deployed_height": "m",
    "route_distance": "m", "route_turn_angle": "deg", "route_translation_time": "s",
    "route_turn_time": "s", "route_peak_acceleration": "m/s**2",
}

# Discipline contracts are also the report's source of units. Include diagnostics
# as well as constraint values so students can explain WHY a design is infeasible.
from rover.excavation import Excavation
from rover.structure import StructureGeometry, StructureLoads
from rover.mass_properties import MassProperties
from rover.relocation import SiteRelocation
from rover.mobility import Mobility
from rover.trench import TrenchTraversal
from rover.tip_stability import TipStability
from rover.alternatives.obstacle_stability import ObstacleStability
from rover.alternatives.parallel_cylinder_clearance import ParallelCylinderClearance
from rover.alternatives.traversal_stability import TraversalStability
from rover.alternatives.slope_traversal import SlopeTraversal
for component in (Excavation, StructureGeometry, StructureLoads, MassProperties, SiteRelocation, Mobility, TrenchTraversal, TipStability, ObstacleStability, ParallelCylinderClearance, TraversalStability, SlopeTraversal):
    OUTPUTS.update(component.OUTPUT_UNITS)


def snapshot(problem, configuration=None):
    def get(name):
        return problem.get_val(name).item()

    specs = configuration or getattr(problem, "model_metadata", {}).get("configuration", {})
    design = specs.get("design", DESIGN)
    assumptions = specs.get("assumptions", ASSUMPTIONS)
    margins = specs.get("margins", MARGINS)
    output_units = dict(OUTPUTS)
    if specs.get('attachment_max',1.)>1:
        from rover.alternatives.extended_attachment import EXTRA_OUTPUTS
        output_units.update(EXTRA_OUTPUTS)
    if specs.get("traversal_mode", "fixed") == "calculated":
        from rover.alternatives.calculated_traversal import TIMING_OUTPUTS
        output_units.update(TIMING_OUTPUTS)
    if getattr(problem, "model_metadata", {}).get("actuator_profile") == "pa04-900" or "actuator_catalog_stroke" in {**design, **assumptions}:
        from rover.alternatives.pa04_actuator import PA04LinkageGeometry, PA04LinkageLoads
        output_units.update(PA04LinkageGeometry.OUTPUT_UNITS)
        output_units.update(PA04LinkageLoads.OUTPUT_UNITS)
    if getattr(problem, "model_metadata", {}).get("structure_profile") == "plate-frame" or "plate_thickness" in assumptions:
        from rover.alternatives.plate_frame import PlateStructureGeometry, PlateStructureLoads
        output_units.update(PlateStructureGeometry.OUTPUT_UNITS)
        output_units.update(PlateStructureLoads.OUTPUT_UNITS)
    constraints = {}
    for name, (upper, units) in REQUIREMENTS.items():
        margin = upper - get(name)
        constraints[name] = {"margin": margin, "units": units, "passed": margin >= -1e-6 * upper}
    for name, (scale, units) in margins.items():
        constraints[name] = {"margin": get(name), "units": units, "passed": get(name) >= -1e-6 * scale}
        if name in ("actuator_stroke_domain_margin", "actuator_curve_margin"):
            # An out-of-domain chart diagnostic is never an accepted prediction.
            constraints[name]["passed"] = bool(get(name) >= 0)
            if name == "actuator_curve_margin":
                constraints[name]["passed"] &= bool(get("actuator_curve_valid") == 1)
        if name in STRICT_SLOPE_MARGINS:
            state = STRICT_SLOPE_MARGINS[name]
            constraints[name].update(passed=bool(get(state+"_slope_solution_valid") == 1
                and 0 <= get(state+"_slope_slip") < 1 and get(name) > 0),
                criterion="Solved BWR driving slip < 1 (strict; failed-solution sentinel is not a prediction)")
    for name, (_, low, high, units) in {**design, **specs.get("frozen_design", {})}.items():
        margin = min(get(name) - low, high - get(name))
        constraints[name + "_bounds"] = {"margin": margin, "units": units,
                                        "passed": margin >= -1e-6 * (high - low)}
    for name in specs.get("ignored_optimizer_constraints", ()):
        constraints[name]["enforced_in_optimization"] = False
    cycles = math.floor(get("cycles"))
    idle_time = get("run_time") - cycles * get("cycle_time")
    whole_volume = cycles * get("payload_mass") * get("delivery_efficiency") / get("berm_density")
    whole_energy = cycles * get("cycle_energy") + idle_time * get("electronics_power") / 3600
    whole_mass_points = 4.4 * whole_volume * 1e6 / (get("run_time") / 60 * get("dry_mass"))
    whole_energy_points = 1.5 * whole_volume * 1e6 / (get("run_time") / 60 * whole_energy) if whole_energy > 0 else 0.0
    return {
        "model_metadata": getattr(problem, "model_metadata", {}),
        "feasible": all(c["passed"] for c in constraints.values()),
        "design": {name: {"value": get(name), "lower": spec[1], "upper": spec[2], "units": spec[-1]} for name, spec in design.items()},
        "assumptions": {name: {"value": get(name), "units": spec[-1]} for name, spec in assumptions.items()},
        "outputs": {name: {"value": get(name), "units": units} for name, units in output_units.items()},
        "constraints": constraints,
        "whole_cycles": {
            "completed": cycles, "idle_time_s": idle_time, "berm_volume_m3": whole_volume,
            "run_energy_Wh": whole_energy,
            "construction_points": get("mass_score_weight") * whole_mass_points + get("energy_score_weight") * whole_energy_points,
            "assumption": "Complete cycles only; idle for remainder. Partial second-pair dumps not credited.",
        },
    }


def print_report(label, report):
    print(f"\n{label}: {'FEASIBLE' if report['feasible'] else 'INFEASIBLE'} (modeled constraints only)")
    timing_mode = report.get("model_metadata", {}).get("traversal_mode", "fixed")
    print(f"Traversal timing: {timing_mode}")
    if timing_mode == "calculated":
        out = report["outputs"]
        print(f"  Outbound {out['outbound_travel_time']['value']:.4f} s; return {out['return_travel_time']['value']:.4f} s")
        print("  traversal_time_one_way is inactive; point-turn physics, deposition and relocation timing retain reference assumptions.")
        for prefix in ("outbound", "return"):
            count = out[prefix+"_bwr_transfer_flag_count"]["value"]
            if count:
                print(f"  BWR {prefix}: {count:g} source-range transfer flags; see model evidence.")
    ignored = [name for name, item in report["constraints"].items()
               if item.get("enforced_in_optimization") is False]
    if ignored:
        print("Excluded from optimizer (still checked in full feasibility): "+", ".join(ignored))
    frozen = report.get("model_metadata", {}).get("configuration", {}).get("frozen_design", {})
    if frozen:
        print("Frozen design variables (held fixed):")
        for name in frozen:
            item = report["assumptions"][name]
            print(f"  {name:25s} {item['value']:10.4f} {item['units'] or ''}")
    print("Design variables:")
    for name, item in report["design"].items():
        print(f"  {name:25s} {item['value']:10.4f} {item['units'] or ''}")
    print("Performance:")
    for name in ("construction_points", "mass_points", "energy_points", "dry_mass", "payload_mass",
                 "cycle_time", "dig_time", "travel_time", "dump_time", "turn_time", "actuation_time", "cycles",
                 "berm_volume", "run_energy", "average_power", "peak_power"):
        item = report["outputs"][name]
        print(f"  {name:25s} {item['value']:10.4f} {item['units'] or ''}")
    whole = report["whole_cycles"]
    print(f"  Whole-cycle check: {whole['completed']} cycles, {whole['berm_volume_m3']:.4f} m^3, "
          f"{whole['construction_points']:.1f} points")
    print("Constraint margins (positive means satisfied):")
    for name, item in report["constraints"].items():
        if name.endswith("_bounds") and item["passed"]:
            continue
        print(f"  {'PASS' if item['passed'] else 'FAIL'} {name:27s} {item['margin']:11.5f} {item['units'] or ''}")
