"""Physical-unit results, feasibility checks, and a separate whole-cycle estimate."""

import math
from .config import DESIGN, ASSUMPTIONS, REQUIREMENTS
from .problem import MARGINS

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
    "actuation_time": "s", "arm_energy": "J", "arm_peak_power": "W",
    "required_actuator_force": "N", "linkage_mass": "kg", "stroke": "m",
    "stowed_x": "m", "stowed_y": "m", "travel_x": "m", "travel_y": "m", "extended_x": "m", "extended_y": "m",
    "sweep_x_min": "m", "sweep_x_max": "m", "actuator_leverage_max": None,
    "actuator_attachment_distance": "m",
    "stowed_length": "m", "stowed_height": "m", "deployed_length": "m", "deployed_height": "m",
    "route_distance": "m", "route_turn_angle": "deg", "route_translation_time": "s",
    "route_turn_time": "s", "route_peak_acceleration": "m/s**2",
}

# Discipline contracts are also the report's source of units. Include diagnostics
# as well as constraint values so students can explain WHY a design is infeasible.
from .excavation import Excavation
from .structure import StructureGeometry, StructureLoads
from .mass_properties import MassProperties
from .relocation import SiteRelocation
from .trench import TrenchTraversal
from .tip_stability import TipStability
from .alternatives.obstacle_stability import ObstacleStability
from .alternatives.obstacle_clearance import ObstacleClearance
for component in (Excavation, StructureGeometry, StructureLoads, MassProperties, SiteRelocation, TrenchTraversal, TipStability, ObstacleStability, ObstacleClearance):
    OUTPUTS.update(component.OUTPUT_UNITS)


def snapshot(problem, configuration=None):
    def get(name):
        return problem.get_val(name).item()

    specs = configuration or getattr(problem, "model_metadata", {}).get("configuration", {})
    design = specs.get("design", DESIGN)
    assumptions = specs.get("assumptions", ASSUMPTIONS)
    margins = specs.get("margins", MARGINS)
    output_units = dict(OUTPUTS)
    if getattr(problem, "model_metadata", {}).get("structure_profile") == "plate-frame" or "plate_thickness" in assumptions:
        from .alternatives.plate_frame import PlateStructureGeometry, PlateStructureLoads
        output_units.update(PlateStructureGeometry.OUTPUT_UNITS)
        output_units.update(PlateStructureLoads.OUTPUT_UNITS)

    constraints = {}
    for name, (upper, units) in REQUIREMENTS.items():
        margin = upper - get(name)
        constraints[name] = {"margin": margin, "units": units, "passed": margin >= -1e-6 * upper}
    for name, (scale, units) in margins.items():
        constraints[name] = {"margin": get(name), "units": units, "passed": get(name) >= -1e-6 * scale}
    for name, (_, low, high, units) in design.items():
        margin = min(get(name) - low, high - get(name))
        constraints[name + "_bounds"] = {"margin": margin, "units": units,
                                        "passed": margin >= -1e-6 * (high - low)}
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
