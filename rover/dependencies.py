"""Structural input dependencies for N2 and component finite differences.

Each set lists the inputs that can affect an output, including alternate min/max
branches. These are NOT sparsity guesses made at one operating point. Keep this
file beside the equations; tests perturb every omitted pair to catch stale maps.
"""


def names(text):
    return set(text.split())


def dependencies(component):
    from .excavation import Excavation
    from .structure import StructureGeometry, StructureLoads
    from .mass_properties import MassProperties
    from .relocation import SiteRelocation
    from .trench import TrenchTraversal
    from .tip_stability import TipStability
    for cls in (Excavation, StructureGeometry, StructureLoads, MassProperties, SiteRelocation, TrenchTraversal, TipStability):
        if component == cls.__name__:
            return cls.DEPENDENCIES
    if component == "GeometryMass":
        drive = names("drive_fixed_mass motor_mass_per_watt drive_motor_power gear_mass_per_torque motor_rated_rpm drive_gear_ratio drive_gear_efficiency")
        battery = names("battery_capacity battery_specific_energy")
        return {"drive_mass": drive, "battery_mass": battery,
                "dry_mass": drive | battery | names("fixed_mass linkage_mass structure_mass wheel_mass drum_mass"),
                "rover_length": {"stowed_length"}, "rover_height": {"stowed_height"},
                "rover_width": names("track_width wheel_width drum_length drum_pair_gap")}
    if component == "LinkageGeometry":
        core = names("arm_length top_pivot_height bottom_pivot_height pivot_x actuator_pivot_x actuator_attachment_fraction")
        retract = core | {"actuator_retracted_length"}
        travel = core | {"travel_actuator_length"}
        extend = core | {"actuator_extended_length"}
        sweep = travel | extend
        envelope = names("wheelbase wheel_diameter drum_diameter pivot_x actuator_pivot_x")
        high = names("wheel_diameter chassis_height drum_diameter top_pivot_height")
        return {
            "stowed_x": retract, "stowed_y": retract,
            "travel_x": travel, "travel_y": travel,
            "extended_x": extend, "extended_y": extend,
            "sweep_x_min": sweep, "sweep_x_max": sweep,
            "actuator_leverage_max": sweep,
            "actuator_attachment_distance": names("arm_length actuator_attachment_fraction"),
            "actuator_toggle_margin": sweep | {"mechanism_clearance"},
            "stowed_length": retract | envelope, "stowed_height": retract | high,
            "deployed_length": sweep | envelope, "deployed_height": sweep | high,
            "deployed_height_margin": sweep | high,
            "stroke": names("actuator_extended_length travel_actuator_length"),
            "actuation_time": names("actuator_extended_length travel_actuator_length actuator_speed"),
            "pit_depth_margin": extend | names("pit_depth drum_diameter"),
            "transport_clearance_margin": retract | names("drum_diameter transport_clearance"),
            "wheel_first_margin": travel | names("drum_diameter wheel_diameter"),
            "startup_position_margin": names("actuator_retracted_length travel_actuator_length"),
            "dig_position_margin": names("actuator_extended_length actuator_retracted_length"),
            "pivot_spacing_margin": names("top_pivot_height bottom_pivot_height pivot_x actuator_pivot_x minimum_pivot_spacing"),
            "pivot_mount_margin": names("wheel_diameter chassis_height top_pivot_height"),
            "bottom_mount_margin": names("bottom_pivot_height wheel_diameter"),
            "pivot_longitudinal_margin": names("wheelbase pivot_x"),
            "actuator_pivot_longitudinal_margin": names("wheelbase actuator_pivot_x"),
            "stroke_margin": names("actuator_extended_length travel_actuator_length minimum_stroke"),
            "actuator_body_margin": names("travel_actuator_length actuator_extended_length actuator_housing_allowance"),
            "triangle_retracted_margin": travel | {"mechanism_clearance"},
            "triangle_extended_margin": extend | {"mechanism_clearance"},
            "toggle_margin": sweep | {"mechanism_clearance"},
            "wheel_clearance_margin": sweep | envelope | {"mechanism_clearance"},
            "chassis_clearance_margin": sweep | envelope | names("mechanism_clearance chassis_height"),
            "linkage_mass": names("actuator_fixed_mass actuator_mass_per_force actuator_force_rating actuator_mass_per_power actuator_speed actuator_efficiency actuator_mass_per_length actuator_extended_length arm_linear_mass arm_length"),
        }
    if component == "LinkageLoads":
        weights = names("drum_mass arm_linear_mass arm_length payload_mass gravity")
        force = weights | names("breakout_force actuator_leverage_max")
        return {"required_actuator_force": force,
                "actuator_force_margin": force | names("actuator_force_rating actuator_safety_factor"),
                "arm_energy": weights | names("travel_y extended_y breakout_force cut_depth actuator_efficiency actuator_idle_power actuation_time"),
                "arm_peak_power": force | names("actuator_speed actuator_efficiency actuator_idle_power")}
    if component == "Mobility":
        profile = names("track_width travel_speed acceleration turn_speed_fraction turn_accel_fraction traversal_time_one_way")
        deposit = names("track_width turn_angle travel_speed acceleration turn_speed_fraction turn_accel_fraction")
        cg = names("empty_cg_x empty_cg_z loaded_cg_x loaded_cg_z half_cg_x half_cg_z wheelbase")
        inertia = names("empty_yaw_inertia loaded_yaw_inertia half_yaw_inertia")
        pressure = cg | profile | deposit | names("rocker_load_factor dry_mass payload_mass gravity wheel_width wheel_diameter wheel_deflection_fraction")
        drag = names("rocker_load_factor dry_mass payload_mass gravity wheel_width wheel_diameter wheel_deflection_fraction rolling_base rolling_pressure_scale")
        force = drag | profile
        turnforce = force | deposit | inertia | names("wheelbase scrub_coefficient")
        hardware = names("drive_motor_power motor_rated_rpm drive_gear_ratio drive_gear_efficiency wheel_diameter")
        power = force | profile
        turnpower = turnforce | deposit
        return {
            "mobility_stability_margin": cg | profile | deposit | {"gravity"},
            "travel_time": {"traversal_time_one_way"}, "turn_time": deposit,
            "route_distance": set(), "route_turn_angle": set(),
            "route_translation_time": profile, "route_turn_time": profile,
            "actual_travel_speed": profile, "route_peak_acceleration": profile,
            "mobility_energy": turnpower | names("drive_efficiency actuator_idle_power"),
            "drive_peak_power": power | names("drive_efficiency actuator_idle_power"),
            "turn_peak_power": turnpower | names("drive_efficiency actuator_idle_power"),
            "ground_pressure": pressure, "ground_pressure_margin": pressure | {"allowable_ground_pressure"},
            "no_load_speed": names("motor_rated_rpm wheel_diameter drive_gear_ratio"),
            "stall_drive_force": hardware,
            "loaded_motor_speed_limit": force | hardware,
            "loaded_power_speed_limit": force | names("drive_motor_power drive_gear_efficiency"),
            "straight_motor_margin": power | hardware, "turn_motor_margin": turnpower | hardware,
            "straight_traction_margin": force | cg | {"traction_coefficient"},
            "turn_traction_margin": turnforce | cg | {"traction_coefficient"},
            "straight_power_margin": power | names("drive_motor_power drive_gear_efficiency"),
            "turn_power_margin": turnpower | names("drive_motor_power drive_gear_efficiency"),
        }
    if component == "Mission":
        cycle = names("dig_time dump_time travel_time turn_time overhead_time actuation_time relocation_time dig_overlap_time")
        energy = cycle | names("dig_energy dump_energy relocation_energy mobility_energy electronics_power arm_energy")
        payload = names("payload_mass delivery_efficiency berm_density")
        volume = cycle | payload | {"run_time"}
        mass_points = cycle | payload | {"dry_mass"}
        energy_points = energy | payload | {"run_time"}
        peak = names("electronics_power dig_power dump_power drive_peak_power turn_peak_power arm_peak_power relocation_peak_power")
        return {
            "excavation_area_margin": cycle | names("available_excavation_area run_time excavation_footprint"),
            "cycle_time": cycle, "cycle_energy": energy, "cycles": cycle | {"run_time"},
            "berm_volume": volume, "run_energy": energy | {"run_time"},
            "average_power": energy, "peak_power": peak,
            "battery_energy_margin": energy | names("run_time battery_capacity battery_usable_fraction"),
            "battery_power_margin": peak | names("battery_capacity battery_c_rate"),
            "mass_points": mass_points, "energy_points": energy_points,
            "construction_points": mass_points | energy_points | names("mass_score_weight energy_score_weight"),
            "complete_cycle_margin": cycle | {"run_time"},
        }
    raise ValueError(f"Missing dependency declarations for {component}")
