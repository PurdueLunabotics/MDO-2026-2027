"""Edit this file first. SI units except explicitly marked deg, rpm and W*h.

Every numerical physics coefficient below is an UNCALIBRATED estimate.
Bounds are exploratory modeling bounds, not requirements, except the 359 mm caps.
Power ratings are mechanical shaft power, per motor (four drive, two drum).
"""

import math

# name: (initial value, lower bound, upper bound, OpenMDAO units)
DESIGN = {
    "lin_act_side_arm_offset": (0.0, -0.15, 0.15, "m"),
    "lin_act_top_arm_offset": (0.0, -0.15, 0.15, "m"),
    "wheel_diameter": (0.30, 0.18, 0.359, "m"),
    "wheel_width": (0.10, 0.05, 0.359, "m"),
    "drum_diameter": (0.28, 0.16, 0.359, "m"),
    "drum_length": (0.25, 0.12, 0.359, "m"),  # each of four drums
    "drive_motor_power": (100.0, 25.0, 300.0, "W"),
    "drive_gear_ratio": (100.0, 15.0, 180.0, None),
    "drum_motor_power": (100.0, 20.0, 400.0, "W"),  # each pair shares one
    "drum_rpm": (20.0, 5.0, 120.0, "rpm"),
    "dump_rpm": (15.0, 5.0, 60.0, "rpm"),
    "drum_gear_ratio": (120.0, 30.0, 400.0, None),
    "cut_depth": (0.012, 0.003, 0.025, "m"),  # scoop protrusion / maximum bite
    "frame_tube_diameter": (0.055, 0.025, 0.10, "m"),
    "frame_wall": (0.002, 0.001, 0.005, "m"),
    "arm_tube_diameter": (0.050, 0.025, 0.10, "m"),
    "arm_wall": (0.002, 0.001, 0.005, "m"),
    "battery_capacity": (50.0, 10.0, 300.0, "W*h"),
    "actuator_speed": (0.025, 0.005, 0.08, "m/s"),
    "travel_actuator_length": (0.38, 0.30, 0.65, "m"),
    "actuator_retracted_length": (0.45, 0.25, 0.65, "m"),  # compact starting pose
    "actuator_extended_length": (0.68, 0.45, 1.05, "m"),
    "top_pivot_height": (0.45, 0.35, 0.70, "m"),  # above level ground
    "actuator_pivot_x": (0.15, 0.02, 0.30, "m"),  # actuator mount; independent of arm pivot_x
    "bottom_pivot_height": (0.15, 0.09, 0.30, "m"),
    "arm_length": (0.60, 0.30, 0.70, "m"),
    "pivot_x": (0.13, 0.05, 0.30, "m"),  # front station from rover center; rear mirrored
    "wheelbase": (0.40, 0.38, 0.65, "m"),  # exploratory upper bound expanded by user; overall length cap still applies
    "track_width": (0.54, 0.50, 0.699, "m"),  # wheel center spacing; width envelope also applies
    "actuator_force_rating": (2000.0, 500.0, 6000.0, "N"),  # each of two
}

# name: (value, units). These are fixed during one optimization, but editable.
ASSUMPTIONS = {
    # Fixed for each solve; CLI --set accepts degrees. Separate from static slope_angle.
    "empty_slope_degrees": (30.0, "deg"),
    "loaded_slope_degrees": (20.0, "deg"),
    "gravity": (9.80665, "m/s**2"),
    "run_time": (900.0, "s"),
    "traversal_time_one_way": (45.0, "s"),  # includes all route turns, excludes deposition
    "travel_speed": (1.0, "m/s"),  # reference profile shape, rescaled to 45 s
    "acceleration": (0.40, "m/s**2"),  # reference profile shape, not an optimized quantity
    "turn_angle": (3.141592653589793, "rad"),  # mandatory half-loaded deposit turn
    "overhead_time": (8.0, "s"),  # alignment only; linkage motion is modeled separately
    "electronics_power": (25.0, "W"),
    "actuator_idle_power": (2.0, "W"),  # per active motor, plus mechanical/efficiency
    "soil_density": (1500.0, "kg/m**3"),  # excavated bulk density, uncertain
    "berm_density": (1500.0, "kg/m**3"),  # deposited volume = retained mass / density
    "fill_fraction": (0.40, None),
    "delivery_efficiency": (0.90, None),  # fraction retained ABOVE grade IN target
    "capture_fraction": (0.65, None),  # retained fraction of reachable fresh soil; uncalibrated
    "cut_pressure": (18000.0, "Pa"),  # effective cutting resistance, not BP-1 data
    "cut_engagement": (0.15, None),  # fraction of full-width cut engaged
    "internal_lift_factor": (0.15, None),  # internal soil torque / (payload*g*radius)
    "vertical_cut_fraction": (0.50, None),
    "force_imbalance": (0.15, None),  # residual horizontal force after cancellation
    "dump_revolutions": (3.0, None),  # each pair unloads sequentially
    "drum_efficiency": (0.70, None),  # combined motor + gearbox electrical efficiency
    "drum_gear_efficiency": (0.80, None),
    "drive_efficiency": (0.72, None),  # provisional combined motor + gearbox efficiency
    "drive_gear_efficiency": (0.80, None),
    "motor_rated_rpm": (3000.0, "rpm"),
    "traction_coefficient": (0.60, None),
    "rocker_load_factor": (1.20, None),  # worst contact pressure / equal-share pressure
    "wheel_deflection_fraction": (0.07, None),  # deflection / wheel diameter
    "rolling_base": (0.04, None),
    "rolling_pressure_scale": (200000.0, "Pa"),
    "allowable_ground_pressure": (25000.0, "Pa"),  # engineering screening assumption
    "scrub_coefficient": (0.25, None),
    "turn_speed_fraction": (0.35, None),  # wheel rim speed relative to travel speed
    "turn_accel_fraction": (0.50, None),
    "chassis_height": (0.35, "m"),  # top = wheel diameter + chassis height
    "drum_pair_gap": (0.06, "m"),  # combined shaft/bearing/center clearance
    "fixed_mass": (6.0, "kg"),  # electronics, wiring, guards ONLY; structure is separate
    "wheel_areal_mass": (8.0, "kg/m**2"),
    "drum_areal_mass": (5.0, "kg/m**2"),
    "motor_mass_per_watt": (0.006, "kg/W"),
    "gear_mass_per_torque": (0.05, "kg/(N*m)"),
    "drive_fixed_mass": (0.35, "kg"),  # per wheel motor/gearbox
    "drum_fixed_mass": (0.50, "kg"),  # per pair motor/gearbox
    "battery_specific_energy": (140.0, "W*h/kg"),  # complete pack
    "battery_usable_fraction": (0.80, None),
    "battery_c_rate": (15.0, "1/h"),  # continuous pack power / nominal energy
    "mass_score_weight": (1.0, None),
    "energy_score_weight": (1.0, None),
    "transport_clearance": (0.08, "m"),
    "mechanism_clearance": (0.015, "m"),
    "minimum_pivot_spacing": (0.08, "m"),
    "minimum_stroke": (0.04, "m"),
    "actuator_housing_allowance": (0.04, "m"),  # generic body-length diagnostic only, not a constraint
    "pit_depth": (0.45, "m"),
    "actuator_efficiency": (0.65, None),
    "actuator_safety_factor": (1.50, None),
    "breakout_force": (100.0, "N"),  # each pair; preliminary extraction resistance
    "actuator_fixed_mass": (0.60, "kg"),
    "actuator_mass_per_force": (0.0004, "kg/N"),
    "actuator_mass_per_power": (0.003, "kg/W"),
    "actuator_mass_per_length": (1.0, "kg/m"),
    "stored_soil_density": (1250.0, "kg/m**3"),
    "usable_volume_fraction": (0.90, None),
    "scoop_opening": (0.035, "m"),
    "scoop_depth_fraction": (0.50, None),  # empirical screen, not a universal law
    "scoops_per_revolution": (4.0, None),
    "max_dig_froude": (0.30, None),  # legacy diagnostic reference only; NOT enforced
    "max_dump_froude": (0.20, None),  # legacy diagnostic reference only; NOT enforced
    "drum_spinup_time": (2.0, "s"),
    "drum_torque_service_factor": (1.25, None),
    "minimum_dig_depth": (0.005, "m"),
    "minimum_payload": (0.05, "kg"),
    "fresh_site_distance": (0.80, "m"),  # move to a new footprint EACH cycle
    "site_speed": (0.10, "m/s"),
    "available_excavation_area": (12.0, "m**2"),  # explicit planning assumption, not a rule
    "tube_density": (2700.0, "kg/m**3"),
    "tube_youngs_modulus": (69e9, "Pa"),
    "tube_yield_strength": (240e6, "Pa"),  # replace with actual material/joint allowables
    "structure_safety_factor": (2.0, None),
    "structure_load_factor": (1.5, None),
    "allowable_deflection": (0.005, "m"),
    "joint_mass": (2.0, "kg"),
    "differential_mass": (1.0, "kg"),
    "obstacle_height": (0.40, "m"),  # fully loaded, travel pose, one wheel raised
    "slope_angle": (math.pi/6, "rad"),  # USER: fully loaded 30-degree slope, travel pose
    "rocker_pivot_offset": (0.10, "m"),  # estimated above wheel-center line; editable
}

# One-way path: translate [m], then rotate [degrees]. All outbound turns are left.
# Reverse traversal follows these segments in reverse order with opposite yaw.
# Each segment is rest-to-rest. Total = 9 m and 180 degrees each way.
ROUTE = ((2.0, 90.0), (3.0, 45.0), (3.0, 45.0), (1.0, 0.0))

# Optional plate-frame profile. Reference DESIGN/ASSUMPTIONS above stay intact.
# Bounds and starting section sizes are exploratory, not purchased hardware.
PLATE_FRAME_DESIGN = {
    "arm_box_width": (0.0508, 0.025, 0.10, "m"),  # transverse; USER baseline: 2 inches
    "arm_box_height": (0.0508, 0.025, 0.10, "m"),  # section depth in arm-motion plane
    "arm_wall": (0.0015875, 0.001, 0.005, "m"),  # USER baseline: 1/16 inch
    "cf_tube_span": (0.28, 0.16, 0.46, "m"),  # plate center-to-center spacing
    "cf_tube_wall": (0.002, 0.001, 0.006, "m"),  # OD = fixed ID + 2*wall
    "plate_spoke_width": (0.035, 0.012, 0.070, "m"),  # in-plane; all seven strips share it
    "rocker_box_width": (0.035, 0.020, 0.065, "m"),  # transverse dimension
    "rocker_box_height": (0.055, 0.025, 0.090, "m"),
    "rocker_wall": (0.002, 0.001, 0.005, "m"),
    "axle_outer_diameter": (0.055, 0.025, 0.090, "m"),
    "axle_wall": (0.003, 0.001, 0.006, "m"),
    "rocker_pivot_offset": (0.10, 0.020, 0.150, "m"),  # centered longitudinally; adjustable axle height
}
PLATE_FRAME_ASSUMPTIONS = {
    "plate_thickness": (0.00635, "m"),  # USER: 1/4 inch 6061-T6 sheet
    "cf_tube_inner_diameter": (0.0254, "m"),  # USER: connector requires 1 inch ID
    "wheel_rocker_gap": (0.0127, "m"),  # USER: wheel inner face to rocker outer face
    "plate_hub_radius": (0.030, "m"),  # provisional central mounting pad
    "plate_cutout_clearance": (0.005, "m"),  # provisional minimum cutout inradius after strips
    "frame_mount_clearance": (0.003, "m"),  # provisional material beyond CF tube envelope
    "minimum_axle_overhang": (0.010, "m"),  # provisional space between plate and rocker bearing
    "plate_buckling_k": (1.0, None),  # pinned-ended weak-axis strip screen, not whole-plate buckling
    "cf_density": (1600.0, "kg/m**3"),  # PROVISIONAL: actual tube/layup not selected
    "cf_flexural_modulus": (70e9, "Pa"),  # PROVISIONAL effective bending modulus, not fiber modulus
    "cf_flexural_strength": (350e6, "Pa"),  # PROVISIONAL screening value; not a certified allowable
}

X44_ASSUMPTIONS = {
    "motor_bus_voltage": (12.0, "V"),  # user; this curve profile supports 12 V only
    "x44_motor_mass": (0.75 * 0.45359237, "kg"),  # WCP-0941 incl. Talon FX, official specification
    "drive_gearbox_mass": (0.261 + 0.133 + 2*0.138 + 0.027, "kg"),  # REV base + 4/5/5 + shaft
    "drum_gearbox_mass": (0.261 + 3*0.155 + 0.027, "kg"),  # PROVISIONAL three-stage inventory budget, not selected BOM
    "drive_stator_current_limit": (0.0, "A"),  # 0 = undecided; voltage envelope ONLY, no continuous rating
    "drum_stator_current_limit": (0.0, "A"),
}


PA04_DESIGN = {"actuator_catalog_stroke": (0.1524, 0.0508, 1.016, "m")}
PA04_ASSUMPTIONS = {
    "actuator_installation_allowance": (0.1613408, "m"),  # USER, added once to EACH endpoint
}


def configuration(structure="reference", motors="reference", actuators="reference"):
    """Independent dictionaries per Problem; no mutation of reference config."""
    if structure not in ("reference", "plate-frame"):
        raise ValueError(f"Unknown structure profile {structure!r}")
    design, assumptions = dict(DESIGN), dict(ASSUMPTIONS)
    if structure == "plate-frame":
        # Generic frame tubes are replaced by explicit rockers, CF rods and axle.
        for name in ("frame_tube_diameter", "frame_wall", "arm_tube_diameter"):
            entry = design.pop(name)
            assumptions[name] = (entry[0], entry[-1])
        assumptions.pop("rocker_pivot_offset", None)
        design.update(PLATE_FRAME_DESIGN)
        assumptions.update(PLATE_FRAME_ASSUMPTIONS)
    if motors not in ("reference", "x44-standard"):
        raise ValueError(f"Unknown motor profile {motors!r}")
    if motors == "x44-standard":
        for name in ("drive_motor_power", "drum_motor_power", "drive_gear_ratio"):
            entry = design.pop(name)
            assumptions[name] = ((100.0 if name == "drive_gear_ratio" else 835.4), entry[-1])
        # Continuous exploratory ratio; select and recheck a real cartridge stack later.
        design["drum_gear_ratio"] = (75.0, 9.0, 125.0, None)
        assumptions.update(X44_ASSUMPTIONS)
    if actuators not in ("reference", "pa04-900"):
        raise ValueError(f"Unknown actuator profile {actuators!r}")
    if actuators == "pa04-900":
        for name, value in (("actuator_force_rating", 900*4.4482216152605),
                            ("actuator_speed", .39*.0254)):
            entry = design.pop(name)
            assumptions[name] = (value, entry[-1])
        design.update(PA04_DESIGN)
        assumptions.update(PA04_ASSUMPTIONS)
    return design, assumptions


def input_spec(name):
    """Unit/default lookup for adapters, including optional profile inputs."""
    return DESIGN.get(name, ASSUMPTIONS.get(name,
           PLATE_FRAME_DESIGN.get(name, PLATE_FRAME_ASSUMPTIONS.get(name,
           X44_ASSUMPTIONS.get(name, PA04_DESIGN.get(name, PA04_ASSUMPTIONS.get(name)))))))

# name: (upper limit, units); guidebook section 13. These are stowed dimensions.
REQUIREMENTS = {
    "dry_mass": (80.0, "kg"),
    "rover_length": (1.50, "m"),
    "rover_width": (0.75, "m"),
    "rover_height": (0.75, "m"),
}
