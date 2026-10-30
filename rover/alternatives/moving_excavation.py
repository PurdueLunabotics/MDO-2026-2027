"""Tandem, straight-feed excavation; analytical assumptions, not calibrated soil.

The front pair removes an inlet-depth layer and the rear removes the next
layer in the same tracks. A staged pass starts the rear at the front's entry
and raises the front at its exit. Each pair cuts the same lane length; the
rover drives that length plus the pair separation. No plunge/endcap soil is
credited. Entry/exit transients and terrain evolution need hardware validation.
The existing full linkage sweep remains a conservative motion/load allowance.
"""
import math

from ..components import scalar, travel_profile
from ..discipline import Discipline, names
from ..excavation import Excavation
from ..motors import ReferenceMotorProvider
from .offset_attachment import attachment_point


def moving_supply(radius, length, inlet_depth, fill, usable,
                  stored_density, soil_density, capture, speed, rpm, scoops):
    """Full-capacity lane demand and un-clipped advance per scoop, in SI units."""
    args = (radius, length, inlet_depth, stored_density, soil_density,
            speed, rpm, scoops)
    if any(not math.isfinite(x) or x <= 0 for x in args):
        raise ValueError("Moving excavation dimensions, densities and speeds must be positive")
    if not inlet_depth < radius:
        raise ValueError("A positive shell and rear penetration below one diameter are required")
    if any(not math.isfinite(x) or not 0 < x <= 1 for x in (fill, usable, capture)):
        raise ValueError("Fill, usable volume and capture must be in (0, 1]")
    capacity = 4*math.pi*(radius-inlet_depth)**2*length*usable*fill
    payload = capacity*stored_density
    fresh = payload/(soil_density*capture)
    cross_section = 4*length*inlet_depth  # two full-width tracks, two fresh layers
    frequency = scoops*rpm/60
    distance = fresh/cross_section
    return {"storage_capacity": capacity, "payload_mass": payload,
            "fresh_volume": fresh, "feed_distance": distance,
            "pair_cutting_time": distance/speed,
            "soil_supply_rate": cross_section*speed,
            "scoop_throughput": 4*length*inlet_depth**2*frequency,
            "feed_per_scoop": speed/frequency,
            "feed_speed_limit": inlet_depth*frequency,
            "feed_margin": inlet_depth-speed/frequency}


def operating_pose(arm, bottom_x, bottom_z, upper_x, upper_z, height,
                   side=0., top=0., *, guarded=False):
    """Outward inverse arm geometry, plus signed vertical reach reserve [m].

    The guard is exclusively for finite optimizer diagnostics. Its negative
    reserve is retained and enforced; the constructed point is not called valid.
    """
    if (not all(math.isfinite(x) for x in (arm, bottom_x, bottom_z,
                                          upper_x, upper_z, height, side, top))
            or arm <= 0 or arm+side <= 0):
        raise ValueError("Finite arm coordinates and a positive pin station are required")
    dz = height-bottom_z
    reserve = arm-abs(dz)
    if reserve < 0 and not guarded:
        raise ValueError("Requested axle height is outside the arm circle")
    dx = math.sqrt(max(arm*arm-dz*dz, 0.))
    x = bottom_x+dx
    # An unreachable diagnostic still has finite coordinates and a failing reserve.
    if dx == 0 and dz == 0:
        dx = arm
        x = bottom_x+dx
    pin_x, pin_z = attachment_point(bottom_x, bottom_z, x, height, side, top)
    return x, height, math.hypot(pin_x-upper_x, pin_z-upper_z), reserve


class MovingExcavation(Excavation):
    """Replacement through ModelSet; original stationary helper remains available."""
    MOVING_EXCAVATION = True
    REQUIRED_OUTPUT_UNITS = Excavation.OUTPUT_UNITS
    CONFIG_INPUTS = tuple(sorted((set(Excavation.CONFIG_INPUTS) - names(
        "force_imbalance minimum_dig_depth actuator_speed cut_depth scoop_opening scoop_depth_fraction cut_engagement")) | names("""
        inlet_depth feed_speed wheel_diameter rolling_base acceleration
        arm_length pivot_x bottom_pivot_height actuator_pivot_x top_pivot_height
        lin_act_side_arm_offset lin_act_top_arm_offset
        drive_motor_power drive_gear_ratio drive_gear_efficiency drive_efficiency
    """)))
    COUPLED_INPUTS = {**Excavation.COUPLED_INPUTS, "travel_y": "m"}
    OUTPUT_UNITS = {**Excavation.OUTPUT_UNITS,
        **{n: "m" for n in names("""
            front_dig_depth rear_dig_depth front_dig_x front_dig_y rear_dig_x rear_dig_y
            front_dig_actuator_length rear_dig_actuator_length operating_reach_margin
            axle_ground_margin excavation_domain_margin feed_per_scoop feed_margin inlet_width cut_depth
            feed_distance feed_drive_distance
        """)},
        "feed_speed_limit": "m/s", "feed_time": "s", "pair_cutting_time": "s",
        "soil_supply_rate": "m**3/s", "scoop_throughput": "m**3/s",
        "feed_drive_energy": "J", "feed_drive_peak_power": "W",
        "feed_motor_margin": "N", "feed_traction_margin": "N", "feed_power_margin": "W"}
    _capacity = names("drum_diameter drum_length inlet_depth fill_fraction usable_volume_fraction")
    _mass = _capacity | {"stored_soil_density"}
    _fresh = _mass | names("soil_density capture_fraction")
    _path = _fresh | {"inlet_depth"}
    _rate = names("drum_length inlet_depth feed_speed")
    _bite = names("feed_speed drum_rpm scoops_per_revolution")
    _force = _bite | names("drum_length cut_pressure")
    _pose = names("drum_diameter inlet_depth arm_length pivot_x bottom_pivot_height actuator_pivot_x top_pivot_height lin_act_side_arm_offset lin_act_top_arm_offset")
    _reach = _pose | names("travel_y extended_y travel_actuator_length actuator_extended_length")
    _time = _path | _pose | names("feed_speed acceleration")
    _torque = _mass | _force | names("gravity drum_diameter internal_lift_factor")
    _spin = _mass | names("drum_mass drum_diameter drum_rpm dump_rpm drum_spinup_time")
    _driveforce = _force | _mass | names("dry_mass gravity rolling_base acceleration vertical_cut_fraction traction_coefficient")
    _drivework = _driveforce | _time | names("feed_speed wheel_diameter")

    DEPENDENCIES = {
        "storage_capacity": _capacity, "target_payload_mass": _mass,
        "payload_mass": _mass, "payload_volume": _capacity,
        "payload_margin": _mass | {"minimum_payload"},
        "fresh_volume": _fresh, "reachable_soil_mass": _mass,
        "trench_depth": {"inlet_depth"},
        "front_dig_depth": {"inlet_depth"}, "rear_dig_depth": {"inlet_depth"},
        "inlet_width": {"drum_length"}, "cut_depth": {"inlet_depth"},
        **dict.fromkeys(names("front_dig_x front_dig_y rear_dig_x rear_dig_y front_dig_actuator_length rear_dig_actuator_length"), _pose),
        "operating_reach_margin": _reach, "dig_reach_margin": _reach,
        "axle_ground_margin": {"extended_y"},
        "excavation_domain_margin": _pose,
        "feed_distance": _path, "feed_drive_distance": _path | _pose,
        "feed_time": _time, "pair_cutting_time": _time,
        "soil_supply_rate": _rate,
        "scoop_throughput": names("drum_length inlet_depth scoops_per_revolution drum_rpm"),
        "feed_per_scoop": _bite, "feed_margin": _bite | {"inlet_depth"},
        "feed_speed_limit": names("inlet_depth scoops_per_revolution drum_rpm"),
        "excavation_footprint": _path | _pose | names("drum_pair_gap drum_length drum_diameter"),
        "cut_force": _force,
        "drum_froude": names("drum_rpm drum_diameter gravity"),
        "dig_froude_margin": names("drum_rpm drum_diameter gravity max_dig_froude"),
        "dump_froude_margin": names("dump_rpm drum_diameter gravity max_dump_froude"),
        "dig_time": _time | {"drum_spinup_time"}, "dig_overlap_time": set(),
        "dump_time": names("dump_revolutions dump_rpm drum_spinup_time"),
        "penetration_margin": _force | names("dry_mass gravity vertical_cut_fraction"),
        "dig_traction_margin": _driveforce,
        "feed_traction_margin": _driveforce,
        "cut_depth_margin": names("inlet_depth drum_diameter"),
        "dump_power": _torque | _spin | names("actuator_idle_power"),
        "dump_energy": _torque | _spin | names("dump_revolutions actuator_idle_power"),
        "dump_torque_margin": _torque | _spin | {"drum_torque_service_factor"},
        "drum_torque_margin": _torque | _spin | {"drum_torque_service_factor"},
        "drum_power_margin": _torque | _spin | {"drum_torque_service_factor"},
        "dig_energy": _torque | _spin | _drivework | names("actuator_idle_power drum_spinup_time"),
        "dig_power": _torque | _spin | _driveforce | names("feed_speed actuator_idle_power"),
        "feed_drive_energy": _drivework | {"actuator_idle_power"},
        "feed_drive_peak_power": _driveforce | names("feed_speed actuator_idle_power"),
        "feed_motor_margin": _driveforce | names("feed_speed wheel_diameter"),
        "feed_power_margin": _driveforce | {"feed_speed"},
    }

    def setup(self):
        provider = self.options["motor_model"]
        base = set(self.CONFIG_INPUTS)
        old = ReferenceMotorProvider.input_names("drum") | ReferenceMotorProvider.input_names("drive")
        drum = provider.input_names("drum")
        drive = provider.input_names("drive")
        self.CONFIG_INPUTS = tuple(sorted(base-old | drum | drive))
        mapping = {n: set(s) for n, s in self.DEPENDENCIES.items()}
        for n in names("dump_power dump_energy dump_torque_margin drum_torque_margin drum_power_margin dig_energy dig_power"):
            mapping[n] |= drum
        for n in names("feed_drive_energy feed_drive_peak_power feed_motor_margin feed_power_margin dig_energy dig_power"):
            mapping[n] |= drive
        self.DEPENDENCIES = mapping
        self.OUTPUT_UNITS = dict(self.OUTPUT_UNITS)
        Discipline.setup(self)

    def compute(self, inputs, outputs):
        values = {name: scalar(inputs, name) for name in inputs}
        radius = values["drum_diameter"]/2
        fr = (values["drum_rpm"]*math.pi/30)**2*radius/values["gravity"]
        capture, lift = self.granular_response(inputs, fr)
        dump_fr = (values["dump_rpm"]*math.pi/30)**2*radius/values["gravity"]
        _, dump_lift = self.granular_response(inputs, dump_fr)
        result = moving_performance(values, self.options["motor_model"],
                                    responses=(capture, lift, dump_lift))
        for name, value in result.items():
            outputs[name] = value


def moving_performance(values, motor_provider=ReferenceMotorProvider, *, responses=None):
    """Pure staged-pass force, motion, work and electrical accounting.

    ``values`` contains scalar SI configuration and coupled state. A selected
    deterministic motor provider and optional (capture, dig lift, dump lift)
    response values preserve the replacement seam without simulator side effects.
    """
    v = values.__getitem__
    radius, length, depth = v("drum_diameter")/2, v("drum_length"), v("inlet_depth")
    omega, dump_omega = v("drum_rpm")*math.pi/30, v("dump_rpm")*math.pi/30
    fr = omega*omega*radius/v("gravity")
    capture, lift, dump_lift = responses if responses is not None else (v("capture_fraction"), v("internal_lift_factor"), v("internal_lift_factor"))
    # Invalid geometric optimizer trials remain finite with signed failures.
    result = moving_supply(radius, length, min(depth, radius*(1-1e-9)), v("fill_fraction"),
                           v("usable_volume_fraction"), v("stored_soil_density"),
                           v("soil_density"), capture, v("feed_speed"),
                           v("drum_rpm"), v("scoops_per_revolution"))
    # Preserve requested/raw domain diagnostics, never declare guarded soil feasible.
    front = operating_pose(v("arm_length"), v("pivot_x"), v("bottom_pivot_height"),
                           v("actuator_pivot_x"), v("top_pivot_height"), radius-depth,
                           v("lin_act_side_arm_offset"), v("lin_act_top_arm_offset"), guarded=True)
    rear = operating_pose(v("arm_length"), v("pivot_x"), v("bottom_pivot_height"),
                          v("actuator_pivot_x"), v("top_pivot_height"), radius-2*depth,
                          v("lin_act_side_arm_offset"), v("lin_act_top_arm_offset"), guarded=True)
    reach = min(*(p[3] for p in (front, rear)),
                *(v("travel_y")-p[1] for p in (front, rear)),
                *(p[1]-v("extended_y") for p in (front, rear)),
                *(p[2]-v("travel_actuator_length") for p in (front, rear)),
                *(v("actuator_extended_length")-p[2] for p in (front, rear)))
    mass, fresh = result["payload_mass"], result["fresh_volume"]
    separation = abs(front[0])+abs(rear[0])
    distance = result["feed_distance"]+separation
    # Stop at rear entry and front exit to stage the two independent arms.
    # Sorting these stations handles lanes shorter than the station spacing.
    short = min(result["feed_distance"], separation)
    middle = abs(result["feed_distance"]-separation)
    first_time, first_peak = travel_profile(short, v("feed_speed"), v("acceleration"))
    if middle > 1e-12:
        middle_time, middle_peak = travel_profile(middle, v("feed_speed"), v("acceleration"))
    else:
        middle_time, middle_peak = 0., 0.
    feed_time = 2*first_time+middle_time
    cutting_time = first_time+(middle_time if result["feed_distance"] > separation else 0.)
    # The entire axial inlet width cuts; no fractional-width credit.
    force = v("cut_pressure")*4*length*result["feed_per_scoop"]
    internal = mass*v("gravity")*radius*lift
    dump_internal = mass*v("gravity")*radius*dump_lift
    inertia = (v("drum_mass")+mass)*radius*radius
    spin_torque = inertia*omega/v("drum_spinup_time")
    dump_spin = inertia/2*dump_omega/v("drum_spinup_time")
    cutting_torque = force*radius+internal
    peak_torque = max(cutting_torque, spin_torque)
    dumping_torque = max(dump_internal/2, dump_spin)
    drum_peak = peak_torque*omega
    dump_peak = dumping_torque*dump_omega
    # Stop drums during staged arm motion. Two starts per pair cover the
    # shared-cut interval; retain that conservative allowance for short lanes.
    dig_time = feed_time+4*v("drum_spinup_time")
    dump_time = 2*(v("dump_revolutions")*60/v("dump_rpm")+v("drum_spinup_time"))
    # Pressure-law cutting force is proportional to instantaneous feed.
    # Integrate that term by distance; internal lifting follows rotation time.
    drum_work = (force*radius*omega*result["feed_distance"]/v("feed_speed")
                 +.5*internal*omega*cutting_time+inertia*omega*omega)
    dump_work = .5*dump_internal*dump_omega*(dump_time-2*v("drum_spinup_time"))/2+.5*inertia*dump_omega*dump_omega
    provider = motor_provider
    drum_motor, drive_motor = provider.create(v, "drum"), provider.create(v, "drive")
    drum_e = drum_motor.evaluate(drum_work/2, drum_peak/2)
    dump_e = drum_motor.evaluate(dump_work/2, dump_peak)
    # Full horizontal tool force is resisted; no opposing-drum cancellation credit.
    full_mass = v("dry_mass")+mass
    steady_force = full_mass*v("gravity")*v("rolling_base")+force
    drive_force = steady_force+full_mass*v("acceleration")
    normal = v("dry_mass")*v("gravity")-force*v("vertical_cut_fraction")
    traction = v("traction_coefficient")*normal-drive_force
    wheel_rpm = v("feed_speed")/(v("wheel_diameter")/2)*30/math.pi
    drive_work = ((v("dry_mass")+.5*mass)*v("gravity")*v("rolling_base")*distance
                  + force*result["feed_distance"]
                  +.5*full_mass*(2*first_peak**2+middle_peak**2))
    drive_peak = drive_force*v("feed_speed")
    drive_e = drive_motor.evaluate(drive_work/4, drive_peak/4)
    idle = v("actuator_idle_power")
    feed_energy = 4*drive_e["energy"]+4*idle*feed_time
    feed_peak = 4*drive_e["peak"]+4*idle
    domain = min(radius-depth, front[3], rear[3])
    # Strictly singular domains must not pass a nonnegative-margin check.
    domain = domain if domain > 0 else min(domain, -1e-5)
    result.update({
        "payload_volume": mass/v("stored_soil_density"), "reachable_soil_mass": mass,
        "target_payload_mass": mass, "payload_margin": mass-v("minimum_payload"),
        "trench_depth": 2*depth, "front_dig_depth": depth, "rear_dig_depth": 2*depth,
        "inlet_width": length, "cut_depth": depth,  # derived compatibility port for linkage work
        "front_dig_x": front[0], "front_dig_y": front[1],
        "rear_dig_x": -rear[0], "rear_dig_y": rear[1],
        "front_dig_actuator_length": front[2], "rear_dig_actuator_length": rear[2],
        "operating_reach_margin": reach, "dig_reach_margin": reach,
        "axle_ground_margin": -v("extended_y"),
        "excavation_domain_margin": domain,
        "feed_drive_distance": distance, "feed_time": feed_time, "pair_cutting_time": cutting_time,
        "excavation_footprint": (distance+2*radius)*(2*length+v("drum_pair_gap")),
        "cut_force": force, "drum_froude": fr, "dig_time": dig_time,
        "dig_overlap_time": 0., "dump_time": dump_time,
        "dig_power": 2*drum_e["peak"]+2*idle+feed_peak,
        "dump_power": dump_e["peak"]+idle,
        "dig_energy": 2*drum_e["energy"]+2*idle*(cutting_time+2*v("drum_spinup_time"))+feed_energy,
        "dump_energy": 2*dump_e["energy"]+idle*dump_time,
        "drum_power_margin": 2*drum_motor.peak_power()-drum_peak*v("drum_torque_service_factor"),
        "drum_torque_margin": 2*drum_motor.available_torque(v("drum_rpm"))-peak_torque*v("drum_torque_service_factor"),
        "dump_torque_margin": drum_motor.available_torque(v("dump_rpm"))-dumping_torque*v("drum_torque_service_factor"),
        "dig_froude_margin": v("max_dig_froude")-fr,
        "dump_froude_margin": v("max_dump_froude")-dump_omega*dump_omega*radius/v("gravity"),
        "penetration_margin": v("dry_mass")*v("gravity")-force*v("vertical_cut_fraction"),
        "dig_traction_margin": traction, "feed_traction_margin": traction,
        "feed_motor_margin": 4*drive_motor.available_torque(wheel_rpm)/(v("wheel_diameter")/2)-drive_force,
        "feed_power_margin": 4*drive_motor.peak_power()-drive_peak,
        "feed_drive_energy": feed_energy, "feed_drive_peak_power": feed_peak,
        "cut_depth_margin": radius*.25-depth,
    })
    return result
