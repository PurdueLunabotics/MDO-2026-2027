"""Finite stationary-plunge excavation and drum drive screens (uncalibrated).

Drum diameter is the outer cutting envelope. cut_depth is radial scoop protrusion
and a maximum per-scoop bite; shell radius is R-cut_depth. Only the endpoint
below-grade segment is credited as fresh soil, conservatively omitting extra arc
sweep. Each cycle must move to a fresh site. No collapse/repeated-soil credit.
"""
import math
from .components import scalar
from .discipline import Discipline, names
from .motors import ReferenceMotorProvider, motor_dependencies


def segment_area(radius, depth):
    """Circle segment below a horizontal grade [m2]; exact empty/full limits."""
    h = min(max(depth, 0.0), 2*radius)
    return radius**2*math.acos((radius-h)/radius) - (radius-h)*math.sqrt(max(2*radius*h-h*h, 0.0))


def motor_torque(power, rated_rpm, ratio, efficiency, output_rpm):
    """Legacy helper retained for callers; the envelope has one shared owner."""
    from .motors import ReferenceMotor, Gearmotor
    return Gearmotor(ReferenceMotor(power,rated_rpm),ratio,efficiency,1.).available_torque(output_rpm)


class Excavation(Discipline):
    CONFIG_INPUTS = tuple(sorted(names("""
        drum_diameter drum_length cut_depth usable_volume_fraction
        fill_fraction soil_density stored_soil_density capture_fraction
        scoop_opening scoop_depth_fraction scoops_per_revolution drum_pair_gap
        drum_rpm dump_rpm max_dig_froude max_dump_froude dump_revolutions
        drum_motor_power drum_gear_ratio motor_rated_rpm drum_gear_efficiency
        drum_efficiency drum_spinup_time drum_torque_service_factor
        cut_pressure cut_engagement internal_lift_factor
        vertical_cut_fraction force_imbalance gravity traction_coefficient
        minimum_dig_depth minimum_payload actuator_idle_power actuator_speed
        travel_actuator_length actuator_extended_length
    """)))
    COUPLED_INPUTS = {"dry_mass": "kg", "drum_mass": "kg", "extended_y": "m", "extended_x": "m", "lowering_time": "s"}
    OUTPUT_UNITS = {
        "payload_mass": "kg", "payload_volume": "m**3", "fresh_volume": "m**3", "storage_capacity": "m**3",
        "reachable_soil_mass": "kg", "target_payload_mass": "kg",
        "trench_depth": "m", "excavation_footprint": "m**2", "cut_force": "N", "drum_froude": None,
        "dig_time": "s", "dig_overlap_time": "s", "dump_time": "s", "dig_power": "W", "dump_power": "W",
        "dig_energy": "J", "dump_energy": "J", "drum_power_margin": "W", "drum_torque_margin": "N*m",
        "dump_torque_margin": "N*m", "dig_froude_margin": None, "dump_froude_margin": None,
        "penetration_margin": "N", "dig_traction_margin": "N", "cut_depth_margin": "m",
        "dig_reach_margin": "m", "payload_margin": "kg",
    }
    geom = names("drum_diameter drum_length extended_y")
    capacity = names("drum_diameter cut_depth drum_length fill_fraction usable_volume_fraction")
    fr = names("drum_rpm drum_diameter gravity")
    captured = geom | names("capture_fraction soil_density")
    mass = captured | capacity | {"stored_soil_density"}
    force = geom | names("cut_depth cut_pressure cut_engagement")
    torque = mass | names("internal_lift_factor gravity drum_diameter")
    lowering = {"lowering_time"}
    time = geom | lowering | names("scoops_per_revolution cut_depth scoop_opening drum_rpm drum_spinup_time")
    spin = names("drum_mass drum_diameter drum_rpm dump_rpm drum_spinup_time") | mass
    hardware = names("drum_motor_power motor_rated_rpm drum_gear_ratio drum_gear_efficiency drum_torque_service_factor")
    DEPENDENCIES = {
        "fresh_volume": geom, "trench_depth": names("drum_diameter extended_y"),
        "excavation_footprint": names("drum_diameter drum_length drum_pair_gap extended_x"), "storage_capacity": capacity,
        "reachable_soil_mass": captured, "target_payload_mass": capacity | {"stored_soil_density"},
        "payload_mass": mass, "payload_volume": mass | {"stored_soil_density"}, "payload_margin": mass | {"minimum_payload"},
        "cut_force": force, "drum_froude": fr,
        "dig_time": time, "dig_overlap_time": lowering,
        "dump_time": names("dump_revolutions dump_rpm drum_spinup_time"),
        "dig_power": torque | force | spin | names("drum_efficiency actuator_idle_power"),
        "dump_power": torque | spin | names("drum_efficiency actuator_idle_power"),
        "dig_energy": torque | force | spin | time | names("drum_efficiency actuator_idle_power"),
        "dump_energy": torque | spin | names("dump_revolutions drum_efficiency actuator_idle_power"),
        "drum_power_margin": torque | force | spin | hardware,
        "drum_torque_margin": torque | force | spin | hardware,
        "dump_torque_margin": torque | spin | hardware,
        "dig_froude_margin": fr | {"max_dig_froude"},
        "dump_froude_margin": names("dump_rpm drum_diameter gravity max_dump_froude"),
        "penetration_margin": force | names("dry_mass gravity vertical_cut_fraction"),
        "dig_traction_margin": force | names("dry_mass gravity vertical_cut_fraction traction_coefficient force_imbalance"),
        "cut_depth_margin": names("cut_depth scoop_opening scoop_depth_fraction drum_diameter"),
        "dig_reach_margin": names("drum_diameter extended_y minimum_dig_depth"),
    }

    def initialize(self):
        self.options.declare("motor_model", default=ReferenceMotorProvider)

    def setup(self):
        motor = self.options["motor_model"]
        self.CONFIG_INPUTS = tuple(sorted((set(self.CONFIG_INPUTS)-ReferenceMotorProvider.input_names("drum"))
                                          | motor.input_names("drum")))
        changed = names("dig_power dump_power dig_energy dump_energy drum_power_margin drum_torque_margin dump_torque_margin")
        mechanical = self.torque | self.force | self.spin | self.time | names(
            "dump_revolutions dump_rpm drum_rpm drum_torque_service_factor actuator_idle_power")
        self.DEPENDENCIES = motor_dependencies(self.DEPENDENCIES, changed, motor, "drum", mechanical)
        self.OUTPUT_UNITS = dict(self.OUTPUT_UNITS)
        super().setup()

    def granular_response(self, inputs, froude):
        """Constant reference retention/lift; measured data may replace this hook."""
        v = lambda n: scalar(inputs, n)
        return v("capture_fraction"), v("internal_lift_factor")

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        radius, width = v("drum_diameter")/2, 4*v("drum_length")
        h = max(radius-v("extended_y"), 0.0)
        omega, dump_omega = v("drum_rpm")*math.pi/30, v("dump_rpm")*math.pi/30
        fr = omega**2*radius/v("gravity")
        capture, lift = self.granular_response(inputs, fr)
        _, dump_lift = self.granular_response(inputs, dump_omega**2*radius/v("gravity"))
        fresh = width*segment_area(radius, h)
        inner = max(radius-v("cut_depth"), 1e-4)
        capacity = 4*math.pi*inner**2*v("drum_length")*v("usable_volume_fraction")*v("fill_fraction")
        target_mass = capacity*v("stored_soil_density")
        reachable_mass = fresh*v("soil_density")*capture
        mass = min(target_mass, reachable_mass)
        # Rotate while lowering, dwell only if the scoops need more time, then
        # raise. Repeated revolutions do not add soil outside the endpoint cut.
        lowering_time = v("lowering_time")
        scoop_rate = width*v("cut_depth")*v("scoop_opening")*v("scoops_per_revolution")*v("drum_rpm")/60
        cutting_time = max(fresh/max(scoop_rate,1e-9),lowering_time)
        dig_time = cutting_time+v("drum_spinup_time")
        dump_time = 2*(v("dump_revolutions")*60/v("dump_rpm")+v("drum_spinup_time"))
        # Maximum bite is constrained by both scoop size and actual soil engagement.
        force = v("cut_pressure")*width*min(v("cut_depth"),h)*v("cut_engagement")
        internal = mass*v("gravity")*radius*lift
        dump_internal = mass*v("gravity")*radius*dump_lift
        # Conservative thin-ring spin inertia for shells plus payload. Spinup from
        # rest before cutting; no regenerative braking credit. One dump pair at a time.
        inertia = (v("drum_mass")+mass)*radius**2
        spin_torque = inertia*omega/v("drum_spinup_time")
        dump_spin = inertia/2*dump_omega/v("drum_spinup_time")
        cutting_torque = force*radius+internal
        peak_torque = max(cutting_torque, spin_torque)
        dumping_torque = max(dump_internal/2, dump_spin)
        dig_peak = peak_torque*omega
        dump_peak = dumping_torque*dump_omega
        dig_energy = (force*radius+0.5*internal)*omega*cutting_time + 0.5*inertia*omega**2
        dump_energy = 0.5*dump_internal*dump_omega*(dump_time-2*v("drum_spinup_time"))/2 + 0.5*inertia*dump_omega**2
        # Work/peak are at the output shafts. Both pairs dig together;
        # each pair dumps separately. Combined efficiency is applied once.
        motor = self.options["motor_model"].create(v, "drum")
        dig_electrical = motor.evaluate(dig_energy/2, dig_peak/2)
        dump_electrical = motor.evaluate(dump_energy/2, dump_peak)
        idle = v("actuator_idle_power")
        result = {
            "payload_mass": mass, "payload_volume": mass/v("stored_soil_density"),
            "reachable_soil_mass": reachable_mass, "target_payload_mass": target_mass,
            "fresh_volume": fresh, "storage_capacity": capacity, "trench_depth": h,
            # Reserve a rectangle for both stations.
            "excavation_footprint": (2*abs(v("extended_x"))+2*radius)*(2*v("drum_length")+v("drum_pair_gap")),
            "cut_force": force, "drum_froude": fr,
            "dig_time": dig_time, "dig_overlap_time": lowering_time, "dump_time": dump_time,
            "dig_power": 2*dig_electrical["peak"]+2*idle,
            "dump_power": dump_electrical["peak"]+idle,
            "dig_energy": 2*dig_electrical["energy"]+2*idle*dig_time,
            "dump_energy": 2*dump_electrical["energy"]+idle*dump_time,
            "drum_power_margin": 2*motor.peak_power()-dig_peak*v("drum_torque_service_factor"),
            "drum_torque_margin": 2*motor.available_torque(v("drum_rpm"))-peak_torque*v("drum_torque_service_factor"),
            "dump_torque_margin": motor.available_torque(v("dump_rpm"))-dumping_torque*v("drum_torque_service_factor"),
            "dig_froude_margin": v("max_dig_froude")-fr,
            "dump_froude_margin": v("max_dump_froude")-dump_omega**2*radius/v("gravity"),
            "penetration_margin": v("dry_mass")*v("gravity")-force*v("vertical_cut_fraction"),
            "dig_traction_margin": v("traction_coefficient")*(v("dry_mass")*v("gravity")-force*v("vertical_cut_fraction"))-force*v("force_imbalance"),
            "cut_depth_margin": min(v("scoop_opening")*v("scoop_depth_fraction"),radius*0.25)-v("cut_depth"),
            "dig_reach_margin": h-v("minimum_dig_depth"),
            "payload_margin": mass-v("minimum_payload"),
        }
        for name,value in result.items():
            outputs[name] = value
