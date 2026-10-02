"""Fixed-time route with physically checked, time-scaled motion profiles."""
import math
from .components import SimpleComponent, scalar, travel_profile
from .config import ROUTE


def route_profiles(track, speed, accel, turn_fraction, turn_accel_fraction, one_way_time):
    """(kind, distance at wheel rim, duration, peak rim speed, acceleration).

    First construct rest-to-rest segments. Scale all durations by the SAME factor,
    velocities inversely and accelerations inversely squared, preserving distance.
    Hardware must satisfy these profiles; changing motors cannot change the 45 s.
    """
    profiles = []
    for distance, degrees in ROUTE:
        if distance:
            t, peak = travel_profile(distance, speed, accel)
            profiles.append(("straight", distance, t, peak, accel))
        if degrees:
            d = abs(math.radians(degrees)) * track / 2
            a = accel * turn_accel_fraction
            t, peak = travel_profile(d, speed * turn_fraction, a)
            profiles.append(("turn", d, t, peak, a))
    scale = one_way_time / sum(p[2] for p in profiles)
    return [(kind, d, t * scale, peak / scale, a / scale**2) for kind, d, t, peak, a in profiles]


class Mobility(SimpleComponent):
    OUTPUT_UNITS = {
            "travel_time": "s", "turn_time": "s", "mobility_energy": "J",
            "drive_peak_power": "W", "turn_peak_power": "W", "ground_pressure": "Pa",
            "ground_pressure_margin": "Pa", "straight_motor_margin": "N",
            "turn_motor_margin": "N", "straight_traction_margin": "N",
            "turn_traction_margin": "N", "straight_power_margin": "W",
            "turn_power_margin": "W", "actual_travel_speed": "m/s",
            "no_load_speed": "m/s", "loaded_motor_speed_limit": "m/s",
            "loaded_power_speed_limit": "m/s", "stall_drive_force": "N",
            "route_distance": "m", "route_turn_angle": "deg", "route_translation_time": "s",
            "route_turn_time": "s", "route_peak_acceleration": "m/s**2",
            "mobility_stability_margin": "m",
    }

    def setup(self):
        self.inputs_from_config(
            "wheel_diameter", "wheel_width", "drive_motor_power", "drive_gear_ratio",
            "travel_speed", "acceleration", "gravity", "traversal_time_one_way", "turn_angle",
            "turn_speed_fraction", "turn_accel_fraction", "wheelbase", "track_width",
            "motor_rated_rpm", "drive_gear_efficiency", "drive_efficiency",
            "traction_coefficient", "rocker_load_factor", "wheel_deflection_fraction",
            "rolling_base", "rolling_pressure_scale", "allowable_ground_pressure",
            "scrub_coefficient", "actuator_idle_power")
        self.add_input("dry_mass", units="kg")
        self.add_input("payload_mass", units="kg")
        for state in ("empty", "loaded", "half"):
            self.add_input(state+"_yaw_inertia", units="kg*m**2")
            self.add_input(state+"_cg_x", units="m")
            self.add_input(state+"_cg_z", units="m")
        for name, units in self.OUTPUT_UNITS.items():
            self.add_output(name, units=units)

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        radius = v("wheel_diameter") / 2
        delta = v("wheel_deflection_fraction") * 2 * radius
        contact_length = 2 * math.sqrt(2 * radius * delta - delta**2)
        area = 4 * v("wheel_width") * contact_length
        dry, payload, g = v("dry_mass"), v("payload_mass"), v("gravity")

        def resistance(mass):
            pressure = v("rocker_load_factor") * mass * g / area
            return (v("rolling_base") + pressure / v("rolling_pressure_scale")) * mass * g

        profiles = route_profiles(v("track_width"), v("travel_speed"), v("acceleration"),
                                  v("turn_speed_fraction"), v("turn_accel_fraction"), v("traversal_time_one_way"))
        half_track = v("track_width") / 2
        turn_path = v("turn_angle") * half_track
        turn_accel = v("acceleration") * v("turn_accel_fraction")
        turn_time, turn_peak = travel_profile(turn_path, v("travel_speed") * v("turn_speed_fraction"), turn_accel)
        rated_omega = v("motor_rated_rpm") * math.pi / 30
        ratio, eta = v("drive_gear_ratio"), v("drive_gear_efficiency")
        stall_force = 4 * 2 * v("drive_motor_power") / rated_omega * ratio * eta / radius
        no_load_speed = 2 * rated_omega * radius / ratio
        rating = 4 * v("drive_motor_power") * eta
        segments = [(dry + payload, "loaded", p) for p in profiles]
        segments += [(dry, "empty", p) for p in reversed(profiles)]
        segments += [(dry + payload / 2, "half", ("turn", turn_path, turn_time, turn_peak, turn_accel))]
        work = 0
        forces, powers, motor_margins, traction_margins = {}, {}, {}, {}
        for kind in ("straight", "turn"):
            forces[kind], powers[kind], motor_margins[kind], traction_margins[kind] = [], [], [], []
        stability = []
        pressure_factors = []
        for mass, state, (kind, d, t, peak, accel) in segments:
            drag = resistance(mass)
            effective_mass = mass
            cg_x, cg_z = abs(v(state+"_cg_x")), v(state+"_cg_z")
            offset_force = 0.0
            if kind == "turn":
                inertia = v(state+"_yaw_inertia")
                effective_mass = inertia / half_track**2
                drag += v("scrub_coefficient") * mass * g * v("wheelbase") / (4 * half_track)
                # CG offset requires both tangential and centripetal net force.
                offset_accel = cg_x * math.hypot(accel/half_track, (peak/half_track)**2)
                offset_force = mass*offset_accel
                stability.append(half_track-cg_z*offset_accel/g)
            else:
                stability.append(v("wheelbase")/2-cg_x-cg_z*accel/g)
            load_factor = 1+2*(cg_x+cg_z*(accel if kind == "straight" else 0)/g)/v("wheelbase")
            pressure_factors.append((mass,load_factor))
            force = drag + effective_mass * accel
            power = force * peak
            work += drag * d + 0.5 * effective_mass * peak**2
            forces[kind].append(force)
            powers[kind].append(power)
            motor_margins[kind].append(stall_force * (1 - peak / no_load_speed) - force)
            # Equal wheel torque is limited by the lighter axle; a torque allocator
            # can replace this conservative screen when tested.
            light_axle_factor = max(0,2-load_factor)
            traction_margins[kind].append(v("traction_coefficient") * mass * g * light_axle_factor - math.hypot(force,offset_force))
        idle = 4 * v("actuator_idle_power")
        outputs["travel_time"] = 2 * v("traversal_time_one_way")
        outputs["turn_time"] = turn_time  # deposition-only; route turns already in 90 s
        outputs["route_translation_time"] = 2 * sum(p[2] for p in profiles if p[0] == "straight")
        outputs["route_turn_time"] = 2 * sum(p[2] for p in profiles if p[0] == "turn")
        outputs["route_distance"] = 2 * sum(p[0] for p in ROUTE)
        outputs["route_turn_angle"] = 2 * sum(abs(p[1]) for p in ROUTE)
        outputs["actual_travel_speed"] = max(p[3] for p in profiles if p[0] == "straight")
        outputs["route_peak_acceleration"] = max(p[4] for p in profiles)
        outputs["mobility_energy"] = work / v("drive_efficiency") + idle * (2 * v("traversal_time_one_way") + turn_time)
        outputs["drive_peak_power"] = max(powers["straight"]) / v("drive_efficiency") + idle
        outputs["turn_peak_power"] = max(powers["turn"]) / v("drive_efficiency") + idle
        outputs["ground_pressure"] = v("rocker_load_factor") * max(m*f for m,f in pressure_factors) * g / area
        outputs["mobility_stability_margin"] = min(stability)
        outputs["ground_pressure_margin"] = v("allowable_ground_pressure") - outputs["ground_pressure"]
        outputs["no_load_speed"], outputs["stall_drive_force"] = no_load_speed, stall_force
        outputs["loaded_motor_speed_limit"] = no_load_speed * (1 - max(forces["straight"]) / stall_force)
        outputs["loaded_power_speed_limit"] = rating / max(forces["straight"])
        for kind in ("straight", "turn"):
            outputs[kind + "_motor_margin"] = min(motor_margins[kind])
            outputs[kind + "_traction_margin"] = min(traction_margins[kind])
            outputs[kind + "_power_margin"] = rating - max(powers[kind])
