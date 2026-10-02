"""Move the empty rover to fresh ground between stationary excavation cycles.

This does not predict whether the rover can climb out of a trench. That screen
was removed pending measured terrain and drivetrain data.
"""
import math
from .components import scalar, travel_profile
from .discipline import Discipline, names
from .excavation import motor_torque


class SiteRelocation(Discipline):
    CONFIG_INPUTS = tuple(sorted(names("""
        wheel_diameter wheelbase track_width drive_motor_power motor_rated_rpm
        drive_gear_ratio drive_gear_efficiency gravity traction_coefficient
        fresh_site_distance site_speed drive_efficiency rolling_base
        actuator_idle_power drum_length drum_pair_gap scrub_coefficient
        acceleration turn_accel_fraction
    """)))
    COUPLED_INPUTS = {"dry_mass": "kg", "empty_yaw_inertia": "kg*m**2"}
    OUTPUT_UNITS = {
        "relocation_time": "s", "relocation_energy": "J", "relocation_peak_power": "W",
        "fresh_site_margin": "m", "site_drive_margin": "N", "site_turn_time": "s",
    }
    profile = names("fresh_site_distance site_speed track_width acceleration turn_accel_fraction")
    rolling = names("dry_mass gravity rolling_base scrub_coefficient wheelbase empty_yaw_inertia")
    drive = names("wheel_diameter drive_motor_power motor_rated_rpm drive_gear_ratio drive_gear_efficiency traction_coefficient")
    DEPENDENCIES = {
        "relocation_time": profile,
        "relocation_energy": profile | rolling | names("drive_efficiency actuator_idle_power"),
        "relocation_peak_power": profile | rolling | names("drive_efficiency actuator_idle_power"),
        "fresh_site_margin": names("fresh_site_distance drum_length drum_pair_gap"),
        "site_drive_margin": profile | rolling | drive,
        "site_turn_time": names("site_speed track_width acceleration turn_accel_fraction"),
    }

    def compute(self, inputs, outputs):
        v = lambda name: scalar(inputs, name)
        radius, half_track = v("wheel_diameter")/2, v("track_width")/2
        translation, site_peak = travel_profile(v("fresh_site_distance"), v("site_speed"), v("acceleration"))
        turn_time, turn_peak = travel_profile(math.pi*half_track/2, v("site_speed"),
                                              v("acceleration")*v("turn_accel_fraction"))
        duration = translation + 2*turn_time
        weight = v("dry_mass")*v("gravity")
        turn_drag = weight*(v("rolling_base") + v("scrub_coefficient")*v("wheelbase")/(4*half_track))
        turn_mass = v("empty_yaw_inertia")/half_track**2
        work = weight*v("rolling_base")*v("fresh_site_distance") + 0.5*v("dry_mass")*site_peak**2
        work += turn_drag*math.pi*half_track + turn_mass*turn_peak**2
        site_force = weight*v("rolling_base") + v("dry_mass")*v("acceleration")
        turn_force = turn_drag + turn_mass*v("acceleration")*v("turn_accel_fraction")
        margins = []
        for required, speed in ((site_force, site_peak), (turn_force, turn_peak)):
            rpm = speed/radius*30/math.pi
            available = 4*motor_torque(v("drive_motor_power"), v("motor_rated_rpm"),
                                       v("drive_gear_ratio"), v("drive_gear_efficiency"), rpm)/radius
            margins.append(min(available, weight*v("traction_coefficient")) - required)
        idle = 4*v("actuator_idle_power")
        outputs["relocation_time"] = duration
        outputs["relocation_energy"] = work/v("drive_efficiency") + idle*duration
        outputs["relocation_peak_power"] = max(site_force*site_peak, turn_force*turn_peak)/v("drive_efficiency") + idle
        outputs["fresh_site_margin"] = v("fresh_site_distance") - (2*v("drum_length") + v("drum_pair_gap"))
        outputs["site_drive_margin"] = min(margins)
        outputs["site_turn_time"] = 2*turn_time
