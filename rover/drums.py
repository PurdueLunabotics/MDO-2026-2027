"""Reference drum assembly mass. Excavation physics lives separately."""
import math
from .components import scalar
from .discipline import Discipline, names


def reference_drum_mass(diameter, length, areal_mass, fixed_mass_per_pair,
                        motor_mass_per_watt, motor_power_per_pair,
                        gear_mass_per_torque, motor_rated_rpm, gear_ratio,
                        gear_efficiency):
    """Four shells plus two motor/gear assemblies [kg]; uncalibrated."""
    shell = 4 * areal_mass * (math.pi * diameter * length + math.pi * diameter**2 / 2)
    motor_omega = motor_rated_rpm * math.pi / 30
    output_torque = motor_power_per_pair / motor_omega * gear_ratio * gear_efficiency
    return shell + 2 * (fixed_mass_per_pair + motor_mass_per_watt * motor_power_per_pair
                        + gear_mass_per_torque * output_torque)


class DrumMass(Discipline):
    CONFIG_INPUTS = tuple(sorted(names(
        "drum_diameter drum_length drum_areal_mass drum_fixed_mass "
        "motor_mass_per_watt drum_motor_power gear_mass_per_torque "
        "motor_rated_rpm drum_gear_ratio drum_gear_efficiency")))
    OUTPUT_UNITS = {"drum_mass": "kg"}
    DEPENDENCIES = {"drum_mass": set(CONFIG_INPUTS)}

    def compute(self, inputs, outputs):
        outputs["drum_mass"] = reference_drum_mass(
            *(scalar(inputs, name) for name in (
                "drum_diameter", "drum_length", "drum_areal_mass", "drum_fixed_mass",
                "motor_mass_per_watt", "drum_motor_power", "gear_mass_per_torque",
                "motor_rated_rpm", "drum_gear_ratio", "drum_gear_efficiency")))
