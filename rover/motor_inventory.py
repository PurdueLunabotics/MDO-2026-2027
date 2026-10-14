"""Motor/gear assembly mass only; independent of motion and electrical losses."""
import math


def reference_assembly_mass(fixed, motor_mass_per_watt, power, gear_mass_per_torque,
                            rated_rpm, ratio, efficiency):
    torque = power/(rated_rpm*math.pi/30)*ratio*efficiency
    return fixed+motor_mass_per_watt*power+gear_mass_per_torque*torque


class ReferenceAssemblyMass:
    @staticmethod
    def input_names(role):
        return {role+"_fixed_mass", role+"_motor_power", role+"_gear_ratio",
                role+"_gear_efficiency", "motor_mass_per_watt", "gear_mass_per_torque", "motor_rated_rpm"}

    @staticmethod
    def per_motor(get, role):
        return reference_assembly_mass(get(role+"_fixed_mass"), get("motor_mass_per_watt"),
            get(role+"_motor_power"), get("gear_mass_per_torque"), get("motor_rated_rpm"),
            get(role+"_gear_ratio"), get(role+"_gear_efficiency"))


class CatalogAssemblyMass:
    """Selected X44 inventory values live in config, not a mass-vs-power fit."""
    @staticmethod
    def input_names(role):
        return {"x44_motor_mass", role+"_gearbox_mass"}

    @staticmethod
    def per_motor(get, role):
        return get("x44_motor_mass")+get(role+"_gearbox_mass")
