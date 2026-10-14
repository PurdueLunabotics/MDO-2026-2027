"""Shared motor envelope, gearbox conversion and simple electrical accounting.

Disciplines own mechanical work [J], peak power [W] and torque/speed demands.
Electrical energy/power use one provisional combined motor+gear efficiency.
No electrical time histories, controller simulation or thermal predictions.
"""
import math
from dataclasses import dataclass


def checked(*values):
    if any(not math.isfinite(x) or x < 0 for x in values):
        raise ValueError("Motor and work arguments must be finite and nonnegative.")


@dataclass(frozen=True)
class ReferenceMotor:
    """Original generic torque-speed envelope; Gearmotor handles energy.

    Mobility historically used the full linear envelope; drums and relocation
    additionally capped torque at the rated value. rated_torque_limit preserves
    that distinction explicitly. This is not a measured motor or thermal model.
    """
    power: float
    rated_rpm: float
    rated_torque_limit: bool = True

    @property
    def free_rpm(self):
        return 2*self.rated_rpm

    @property
    def stall_torque(self):
        return 2*self.power/(self.rated_rpm*math.pi/30)

    def available_torque(self, rpm):
        linear = self.stall_torque*(1-rpm/self.free_rpm)
        return min(self.stall_torque/2, linear) if self.rated_torque_limit else linear

    def peak_power(self):
        return self.power

    def speed_limit(self, torque):
        return self.free_rpm*(1-torque/self.stall_torque)


@dataclass(frozen=True)
class Gearmotor:
    """One motor and reduction; all public demands refer to the OUTPUT shaft."""
    motor: object
    ratio: float
    efficiency: float  # mechanical gearbox efficiency, for torque capability
    electrical_efficiency: float  # combined motor+gear efficiency, for energy/power

    def __post_init__(self):
        if (not math.isfinite(self.ratio) or self.ratio <= 0
                or not 0 < self.efficiency <= 1 or not 0 < self.electrical_efficiency <= 1):
            raise ValueError("Positive gearing and efficiency in (0,1] are required.")

    @property
    def free_rpm(self):
        return self.motor.free_rpm/self.ratio

    def available_torque(self, rpm):
        checked(rpm)
        return self.motor.available_torque(rpm*self.ratio)*self.ratio*self.efficiency

    def peak_power(self):
        return self.motor.peak_power()*self.efficiency

    def speed_limit(self, torque):
        checked(torque)
        return self.motor.speed_limit(torque/(self.ratio*self.efficiency))/self.ratio

    def evaluate(self, work, peak):
        """Electrical J and peak W for ONE motor, from output-shaft J and W.

        The combined efficiency includes gear loss: do not divide by gear
        efficiency again. Auxiliary idle/electronics loads belong to callers.
        """
        checked(work, peak)
        return {"energy": work/self.electrical_efficiency,
                "peak": peak/self.electrical_efficiency}


class ReferenceMotorProvider:
    """OpenMDAO input declaration + construction, separate from motor equations.

    A replacement provider implements input_names(role), create(get, role,
    rated_torque_limit=...). get(name) reads a scalar
    config input. It returns a Gearmotor; it never replaces a discipline.
    """

    @staticmethod
    def input_names(role):
        return {role+"_motor_power", role+"_gear_ratio", role+"_gear_efficiency",
                role+"_efficiency", "motor_rated_rpm"}

    @staticmethod
    def create(get, role, *, rated_torque_limit=True):
        eta = get(role+"_gear_efficiency")
        motor = ReferenceMotor(get(role+"_motor_power"), get("motor_rated_rpm"), rated_torque_limit)
        return Gearmotor(motor, get(role+"_gear_ratio"), eta, get(role+"_efficiency"))


def motor_dependencies(mapping, outputs, provider, role, mechanical=()):
    """Select real possible dependencies, including both sides of load branches."""
    result = {name:set(inputs) for name,inputs in mapping.items()}
    old = ReferenceMotorProvider.input_names(role)
    for name in outputs:
        result[name] = (result[name] | set(mechanical))-old | provider.input_names(role)
    return result

