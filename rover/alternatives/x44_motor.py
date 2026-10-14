"""Standard X44 motor performance only. No robot loads or OpenMDAO components.

The official 12 V torque-speed envelope is separate from provisional constant-
efficiency electrical accounting in rover/motors.py.
"""
import math
import hashlib
import json
from dataclasses import dataclass, replace
from pathlib import Path
from ..motors import Gearmotor, checked


@dataclass(frozen=True)
class X44Curve:
    voltage: float = 12.0
    free_rpm: float = 7757.6047386285445
    stall_torque: float = 4.113357107
    stall_current: float = 279.0986925
    free_current: float = 3.156052589044805
    stator_current_limit: float = 0.0

    @property
    def kt(self):
        return self.stall_torque/(self.stall_current-self.free_current)

    def available_torque(self, rpm, current_limit=None):
        current_limit = self.stator_current_limit if current_limit is None else current_limit
        checked(rpm, current_limit)
        voltage_torque = self.stall_torque*(1-rpm/self.free_rpm)
        if current_limit > 0:
            return min(voltage_torque, self.kt*(current_limit-self.free_current))
        return voltage_torque  # negative beyond free speed: failed screen, not a prediction

    def peak_power(self, current_limit=None):
        current_limit = self.stator_current_limit if current_limit is None else current_limit
        checked(current_limit)
        torque = self.stall_torque/2
        if current_limit > 0:
            torque = max(0., min(torque, self.kt*(current_limit-self.free_current)))
        return torque*self.free_rpm*(1-torque/self.stall_torque)*math.pi/30

    def speed_limit(self, torque):
        checked(torque)
        speed = self.free_rpm*(1-torque/self.stall_torque)
        return speed if self.available_torque(0) >= torque else -abs(speed)


CURVE = X44Curve()
EVIDENCE = {"revision": 2, "manifest": "data/motors/manifest.json",
            "status": "manufacturer 12 V torque-speed curve; combined efficiencies provisional; thermal operation unverified",
            "continuous_operation": "unverified; thermal duty has not been established"}


class X44MotorProvider:
    """Declares config inputs and builds a shared Gearmotor for either role."""
    curve = CURVE
    model_evidence = EVIDENCE

    @staticmethod
    def input_names(role):
        return {role+"_gear_ratio", role+"_gear_efficiency", role+"_stator_current_limit", role+"_efficiency"}

    @classmethod
    def create(cls, get, role, *, rated_torque_limit=True):
        motor = replace(cls.curve, stator_current_limit=get(role+"_stator_current_limit"))
        return Gearmotor(motor, get(role+"_gear_ratio"), get(role+"_gear_efficiency"),
                         get(role+"_efficiency"))


def verify_source_data():
    """Explicit pre-run verification, never file IO in a motor evaluation."""
    folder = Path(__file__).resolve().parents[2]/"data/motors"
    manifest = json.loads((folder/"manifest.json").read_text(encoding="utf-8"))
    if hashlib.sha256((folder/manifest["file"]).read_bytes()).hexdigest() != manifest["sha256"]:
        raise ValueError("X44 curve differs from its versioned manifest; review coefficients and validation before using it.")
