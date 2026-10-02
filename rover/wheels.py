"""Reference wheel mass. Replace this component for a CAD/BOM-based wheel."""
import math
from .components import scalar
from .discipline import Discipline, names


def reference_wheel_mass(diameter, width, areal_mass):
    """Total mass of four closed cylindrical wheels [kg]; uncalibrated."""
    return 4 * areal_mass * (math.pi * diameter * width + math.pi * diameter**2 / 2)


class WheelMass(Discipline):
    CONFIG_INPUTS = tuple(sorted(names("wheel_diameter wheel_width wheel_areal_mass")))
    OUTPUT_UNITS = {"wheel_mass": "kg"}
    DEPENDENCIES = {"wheel_mass": set(CONFIG_INPUTS)}

    def compute(self, inputs, outputs):
        outputs["wheel_mass"] = reference_wheel_mass(
            *(scalar(inputs, name) for name in
              ("wheel_diameter", "wheel_width", "wheel_areal_mass")))
