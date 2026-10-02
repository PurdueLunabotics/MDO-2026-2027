"""Provisional trench traversal screen; replace with validated terrain data."""
from .components import scalar
from .discipline import Discipline


class TrenchTraversal(Discipline):
    CONFIG_INPUTS = ("wheel_diameter",)
    COUPLED_INPUTS = {"trench_depth": "m"}
    OUTPUT_UNITS = {"trench_traverse_margin": "m"}
    DEPENDENCIES = {"trench_traverse_margin": {"wheel_diameter", "trench_depth"}}

    def compute(self, inputs, outputs):
        outputs["trench_traverse_margin"] = (scalar(inputs, "wheel_diameter")
                                              - scalar(inputs, "trench_depth"))
