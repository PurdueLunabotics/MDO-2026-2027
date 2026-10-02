"""Small, explicit OpenMDAO adapter for independently replaceable disciplines.

CONFIG_INPUTS come from config; COUPLED_INPUTS come from other components.
Every output has SI units and an explicit structural dependency set, including
inactive branches. No simulator is launched during an optimizer evaluation.
"""
from .components import SimpleComponent


class Discipline(SimpleComponent):
    CONFIG_INPUTS = ()
    COUPLED_INPUTS = {}
    OUTPUT_UNITS = {}
    DEPENDENCIES = {}

    def setup(self):
        self.inputs_from_config(*self.CONFIG_INPUTS)
        for name, units in self.COUPLED_INPUTS.items():
            self.add_input(name, val=0.0, units=units)
        for name, units in self.OUTPUT_UNITS.items():
            self.add_output(name, val=0.0, units=units)

    def setup_partials(self):
        for output, inputs in self.DEPENDENCIES.items():
            if inputs:
                self.declare_partials(output, sorted(inputs), method="fd", form="central", step=1e-5)


def names(value):
    return set(value.split())
