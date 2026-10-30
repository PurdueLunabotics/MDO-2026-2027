"""Loaded travel-to-dump motion and empty return, two parallel actuators.

Reference work/loss assumptions are retained; PA-04 uses chart electrical
accounting. Gravity-assisted operation and dynamic motion remain unvalidated.
"""
from ..components import scalar
from ..discipline import Discipline, names
from .offset_attachment import physical_length


def deposition_motion(values, profile="reference"):
    """Total added duration [s], electrical energy [J], and peak power [W]."""
    v = values.__getitem__
    stroke = abs(v("deposition_actuator_length")-v("travel_actuator_length"))
    if stroke == 0:
        return dict(deposition_actuation_time=0., deposition_arm_energy=0.,
                    deposition_arm_peak_power=0.)
    if profile == "pa04-900":
        from .pa04_actuator import cycle_performance, RATED_FORCE, INCH
        # Existing load/curve margins retain the original invalid force.
        # Guarded values are finite rejected-trial diagnostics only.
        force = min(max(v("required_actuator_force"), 0.), RATED_FORCE)
        result = cycle_performance(force, stroke, stroke/(.39*INCH))
        return {"deposition_"+name: result[name] for name in
                ("actuation_time", "arm_energy", "arm_peak_power")}
    if profile != "reference":
        raise ValueError("Unknown deposition actuator profile")
    empty = (v("drum_mass")/2 + v("arm_linear_mass")*
             physical_length(v("arm_length"), v("lin_act_side_arm_offset"))**2 /
             (2*v("arm_length")))*v("gravity")
    loaded = empty+v("payload_mass")/2*v("gravity")
    duration = 2*stroke/v("actuator_speed")
    height = abs(v("deposition_y")-v("travel_y"))
    energy = 2*(loaded+empty)*height/v("actuator_efficiency")
    energy += 2*v("actuator_idle_power")*duration
    peak = 2*v("required_actuator_force")*v("actuator_speed")/v("actuator_efficiency")
    peak += 2*v("actuator_idle_power")
    return dict(deposition_actuation_time=duration, deposition_arm_energy=energy,
                deposition_arm_peak_power=peak)


class DepositionActuation(Discipline):
    OUTPUT_UNITS = {"deposition_actuation_time": "s", "deposition_arm_energy": "J",
                    "deposition_arm_peak_power": "W"}

    def initialize(self):
        self.options.declare("actuator_profile", default="reference",
                             values=("reference", "pa04-900"))

    def setup(self):
        lengths = names("travel_actuator_length deposition_actuator_length")
        if self.options["actuator_profile"] == "pa04-900":
            self.CONFIG_INPUTS = tuple(sorted(lengths))
            self.COUPLED_INPUTS = {"required_actuator_force": "N"}
            self.DEPENDENCIES = {n: lengths | {"required_actuator_force"}
                                 for n in self.OUTPUT_UNITS}
        else:
            self.CONFIG_INPUTS = tuple(sorted(lengths | names(
                "gravity arm_length lin_act_side_arm_offset actuator_speed actuator_efficiency actuator_idle_power")))
            self.COUPLED_INPUTS = {"travel_y": "m", "deposition_y": "m",
                "drum_mass": "kg", "payload_mass": "kg", "arm_linear_mass": "kg/m",
                "required_actuator_force": "N"}
            self.DEPENDENCIES = {
                "deposition_actuation_time": lengths | {"actuator_speed"},
                "deposition_arm_energy": set(self.CONFIG_INPUTS) | set(self.COUPLED_INPUTS)-{"required_actuator_force"},
                "deposition_arm_peak_power": lengths | names("required_actuator_force actuator_speed actuator_efficiency actuator_idle_power")}
        super().setup()

    def compute(self, inputs, outputs):
        for name, value in deposition_motion({n: scalar(inputs, n) for n in inputs},
                                             self.options["actuator_profile"]).items():
            outputs[name] = value
