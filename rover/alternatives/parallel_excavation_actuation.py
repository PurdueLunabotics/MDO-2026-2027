"""Concurrent front/rear excavation positioning at the actual cutting poses.

Capacity/load checks retain the full mechanism sweep. Motion time is the slower
arm in each direction; electrical work sums both arms without regeneration.
"""
import math

from ..components import scalar
from ..dependencies import dependencies
from ..discipline import names
from ..linkage import LinkageLoads
from .extended_attachment import ExtendedLinkageLoads
from .offset_attachment import physical_length
from .pa04_actuator import (
    PA04LinkageLoads, ExtendedPA04LinkageLoads, cycle_performance, RATED_FORCE, INCH,
)


COMMANDS = names("travel_actuator_length front_dig_actuator_length rear_dig_actuator_length")
MOTION_OUTPUT_UNITS = {"excavation_lowering_time": "s", "excavation_raising_time": "s"}


def excavation_motion(values, profile="reference"):
    """Parallel elapsed seconds and summed electrical joules for two arms.

    Invalid operating poses retain their upstream signed failures. Absolute
    command distances provide finite motion diagnostics for those trials.
    PA-04 force guards retain the parent's failed force/curve margins.
    """
    v = values.__getitem__
    if profile not in ("reference", "pa04-900"):
        raise ValueError("Unknown excavation actuator profile")
    commands = [v(n) for n in ("travel_actuator_length", "front_dig_actuator_length", "rear_dig_actuator_length")]
    if not all(math.isfinite(x) and x >= 0 for x in commands) or commands[0] <= 0:
        raise ValueError("Excavation actuator commands must be finite and nonnegative, with positive travel length")
    distances = [abs(x-commands[0]) for x in commands[1:]]
    if max(distances) == 0:
        return dict(actuation_time=0., arm_energy=0., lowering_duration=0., raising_duration=0.)
    if profile == "pa04-900":
        force = v("required_actuator_force")
        if not math.isfinite(force):
            raise ValueError("Excavation actuator force must be finite")
        results = [cycle_performance(min(max(force, 0.), RATED_FORCE), d, d/(.39*INCH))
                   for d in distances]
        lowering = max(distances)/(.39*INCH)
        raising = max(r["actuator_raising_time"] for r in results)
        # cycle_performance returns energy for two equal parallel actuators;
        # each result here describes one of two unequal actual arm movements.
        energy = sum(r["arm_energy"]/2 for r in results)
    else:
        speed, efficiency = v("actuator_speed"), v("actuator_efficiency")
        if not math.isfinite(speed) or speed <= 0 or not math.isfinite(efficiency) or not 0 < efficiency <= 1:
            raise ValueError("Excavation motion needs positive speed and efficiency in (0,1]")
        nonnegative = names("drum_mass arm_linear_mass payload_mass gravity actuator_idle_power")
        if (any(not math.isfinite(v(n)) or v(n) < 0 for n in nonnegative)
                or not math.isfinite(v("arm_length")) or v("arm_length") <= 0
                or not math.isfinite(v("lin_act_side_arm_offset"))
                or v("arm_length")+v("lin_act_side_arm_offset") <= 0):
            raise ValueError("Excavation work needs finite nonnegative loads and a positive arm length")
        heights = [abs(v("travel_y")-v(n)) for n in ("front_dig_y", "rear_dig_y")]
        if not all(math.isfinite(x) for x in heights):
            raise ValueError("Excavation heights must be finite")
        empty = (v("drum_mass")/2 + v("arm_linear_mass")*
                 physical_length(v("arm_length"), v("lin_act_side_arm_offset"))**2 /
                 (2*v("arm_length")))*v("gravity")
        loaded = empty+v("payload_mass")*v("gravity")/2
        active_heights = sum(h for h, d in zip(heights, distances) if d > 0)
        energy = ((empty+loaded)*active_heights)/efficiency
        energy += v("actuator_idle_power")*2*sum(distances)/speed
        lowering = raising = max(distances)/speed
    return dict(actuation_time=lowering+raising, arm_energy=energy,
                lowering_duration=lowering, raising_duration=raising)


class ParallelExcavationMotion:
    PARALLEL_EXCAVATION_MOTION = True
    ACTUATOR_PROFILE = "reference"

    def setup(self):
        super().setup()
        self.inputs_from_config("travel_actuator_length")
        for name in ("front_dig_actuator_length", "rear_dig_actuator_length"):
            self.add_input(name, units="m")
        mapping = getattr(self, "DEPENDENCIES", None) or dependencies("LinkageLoads")
        self.DEPENDENCIES = {n: set(deps) for n, deps in mapping.items()}
        if self.ACTUATOR_PROFILE == "pa04-900":
            for name in ("actuation_time", "arm_energy"):
                self.DEPENDENCIES[name] = COMMANDS | self.force_inputs
        else:
            for name in ("front_dig_y", "rear_dig_y"):
                self.add_input(name, units="m")
            self.DEPENDENCIES["actuation_time"] = COMMANDS | {"actuator_speed"}
            self.DEPENDENCIES["arm_energy"] = COMMANDS | names(
                "actuator_speed actuator_efficiency actuator_idle_power drum_mass payload_mass "
                "arm_linear_mass arm_length lin_act_side_arm_offset gravity travel_y front_dig_y "
                "rear_dig_y")
        self.DEPENDENCIES["excavation_lowering_time"] = (COMMANDS if self.ACTUATOR_PROFILE == "pa04-900"
                                                       else COMMANDS | {"actuator_speed"})
        self.DEPENDENCIES["excavation_raising_time"] = set(self.DEPENDENCIES["actuation_time"])

    def compute(self, inputs, outputs):
        super().compute(inputs, outputs)
        values = {name: scalar(inputs, name) for name in inputs}
        force = outputs["required_actuator_force"]
        values["required_actuator_force"] = force.item() if hasattr(force, "item") else force
        result = excavation_motion(values, self.ACTUATOR_PROFILE)
        outputs["actuation_time"] = result["actuation_time"]
        outputs["arm_energy"] = result["arm_energy"]
        outputs["excavation_lowering_time"] = result["lowering_duration"]
        outputs["excavation_raising_time"] = result["raising_duration"]


class ParallelLinkageLoads(ParallelExcavationMotion, LinkageLoads):
    OUTPUT_UNITS = {**LinkageLoads.OUTPUT_UNITS, **MOTION_OUTPUT_UNITS}


class ParallelPA04LinkageLoads(ParallelExcavationMotion, PA04LinkageLoads):
    ACTUATOR_PROFILE = "pa04-900"
    OUTPUT_UNITS = {**PA04LinkageLoads.OUTPUT_UNITS, **MOTION_OUTPUT_UNITS}


class ParallelExtendedLinkageLoads(ParallelExcavationMotion, ExtendedLinkageLoads):
    OUTPUT_UNITS = {**ExtendedLinkageLoads.OUTPUT_UNITS, **MOTION_OUTPUT_UNITS}


class ParallelExtendedPA04LinkageLoads(ParallelExcavationMotion, ExtendedPA04LinkageLoads):
    ACTUATOR_PROFILE = "pa04-900"
    OUTPUT_UNITS = {**ExtendedPA04LinkageLoads.OUTPUT_UNITS, **MOTION_OUTPUT_UNITS}


def parallel_load_factory(factory):
    """Adapt known load factories; preserve explicit custom replacements."""
    return {LinkageLoads: ParallelLinkageLoads,
            PA04LinkageLoads: ParallelPA04LinkageLoads,
            ExtendedLinkageLoads: ParallelExtendedLinkageLoads,
            ExtendedPA04LinkageLoads: ParallelExtendedPA04LinkageLoads}.get(factory, factory)
