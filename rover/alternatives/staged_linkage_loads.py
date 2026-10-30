"""Conservative sequential arm-motion allowance for tandem moving excavation.

The reference load/work calculation retains the complete travel-to-maximum-
extension capability sweep, bounding the shallower operating moves. Front/rear
arms position in separate stages so the rear enters the front's cut and the front
can leave when full. Summing both arms' phase durations doubles the parallel
duration, without changing their already-total work or electrical energy.
Peak power retains the conservative two-actuator simultaneous envelope.
"""
from ..linkage import LinkageLoads
from ..dependencies import dependencies
from .pa04_actuator import PA04LinkageLoads, ExtendedPA04LinkageLoads
from .extended_attachment import ExtendedLinkageLoads


class StagedMotion:
    def compute(self, inputs, outputs):
        super().compute(inputs, outputs)
        outputs["actuation_time"] *= 2


class StagedLinkageLoads(StagedMotion, LinkageLoads):
    DEPENDENCIES = dependencies("LinkageLoads")


class StagedPA04LinkageLoads(StagedMotion, PA04LinkageLoads):
    pass


class StagedExtendedLinkageLoads(StagedMotion, ExtendedLinkageLoads):
    pass


class StagedExtendedPA04LinkageLoads(StagedMotion, ExtendedPA04LinkageLoads):
    pass


def staged_load_factory(factory):
    """Adapt known providers; preserve explicitly supplied student components."""
    return {LinkageLoads: StagedLinkageLoads,
            PA04LinkageLoads: StagedPA04LinkageLoads,
            ExtendedLinkageLoads: StagedExtendedLinkageLoads,
            ExtendedPA04LinkageLoads: StagedExtendedPA04LinkageLoads}.get(factory, factory)
