"""Explicit model replacement seam. Factories return NEW components per Problem.

Example: build_problem(models=ModelSet(drum_mass=MyDrumMass,
                                      excavation=MyExcavation)).
Replacements must preserve required output names, units, meanings and margins.
Keep external EDEM/Chrono jobs offline; evaluate validated deterministic responses
inside OpenMDAO. See docs/MODEL_DEVELOPMENT.md and examples/empirical_excavation.py.
"""
from dataclasses import dataclass, replace
from typing import Callable
from .excavation import Excavation
from .structure import StructureGeometry, StructureLoads
from .mass_properties import MassProperties
from .relocation import SiteRelocation
from .trench import TrenchTraversal
from .tip_stability import TipStability
from .alternatives.obstacle_stability import ObstacleStability
from .alternatives.parallel_cylinder_clearance import ParallelCylinderClearance
from .alternatives.traversal_stability import TraversalStability
from .alternatives.bekker_wong_reece import BekkerWongReeceMobility
from .alternatives.slope_traversal import SlopeTraversal
from .wheels import WheelMass
from .drums import DrumMass
from .components import GeometryMass
from .linkage import LinkageGeometry, LinkageLoads


@dataclass(frozen=True)
class ModelSet:
    structure_geometry: Callable = StructureGeometry
    linkage: Callable = LinkageGeometry
    wheel_mass: Callable = WheelMass
    drum_mass: Callable = DrumMass
    geometry: Callable = GeometryMass
    excavation: Callable = Excavation
    linkage_loads: Callable = LinkageLoads
    mass_properties: Callable = MassProperties
    structure_loads: Callable = StructureLoads
    relocation: Callable = SiteRelocation
    trench_traversal: Callable = TrenchTraversal
    tip_stability: Callable = TipStability
    obstacle_stability: Callable = ObstacleStability
    obstacle_clearance: Callable = ParallelCylinderClearance
    traversal_stability: Callable = TraversalStability
    mobility: Callable = BekkerWongReeceMobility
    slope_traversal: Callable = SlopeTraversal

    def metadata(self):
        return {name: f"{getattr(factory, '__module__', type(factory).__module__)}.{getattr(factory, '__qualname__', type(factory).__qualname__)}"
                for name,factory in vars(self).items()}


def plate_frame_models():
    """Explicit coordinated profile; reference ModelSet() remains unchanged."""
    from .alternatives.plate_frame import PlateMassProperties, PlateObstacleStability
    from .alternatives.aluminum_box_arm import BoxArmPlateStructureGeometry, BoxArmPlateStructureLoads
    from .alternatives.traversal_stability import PlateTraversalStability
    return ModelSet(structure_geometry=BoxArmPlateStructureGeometry, structure_loads=BoxArmPlateStructureLoads,
                    mass_properties=PlateMassProperties, obstacle_stability=PlateObstacleStability,
                    traversal_stability=PlateTraversalStability)


def traversal_models(models, mode):
    """Select optional timing through factories without replacing student models."""
    if mode not in ("fixed", "calculated"):
        raise ValueError("traversal_mode must be fixed or calculated")
    selected = getattr(models.mobility, "TRAVERSAL_MODE", "fixed")
    if mode == selected:
        return models
    if mode == "calculated" and models.mobility is BekkerWongReeceMobility:
        from .alternatives.calculated_traversal import CalculatedTraversalMobility
        return replace(models, mobility=CalculatedTraversalMobility)
    raise ValueError("The selected mobility factory does not support this traversal mode; "
                     "select a compatible ModelSet explicitly. Student components were not replaced.")


def extended_attachment_models(models):
    """Coordinated box-arm extension factories; never override student models."""
    from .alternatives.aluminum_box_arm import BoxArmPlateStructureLoads
    from .alternatives.plate_frame import PlateMassProperties
    from .alternatives.extended_attachment import (
        ExtendedLinkageGeometry,ExtendedLinkageLoads,ExtendedPlateStructureLoads,ExtendedPlateMassProperties)
    expected={'linkage':LinkageGeometry,'linkage_loads':LinkageLoads,
              'structure_loads':BoxArmPlateStructureLoads,'mass_properties':PlateMassProperties}
    if any(getattr(models,n) is not cls for n,cls in expected.items()):
        raise ValueError('Attachment beyond the drum requires the coordinated plate/box-arm factories; student models were not replaced')
    return replace(models,linkage=ExtendedLinkageGeometry,linkage_loads=ExtendedLinkageLoads,
                   structure_loads=ExtendedPlateStructureLoads,mass_properties=ExtendedPlateMassProperties)
