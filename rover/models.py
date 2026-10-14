"""Explicit model replacement seam. Factories return NEW components per Problem.

Example: build_problem(models=ModelSet(drum_mass=MyDrumMass,
                                      excavation=MyExcavation)).
Replacements must preserve required output names, units, meanings and margins.
Keep external EDEM/Chrono jobs offline; evaluate validated deterministic responses
inside OpenMDAO. See docs/MODEL_DEVELOPMENT.md and examples/empirical_excavation.py.
"""
from dataclasses import dataclass
from typing import Callable
from .excavation import Excavation
from .structure import StructureGeometry, StructureLoads
from .mass_properties import MassProperties
from .relocation import SiteRelocation
from .trench import TrenchTraversal
from .tip_stability import TipStability
from .alternatives.obstacle_stability import ObstacleStability
from .alternatives.obstacle_clearance import ObstacleClearance
from .alternatives.traversal_stability import TraversalStability
from .mobility import Mobility
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
    obstacle_clearance: Callable = ObstacleClearance
    traversal_stability: Callable = TraversalStability
    mobility: Callable = Mobility

    def metadata(self):
        return {name: f"{getattr(factory, '__module__', type(factory).__module__)}.{getattr(factory, '__qualname__', type(factory).__qualname__)}"
                for name,factory in vars(self).items()}


def plate_frame_models():
    """Explicit coordinated profile; reference ModelSet() remains unchanged."""
    from .alternatives.plate_frame import PlateStructureGeometry, PlateStructureLoads, PlateMassProperties, PlateObstacleStability
    from .alternatives.traversal_stability import PlateTraversalStability
    return ModelSet(structure_geometry=PlateStructureGeometry, structure_loads=PlateStructureLoads,
                    mass_properties=PlateMassProperties, obstacle_stability=PlateObstacleStability,
                    traversal_stability=PlateTraversalStability)
