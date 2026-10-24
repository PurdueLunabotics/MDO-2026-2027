"""Aluminum box drum arms for the coordinated plate-frame ModelSet.

Width is transverse to the vehicle; height is the section depth normal to the
arm centerline in its x/z motion plane. Sharp corners, uniform wall, elastic
beam/Euler screens only; no local wall, weld, joint or torsional validation.
Material inputs are the existing aluminum tube_density/modulus/strength.
"""
from .plate_frame import (
    box_section, PlateStructureGeometry, PlateStructureLoads, EVIDENCE,
)


def box_arm_section(values):
    width, height, wall = (values[n] for n in ('arm_box_width', 'arm_box_height', 'arm_wall'))
    area, bending, lateral = box_section(width, height, wall)
    return dict(area=area, bending_inertia=bending,
                buckling_inertia=min(bending, lateral), depth=height,
                section_margin=min(width, height)/4-wall)


ARM_INPUTS = {'arm_box_width', 'arm_box_height'}
BOX_GEOMETRY_INPUTS = (set(PlateStructureGeometry.CONFIG_INPUTS)-{'arm_tube_diameter'}) | ARM_INPUTS
BOX_LOAD_INPUTS = (set(PlateStructureLoads.CONFIG_INPUTS)-{'arm_tube_diameter'}) | ARM_INPUTS
BOX_ARM_EVIDENCE = {
    **EVIDENCE, 'revision': 3,
    'arm_section': 'aluminum rectangular hollow section; baseline 50.8 x 50.8 x 1.5875 mm; sharp corners',
    'arm_material': 'existing aluminum density/Young modulus/yield inputs; no purchased-stock alloy/temper certification',
    'arm_axes': 'width transverse; height in arm-motion plane; bending uses width-axis I, Euler uses min(I_width,I_height)',
    'arm_mass': 'two uniform arms, rho*A*L each; LinkageGeometry owns arm mass exactly once',
}


class BoxArmPlateStructureGeometry(PlateStructureGeometry):
    ARM_SECTION = staticmethod(box_arm_section)
    CONFIG_INPUTS = tuple(sorted(BOX_GEOMETRY_INPUTS))
    DEPENDENCIES = {n: set(BOX_GEOMETRY_INPUTS) for n in PlateStructureGeometry.OUTPUT_UNITS}
    model_evidence = BOX_ARM_EVIDENCE


class BoxArmPlateStructureLoads(PlateStructureLoads):
    ARM_SECTION = staticmethod(box_arm_section)
    CONFIG_INPUTS = tuple(sorted(BOX_LOAD_INPUTS))
    DEPENDENCIES = {n: set(BOX_LOAD_INPUTS) | set(PlateStructureLoads.COUPLED_INPUTS)
                    for n in PlateStructureLoads.OUTPUT_UNITS}
    model_evidence = BOX_ARM_EVIDENCE
