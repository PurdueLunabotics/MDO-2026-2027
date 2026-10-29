"""Level-ground axle clearance with the student's fixed deflection allowances."""
import math

from .components import scalar
from .discipline import Discipline


def axle_ground_clearance(wheel_diameter, bottom_pivot_height, rocker_pivot_offset,
                          radius_deflection_fraction=.20, differential_allowance=.0254,
                          minimum_clearance=.1524, *, bottom_axle_radius=0., rocker_axle_radius=0.):
    """Signed clearances [m]; bottom height is above ground, rocker offset above axle.

    Wheel compression lowers both axle centers by fraction*undeformed_radius.
    Subtract each axle radius to measure its underside, then the bar allowance.
    Zero radii provide the centerline limit; model adapters supply actual sizes.
    Negative clearances stay negative. This is not an articulated terrain model.
    """
    values=(wheel_diameter,bottom_pivot_height,rocker_pivot_offset,
            radius_deflection_fraction,differential_allowance,minimum_clearance,
            bottom_axle_radius,rocker_axle_radius)
    if (not all(math.isfinite(x) for x in values) or wheel_diameter <= 0
            or not 0 <= radius_deflection_fraction < 1
            or differential_allowance < 0 or minimum_clearance <= 0
            or min(bottom_axle_radius,rocker_axle_radius) < 0):
        raise ValueError('Clearance needs finite geometry, positive wheel/requirement, '
                         '0 <= radial deflection < 1 and nonnegative bar allowance/axle radii.')
    radius=wheel_diameter/2
    deduction=radius_deflection_fraction*radius+differential_allowance
    bottom=bottom_pivot_height-bottom_axle_radius-deduction
    rocker=radius+rocker_pivot_offset-rocker_axle_radius-deduction
    clearance=min(bottom,rocker)
    return {'bottom_pivot_ground_clearance':bottom,
            'rocker_pivot_ground_clearance':rocker,
            'ground_clearance':clearance,
            'ground_clearance_margin':clearance-minimum_clearance}


class GroundClearance(Discipline):
    CONFIG_INPUTS=('wheel_diameter','bottom_pivot_height','rocker_pivot_offset',
                   'clearance_radius_deflection_fraction','differential_bar_clearance_allowance',
                   'minimum_ground_clearance','cf_tube_inner_diameter','cf_tube_wall','axle_outer_diameter')
    OUTPUT_UNITS={n:'m' for n in ('bottom_pivot_ground_clearance','rocker_pivot_ground_clearance',
                                 'ground_clearance','ground_clearance_margin',
                                 'bottom_pivot_axle_radius','rocker_pivot_axle_radius')}
    shared={'wheel_diameter','clearance_radius_deflection_fraction','differential_bar_clearance_allowance'}
    bottom_size={'cf_tube_inner_diameter','cf_tube_wall'}
    rocker_size={'axle_outer_diameter'}
    DEPENDENCIES={
        'bottom_pivot_axle_radius':bottom_size,
        'rocker_pivot_axle_radius':rocker_size,
        'bottom_pivot_ground_clearance':shared|{'bottom_pivot_height'}|bottom_size,
        'rocker_pivot_ground_clearance':shared|{'rocker_pivot_offset'}|rocker_size,
        'ground_clearance':shared|{'bottom_pivot_height','rocker_pivot_offset'}|bottom_size|rocker_size,
        'ground_clearance_margin':set(CONFIG_INPUTS),
    }
    model_evidence={
        'status':'User-specified static level-ground clearance screen; no calibration',
        'assumptions':'20% radial compression, axle underside radii, 25.4 mm bar allowance, 152.4 mm required clearance',
        'axles':'Bottom radius=CF ID/2+wall; rocker radius=suspension axle OD/2; plate-frame dimensions vary with design',
        'coordinates':'bottom_pivot_height above undeformed ground; rocker_pivot_offset above wheel centers',
        'scope':'Uniform wheel compression used only here; no slope, articulation or actual bar geometry',
    }

    def compute(self, inputs, outputs):
        v=lambda n:scalar(inputs,n)
        bottom_radius=v('cf_tube_inner_diameter')/2+v('cf_tube_wall')
        rocker_radius=v('axle_outer_diameter')/2
        result=axle_ground_clearance(v('wheel_diameter'),v('bottom_pivot_height'),v('rocker_pivot_offset'),
                                    v('clearance_radius_deflection_fraction'),
                                    v('differential_bar_clearance_allowance'),v('minimum_ground_clearance'),
                                    bottom_axle_radius=bottom_radius,rocker_axle_radius=rocker_radius)
        outputs['bottom_pivot_axle_radius']=bottom_radius
        outputs['rocker_pivot_axle_radius']=rocker_radius
        for name,value in result.items():outputs[name]=value
