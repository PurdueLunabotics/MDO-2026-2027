"""Static center-contact tipping with user-specified uniform wheel compression.

The 20% law is a design assumption, not a measured spring/soil fit. Overall
wheel diameter includes rigid grousers. No footprint-edge or sinkage credit is
taken. Original rigid tipping factories remain available for comparisons.
"""
import math

from ..components import scalar
from .obstacle_stability import ObstacleStability
from .plate_frame import PlateObstacleStability
from .traversal_stability import TraversalStability, slope_support, rock_path_screen


COMPLIANCE_CONFIG = ("grouser_height", "tipping_radius_deflection_fraction",
                     "tipping_reference_fill_fraction", "fill_fraction")
COMPLIANCE_MASSES = {"dry_mass": "kg", "payload_mass": "kg"}
REFERENCE_MASS_INPUTS = {"dry_mass", "payload_mass", "fill_fraction", "tipping_reference_fill_fraction"}
RATIO_INPUTS = REFERENCE_MASS_INPUTS | {"tipping_radius_deflection_fraction"}
DEFLECTION_INPUTS = RATIO_INPUTS | {"wheel_diameter"}
COMPLIANCE_OUTPUTS = {
    "tipping_reference_mass": "kg", "tipping_wheel_outer_radius": "m",
    "tipping_wheel_tread_radius": "m", "tipping_wheel_deflection": "m",
    "tipping_wheel_deflection_ratio": None, "tipping_wheel_support_height": "m",
    "tipping_loaded_cg_z": "m",
}


def wheel_support_geometry(diameter, grouser_height, rated_ratio, mass, reference_mass):
    """Uniform static heave at a specified total mass, including the rover [SI].

    Each wheel carries one quarter of the mass and reference mass, so those
    factors cancel in delta/R = rated_ratio * mass/reference_mass. R is the
    unloaded OVERALL radius, as specified by the student. Grousers retain their
    height: (R-grouser_height-delta)+grouser_height = R-delta.
    Supported mass is [0, reference_mass]; extrapolation is rejected.
    """
    if (not all(math.isfinite(x) for x in
                (diameter, grouser_height, rated_ratio, mass, reference_mass))
            or diameter <= 0 or reference_mass <= 0 or mass < 0
            or not 0 <= grouser_height < diameter/2 or not 0 <= rated_ratio < 1):
        raise ValueError("Static wheel compression needs finite positive diameter/reference mass, "
                         "nonnegative mass/grouser height and radial ratio in [0, 1).")
    if mass > reference_mass + 1e-12*reference_mass:
        raise ValueError("Static wheel compression exceeds the reference load; "
                         "the supplied linear law is not defined above the reference fill.")
    outer = diameter/2
    tread = outer-grouser_height
    ratio = rated_ratio*mass/reference_mass
    deflection = outer*ratio
    if rated_ratio*outer >= tread:
        raise ValueError("Rated wheel compression must leave a positive tread radius.")
    return {"tipping_reference_mass": reference_mass,
            "tipping_wheel_outer_radius": outer, "tipping_wheel_tread_radius": tread,
            "tipping_wheel_deflection_ratio": ratio, "tipping_wheel_deflection": deflection,
            "tipping_wheel_support_height": tread-deflection+grouser_height}


def loaded_wheel_compliance(values):
    """Reference stiffness to the SAME design's payload at 0.9 drum fill.

    Requires the moving-excavation storage contract payload proportional to
    fill_fraction. Scaling from the current payload avoids a second storage
    equation or any mass addition. The reference does not follow the run's fill.
    """
    v = values.__getitem__
    fill, reference_fill = v("fill_fraction"), v("tipping_reference_fill_fraction")
    dry, payload = v("dry_mass"), v("payload_mass")
    if (not all(math.isfinite(x) for x in (fill, reference_fill, dry, payload))
            or not 0 < fill <= reference_fill <= 1 or dry <= 0 or payload < 0):
        raise ValueError("Static compression requires positive dry mass, nonnegative payload "
                         "and 0 < fill_fraction <= tipping_reference_fill_fraction <= 1.")
    reference_mass = dry+payload*reference_fill/fill
    return wheel_support_geometry(v("wheel_diameter"), v("grouser_height"),
                                  v("tipping_radius_deflection_fraction"),
                                  dry+payload, reference_mass)


EVIDENCE = {
    "revision": 1,
    "status": "user-specified static compression assumption; no measured calibration or validation",
    "law": "delta/overall_radius = rated_ratio*(dry_mass+payload_mass)/(dry_mass+payload_mass*reference_fill/fill)",
    "reference": "same design at 0.9 fill; 20% of unloaded overall radius; zero at zero total mass",
    "grousers": "18 mm radial, rigid, already inside wheel_diameter; support at tips; no assumed soil penetration",
    "support": "wheel-center contacts retained by student choice; no footprint-edge stability credit",
    "scope": "equal compression at the level-load equivalent mass; uniform heave also applied to rock CG height",
    "omissions": "unequal wheel compression/load transfer, lateral compliance, soil bearing/sinkage, grouser penetration, contact loss and dynamics",
}


class CompliantTraversalStability(TraversalStability):
    """Optional reference-inventory adapter; plate variant is selected below."""
    STATIC_WHEEL_COMPLIANCE = True
    REQUIRES_LINEAR_FILL_PAYLOAD = True
    CONFIG_INPUTS = TraversalStability.CONFIG_INPUTS + COMPLIANCE_CONFIG
    COUPLED_INPUTS = {**TraversalStability.COUPLED_INPUTS, **COMPLIANCE_MASSES}
    REQUIRED_OUTPUT_UNITS = TraversalStability.OUTPUT_UNITS
    OUTPUT_UNITS = {**TraversalStability.OUTPUT_UNITS, **COMPLIANCE_OUTPUTS}
    DEPENDENCIES = {n: set(deps) for n, deps in TraversalStability.DEPENDENCIES.items()}
    for n in ("slope_pitch_tip_margin", "slope_roll_tip_margin", "slope_tip_margin", "slope_tip_angle",
              "rock_sweep_cg_z"):
        DEPENDENCIES[n] |= DEFLECTION_INPUTS
    DEPENDENCIES.update({
        "tipping_reference_mass": REFERENCE_MASS_INPUTS,
        "tipping_wheel_outer_radius": {"wheel_diameter"},
        "tipping_wheel_tread_radius": {"wheel_diameter", "grouser_height"},
        "tipping_wheel_deflection_ratio": RATIO_INPUTS,
        "tipping_wheel_deflection": DEFLECTION_INPUTS,
        "tipping_wheel_support_height": DEFLECTION_INPUTS,
        "tipping_loaded_cg_z": DEFLECTION_INPUTS | {"loaded_cg_z"},
    })
    model_evidence = {**TraversalStability.model_evidence, "wheel_compliance": EVIDENCE}

    def compute(self, inputs, outputs):
        values = {n: scalar(inputs, n) for n in inputs}
        compression = loaded_wheel_compliance(values)
        delta = compression["tipping_wheel_deflection"]
        height = values["loaded_cg_z"]-delta
        result = slope_support(values["wheelbase"], values["track_width"],
                               values["loaded_cg_x"], height, values["slope_angle"])
        result.update(rock_path_screen(values, rocker_centroid_fraction=self.rocker_centroid_fraction))
        # Equal compression lowers all hardware at fixed platform contacts.
        # Projected contacts/CG and differential reaction moments are invariant.
        result["rock_sweep_cg_z"] -= delta
        result.update(compression)
        result["tipping_loaded_cg_z"] = height
        for name in self.OUTPUT_UNITS:
            outputs[name] = result[name]


class CompliantPlateTraversalStability(CompliantTraversalStability):
    rocker_centroid_fraction = .5
    model_evidence = {**CompliantTraversalStability.model_evidence,
                      "rocker_mass": "plate-profile uniform sloping half-arms; centroid midway between shaft and wheel midpoint"}


class _CompressedObstacleHeight:
    REQUIRES_LINEAR_FILL_PAYLOAD = True

    def compute(self, inputs, outputs):
        super().compute(inputs, outputs)
        compression = loaded_wheel_compliance({n: scalar(inputs, n) for n in inputs})
        outputs["obstacle_loaded_cg_z"] -= compression["tipping_wheel_deflection"]


class CompliantObstacleStability(_CompressedObstacleHeight, ObstacleStability):
    CONFIG_INPUTS = ObstacleStability.CONFIG_INPUTS + COMPLIANCE_CONFIG
    COUPLED_INPUTS = {**ObstacleStability.COUPLED_INPUTS, **COMPLIANCE_MASSES}
    DEPENDENCIES = {n: set(deps) for n, deps in ObstacleStability.DEPENDENCIES.items()}
    DEPENDENCIES["obstacle_loaded_cg_z"] |= DEFLECTION_INPUTS
    model_evidence = {**ObstacleStability.model_evidence, "wheel_compliance": EVIDENCE}


class CompliantPlateObstacleStability(_CompressedObstacleHeight, PlateObstacleStability):
    CONFIG_INPUTS = CompliantObstacleStability.CONFIG_INPUTS
    COUPLED_INPUTS = CompliantObstacleStability.COUPLED_INPUTS
    DEPENDENCIES = CompliantObstacleStability.DEPENDENCIES
    model_evidence = {**PlateObstacleStability.model_evidence, "wheel_compliance": EVIDENCE}
