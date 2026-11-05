# Static tipping with wheel compression

On 2026-11-05 the student specified a linear wheel-compression law, 18 mm
radial grousers, and retention of the existing wheel-center contact boundary.
They confirmed that the 20% ratio uses the unloaded **overall radius**, and
that `wheel_diameter` **includes** the grousers. These are design assumptions,
not measured wheel stiffness or soil calibration. No dataset was fitted.

## Equations and conventions

For the same design's dry mass m_d, current four-drum payload m_p, fill f and
reference fill f_ref = 0.9:

```text
reference_payload = m_p * f_ref / f
reference_mass = m_d + reference_payload
loaded_mass = m_d + m_p
R_outer = wheel_diameter / 2
R_tread = R_outer - grouser_height
deflection_ratio = tipping_radius_deflection_fraction * loaded_mass / reference_mass
delta = R_outer * deflection_ratio
wheel_support_height = (R_tread - delta) + grouser_height = R_outer - delta
tipping_loaded_cg_z = loaded_cg_z - delta
```

`tipping_radius_deflection_fraction` defaults to 0.20. Each of the four wheels
is assigned the same compression at the level-load equivalent mass; the
quarter-load factors cancel. Empty rover weight still compresses the wheels;
zero compression occurs only at zero total supported mass. The pure helper
accepts zero supported mass for this physical limit. The same-design reference
mass remains fixed when comparing different fill fractions; it is recomputed
when sizing a different design. This assumes wheels designed for that design's
rated load, rather than a measured fixed stiffness shared by all designs.

The moving-excavation storage contract is linear in fill, so scaling the
existing payload gives the reference load without duplicating storage physics
or adding mass. Valid run fill is `0 < f <= f_ref <= 1`. Legacy stationary
excavation can be soil-supply limited and does not support this normalization;
it requires the original rigid tipping factories. Above-reference loads and
nonpositive remaining tread radius are rejected, without clipping or silent
extrapolation. Dimensions, masses and ratios must be finite and physical.

For the baseline's 359 mm overall diameter and 18 mm grousers, the unloaded
tread radius is 161.5 mm. At the reference mass, compression is 35.9 mm and
the axle is 143.6 mm above the support plane. Counting the grousers again
outside the 359 mm diameter would double-count their height. At fixed overall
diameter and the specified overall-radius deflection law, grouser height
changes the body/grouser split but does not independently raise the axle.

Grousers are assumed rigid and supported at their tips. Their penetration into
soil is unknown and is not assumed equal to their length. Soil sinkage is also
not subtracted or borrowed from the separate rigid-wheel driving model.

## What changes in the static screens

On the planar slope, the compressed CG height is normal to the plane:

```text
pitch reserve = wheelbase/2 - abs(loaded_cg_x) - tipping_loaded_cg_z*tan(slope_angle)
roll reserve = track_width/2 - tipping_loaded_cg_z*tan(slope_angle)
slope_tip_margin = min(pitch reserve, roll reserve)
slope_tip_angle = atan2(min(wheelbase/2-abs(loaded_cg_x), track_width/2), tipping_loaded_cg_z)
```

Margins retain metres and positive-feasible semantics. Weight cancels out of
the static gravity moment balance. Fill therefore affects slope stability
through CG distribution and compression, rather than directly through mass.
Higher payload can raise the nominal CG while greater compression lowers it;
those effects may partly cancel. A more detailed model need not produce a
larger difference between fill settings.

For the horizontal-platform rock screens, uniform compression lowers the wheel
centers, shaft and mounted masses by delta at fixed platform contact heights.
Every projected contact and CG x/y coordinate is unchanged, as are the
differential reaction moment arms.
Endpoint/sweep tip reserves, reach and signed wheel reactions therefore retain
their original values. Endpoint and sweep world CG-z diagnostics decrease by
delta. Uniform translation also leaves the level-ground asymmetric dumping
margin unchanged. This is an exact limit of the chosen simplified model.

Unequal load-dependent compression on an incline or rock is **not solved**.
The model uses the prescribed total-mass law, without a cos(slope) load
correction, per-wheel elastic equilibrium, lateral compliance or a changing
contact sequence. It remains a static conceptual screen, without experimental
prediction-error estimates. Incline holding friction/torque remains assumed.

## Selection and report contract

`rover/alternatives/compliant_tipping.py` supplies pure helpers and optional
OpenMDAO adapters. `compliant_plate_frame_models()` selects
`CompliantPlateTraversalStability` and `CompliantPlateObstacleStability`, so
the default `build_problem(structure="plate-frame")` and the student's saved
plate-frame baseline use the change automatically. Explicit `ModelSet()` and
`plate_frame_models()` retain their original rigid selection. The original `TraversalStability`,
`PlateTraversalStability`, `ObstacleStability` and `PlateObstacleStability`
remain available and retain their equations. Explicit custom factories are
preserved. `CompliantTraversalStability` and `CompliantObstacleStability` can
be selected together for a moving-excavation reference-frame comparison.

Schema 10 adds fixed inputs `grouser_height` [m],
`tipping_radius_deflection_fraction` [dimensionless], and
`tipping_reference_fill_fraction` [dimensionless]. Selected compliant reports
include seven diagnostics: reference mass [kg], outer/tread radii [m],
deflection [m], radial ratio, support height [m], and corrected loaded CG height
[m]. Configuration records `static_wheel_compliance` so saved optimizer cases
retain the same output units. Existing margin names, loading states, units,
constraints and numerical reserves are preserved. Explicit dependencies include
the fill/reference-mass path; the rock margins have no uniform-heave dependency.

`loaded_cg_z`, sprung CG ports, inertia, geometry, collision screens, the
independent worst-case ground-clearance deduction, and mission energy retain
their nominal conventions. The new `tipping_loaded_cg_z` is used only for
static slope tipping. Existing slope curves consume that same height; no new
plots are added. Geometry drawings remain nominal and the report notes that
convention. Historical reports must be regenerated for comparisons.

For the original rigid plate comparison, pass `models=plate_frame_models()`
explicitly:

```python
from rover.models import plate_frame_models
from rover.problem import build_problem

rigid = build_problem(structure="plate-frame", models=plate_frame_models())
```

## Contact-boundary research and student decision

[Caron, Pham and Nakamura (2015)](https://arxiv.org/abs/1501.04719) derive
rigid surface-contact feasibility conditions, including a center of pressure
inside the support area and friction/yaw constraints. Applied to our static
problem, this supports viewing the tipping boundary as the boundary of
available contact-force support, rather than an immutable axle projection.
Crediting a finite footprint requires it to sustain the required pressure and
moments; that rigid-surface result does not validate a soil footprint.

[Ding et al. (2014), Section 2.1](https://onlinelibrary.wiley.com/doi/10.1155/2014/793526)
explicitly describe the limitation of a single contact point beneath a wheel
on deformable rough terrain and construct contact geometry from the terrain.
Their path-following study is not a measured tipping correction for this rover.

[Sharma et al. (2018)](https://doi.org/10.1016/j.jterra.2017.12.002) test a
different flexible lunar wheel, separating wheel deformation and terrain
sinkage. They report stiffness changes when rim/spoke contact changes. This
supports measuring wheel load-deflection and soil contact rather than treating
the student's linear design target as universally calibrated wheel behavior.
Their coefficients are not transferred here.

After this briefing, the student chose to retain center contacts for this pass.
No finite-patch extension, extra soft-soil restoring moment or calibrated
fulcrum offset is credited. A next step would require wheel load-deflection,
effective supported footprint, grouser penetration and soil settlement
measurements under the relevant normal loads, followed by loaded tilt tests.

## Verification

`tests/test_compliant_tipping.py` checks zero/half/rated total-load limits,
rover-weight inclusion, fixed reference mass across fills, independent gravity
moment balance, grouser accounting, signed instability, invalid/collapsed and
above-reference cases, uniform-heave invariance of rock/dump/energy outputs,
factory/recorded-output contracts, N2 dependencies and smooth derivatives.
See [VERIFICATION.md](VERIFICATION.md) for actual run outcomes. These checks
are software/analytical verification, not hardware or soil validation.
