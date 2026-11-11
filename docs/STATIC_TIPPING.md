# Static tipping with wheel compression

The plate-frame tipping model includes unequal wheel compression and downhill
body lean. The student specifies 20% of unloaded **overall radius** at the same
design's 0.9-fill reference load, a provisional linear force/displacement law
through **6.4 cm (64 mm)**, and a rigid stop thereafter. These are assumptions
pending nonlinear FEA and independent hardware tests; no fit or measured error
is available. Center contacts remain selected pending the wheel team's work.

## Wheel law and geometry

For dry mass m_d, current four-drum payload m_p and current fill f:

```text
M_ref = m_d + m_p*tipping_reference_fill_fraction/f
R = wheel_diameter/2
k = M_ref*g/(4*tipping_radius_deflection_fraction*R)
delta_i = min(N_i/k, wheel_compression_hard_stop)
N_i = k*delta_i + S_i; S_i >= 0; S_i*(hard_stop-delta_i) = 0
```

Defaults are reference fill 0.9, radial fraction 0.20, and stop 0.064 m.
The quarter-load reference sets stiffness, rather than capping wheel force.
Beyond the stop, S_i carries extra reaction without further compression.
Zero radial fraction selects the rigid limit. Empty rover weight compresses
the wheels; zero force gives zero compression.

Moving-excavation payload is proportional to fill, so M_ref stays fixed across
same-design fill comparisons. Legacy soil-supply-limited excavation requires
the rigid factories. Candidate designs imply wheels sized for each design's
reference load, rather than one measured spring shared across all designs.
Inputs must be finite and physical; reference compression cannot exceed the
stop, and full travel must leave positive tread radius.

The 18 mm radial grousers are rigid and **inside wheel_diameter**. Thus tread
radius is R-0.018, and tip-supported axle height is R-delta_i. Grousers are not
added twice or assumed buried. For the saved 359 mm wheel, nominal reference
compression is 35.9 mm; the stop permits further individual wheel compression.

## Coupled static equilibrium

The existing `compliant_tipping.py` supplies pure helpers and adapters. It reuses
`rocker_pose`, `articulated_cg`, `wheel_reactions` and `rock_path_screen` in the
existing obstacle/traversal modules. Shaft height, shaft roll and left/right
rocker pitches satisfy four wheel-height equations:

```text
wheel_center_z_i = terrain_height_i + R - delta_i
body_pitch = (left_rocker_pitch + right_rocker_pitch)/2
```

At each pose, four normal reactions balance weight, global moments and equal
left/right external rocker pitch torques for the ideal differential. Sprung,
wheel/drive and rocker masses retain their existing placements and are counted
once. On slopes, holding reactions are parallel to opposite gravity, assuming
the same tangential/normal ratio at each wheel and adequate friction/motor
holding torque. On horizontal platforms this reduces to vertical reactions.

Negative normals remain unloading failures. Their zero-compression arithmetic
continuation is guarded, not a physical four-contact pose or a prediction after
lift-off. Root closure, nonfolded geometry and positive incremental-compliance
branch indicator are checked separately. An algebraic root alone is insufficient.

Slope headings use a 5-degree grid with local bounded refinement and exact
mirror reductions. This is a numerical search, not an analytical continuum
proof. Pitch/roll margins and existing graph curves evaluate coupled cardinal
poses. Global hull zero (`slope_tip_angle`) and first wheel unloading
(`slope_contact_limit_angle`) are separate; the former may occur after the
four-contact model has ceased to be physical. Losing one wheel is not necessarily
complete rollover. The 41-height rock sweep and endpoint use the same equilibrium
and retain the original rigid reach screen as an additional conservative gate.

## Selection and report contract

`compliant_plate_frame_models()` keeps the existing
`CompliantPlateTraversalStability` / `CompliantPlateObstacleStability` classes;
the uniform-heave approximation is replaced in place. `ModelSet()` and explicit
`plate_frame_models()` retain rigid reference physics. Custom factories are
preserved. Schema 11 adds stop travel, per-wheel normal/compression/stop-reaction
diagnostics, heading/limit and numerical-validity outputs. Existing compression
and support-height diagnostics retain their nominal equal-load, level-ground
meanings; `tipping_loaded_cg_z` is the solved level CG, not the inclined CG.

Two additional active screens are `slope_wheel_load_margin` [N] and
`wheel_equilibrium_margin` [dimensionless]. Existing margin names, SI units,
positive-feasible sign and loading states remain. No constraints are relaxed.
Recorded configuration identifies optional outputs. Nominal mass properties,
level dumping, geometry/clearance drawings, route dynamics, energy and score
remain outside this change. No additional graph is introduced.

Finite footprints, camber contact corrections, lateral wheel elasticity, soil
bearing/sinkage, grouser penetration, stop/joint strength, post-lift contact
sequence and dynamics remain unmodeled. The absolute 64 mm travel is a current-wheel
assumption; alternate hardware needs its own travel input. FEA agreement alone
will not establish experimental validation.

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

`tests/test_compliant_tipping.py` checks force/moment/differential conservation,
flat and rigid limits, reference-fill/grouser accounting, downhill lean, hard-stop
load conservation, unloading/unstable/invalid cases, dense headings, mass/energy
invariance, records/N2/dependencies and derivatives. Actual run outcomes are in
[VERIFICATION.md](VERIFICATION.md); these are software checks, not soil validation.
