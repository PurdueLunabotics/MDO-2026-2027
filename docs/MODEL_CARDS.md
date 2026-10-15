# Reference model cards — version 2

All reference cards are **uncalibrated conceptual models**. Tests verify software,
conservation and selected physical limits, not predictive accuracy. A replacement
needs a card with provenance, domain, held-out validation error and reproduction.

| Component | Intended use | Important omissions | Replacement evidence |
| --- | --- | --- | --- |
| WheelMass | Four closed cylindrical wheels from one effective areal-mass coefficient | Spokes/open sides, grousers, hubs, fasteners, strength-based thickness | Parametric CAD/BOM and weighed wheels at multiple sizes |
| DrumMass | Four closed cylindrical shells plus two motor/gear assemblies | Architecture-specific scoops, helix/baffles, shafts, hubs, joints and actual part ratings | Component-level CAD/BOM and weighed drums with a clear motor/gear boundary |
| GeometryMass | Sum wheel, drum, linkage, structure, drive, battery and fixed mass; stowed envelope | Detailed drive/gear catalog and mounting mass | Measured assembly mass and packaging CAD |
| LinkageGeometry/Loads | Mirrored planar arms with independently located actuator and arm pivots; valid triangle/clearance; provisional wheel-diameter step clearance; sampled maximum virtual-work leverage and rated force screen | Validated obstacle capability, cylinder envelopes, real force-speed/thermal curves, joint/rod strength, lowering load history | CAD sweep, obstacle/actuator tests and extraction tests |
| StructureGeometry | Two longitudinal tubes, crossmember, two towers, two arms | Exact hardware/joint manufacturing mass | BOM, weighed prototype, CAD |
| StructureLoads | Small-deflection elastic beam envelopes, valid tube sections | Assembled frame load sharing, joint strength, local shell buckling, fatigue | Beam/FEA plus load/deflection tests |
| Plate-frame profile (optional) | Moving-mount axial plate strips, CF rod bending, box rocker/axle/arm gravity screens, explicit mass placement | Provisional topology/CF properties; junction bending, whole-plate/local wall buckling, joints, recovery/cutting and obstacle reactions omitted | Independent equilibrium/energy tests, selected tube data, CAD/BOM, shell FEA and physical load tests |
| Excavation | Stationary plunge, fresh site each cycle, constant capture/lift; Fr diagnostic only | Unverified RPM-dependent retention/dumping, collapse, recutting, impact, complete material-flow dynamics | Bench/EDEM torque, force, mass and energy histories |
| MassProperties | Empty/full/front-pair-only loads; rigid payload; central yaw axis | Slosh and exact CAD mass distribution | CAD and measured CG/inertia |
| Mobility | Fixed-time level route, prescribed profiles, equal wheel torque | Validated slip/sinkage and controller tracking, wheel rotational inertia | Loaded route tests or validated Chrono |
| SiteRelocation | Empty move to a fresh lane with two turns | Trench exit, slip/sinkage and site topology | Timed relocation and power tests/Chrono |
| TrenchTraversal | Wheel diameter >= stationary trench depth | Slip, sinkage, edge collapse and vehicle dynamics | SCM/Chrono runs and held-out physical exit tests |
| TipStability | Level-ground center of gravity within wheel-center rectangle for one full pair and one empty pair, both fully extended | Other half-loaded arm poses need a software interlock; soil motion, dumping reaction, slopes, individual wheel lift/contact and compliance omitted | Asymmetric-load balance tests; validated multibody and terrain simulations |
| ObstacleStability | Fully loaded travel pose, each wheel raised 0.40 m; centered rockers with average-angle differential; geometric reach and projected gravity support | Estimated +0.10 m pivot offset; point contacts, no joint stops/load equilibrium, climb path, traction, collisions, soil or dynamics; 0.50 m ditch unverified | CAD geometry/CG and loaded articulation/balance tests; full crossing trials |
| ObstacleClearance (revision 1) | Lower bound on drum–wheel surface gap from spheres enclosing rigid cylindrical envelopes; all 16 pairs in each of four prescribed single-wheel-raised travel poses; subtracts mechanism_clearance | Conservative over-rejection is possible; negative bounds do not prove cylinder collision. No intermediate crossing path, deflection, terrain, hubs/shafts/arms/frame interference; invalid poses have flagged guarded diagnostics and failing margins | Analytic software tests with synthetic geometry; no fitted data or hardware validation. Check CAD envelopes and measured loaded articulation, then use detailed cylinder/CAD geometry if the bounds over-reject |
| Mission | Repeated full cycles, explicit overlap and fresh lanes | Detailed site layout, partial delivery, battery voltage/thermal history | Timed full mission and electrical logs |
| Standard X44 motor profile (optional, physics revision 2 / interface revision 3) | Fixed six standard X44s at 12 V; fixed 100:1 drive ratio; exploratory drum ratio; original disciplines supply mechanical work/peak to shared Gearmotor; catalog assembly mass | Combined drive/drum efficiencies 0.72/0.70 are provisional; current/thermal ratings undecided; gearbox behavior, adapters and drum stage budget unvalidated; climbing unverified | Versioned official curve/manifest, interior reproduction, independent work/kinetic-energy and one-time loss tests, mechanical-invariant regression and replacement composition; no independent hardware validation. See X44_MOTORS.md |
| TraversalStability (revision 1) | Full-load travel: planar 30-degree gravity support, every heading; four separate one-wheel lifts sampled from 0 to 40 cm; global hull and ideal differential reactions | Assumes slope friction/holding torque; finite grid, horizontal rock platforms and ideal differential. No approach profile, combined slope/rock, wheel-pair cases, post-lift contacts, joint limits, traction/climbing, impacts or validation | Independent slope, force/moment, negative-load, mirrored/dense grid and mass tests; measured CG and loaded tilt/articulation tests needed. See OBSTACLE_TRAVERSAL.md |

## Assumptions requiring particular attention

- Arm attachment revision: `actuator_attachment_fraction` selects the pin along
  the straight arm, measured from the chassis pivot (default 1; domain 0<f<=1).
  It is fixed in ASSUMPTIONS by default; moving it into DESIGN makes it optimizable.
  Geometry intersects circles at that pin and extends to the drum tip. Loads use
  virtual work at the pin; gravity energy remains based on drum lift. MassProperties
  retains an equivalent rod between mean proximal and distal link endpoints.
  Evidence is analytical geometry, numerical virtual work, gravity-work conservation,
  and invalid-triangle tests in `tests/test_arm_attachment.py`; no hardware data
  or fitted relationships are used. Calibration status remains uncalibrated.
  Actuator-body interference, bracket mass/strength, actual link mass distribution,
  and assembled-frame load sharing still require CAD and physical tests.

- At the student's request the uncalibrated dig/dump Fr caps are removed.
  Legacy 0.30/0.20 reference values and signed differences remain diagnostics,
  without PASS/FAIL semantics. Constant capture/lift assumes no additional
  RPM-dependent granular losses; actual retained mass/dumping behavior is
  unverified at every speed. Torque, power and startup energy are still modeled.
  Optional empirical models must still enforce their own measured domains.
- The current drum is full-width; fill_fraction is a storage ceiling, not a
  required achieved fill.
- The wheel-diameter/trench-depth screen is provisional. It does not guarantee
  exit from disturbed soil; replace it with validated SCM or physical-test data.
- The travel drum's wheel-diameter step clearance is a separate provisional
  obstacle screen. Loaded stability here covers the prescribed level-ground
  accelerations/turns, not slopes or wheel-on-rock transients.
- The uneven-load tip screen is static. It checks extended arm positions but the
  reference mission still deposits from the raised travel pose. A nonnegative
  margin is not proof that moving or dumping on uneven terrain cannot tip.
- Tube strength/safety factors are illustrative; actual joint/weld/printed material
  allowables and local buckling must be checked before building hardware.
- Fresh-lane geometry and available excavation area are planning assumptions;
  confirm the actual legal zone and maneuver space.

## Plate-frame evidence and scope

Select with `--structure plate-frame` or `build_problem(structure="plate-frame")`.
Five coordinated factories in `rover/alternatives/plate_frame.py` and
`rover/alternatives/traversal_stability.py` preserve scalar mass/output boundaries.
Fixed 6.35 mm 6061-T6 sheet and 25.4 mm CF ID come from
the student. Spoke width, CF span/wall, box rocker/axle sections and centered axle
height are adjustable. The four-radial/three-perimeter topology and CF properties
are **provisional**, not calibrated CAD/material data. No fitted response or
experimental dataset was introduced. Valid only for nondegenerate geometry and
mirrored fully loaded level-ground gravity; central pin cannot carry unbalanced
moments. Failed arithmetic remains failed. Plate strip buckling is not whole-plate
buckling validation; CF local wall/joint behavior and excluded operational loads
need separate checks. Detailed equations, domains, output meanings and material
sources: [PLATE_FRAME.md](PLATE_FRAME.md). Numerical checks: VERIFICATION.md.

The lower boundary is B+-S-B-; removing the direct B+-B- strip eliminates the
collapsed fourth cutout when hub and lower mounts share an elevation. Real
pad-distance, signed triangle opening and wheel/axle packing checks remain.
The filled-chassis rectangle is not an enforced collision screen for this open
frame; its legacy value is reported as diagnostic only. No actual frame-member
or equipment collision model has been substituted for it.

The active obstacle screen and pending ditch requirement are documented in
[OBSTACLE_TRAVERSAL.md](OBSTACLE_TRAVERSAL.md). The user confirmed centered pivots
and an average-angle differential; +0.10 m pivot offset is an editable estimate.
Geometry/mass/support tests are analytical, without calibration data. A positive
obstacle screen is not experimental validation or proof of climbing capability.
The circular-like ditch still lacks a width/radius and is explicitly unverified.

## Replacement card template

```text
Name, revision, author:
Component or response replaced:
Physical question:
Inputs/outputs/units and loading/operating states:
Geometry/material/gravity/operating domain:
Raw-data URI and hash; tool/version; processing script:
Fitting runs and independent validation runs:
Validation metrics and preselected acceptance thresholds:
Uncertainty, failure-domain and extrapolation treatment:
Known omissions:
Reproduction commands, data revision and offline random seeds:
Default changed? Evidence and reviewer:
```
