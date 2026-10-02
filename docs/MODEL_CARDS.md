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
| Excavation | Stationary plunge, fresh site each cycle, stated Fr domain | Collapse, recutting, impact, complete material-flow dynamics | Bench/EDEM torque, force, mass and energy histories |
| MassProperties | Empty/full/front-pair-only loads; rigid payload; central yaw axis | Slosh and exact CAD mass distribution | CAD and measured CG/inertia |
| Mobility | Fixed-time level route, prescribed profiles, equal wheel torque | Validated slip/sinkage and controller tracking, wheel rotational inertia | Loaded route tests or validated Chrono |
| SiteRelocation | Empty move to a fresh lane with two turns | Trench exit, slip/sinkage and site topology | Timed relocation and power tests/Chrono |
| TrenchTraversal | Wheel diameter >= stationary trench depth | Slip, sinkage, edge collapse and vehicle dynamics | SCM/Chrono runs and held-out physical exit tests |
| TipStability | Level-ground center of gravity within wheel-center rectangle for one full pair and one empty pair, both fully extended | Other half-loaded arm poses need a software interlock; soil motion, dumping reaction, slopes, individual wheel lift/contact and compliance omitted | Asymmetric-load balance tests; validated multibody and terrain simulations |
| Mission | Repeated full cycles, explicit overlap and fresh lanes | Detailed site layout, partial delivery, battery voltage/thermal history | Timed full mission and electrical logs |

## Assumptions requiring particular attention

- Dig/dump Fr bounds 0.30/0.20 are provisional validity screens, not measured
  cutoffs for these drums. Capture and lift speed functions are hypotheses only
  inside that domain. Do not extrapolate into centrifugation.
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
