# Actuator attachment relative to the drum axle — revision 2

The attachment fraction is replaced by two signed lengths in metres:

| Input | Positive direction | Requested baseline |
| --- | --- | --- |
| `lin_act_side_arm_offset` | Along the arm, past the drum axle away from the chassis pivot | 0.05 m |
| `lin_act_top_arm_offset` | Normal to the arm, above it in the front side elevation | 0.05 m |

Both are ordinary design variables with exploratory bounds −0.15 to +0.15 m.
These are numerical search bounds, not measured bracket dimensions. Generic
defaults are zero, reproducing the original attachment at the axle. Use
`--freeze lin_act_side_arm_offset lin_act_top_arm_offset` to hold both during
optimization. Fixed evaluation of an input case never optimizes them.

For lower chassis pivot B, drum axle C, B-to-axle length L, upper actuator pivot A,
unit arm direction u=(C−B)/L and upward local normal n=(−u_z,u_x):

```text
D = C + side*u + top*n
rho = sqrt((L+side)^2 + top^2)
beta = atan2(top, L+side)
```

D is the actuator pin. The side offset is not the vehicle's lateral y direction.
The rear assembly mirrors the entire front geometry in x: positive top remains
above the rear arm. `arm_length` remains B-to-drum-axle length, independent of D.
`actuator_attachment_distance` now reports the actual B-to-D radial distance rho.

## Kinematics, loads and mass

Intersect the B-centered circle of radius rho with the A-centered actuator
length circle on the existing outward B-D branch. Rotate B-D's unit direction
through −beta to obtain B-C. The same mapping is used for all three commanded
poses, the 81-angle sweep, reports and mass placement. Unreachable circles retain
finite guarded geometry and failing triangle/toggle margins. Public helpers
reject nonfinite offsets, nonpositive L, and L+side ≤ 0. Current design bounds
keep the last condition strictly positive throughout the declared search box.

The gravity virtual-work leverage is
`abs((C_x−B_x)*|A−D| / cross(D−B,A−B))`. Actuator force, PA-04 chart speed/current
and timing therefore use the eccentric pin. Moving the pin does not multiply
gravity energy by that leverage. Lowering/cutting overlap remains subtracted
once, and PA-04 electrical losses remain applied once.

For side > 0 the same uniform arm tube continues beyond C to axial station
L+side. Physical tube length is ell=max(L,L+side). Two tails add
`2*arm_linear_mass*(ell−L)` kg once to linkage mass. Their actual centroids and
rod yaw inertias enter all loading states and sprung mass. Uniform arm gravity
moment uses `arm_linear_mass*g*ell^2/2`; raising/lowering gravity work and
actuator force include this increase. Zero offsets preserve reference physics.

Plate-frame load equilibrium uses the actual force at D. To screen the straight
tube, transfer this force to axial station L+side with the eccentric couple
`−top*F_parallel`. Bending includes both sides of the couple jump, point loads,
distributed tube weight and interior extrema. Euler screening uses ell. Tube
end and pin extents enter stowed/deployed height, length and pit-floor screens.
Existing drum/wheel clearance screens continue to use the solved drum poses.

One common conservative stress envelope (maximum axial stress plus maximum
bending stress) applies at every side/top value, including zero. This avoids
a fictitious jump when crossing zero top offset. Compared with the older
unextended plate-arm screen, it can slightly increase the conservative arm stress
envelope; no constraint or safety factor is relaxed. The generic reference
tube-frame structural equations are unchanged. Side=0 still has the physical
max-length kink where the tube continuation begins; derivative tests use smooth
nonzero side offsets, while continuity is checked separately at zero.

The historical motor-interface fixture explicitly selects its original
round-arm, axle-pin beam screen as well as its original mobility law. Its
numeric expectations and tolerances are unchanged. The current offset beam
screen is verified independently by force/moment limits and continuity tests.

The normal offset is an assumed **rigid, massless bracket**. Its fabrication
mass, section, strength, joints and collision envelope are unknown. Tube beam
calculations include its transferred force/moment, but do not certify the
bracket itself. Pin/centerline envelopes do not verify actuator-body, bracket,
arm/wheel or soil interference. Actual bracket/CAD/BOM and test data are needed
to close these limitations; no measurements or calibration have been invented.

## Interface migration and evidence

Schema 5 additionally makes `actuator_body_margin` diagnostic only at the user’s request; its value is retained without optimizer or pass/fail enforcement. The earlier schema 4 removes `actuator_attachment_fraction` from live configuration and
ports. Named old JSON cases explicitly convert `side=(f−1)*L`, `top=0`, recording
the conversion in source notes; mixed old/new coordinates are rejected.
Programmatic old fraction overrides are rejected. Historical report drawing
support applies the same explicit geometric conversion. Old reports and
optimizer outcomes still require reevaluation. `--optimize-attachment` is a
redundant compatibility flag; offsets are already design variables. The legacy
`--attachment-max` flag is retained only for old study entry points and does not
change the signed offset bounds. Physical continuation is automatic.

New coupled contracts are `arm_envelope_radius` (m, from selected structure),
`arm_physical_length` (m per arm), and `arm_extension_mass` (kg, both tails).
Geometry owns the tail mass; mass properties subtract tails from the approximate
core linkage lump before placing them explicitly. Wheel/drum/motor mass ownership
and all mission requirements, score semantics and margin signs remain unchanged.

The reference tube-frame structural envelope remains an elementary envelope;
the plate-frame profile additionally resolves the eccentric pin moment as above.
Neither profile is assembled-frame FEA or hardware validation.

Independent tests check signed coordinates, circle closure, numerical virtual
work, unchanged gravity work at fixed drum poses without a tail, eccentric force
and moment equilibrium, the analytic axial-force/end-couple beam limit, actual
tail mass/work/inertia, invalid domains, strict failed geometry, mirrored SVG
pins, source migration, declared dependencies and smooth total derivatives.
No fitted relationship or new calibration dataset is introduced.

Reproduce the requested fixed report:

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/arm_offsets_baseline.json
```

All other baseline inputs are retained. Fixed evaluation is not optimization.
Current reports exclude the actuator packaging reserve from feasibility and optimization; all remaining failed checks stay visible. A negative diagnostic still indicates a catalog/body mismatch under that calculation, without an enforced decision. See `VERIFICATION.md` for the verification record.
