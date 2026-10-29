Current attachment interface: [axle-relative side/top offsets, revision 2](ARM_OFFSETS.md). The fraction-based attachment selection described in older sections below is historical; live models use signed metre-valued offsets, with automatic axial tube continuation and an assumed rigid, massless normal bracket.

# Preliminary CF-tube / aluminum-plate frame

Select the coordinated optional profile with:

```powershell
.\.venv\Scripts\python.exe main.py --structure plate-frame --baseline --output results/plate_frame/baseline.json
.\.venv\Scripts\python.exe main.py --structure plate-frame --check-derivatives --output results/plate_frame/optimization.json
```

The Python equivalent is `build_problem(structure="plate-frame")`. This selects
five ModelSet factories: BoxArmPlateStructureGeometry, BoxArmPlateStructureLoads,
PlateMassProperties, PlateObstacleStability, and PlateTraversalStability.
The last is defined in `rover/alternatives/traversal_stability.py`; the others
are in `rover/alternatives/plate_frame.py`, with the box-arm adapters/section
helper in `rover/alternatives/aluminum_box_arm.py`. `ModelSet()` and CLI
`--structure reference` retain the previous tube-frame physics and inputs. The
profile does not mutate the reference config dictionaries. No automatic section
sizing or mass/load fixed-point solve is hidden inside a component.

## Known hardware and provisional choices

The student specified 1/4 inch (0.00635 m) 6061-T6 plates, 1 inch (0.0254 m)
CF tube ID, aluminum box rockers, a 1/2 inch (0.0127 m) wheel-to-rocker gap,
bonded inserts plus single bolts, and arm/actuator bushings rotating around the
tubes. Top tubes carry actuator loads; bottom tubes carry arm-pivot loads.
Tube loads act on their sides, not along the transverse tube axes. Drum arms
now use aluminum box tube; the user-selected baseline is 2 x 2 inches outside,
1/16 inch wall (50.8 x 50.8 x 1.5875 mm). The previous round arms already used
the aluminum material inputs; CF properties apply to transverse rods only.

The following remain explicit assumptions, NOT measured or selected products:

| Choice | Initial model |
| --- | --- |
| Plate topology | Two identical plates, each with four radial and three perimeter strips joining S, front/rear A and front/rear B; three cutouts |
| CF properties | Density 1600 kg/m3, effective bending modulus 70 GPa, flexural strength 350 MPa; all provisional |
| Central pad | 30 mm radius; mass counted as a solid disk |
| Mount material reserve | CF OD/2 plus 3 mm |
| Minimum cutout inradius | 5 mm after allowing half strip width |
| Axle space at each end | 10 mm minimum between plate outside face and rocker bearing center |
| Metal properties | Retain existing 2700 kg/m3, 69 GPa, 240 MPa screening inputs |
| Strip buckling restraint | K=1, pinned weak-axis strip approximation |
| Load and strength factors | Retain 1.5 structural load multiplier and 2.0 strength/buckling factor |

Kaiser's 6061 sheet/plate data lists typical T6/T651 yield strength 276 MPa,
modulus 68.3 GPa and density 2700 kg/m3. The retained 240 MPa screening value
is not an actual purchased-stock allowable or a certified minimum. Actual CF
properties must come from the selected tube/layup, not fiber modulus alone.

Source: [Kaiser 6061 sheet/plate technical data](https://online.kaiseraluminum.com/depot/PublicProductInformation/Document/1015/Kaiser_Aluminum_6061_Sheet_Coil_and_Plate.pdf).
No experimental dataset or fitted relationship was introduced.

## Adjustable dimensions and geometry

Plate thickness and CF ID are fixed assumptions. Profile design variables add
common in-plane `plate_spoke_width`, `cf_tube_span`, `cf_tube_wall`,
`rocker_box_width`, `rocker_box_height`, `rocker_wall`, `axle_outer_diameter`,
`axle_wall` and `rocker_pivot_offset`. The old generic frame diameter/wall are
inactive fixed entries in this profile. Arm variables are `arm_box_width`,
`arm_box_height` (both 25–100 mm exploratory bounds), and `arm_wall` (1–5 mm).
Width is transverse; height is normal to the arm centerline in the x/z motion
plane and rotates with the arm. Width/height can optimize independently, so the
requested square baseline does not force every candidate to be square.
The retained section-validity reserve is min(width,height)/4-wall; hollow
section arithmetic also rejects 2*wall >= min(width,height).

The legacy `arm_tube_diameter` is inactive for these arms. Input files specifying
it must explicitly provide box width/height instead; the builder rejects a lone
old diameter rather than silently ignoring the requested section. The reference
model keeps its original round-arm input. Historical round-arm factories remain
available for explicit comparisons and the strict motor-interface regression.
`cf_tube_outer_diameter = cf_tube_inner_diameter + 2*cf_tube_wall` is reported.
All new starting values/bounds are exploratory; no claim of COTS availability.

Side-elevation nodes (x forward, z up, metres above original level ground):

```text
S   = (0, wheel_diameter/2 + rocker_pivot_offset)
A±  = (±actuator_pivot_x, top_pivot_height)
B±  = (±pivot_x, bottom_pivot_height)
edges = S-A±, S-B±, A+-B+, A--B-, A+-A-
lower boundary = B+-S-B- (no direct B+-B- member)
```

Lengths and directions are recomputed from these coordinates on EVERY evaluation.
The connection pattern is fixed; exact cutout dimensions are not fixed. There
is one shared width for all strips. Minimum lengths, mount/pad clearances,
three signed triangular cutout inradii enter a signed
`plate_geometry_margin`. Invalid geometry stays invalid even if finite forces
can be calculated for its schematic. `plate_geometry_valid` is diagnostic.

Revision 2 removes the direct B+-B- strip and the fourth/lower cutout. The hub
can now be at, below or above the lower mounts' elevation; its actual distance
from each tube mounting pad still has to fit the provisional pad radii and
ligament allowance. There is no hub/lower-mount vertical ordering requirement.
Triangles S-A+-B+, S-B--A-, S-A--A+ retain their orientation and a finite opening;
flipped/collapsed triangles remain infeasible. Connectivity is fixed across
the coplanar lower boundary, so no abrupt model switch is introduced.

The old filled rectangular chassis collision margin is diagnostic only for
this profile, as requested by the student. It remains active for the reference
tube-frame model. Removing it does not verify clearance to the actual plate
strips, tubes, equipment, arms or actuator bodies; those need CAD or a subsequent
member-envelope model. Both drum-wheel clearance screens remain enforced.
Dig/dump Froude caps are also diagnostic only in both profiles: torque/power,
startup energy and the existing RPM design bounds remain enforced. No additional
speed-dependent granular loss is assumed; retention and dumping remain unverified.

For transverse packaging:

```text
rocker_bearing_span = track_width - wheel_width - 2*wheel_rocker_gap - rocker_box_width
frame_packing_margin = (rocker_bearing_span-cf_tube_span-plate_thickness)/2
                       - minimum_axle_overhang
```

`track_width` remains WHEEL CENTER spacing; it is not silently redefined.
Plate spacing is plate CENTER spacing. Axle height can vary via the existing
offset; longitudinal pivot placement remains centered, as confirmed for the
hardware. Arbitrarily moving the axle longitudinally would require updating the
rocker/contact kinematics and load/reaction model; it is not a new independent
degree of freedom in this profile.

## Mass boundaries and placement

Structure mass includes four sloping half-rockers, four CF rods, two plates,
one transverse aluminum round axle and the existing joint/differential allowances.
Rocker mass remains a SUBSET of structure mass. Aluminum arm linear mass enters
LinkageGeometry's two-arm mass once. Four wheels and four drums/two drum motor
assemblies retain their existing ownership. GeometryMass sums the totals once.

Each plate inventory is density*thickness*(width*sum(member lengths)+pi*hub_radius2).
Overlapping strip junctions and hub material are deliberately retained: this is
a conservative line-member mass inventory, not exact CAD union volume. Holes,
real pads/fillets and connector masses need CAD/BOM replacement. Joint mass is
still an explicit allowance, including the undecided metal/printed pivot pieces.

StructureGeometry reports frame CG height and approximate line-member yaw inertia.
PlateMassProperties uses this placement instead of the old rectangular frame lump;
equipment and unequal arm/actuator distributions remain reference approximations.
For obstacle CG, straight sloping half-rockers have an aggregate centroid halfway
between shaft origin and mean wheel centers, distinct from wheel/drive CG. Equal
outboard wheel offsets cancel in that aggregate. Contact-track pose geometry and
average-angle differential are retained; no obstacle joint loads are calculated.

## Gravity loads and failure screens

Scope: both front/rear drum pairs fully loaded and at the SAME arm angle, on level
ground. Include the compact endpoint and 81 equally spaced travel-to-dig angles.
This is a sampled loading screen, not a proven continuous maximum. Cutting,
breakout, self-recovery, impact, obstacle reaction redistribution and asymmetric
drum cases are excluded from this STRUCTURAL profile at the student's request.
Existing excavation, actuator rating/energy and stability models retain their
own cutting/breakout/asymmetric cases; those were not weakened.

At each arm pose, moment equilibrium about B gives the actuator force on the
arm at D along D-A. Drum-pair gravity acts at C; uniform arm gravity acts at
its midpoint. The reaction on the chassis at A is the opposite actuator force;
the reaction at B is actuator force plus the drum/arm gravity vector. Their sum
is exactly the drum/arm gravity vector. Other hardware weight is included in
overall suspension loading but detailed actuator weight loading at the bushings
is omitted from this preliminary arm/CF calculation.

Each transverse tube sends half its point-force reaction to each plate. Its own
distributed weight is included. The tube flexure screen uses two half-span
cantilevers receiving half the midpoint force each:

```text
l = cf_tube_span/2
M_vector = (F/2)*l + q_vector*l2/2
displacement_vector = ((F/2)*l3/3 + q_vector*l4/8)/(E_CF*I_CF)
CF bending stress = norm(M_vector)*OD/(2*I_CF)
```

These are independent flexure screens. Tube-end bending moments are NOT applied
to the axial plate model; out-of-plane plate bending and actual rotational joint
restraint are outside it. It is not an assembled 3D equilibrium/stiffness model.
Ideal rotating bushings do not add a torque about the tube axis. CF allowable is
an effective flexural strength assumption, separately constrained from aluminum
yield. CF local wall instability, shear/delamination and inserts are unverified.

For the plate, solve axial elastic truss equilibrium with A=t*w and stiffness
E*A/l. Strip self-weight is distributed to its endpoints. Central support S is
a PIN. A mathematically exact zero-rotation gauge removes its free rigid rotation
without a fictitious spring. External moment about S and any nonzero gauge
reaction are rejected. The model is appropriate only for mirrored balanced
gravity cases. Additional mechanisms/near-degenerate matrices are rejected.

Signed member force N is positive tension. Check abs(N)/(t*w) against metal
strength/SF and compressed strips against:

```text
I_weak = w*t3/12
Pcr = pi2*E*I_weak/(K*l)2
plate_buckling_margin = min(Pcr/SF - max(-N,0))
```

This is NOT whole-plate buckling or junction bending validation. A monolithic
plate can redistribute forces by bending and buckle in a coupled mode omitted
by these strips. Compare to CAD shell static/buckling analysis and independent
physical testing when available; do not invent a correction factor.

Rockers use nominal wheel reaction (dry+payload)*g/4, multiplied by the retained
load factor. Remove colocated wheel/drive gravity, then include distributed
half-rocker gravity. Use actual sloped pivot-to-wheel length, box-section I and
force components. Axle overhang uses the distance between plate and rocker
bearing centers under the corresponding sprung-side weight. Axle calculation
omits middle-span flexure and joint rotation. Arms use point loads at C and D
plus distributed self-weight; bending extrema are found at endpoints, pin sides
and interior zero-shear points. Root restraint is a relative cantilever-flexure
approximation, not a solved compliant linkage. Box arm area/inertias are exact
outer-minus-inner sharp rectangles. Bending and deflection use the width-axis
inertia `(width*height^3-inner_width*inner_height^3)/12` and outer fiber height/2.
Euler uses the smaller of both principal inertias; lateral instability cannot
be credited with the larger in-plane bending inertia. Arm material retains the
existing aluminum density, modulus and yield screening inputs. Stock corner
radii, alloy/temper certification, local wall buckling, torsion, mount/weld
strength and actual arm collision envelopes require separate verification.

`structure_stress` is an eighth-norm envelope of ALUMINUM arm/rocker/axle/plate
stress; `cf_bending_margin` is separate. `structure_buckling_margin` is the minimum
of weak-axis plate-strip, rocker and arm Euler screens. Rocker/arm effective
length is twice their cantilever length; their compression comes from the same
applied forces used for stress. Separate plate_buckling_margin,
rocker_buckling_margin and arm_buckling_margin identify the controlling family.
These are profile-specific documented contract interpretations.

The unchanged 5 mm `allowable_deflection` limits an eighth-norm upper envelope
of rocker-end, axle-overhang and arm-tip relative flexure, plus the maximum
relative movement between upper and lower mounting points (plate translations
plus transverse tube displacement). Each is also reported separately. This is
not one actual chassis point or assembled-frame displacement. Other displacement
criteria can replace it only through an explicit future engineering decision.

Failed mechanisms/toggles produce `frame_load_solution_valid=0` and finite,
unambiguously FAILED stress/deflection/buckling margins. There is no fallback
to reference physics and no clipping into feasibility. Reported values then are
arithmetic-guard diagnostics, not predictions.

## Verification and limitations

Tests cover hand-solvable actuator loading, force/moment equilibrium, truss nodal
equilibrium and strain-energy/external-work equality, area scaling, section
properties, mass conservation, load/span trends, moving mounting points, geometry
and mechanism failures, insufficient strip thickness, scalar contracts, connected
N2 sources, reference preservation, mission energy and smooth total derivatives.
JSON and HTML include profile inputs, materials/evidence, new margins and local
displacements. A separate SVG shows the selected plate member topology.

Numerical feasibility remains conditional on the stated assumptions. No CF tube
has been selected or tested; no plate FEA or hardware validation was performed.
Connectors and recovery/cutting verification remain outside this profile.

Further primary references:

- [OpenSees axial truss example](https://opensees.github.io/OpenSeesDocumentation/user/examples/basicExamples/basicTruss.html).
- [MIT beam/column mechanics](https://www.ocw.mit.edu/courses/2-002-mechanics-and-materials-ii-spring-2004/bc25a56b5a91ad29ca5c7419616686f7_lec2.pdf).
- [NASA aluminum plates with cutouts](https://ntrs.nasa.gov/citations/19900019543).
