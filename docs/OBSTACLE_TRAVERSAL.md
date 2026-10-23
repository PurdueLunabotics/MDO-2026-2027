# Traversal: obstacles, clearance, slopes, and wheel unloading

The user requires traversal of a 0.40 m obstacle and a 0.50 m deep ditch while
fully loaded in the travel pose. The optimizer now enforces geometric reach,
gravity-support and parallel-cylinder drum–wheel clearance screens for the obstacle. These do not establish climbing
capability. Ditch traversal remains unverified. The existing `trench_traverse_margin` concerns the stationary
excavation trench, and `wheel_first_margin` is a wheel-diameter drum-clearance
placeholder. Neither port is repurposed by this work.

The additive traversal stability component now also checks 41 intermediate
single-wheel heights and nonnegative ideal-differential wheel reactions, plus
the separate 30-degree slope requirement. The endpoint screens below retain
their original meanings. Slope and intermediate-height equations are included below.

## Gravity support calculation

`rover/alternatives/obstacle_stability.py` contains the pure helper
`projected_support_margin(contacts, center_of_mass)`. All supplied coordinates
are metres in the world frame: x forward, y left, z up; gravity is vertical.
The `ObstacleStability` OpenMDAO adapter is selected through the new
`ModelSet.obstacle_stability` factory, active by default for the requested
requirement. It has no
calibration data, fitted coefficients, simulator dependency, or hardware
validation. Its domain is at least three finite, distinct, noncollinear projected
contacts, a finite CG, and contacts capable of upward vertical reaction forces.

For contact reactions N_i >= 0 and total weight W:

```text
sum(N_i) = W
sum(N_i*x_i) = W*CG_x
sum(N_i*y_i) = W*CG_y
```

Thus the projected CG must lie inside the convex hull of active contacts. For
each counterclockwise hull edge a->b the signed reserve is
`cross(b-a, CG_xy-a)/|b-a|`; the returned margin is the minimum edge reserve.
Positive means inside, zero is marginal, and negative rules out global static
equilibrium under these assumptions. Invalid/zero-area supports raise an error.
For an articulated mechanism, passing this global test does not establish
internal joint equilibrium or nonnegative wheel loads under its joint laws.

Height affects the answer through the posed CG and horizontal contact positions.
Simply changing one wheel's z coordinate while retaining level-ground CG and x/y
positions cannot create a valid obstacle test. Do not rotate wheel mass with the
chassis while simultaneously claiming those wheels stay on separate platforms.
The pose calculation must conserve total mass and transform the appropriate
chassis-mounted and suspension-mounted parts separately.

Tests verify the independent level-rectangle limit, moment balance for a triangle
with prescribed positive reactions, coordinate invariance, redundant contacts,
and failure domains. These are analytical/software checks, not traversal trials.

## Confirmed suspension and 0.40 m obstacle constraint

The user confirmed an average-angle differential and centered rocker pivots
estimated 0.10 m above the wheel-center line. Config inputs are
`obstacle_height=0.40 m` and `rocker_pivot_offset=+0.10 m`. The latter is an
editable estimate, not a frozen hardware dimension. Below-axis offsets may be
negative; the supported geometry requires `abs(offset)<wheelbase/2` throughout
the design bounds. The offset is fixed in the reference profile and a design variable in plate-frame.

Evaluate each raised wheel separately, with all four drums full and both arms
fixed at travel actuator length. Assume rigid symmetric side rockers, a rigid
shaft of length T=track_width, centered wheel spacing L=wheelbase, horizontal
contact platforms, point contacts vertically below wheel centers, and no yaw.
Wheel camber/contact-patch geometry, soil deformation and joint stops are omitted.
With d=rocker_pivot_offset, the raised-side rocker angle theta and shaft roll phi
are coupled:

```text
h = L*cos(phi)*sin(theta)
T*sin(phi) = h/2 + d*cos(phi)*(cos(theta)-1)
chassis angle about rolled shaft = theta/2  (other rocker angle is zero)
h_max = L*T / sqrt(T^2 + (L/2-d)^2)
obstacle_pose_margin = h_max - requested_height
```

Angles are radians; mirror their signs for the other three wheels. The pose
helper solves the coupled geometry deterministically on the forward-contact
branch. It preserves shaft length, rocker wheel spacing, pivot offset, and the
requested contact heights. Chassis pitch is a rotation about the rolled shaft,
not an average of world-frame elevation angles.

The closed maximum above applies to d>=0. For d<0, use
`q(theta)=(L/2)*sin(theta)+d*(cos(theta)-1)` and
`h(theta)=L*T*sin(theta)/sqrt(T^2+q(theta)^2)`. Find its first maximum where
`T^2*cos(theta)+d*q(theta)*(1-cos(theta))=0`, and invert the increasing branch
below that maximum. Below-axis pivots can reach their maximum before 90 degrees;
the code does not substitute the above-axis limit in that domain.

StructureGeometry now supplies `rocker_mass` (kg), the TWO longitudinal tubes,
already included in `structure_mass`. MassProperties supplies
`loaded_sprung_mass` (kg) and `loaded_sprung_cg_x/z` (m above original ground),
excluding wheel/drive masses and those rockers. Existing level-ground CG/inertia
outputs retain their prior lumped-frame convention unchanged. For obstacle
posing, the rocker mass lies at the two actual wheel-center midpoints. Thus its
level-ground total CG is slightly lower than the older lumped frame CG. No mass
is added twice. CAD replacements must explicitly supply these new ports and
preserve the mass subsets.

The mounted group rotates about the suspension shaft; wheel/drive masses follow
their four centers and uniform rockers follow their two midpoints. Combine their
moments to get the posed loaded CG. `obstacle_tip_margin` is the smallest signed
edge reserve across all four raised-wheel cases, metres, positive feasible.
The optimizer enforces both margins with the existing 0.001-scaled inward reserve
(0.0004 m for each); no old constraints or bounds were relaxed.

For unreachable requests, `obstacle_pose_valid=0` and the actual requested reach
margin remains nonpositive. A guarded pose just inside maximum reach keeps
optimizer arithmetic finite; its height is reported as
`obstacle_evaluated_height`. Tip/CG/angle diagnostics in that case are **guarded
values, not predictions for the requested height**. Reports show this explicitly.
For valid poses, evaluated height equals requested height. Wheel indices are
0=front left, 1=front right, 2=rear left, 3=rear right; symmetric cases may tie.

The generic reference 0.40 m wheelbase start has h_max=0.39331 m and fails reach.
A 0.46 m wheelbase at otherwise baseline inputs has positive reach and tip
reserves. Original failed solver iterates remain failed; see VERIFICATION.md.
Passing this static screen still leaves internal joint/load equilibrium,
climbing torque, traction, contact loss, other hardware interference and terrain failure
unverified. A rock's height alone also does not specify its approach shape.

## Parallel-cylinder clearance (default revision 2)

The current mechanism shares one roll rotation between the chassis-mounted
drums and both wheel/rocker assemblies. Wheel and drum axes remain parallel:
rocker/chassis pitch rotates about their transverse axes and does not change
the direction of those axes. Removing the shared roll leaves wheel centers at
`y=±track/2` and drums at `y=±(drum_length+drum_pair_gap)/2`.

For any common rigid rotation R and translation t,
`|(R*d+t)-(R*w+t)|=|d-w|`. Common roll therefore cancels from relative clearance. The contact-pose solver must still solve roll and rocker pitch together.
Changing track or requested rock height changes that articulation and can change
clearance even though a common rotation at a fixed articulated configuration cannot.

`rover/alternatives/parallel_cylinder_clearance.py` provides the default
`ParallelCylinderClearance` ModelSet factory for every frame/motor profile. In the common unrolled frame:

```text
radial_gap = hypot(delta_x, delta_z) - drum_radius - wheel_radius
axial_gap = abs(delta_y) - (drum_length + wheel_width)/2
signed_gap = hypot(max(radial_gap,0), max(axial_gap,0))
             + min(max(radial_gap,axial_gap),0)
```

Positive signed_gap is exact Euclidean surface separation for the ideal finite
parallel cylinders. Negative is minus the smallest relative translation needed
to separate overlapping solids. Overlapping side-view circles are insufficient
to establish collision when the axial intervals do not overlap. Evaluate all
16 pairs in each of the four endpoint poses. The existing 15 mm mechanism
clearance is subtracted once, with the same numerical reserve and unreachable
pose rejection as the reference. The legacy `obstacle_drum_wheel_gap_bound`
port is retained; its lower bound is now equal to the exact ideal-cylinder gap.
No other port, constraint, mass, energy or score convention changes.

No selection flag or replacement factory is needed:

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_optimized.json --output results/cylinder_evaluation.json
```

`python -m examples.parallel_cylinder_clearance` also evaluates the default
model and writes `results/parallel_cylinder_clearance/cylinders.html`, JSON,
SVGs and an N2 diagram. Neither evaluation edits the saved design inputs.
The former enclosing-envelope implementation and its selection path are removed.

Unreachable requests set `obstacle_clearance_pose_valid=0`, report the guarded
`obstacle_clearance_evaluated_height`, and limit the margin to at most
`requested_reach_margin - 0.0001 m`. This is a numerical invalid-domain marker,
not a physical allowance. Guarded gap values are not predictions at the
requested height and cannot make a failed pose feasible.

This analytical model has no fitted data or calibration. It assumes
parallel right circular envelopes, rigid shared roll and travel-position arms.
Independent wheel camber/steering, grousers outside the specified envelope,
deflection, other hardware and intermediate crossing poses are outside its
domain. A future suspension with independently tilted wheel axes needs a
general 3-D cylinder/CAD check. Tests cover independently calculated radial,
axial and corner distances, penetration, common-rotation invariance, mirrored
cases, invalid domains, guarded poses, native connections and smooth derivatives.

New reports include one front-left wheel-on-rock image using the same pose
helper, alongside a roll-free circular section and axial gap. The support
block's width/shape are illustrative; only the raised contact height is modeled.
The contact model still omits wheel-camber contact corrections. Unreachable
examples are explicitly marked as guarded. The check table records the default
model; this drawing shows the closest pair in its front-left example pose,
while the enforced check still covers all four raised-wheel cases.


## Ditch research and missing terrain definition

Depth alone cannot define ditch crossing. Obtain top width, bottom width or wall
slopes, edge shape, soil properties, and whether crossing means bridging the gap
or descending and climbing out. A narrow slot and a broad depression of the same
depth have different limiting mechanisms. Do not enforce wheel diameter >=0.50 m
as a substitute: the user's wheel dimension cap is still strictly below 0.36 m,
and that inequality is not a validated ditch traversal criterion.

The user describes the ditch as circle-like, but its width/radius is unknown.
For a circular-segment approximation with depth D and top width W,
`R=D/2+W^2/(8D)`. A semicircle would imply W=2D=1.0 m, but that shape is not
confirmed and is not assumed. Record the 0.50 m depth and unresolved profile in
report metadata; do not assign the ditch requirement a passing numeric margin.

Wang, Xu, Xu and Zhou, [Enhancing the Passing Ability of Unmanned Vehicles Using
a Variable-Wheelbase Driving System](https://doi.org/10.1109/ACCESS.2019.2936268),
IEEE Access 7 (2019), Section V, derives trench-width expressions from obstacle
climbing capability and wheel/soil geometry, then studies a six-wheel vehicle
with a movable middle axle. Equations 36/37 concern crossing width; they are not
an empirical depth-only law for this four-wheel side-rocker rover. The paper's
reported crossing verification uses ADAMS simulations, which does not calibrate
our hardware. [Full paper](https://www.researchgate.net/publication/335364908_Enhancing_the_Passing_Ability_of_Unmanned_Vehicles_Using_a_Variable-Wheelbase_Driving_System).

Song, Luo and Xie, [Design and obstacle-crossing analysis of a four-link
rocker-suspension planetary exploration robot](https://ms.copernicus.org/articles/15/137/2024/),
Mechanical Sciences 15 (2024), provides geometric and static obstacle analysis
and prototype experiments for a different six-wheel architecture. Its passive
differential relation averages the two primary rocker angles (Eq. 7); it is a
useful explicitly stated suspension assumption, not evidence that our chassis
uses that mechanism.

Next step after terrain/hardware confirmation: solve a quasi-static sequence of
contact states, and check joint reach, body/drum clearance, nonnegative contact
loads, available drive torque and traction at every state. Use measured trials
to validate that screen before fitting a fast surrogate for optimization.

## Slope and intermediate-height stability

The student requires a 30-degree slope and a 0.40 m rock, with all four drums
loaded and both arms fixed at the travel actuator setting. They explicitly
retained the original **one-wheel-raised** rock case. Slope and rock are separate
requirements, without combined slope/rock or front/rear wheel-pair raises.

`rover/alternatives/traversal_stability.py` supplies pure analytical helpers and
the `ModelSet.traversal_stability` adapter. `PlateTraversalStability` uses the
plate-profile rocker mass placement. The original endpoint obstacle component
and reference factories retain their previous physics. No fitted data or
experimental calibration was introduced. These are necessary quasi-static
screens within stated contact assumptions, not traversal certification.

## Planar slope: global gravity support

`slope_angle` is fixed, in radians, default pi/6. Valid domain: [0, pi/2).
The full-load travel CG is rotated with the rover onto a uniform slope, with
its level-ground height becoming height normal to the plane. Paired masses
make lateral CG zero. Support points are wheel-center x/y stations projected
onto the plane; finite tread patches, compliance and terrain deformation are
omitted. Both rockers have the same pitch on a uniform plane.

For downhill azimuth beta, gravity intersects the slope plane at:

```text
p_x = CG_x - CG_z*tan(alpha)*cos(beta)
p_y =      - CG_z*tan(alpha)*sin(beta)
pitch reserve = wheelbase/2 - abs(CG_x) - CG_z*tan(alpha)
roll reserve  = track_width/2 - CG_z*tan(alpha)
slope_tip_margin = min(pitch reserve, roll reserve)
slope_tip_angle = atan2(min(wheelbase/2-abs(CG_x), track_width/2), CG_z)
```

The two reserves bound every azimuth, including uphill, downhill and either
sideways orientation. Reserves are metres along the plane. This is GLOBAL
support with adequate friction/holding torque assumed. It does not solve
individual slope wheel loads, traction allocation, slip, climbing torque or
structural loading on an incline. A positive margin is not proof of climbing.

## Rock: articulation and differential reactions

The existing `raised_wheel_pose` supplies shaft position, chassis pitch/roll
and contact points. Confirmed centered rockers have a normal pivot offset and
an average-angle differential. Horizontal platforms exert vertical reactions.
No approach profile, side contact force, wheel camber, joint stop or impact is
modeled. Check all four raised-wheel choices at 41 heights including zero and
the requested maximum: 10 mm steps for a 40 cm lift.

Sprung hardware rotates about the shaft. Equal wheel/drive masses follow their
centers. Uniform plate-profile half-rockers put their centroid halfway between
shaft pivot and wheel midpoint; reference uses its older wheel-midpoint
approximation. Denote that rocker centroid fraction by a: 0.5 for plate-frame,
1 for reference. Existing sprung/wheel/drive/rocker groups conserve total mass;
rocker mass remains a subset of structure mass, never counted twice.

The old support hull asks whether ANY nonnegative vertical reactions can
balance whole-rover gravity. The new solve also requires the articulated
mechanism's load sharing. Ideal virtual work for
theta_body=(theta_left+theta_right)/2 gives equal left/right rocker pitch torque
contributions. Gear friction, preload, compliance and torque capacity are omitted.

For normals N in FL, FR, RL, RR order, contact x/y relative to the shaft,
total mass M and posed CG relative to the shaft:

```text
sum(N_i)     = M*g
sum(x_i*N_i) = M*g*CG_x
sum(y_i*N_i) = M*g*CG_y
x_FL*N_FL + x_RL*N_RL - x_FR*N_FR - x_RR*N_RR
    = g*(wheel_drive_mass + a*rocker_mass)/4
          * (x_FL+x_RL-x_FR-x_RR)
```

The fourth row subtracts each side's wheel/drive/rocker gravity moments before
equating rocker pitch moments. Projecting onto the common rolled shaft supplies
the same cos(roll) factor on both sides, which cancels. Sprung gravity enters
through the posed CG. A conditioned 4x4 solve returns signed forces; negatives
are not clipped or replaced with another pose.

`rock_sweep_tip_margin` [m] is the minimum hull edge reserve across the height
grid and four wheel choices. `rock_sweep_wheel_load_margin` [N] is the minimum
normal reaction anywhere on that grid, with independent worst-height/wheel
diagnostics. A negative normal would require a wheel to pull on the platform:
the prescribed four-contact pose cannot hold equilibrium. This is a **wheel-
unloading failure**, not proof of complete rollover. Post-lift contact sequence
and possible stable three-contact poses are not calculated.

`obstacle_pose_margin` still rejects unreachable requested height. Guarded
values below reach have validity false and failed path margins; they are not
predictions at the requested height. Singular supports raise an error instead
of substituting different physics. Original endpoint obstacle ports retain
their meanings; path outputs/constraints are additive.

Finite sampling is a limitation. A dense-grid regression test compares 401 heights
per wheel (1 mm spacing) for a synthetic case; the study does not run that dense
check for every candidate. Agreement is numerical evidence, not a
continuous-height proof or a real crossing test.
Arbitrary rock profiles, simultaneous raised wheels, contact loss, impacts,
slosh and combined rock/slope require additional models or physical tests.

## Interfaces, numerical reserves and validation

- New factory: `ModelSet.traversal_stability`; new fixed input `slope_angle`
  [rad]. No new design variable or mass boundary. Native scalar CG/mass sources
  come from MassProperties, WheelMass, GeometryMass and StructureGeometry.
- Three new active signed margins: `slope_tip_margin` [m],
  `rock_sweep_tip_margin` [m], `rock_sweep_wheel_load_margin` [N].
  JSON/HTML/N2 show their values, worst cases, evidence and real connections.
- Existing 0.001-scaled reserves give 0.3 mm, 0.4 mm and 0.5 N above zero.
  These are numerical buffers, **not hardware uncertainty allowances**. A
  design close to unloading needs a chosen engineering reserve and measured
  mass/CG/joint tolerances before hardware acceptance.
- Tests cover slope moment limits, all-heading bounds, hand-solved flat axle
  loads, gravity/differential moment balance, mirrored/dense grids, positive
  hull with negative wheel load, mass placement, invalid domains and derivatives.
- Earth gravity, dimension caps, structural allowables, power, mission route,
  score, energy boundaries and existing collision screens are retained.

Fundamental basis: [OpenStax force and torque equilibrium](https://openstax.org/books/university-physics-volume-1/pages/12-1-conditions-for-static-equilibrium).
The [MIT/NASA NIAC rover analysis](https://www.niac.usra.edu/files/studies/final_report/161Dubowsky.pdf),
Appendix B, separates body and rocker/bogie equilibrium and assumes equal
left/right pitch contributions in B-20. Its six-wheel geometry is not used:
the four-contact equations above are derived for this ideal two-rocker rover.
Neither source provides a validated safety margin for the student's hardware.
