# Loaded obstacle stability and unverified ditch traversal

The user requires traversal of a 0.40 m obstacle and a 0.50 m deep ditch while
fully loaded in the travel pose. The optimizer now enforces geometric reach,
gravity-support and conservative drum–wheel clearance screens for the obstacle. These do not establish climbing
capability. Ditch traversal remains unverified. The existing `trench_traverse_margin` concerns the stationary
excavation trench, and `wheel_first_margin` is a wheel-diameter drum-clearance
placeholder. Neither port is repurposed by this work.

The additive traversal stability component now also checks 41 intermediate
single-wheel heights and nonnegative ideal-differential wheel reactions, plus
the separate 30-degree slope requirement. The endpoint screens below retain
their original meanings. See [TRAVERSAL_STABILITY.md](TRAVERSAL_STABILITY.md).

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
the design bounds. The offset is not a design variable by default.

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

The unchanged 0.40 m wheelbase baseline has h_max=0.39331 m and fails reach.
A 0.46 m wheelbase at otherwise baseline inputs has positive reach and tip
reserves. Original failed solver iterates remain failed; see VERIFICATION.md.
Passing this static screen still leaves internal joint/load equilibrium,
climbing torque, traction, contact loss, other hardware interference and terrain failure
unverified. A rock's height alone also does not specify its approach shape.

## Conservative drum–wheel clearance (revision 1)

`ModelSet.obstacle_clearance` selects `ObstacleClearance` from
`rover/alternatives/obstacle_clearance.py`, active by default. All existing model
factories, design bounds, masses, route and score equations retain their meanings.
This additive integration introduces four scalar outputs and one enforced margin;
reports retain schema version 2 and include the new fields and model evidence.

The linkage supplies `travel_x` and `travel_y` (its legacy vertical coordinate).
Level-ground drum centers are `(±travel_x, ±(drum_length+drum_pair_gap)/2, travel_y)`.
Subtract the original shaft height `wheel_diameter/2+rocker_pivot_offset`, apply
pitch then roll, and add the posed shaft origin. The same posed wheel centers
used in the suspension geometry are used for clearance.

Each cylinder is enclosed by a sphere of radius `sqrt(R^2+(length/2)^2)`.
For wheels, axial length is wheel_width; for drums it is drum_length and R is the
outer cutting radius. The gap lower bound is the minimum center distance minus
the two sphere radii, across 16 pairs in each of four raised-wheel cases (64 pairs).
It is a sufficient conservative screen, not an exact cylinder-intersection test.
Positive bounds guarantee separation of the modeled envelopes. Negative bounds
are inconclusive about actual cylinder collision and are conservatively rejected.

At reachable poses:

```text
obstacle_drum_wheel_margin = obstacle_drum_wheel_gap_bound - mechanism_clearance
```

The existing mechanism_clearance is used without a new hardware assumption
(default 0.015 m). The margin uses the existing 0.4 m constraint scale and
0.0004 m optimizer inward reserve. No earlier constraint is relaxed.

The strict pure helper rejects unreachable poses. The adapter instead explicitly
evaluates the existing guarded height, sets `obstacle_clearance_pose_valid=0`,
reports `obstacle_clearance_evaluated_height`, and limits its margin to at most
`requested_reach_margin-0.0001 m`. This offset is a numerical invalid-domain
failure marker, not a calibrated clearance or obstacle-height requirement. It
also prevents a guarded boundary pose from appearing feasible within reporting
tolerance. Guarded gap bounds are not predictions at the requested height.

This checks the prescribed static endpoints only. No intermediate crossing path,
wheel/frame deformation, terrain contact, or hub/shaft/arm/frame interference is
modeled. The reference dimensions and travel linkage must be replaced or checked
against CAD and actual fully loaded articulation before hardware conclusions.
Tests use analytic limits and explicitly synthetic geometry; there is no fitted
relationship or experimental calibration dataset.

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
