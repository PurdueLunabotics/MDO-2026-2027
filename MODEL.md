# Rover MDO equations and assumptions — version 2

This is an OpenMDAO conceptual sizing model under Earth gravity. FEASIBLE means only that its declared numeric screens pass. All unmeasured coefficients are editable assumptions, not calibrated BP-1 data. Read [model cards](docs/MODEL_CARDS.md), [development guide](docs/MODEL_DEVELOPMENT.md), and [experiment plans](docs/EXPERIMENTS.md) before replacing a discipline.

## Requirements and architecture

Four wheels with side rockers and skid steering; four bucket drums on front/rear shaft-linked pairs; four wheel motors, two drum motors, two linear actuators. Each fixed arm joins a lower chassis pivot to a drum shaft; its actuator joins a separately located upper pivot to a selectable point along that arm. Front/rear geometry is mirrored. Each wheel/drum diameter and width/length remains strictly below 360 mm.

The supplied 9/17/26 Lunabotics Guidebook sections 13.3/13.4/13.9 supplied the 1.50 x 0.75 x 0.75 m stowed envelope, 80 kg inspection mass (without regolith), 2.50 m deployed height and penetration-force requirement. Section 17.1 supplies the 4.4 and 1.5 score multipliers. Section 11.1 describes BP-1 and an approximately 0.45 m pit. The supplied requirements CSV identifies additional manual checks. Confirm current event interpretations independently.

The one-way route is 2/3/3/1 m with 90/45/45-degree turns, `traversal_time_one_way` seconds each way in default fixed mode (45 s by default). Optional calculated mode predicts separate loaded outbound and empty return times from bounded BWR/motor profiles; the fixed-time input is inactive. Outbound turns are assumed left; return reverses sequence and yaw signs. A separate 180-degree deposition turn carries only the front pair's remaining payload.

## Data flow and interfaces

Optional standard X44 hardware is selected with `--motors x44-standard`.
Both motor-power sizing variables become inactive reference diagnostics;
drive ratio is fixed at 100:1 and drum ratio remains exploratory. The existing
discipline components supply mechanical work and peak power to a shared Gearmotor and
selected motor provider; mass components use a selected assembly inventory.
There are no X44-specific copies of excavation, mobility or relocation.
Electrical energy and peak power use provisional combined efficiencies (drive
0.72, drums 0.70), applied once; no electrical time history or RMS current is
predicted. Torque-speed/current capability is checked separately.
See [the interface/file map](docs/X44_MOTORS.md#shared-helper-and-replacement-api). Power margins in this profile
are instantaneous curve screens, not continuous ratings. Current limits are
undecided (zero explicitly disables the current cap, not the voltage screen).
Full equations, source manifest, domains and missing validation are in
[X44_MOTORS.md](docs/X44_MOTORS.md). Slope climbing is still unverified.

The student expanded the exploratory wheelbase upper bound from 0.50 to 0.65 m;
the 1.50 m stowed-length requirement remains enforced.

StructureGeometry -> LinkageGeometry, with WheelMass and DrumMass calculated independently -> GeometryMass -> Excavation. GeometryMass now sums the wheel/drum outputs with drive, battery, structure and linkage masses; its reference result is numerically unchanged. Excavation feeds LinkageLoads, MassProperties, TrenchTraversal and TipStability. StructureLoads follows linkage loads; SiteRelocation and Mobility follow mass properties; Mission combines their time, energy and constraints. Explicit tube dimensions, actuator/motor sizes and battery capacity avoid hidden mass-sizing loops. Automatic structural resizing would require a coupled solve.

ModelSet factories replace individual components. New disciplines keep CONFIG_INPUTS, COUPLED_INPUTS, OUTPUT_UNITS and DEPENDENCIES beside the equations; older ones use dependencies.py. Named scalar ports use SI, except explicit rpm and W*h. Dependency declarations include both sides of every branch. Global axes are x forward, y left, z up; legacy linkage side-elevation y means global z.

## Linkage and actuator loads

The saved student baseline selects the frozen PA-04 900 lbf / 12 V profile.
It uses continuous catalog stroke sizing, chart-based speed/current and mass,
and adds the configurable 0.1613408 m installation allowance to both hardware
end lengths. Generic reference defaults below remain available. Geometry emits
`lowering_time`; loads emit total `actuation_time`, allowing payload-dependent
raising speed without a circular dependency. See [the actuator equations,
source manifest, updated port contract and limitations](docs/PA04_ACTUATORS.md).

Wheel-center track width is now a design variable alongside wheelbase. Both are screened by the stowed envelope, structural loads and mobility constraints.

The actuator attachment now uses two signed metre-valued coordinates relative to the drum axle: lin_act_side_arm_offset and lin_act_top_arm_offset. With arm direction u=(C-B)/L and its upward normal n=(-u_z,u_x), the pin is D=C+side*u+top*n. Positive side extends past the axle; positive top is above the arm. Rear geometry mirrors the front. Circle closure uses radius hypot(L+side,top), then rotates through -atan2(top,L+side) to recover the drum position. arm_length remains the pivot-to-axle distance. Both offsets are design variables with exploratory bounds [-0.15,0.15] m; generic defaults are zero and the requested baseline is +0.05 m for both. The old fraction is removed from live configuration.

The same eccentric pin feeds virtual-work actuator force, plate-frame equilibrium, mass placement and report drawings. For positive side offset, a uniform tube continuation adds actual mass, gravity work, yaw inertia, physical length and beam load effects. Top offset transfers an eccentric couple into the tube; its normal bracket is assumed rigid and massless pending hardware/CAD data. Actuator catalog stroke/load domains, safety factors, triangle/toggle, mounting, pit-floor, envelope and existing drum/wheel clearance screens remain. At the user’s request actuator_body_margin is diagnostic only: the body/catalog installed-length reserve is computed but excluded from optimization and feasibility. The sweep retains 81 arm-angle samples. One-time electrical loss accounting is retained. Moving excavation now charges sequential positioning with a conservative full-range arm allowance and zero lowering/cutting overlap; see the excavation section below. See [complete equations, interface migration, domains and limitations](docs/ARM_OFFSETS.md).

## Forward-feed excavation, storage and inlet depth

The default `ModelSet.excavation` is `MovingExcavation`. The front pair cuts
to `inlet_depth` below the original ground and the rear pair to twice that
depth in the same tracks. Each pair removes one fresh layer of that depth.
The same `inlet_depth` is the radial scoop protrusion; storage excludes it.
The inlet spans the full axial width `drum_length` of each drum. There is no
independent opening-width or partial-width engagement setting in this model.
`drum_diameter` remains the OUTER cutting envelope.
For R=D/2, d=inlet_depth, L=drum_length and n=scoops_per_revolution:

```text
front_dig_y=R-d; rear_dig_y=R-2*d; trench_depth=2*d
inlet_width=L
storage_capacity=4*pi*(R-d)^2*L*usable_volume_fraction*fill_fraction
payload_mass=storage_capacity*stored_soil_density
fresh_volume=payload_mass/(soil_density*capture_fraction)
feed_distance=fresh_volume/(4*L*d)
feed_per_scoop=60*feed_speed/(n*drum_rpm)
feed_speed_limit=d*n*drum_rpm/60
feed_margin=d-feed_per_scoop
```

Storage is supplied by the required fresh lane, rather than the old stationary
endpoint intersection. Storage and in-situ soil densities remain distinct.
The front pair starts first, the rear enters the same lane after the station
separation, and the front is raised when full while the rear finishes. Feed
drive distance is productive lane length plus station separation; this avoids
assuming a prepared entrance or crediting the front layer twice. No lowering,
endcap, collapse or recutting soil is credited. Fresh-lane area and relocation
remain explicit planning checks.

The current model represents one full-width bucket-drum configuration. Alternate intake geometries need their own measured soil-supply and storage models.

The kinematic feed constraint requires forward advance per scoop <= `inlet_depth`;
equivalently fresh flow `4*L*d*feed_speed` cannot exceed assumed scoop throughput
`4*L*d^2*n*drum_rpm/60`. Overfeed retains its negative margin and raw
force demand. This uncalibrated volume screen is not a measured plunge limit.
The report shows the layered cut and feed-speed/RPM boundary. `dig_overlap_time`
is zero. Full-range arm work/loads remain a conservative capability allowance,
with sequential positioning time charged for the staged pass.

Cut force=`cut_pressure*4*L*feed_per_scoop`, with the full drum width engaged.
The radial geometry screen remains `R/4-inlet_depth >= 0`; the obsolete
half-opening-depth heuristic is retired with the separate opening setting.
The compatibility output `cut_depth` equals `inlet_depth` and supplies linkage
breakout work; it is not an independent input. Feed wheel drive contributes
electrical work/peak power to digging and has separate motor, power and traction
reserves after tool unloading. Both imposed operating poses must be reachable.
The additional `axle_ground_margin=-extended_y` requires maximum-extension axle
reach to ground level, preserving the option of plunge/recovery without modeling
those modes' soil forces, driveability or motor-body clearance. The old stationary
model is no longer active by default. See [equations, domains, staging and
limitations](docs/MOVING_EXCAVATION.md).

## Drum speed, torque and energy

```text
omega=rpm*pi/30; Fr=omega^2*R/g
capture=capture_fraction
lift=internal_lift_factor
internal_torque_full=payload*g*R*lift
```

These are UNCALIBRATED constants, not a high-speed loss law. At the student's
request, dig/dump Froude caps are no longer enforced. `drum_froude` and the legacy
`dig_froude_margin`/`dump_froude_margin` against 0.30/0.20 are diagnostics only,
excluded from feasibility and optimizer constraints. There is no extra assumed
speed-dependent capture/lift loss; motor power, torque, spinup energy and RPM
design bounds still constrain operation. High-speed retention/dumping remains
unverified. Empirical adapters must still reject extrapolation outside their
own measured domains. Fr=1 is an acceleration scale, not a universal threshold.

Drum gear ratio is independent of operating RPM. Continuous output torque=P_rated/omega_motor_rated*ratio*gear_efficiency, also bounded by a linear motor torque-speed envelope with stall twice rated and no-load twice rated speed. Peak cutting/spinup torque, continuous mechanical power and a service factor constrain the drive. Replace this generic envelope with actual voltage/current/thermal/gearbox data.

Both drum pairs take equal fresh layers during the staged forward pass; dumping uses one pair at a time. Conservative ring spin inertia is (drum assembly mass+payload)*R^2. Startup has duration, I*omega/time torque and I*omega^2/2 energy. No regenerative braking is credited. Peak uses full internal load. Dig energy uses a simple half-full mean internal torque through cutting; unloading uses a linear emptying approximation. dig_energy/dump_energy are electrical J including startup and idle. dig_power/dump_power are PEAK electrical W; do not multiply them by duration to estimate energy. Apply drivetrain efficiency exactly once.

## Level-ground ground clearance

The user requires 8 inches (0.2032 m) after 20% wheel-radius compression and a
1-inch (0.0254 m) differential-bar allowance below the lower axle. With nominal
wheel radius R, the existing bottom-pivot height is already above ground;
the rocker axle is at R + rocker_pivot_offset. Each height locates the axle
center, so its own outer radius must be subtracted to locate its underside:

```text
bottom_axle_radius = cross_tube_inner_diameter/2 + cross_tube_wall
rocker_axle_radius = axle_outer_diameter/2
bottom_clearance = bottom_pivot_height - bottom_axle_radius - 0.20*R - 0.0254
rocker_clearance = R + rocker_pivot_offset - rocker_axle_radius - 0.20*R - 0.0254
ground_clearance = min(bottom_clearance, rocker_clearance)
ground_clearance_margin = ground_clearance - 0.2032
```

Equivalently, subtract R from the bottom height before applying the user's
offset formula `min(bottom_offset-bottom_axle_radius, rocker_offset-rocker_axle_radius)
+ 0.8*R - 1 inch`. Subtract the respective radii before choosing the lower surface.
The wheel radius is counted once and both existing coordinate definitions are retained.
`GroundClearance` is selected through `ModelSet.ground_clearance` for both frame
profiles. Reports include both axle radii and underside clearances, their minimum,
and the active signed margin. Plate-frame radii follow the existing cross-tube wall and
axle diameter design variables. The reference profile exposes the same nominal
shaft dimension defaults as fixed clearance inputs, without changing its mass
model. The three allowances/requirement remain fixed configuration inputs.
This is a level-ground screen using user-specified assumptions, without hardware
calibration. The radial deflection applies only here; the existing diameter-based
contact-pressure deflection input, nominal drawings and terrain/stability models
are unchanged. Slope/rock clearance, unequal wheel compression and actual bar
shape are not predicted by this screen.

## Structure and mass

The actuator force acceptance uses `actuator_safety_factor=1.0` by user choice:
calculated actuator demand is compared directly with its rated force. This
does not change the separate structural load factor or structural safety factor.
The saved baseline retains `fill_fraction=0.9`.

The optional `--structure plate-frame` profile replaces the following reference
screens with coordinate-driven two-force plate strips, transverse aluminum cross-tube flexure,
aluminum box rockers and drum arms, and a round suspension axle. It fixes the user-specified
6.35 mm sheet and 25.4 mm cross-tube ID; common spoke width and cross-tube wall/span are optimized.
Only mirrored fully loaded level-ground gravity is in its structural load scope.
Cutting/recovery and local/whole-plate instability require separate verification.
cross-tube properties remain provisional. Its mass placement, output interpretations,
geometry failure domains and unchanged 5 mm envelope are documented in
[PLATE_FRAME.md](docs/PLATE_FRAME.md). `ModelSet()` retains the reference below.

Plate-frame drum arms use `arm_box_width` (transverse), `arm_box_height` (depth
normal to the arm centerline in its motion plane), and `arm_wall`. The baseline
is 0.0508 x 0.0508 m outside with 0.0015875 m wall (2 x 2 x 1/16 inches).
For outer width b, height h, inner bi=b-2t and hi=h-2t:

```text
A = b*h - bi*hi
I_bend = (b*h^3 - bi*hi^3)/12
I_lateral = (h*b^3 - hi*bi^3)/12
arm_linear_mass = tube_density*A
sigma = abs(N)/A + abs(M)*h/(2*I_bend)
Pcr = pi^2*tube_youngs_modulus*min(I_bend,I_lateral)/(2*arm_length)^2
```

Both arms enter linkage mass once. Bending/deflection use I_bend; Euler buckling
uses the weaker axis. Existing aluminum material inputs and safety/load factors
are retained, without certifying an alloy/temper. Sharp corners and uniform
walls are assumed; local wall buckling, torsion, joints and section-envelope
collisions remain unverified. The reference round-arm model remains available
through the original ModelSet factories.

Plate revision 2 uses seven strips and three triangular openings; its lower
boundary runs through the suspension hub, allowing the hub and lower mounts
to share an elevation. The filled rectangular chassis collision screen remains
active only in the reference profile, and is diagnostic only in plate-frame.
Actual frame/equipment interference remains unverified; drum-wheel clearances,
mount packaging and dimension limits are retained.

Frame tubes are two longitudinal wheelbase members, one track-width crossmember and two pivot towers from axle height to the upper pivot, including its horizontal offset. Arms have a separate section family. For diameter d, wall t and di=d-2t:

```text
A=pi*(d^2-di^2)/4; I=pi*(d^4-di^4)/64; J=2I
mass=density*A*length + explicit joint/differential allowances
```

The previous 14 kg frame-inclusive fixed mass becomes computed tube mass plus a 6 kg equipment-only allowance. Arm linear mass enters linkage mass exactly once. Wall<=diameter/4 is a section validity screen. Wheel/drum shells retain areal-mass estimates, motors scale with rated power, gears with rated output torque, battery with pack-specific energy. WheelMass outputs the total for four bare wheels; DrumMass outputs four drums plus the two drum motor/gear assemblies. GeometryMass adds both outputs once to dry mass, so alternatives must preserve these boundaries or revise the shared accounting contract explicitly.

Elementary load envelopes cover longitudinal supports, crossmember torsion under a single loaded pair, actuator towers and arms. Loaded weight, cutting, extraction and a structural load factor enter these cases. Stress combines axial+bending and torsional shear by von Mises; tip deflection uses FL^3/(3EI), buckling uses conservative effective length 2L. Eighth-norm aggregation smoothly upper-bounds maximum stress/deflection. Safety-factor and deflection margins size sections. This is not assembled-frame FEA: joints, stress concentrations, local shell buckling, fatigue and exact load sharing remain outside it.

## Mass properties and fixed-time mobility

Sum local component Izz plus m*(x^2+y^2) about chassis-center yaw. Include wheel/drive units, structure/equipment, linkage, drums and payload at their actual longitudinal/lateral stations. The reference linkage center uses the midpoint between the mean chassis pivots (A+B)/2 and mean distal endpoints (C+D)/2; local yaw inertia retains an equivalent rod between those means. The distal actuator endpoint is the eccentric pin D. Explicit axial tails are removed from the core linkage lump and placed at their actual centroid/inertia. Zero offsets reproduce the previous tip attachment; CAD should resolve the unequal arm and actuator masses. Full/empty states are symmetric; front-pair-only load shifts CG. Equivalent cylinder inertia and payload CG R/3 below the shaft are approximations needing CAD/material-distribution data. A displaced CG's translational kinetic contribution is included by using inertia about the actual yaw axis.

Each reference segment stops/starts: peak=min(requested_speed,sqrt(distance*acceleration)), time=distance/peak+peak/acceleration. Scale total to traversal_time_one_way with velocities /k and accelerations /k^2. Motor upgrades do not shorten the schedule. Turning wheel-rim distance=abs(yaw)*track/2; effective mass=Izz/(track/2)^2. Scrub moment=coefficient*mass*g*wheelbase/4. Energy sums resistance work and rest-to-rest kinetic energy without regeneration.

Half-loaded turns reserve traction for tangential/centripetal CG acceleration. Axle transfer limits equal-wheel-torque traction through the lighter axle; pressure uses the heavier axle and a rocker factor. Pitch/roll support reserves are checked. Rolling/contact-pressure and scrub laws are uncalibrated. No terrain/controller simulation, detailed per-wheel friction circle or wheel rotational kinetic energy is claimed.

`--traversal-mode calculated` selects an optional `ModelSet.mobility` factory.
It searches a fast common-acceleration profile for each loading state, with
straight BWR slip/torque and motor speed/power limiting peak speed. Existing
speed/acceleration inputs are upper bounds. The same profiles determine route
time, work and peak power; there is no post-hoc timing rescale. Point turns
retain the empirical rolling/scrub/yaw law and their existing traction/support
screens. Deposition and relocation timing remain reference calculations. No
physical constraint is automatically waived. The current GRC-1 transfer and
calibration limitations remain, and predicted timing is not hardware validation.
See [the full algorithm and optional output schema](docs/CALCULATED_TRAVERSAL.md).

The default [rigid-wheel Bekker-Wong-Reece model](docs/BEKKER_WONG_REECE.md)
integrates Bekker normal pressure and Janosi-Hanamoto shear with a continuous
Wong-Reece contact distribution. It solves normal equilibrium for sinkage and
net axle-force demand for slip using independently published loose GRC-1 soil
fits and assumed stress-shape coefficients. Straight-segment torque/work replaces
the old rolling law without double-counting compaction or kinetic work. Point
turns and existing pressure/stability screens retain their equations. The earlier
straight-slip interpolation and its executable/data pipeline have been removed.

`ModelSet.slope_traversal` adds two independent quasi-static uphill checks:
`empty_slope_degrees=30` for dry mass and `loaded_slope_degrees=20` for dry mass
plus all four drums' payload. Both are editable degree-valued configuration
inputs, fixed during optimization. Each wheel receives `m*g*cos(alpha)/4` normal
load and must supply `m*g*sin(alpha)/4` net force. Acceptance requires a solved
slip **strictly below 1**, without a 98% numerical cutoff or extra slip reserve.
Imposed slip=1 is supported only as the locked-wheel force limit. A failed force
solve gives an explicitly invalid slip=1 sentinel, negative normalized force
shortfall margin, and `solution_valid=0`; it never passes via report tolerance.
For valid solutions the margin is `1-slip`. Slope motor limits and required speed
are outside this slip-only criterion. The existing 30-degree full-load static
support constraint remains separate.

The student selected this model as a conservative design assumption. Published
coefficients are unchanged and no rover data are fitted. The independent
larger-rover accuracy gate still fails; the documentation retains quantitative
errors, failures, soil preparation mismatch and missing prototype calibration.

## Uneven-load tip stability

The default static screen checks one front/rear drum pair full, the other empty, with both pairs at the common commanded deposition pose. Each full drum holds one quarter of `payload_mass`. Symmetric dry hardware cancels longitudinally, so the reference inventory gives `CG_x = (payload_mass/2)*deposition_x/(dry_mass+payload_mass/2)` and `CG_y = 0`. The signed margin is `min(wheelbase/2 - abs(CG_x), track_width/2)`, using the wheel-center support rectangle. The mass-properties component supplies the CG so replacement inventories can change its distribution. Front-full and rear-full cases are mirrored. `tip_worst_mask` is 3 (front pair full); mask 12 is equivalent.

The half-loaded turn also uses deposition CG and yaw inertia. Loaded outbound and empty return use travel mass properties. The two pairs move together while fully loaded before dumping, stay at the deposition command through both dumps/turn, then return empty to travel. The added transitions have explicit time and energy costs. Control software must prevent other half-loaded arm poses; that interlock is unverified. The static screen omits soil shifting, dumping reaction, slopes and individual wheel unloading; separate route-turn dynamic screens remain enforced. Explicit legacy excavation retains the maximum-extension `TipStability` component. See [DEPOSITION_STATES.md](docs/DEPOSITION_STATES.md) for the schema 8 contracts and equations.

## Fresh-site motion

The requested fully loaded 0.30 m obstacle now has geometric reach and gravity
support constraints through `ModelSet.obstacle_stability`. The user confirmed
centered side-rocker pivots and an average-angle differential, with an editable
+0.10 m pivot height estimate above the axle line. The posed model preserves
rocker/shaft dimensions, transforms mounted mass separately from wheel/rocker
mass, and checks all four raised-wheel cases. Signed `obstacle_pose_margin` and
`obstacle_tip_margin` are metres, positive feasible. Unreachable poses are
flagged and retain a failing requested-height margin; finite guarded diagnostics
are explicitly labeled. See [equations and domain](docs/OBSTACLE_TRAVERSAL.md).
These necessary static screens do not verify the actual climbing path, joint
loads/stops, collision, traction or dynamics. The 0.50 m deep, circle-like ditch
remains unverified because width/radius is unknown. Existing wheel-diameter
screens are not repurposed to claim compliance with that requirement.

`ModelSet.obstacle_clearance` defaults to `ParallelCylinderClearance` for every
frame and motor profile. It checks the signed separation of the ideal finite
wheel/drum cylinders across all 64 pairs in four raised-wheel endpoint poses.
Common roll cancels from relative clearance; rocker pitch relative to chassis
pitch still matters. Combine the circular x/z separation with finite axial
y-interval separation in the common unrolled frame. The pose solver retains
roll to satisfy prescribed contact heights. The margin is the minimum signed
cylinder gap minus mechanism_clearance (default 0.015 m). Negative gaps indicate
overlap of the ideal solids. Unreachable poses retain flagged guarded geometry
and failing margins. The previous enclosing-envelope implementation is removed.
Port names, units, margin sign and numerical reserves are unchanged; the legacy
`obstacle_drum_wheel_gap_bound` now equals the ideal-cylinder signed gap.
Independent camber/steering, deflection, other hardware and the actual crossing
trajectory remain unverified. See [equations and domain](docs/OBSTACLE_TRAVERSAL.md#parallel-cylinder-clearance-default-revision-2).

Empty relocation translates to a neighboring fresh lane with two 90-degree turns; rest-to-rest time, kinetic/scrub/rolling energy, torque and level-ground traction are charged. Lane offset must exceed total drum width. Each cycle reserves the rectangle enclosing the complete forward pass and both drum stations, including the center gap; cumulative area must fit available_excavation_area. A real site planner must still check maneuver space and legal-zone topology. Trench traversal currently requires wheel_diameter >= trench_depth; this is a geometric placeholder, not a traction or exit simulation. These area/path values are planning assumptions, not asserted competition rules.

## Mission and numerical verification

`ModelSet.traversal_stability` additionally enforces 30-degree full-load travel
support using the gravity-line intersection with the planar support rectangle.
It samples each single-wheel raise from zero to obstacle_height at 41 heights,
retaining rocker kinematics. Both the global hull and nonnegative wheel normals
under ideal differential pitch-torque balance are required. Three new margins
are additive to the endpoint obstacle and uneven-dump contracts. Slope and rock
are separate cases; failed four-contact poses are not replaced with invented
three-wheel poses. See [traversal equations](docs/OBSTACLE_TRAVERSAL.md#slope-and-intermediate-height-stability).

Cycle sums dig, dump, travel_time (2*traversal_time_one_way in fixed mode, outbound+return in calculated mode), deposition turn, conservative sequential excavation actuator positioning, parallel loaded/empty deposition pose changes, relocation and alignment. Moving excavation sets lowering/cutting overlap to zero. Electrical energy sums dig/dump, mobility, arms, relocation and electronics over the cycle. Battery usable energy and peak C-rate are constrained. cycles=run_time/cycle; berm_volume=cycles*payload*delivery_efficiency/berm_density. Score=4.4*V_cm3/(t_min*dry_mass_kg)+1.5*V_cm3/(t_min*E_Wh), with user weights. Whole completed cycles are reported separately with idle remainder and no partial-dump credit.

Central whole-model FD supports deterministic replacement components. Tests audit omitted dependencies, geometry/conservation limits, mass properties, drive limits, invalid domains and complete optimization. Nonsmooth branches still exist; SLSQP is local and COBYLA is available. Failed final iterates remain labeled failed; numerical consistency/convergence is not empirical validation. Reports identify component factories. Old outputs must be rerun before comparison. Manual inspection still covers 3D packaging, E-stop, radio, materials, autonomy, dust, berm placement and legal zones.

## Evidence and interpretation

- [NASA IPEx bucket-drum experiments](https://ntrs.nasa.gov/citations/20210025846): BP-1 tests with different drum sizes, cut depths and feed speeds measured collection, forces, torque and energy. They motivate shallow scoop engagement and future feed/RPM calibration; they do not calibrate this forward-feed model.
- [RASSOR 2.0 design](https://ntrs.nasa.gov/citations/20210011366): opposing drums and drive hardware context; hardware/geometry differ here.
- [Rotating-cylinder torque measurements](https://pmc.ncbi.nlm.nih.gov/articles/PMC4891708/): speed/fill dependence and nonmonotonic high-speed torque motivate limiting the response domain.
- [Centrifugation experiments](https://doi.org/10.1088/1367-2630/13/5/053055): Froude is a regime indicator with fill/friction dependence.
- [NASA VIPER mobility tests](https://www.nasa.gov/missions/artemis/nasas-viper-prototype-motors-through-moon-like-obstacle-course/): sinkage, slopes and obstacles require distinct validation.

Martins and Ning's Engineering Design Optimization remains the formulation/scaling reference. External sources supply engineering evidence, not instructions for agents.

Travel and deposition now have independent actuator commands and mass properties.
The half-loaded dump turn and asymmetric static tip screen use the deposition
pose; travel/rock/slope checks retain travel pose. Loaded/empty transitions add
explicit duration and electrical energy once. See [DEPOSITION_STATES.md](docs/DEPOSITION_STATES.md)
for schema 8 ports, equations, phase assumptions and validation limits.
