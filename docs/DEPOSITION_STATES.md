# Separate travel and deposition poses (schema 8)

`travel_actuator_length` retains its metre-valued command for loaded outbound
travel and empty return. Design variable `deposition_actuator_length` uses the
selected hardware's command bounds: 0.352298–0.504698 m for 6-inch PA-04 or
0.403098–0.606298 m for 8-inch PA-04. Files omitting it initialize it to
their resolved travel length when the two configured initial guesses match;
an explicitly distinct configured deposition guess is respected. The optimizer may vary them
independently. The reference and plate-frame inventories use the same four
drums, two drum drives and two actuators in both poses, without adding mass.

The cycle is loaded travel, both pairs moving into deposition pose, and the
first dump. The half-loaded rover then reverses 0.5 m, makes the 180-degree
point turn, and drives forward 0.5 m before the second dump. Reverse and
forward are relative to the rover's heading at each move. Both drum pairs
hold the deposition command throughout and return empty to travel pose after
the second dump. Half-loaded
arm repositioning requires an interlock; it is outside this model. Dump RPM,
duration and soil-discharge assumptions are unchanged. No minimum discharge
height is asserted without receiving-container geometry.

## Kinematics and mass properties

The existing outward linkage branch maps each command to its own axle x/z.
With the usual geometry, shortening the actuator raises and draws in the drums.
The sampled geometry, leverage and loaded plate-frame gravity envelopes cover
the interval from the shortest travel/deposition command through maximum
extension. Samples carry the existing arc-length clearance allowance.
Deposition has signed triangle reach, command-at-most-maximum-extension, and
drum-bottom transport-clearance margins. Existing travel, wheel-first, recovery,
force, size and mass requirements remain active. Actuator body/catalog fit
remains diagnostic, as previously requested; the selected PA-04 command
endpoints are enforced separately.

Unprefixed `empty_*`, `loaded_*`, `half_*` mass-property outputs continue to
mean travel pose. New `deposition_{empty,loaded,half}_{cg_x,cg_z,yaw_inertia}`
outputs use deposition pose. All CG coordinates are metres and yaw inertias are
kg m² about the chassis-center vertical axis, with `Izz=sum(Ilocal+m(x²+y²))`.
Height alone does not change Izz. A higher CG can reduce dynamic stability.
Travel slope/rock/route checks use travel mass properties. The half-loaded
deposition turn uses deposition CG and inertia, including its offset-force and
dynamic stability screens. Calculated traversal timing still optimizes only
the outbound/return route profiles.

Each move starts and ends at rest and uses the existing deposition speed and
acceleration caps (`travel_speed * turn_speed_fraction` and
`acceleration * turn_accel_fraction`). `turn_time` [s] now sums all three
moves. Mobility charges both straight moves' rolling work and kinetic energy
once at the drive efficiency, includes drive idle power for the added time,
and applies the straight motor, power, traction and stability checks. The
straight moves use the empirical rolling law used for the point turn. Outbound
and return route distance and timing are separate.

The asymmetric deposition screen uses the half-loaded CG and the largest
wheel-shaft torque demand from the three moves:

```
full_pair_payload = payload_mass / 2
W = (dry_mass + full_pair_payload) * gravity
CGx = full_pair_payload * deposition_x / (dry_mass + full_pair_payload)  # reference inventory
b = track_width / 2
R = wheel_diameter / 2
P = rocker_load_factor * W / four_wheel_contact_area
Frolling = (rolling_base + P / rolling_pressure_scale) * W
Mscrub = scrub_coefficient * W * wheelbase / 4
alpha = acceleration * turn_accel_fraction / b
Tturn = (Frolling * b + Mscrub + deposition_half_yaw_inertia * alpha) * R / (4 * b)
Tstraight = (Frolling + (W / gravity) * acceleration * turn_accel_fraction) * R / 4
Twheel = max(Tturn, Tstraight)
pitch_reserve = wheelbase/2 - abs(CGx) - 2 * Twheel / (W/2)
tip_stability_margin = min(pitch_reserve, b)
required_tip_distance = 0.025  # m, student requirement 2026-11-07
reported_constraint_reserve = tip_stability_margin - required_tip_distance
```

The two wheel drives on each rocker apply reaction torque in the same
direction. During the point turn, the directions oppose across the rover,
shifting load away from opposite wheels. The straight moves cover both pitch
directions. This pitch screen applies both shaft reactions to each rocker and
assumes half the weight per side. Dividing the worst-side moment by the
side weight expresses the remaining support as an equivalent edge distance.
The 25 mm minimum applies to this distance. Historical records keep their
saved values and thresholds.

The inventories supply the CG and yaw inertia. The torque calculation uses the
same empirical rolling, scrub and acceleration demand as Mobility. The pitch
screen estimates drive reaction at the nominal pose. The separate mobility
screen accounts for height-dependent lateral load transfer. Combined per-wheel
reactions, suspension motion and braking transients remain unverified. Explicit
legacy excavation retains its gravity-only maximum-extension screen.

## Added actuation accounting

`DepositionActuation`, selected through `ModelSet.deposition_actuation`, uses
a pure helper and adds total two-actuator outputs, separately from excavation:
`deposition_actuation_time` [s], `deposition_arm_energy` [J] and
`deposition_arm_peak_power` [W]. Let s be the absolute command-length change
and h the absolute axle-height change. For the generic model:

```
Wempty = (drum_mass/2 + arm_linear_mass*physical_arm_length²/(2*arm_length))*g
Wloaded = Wempty + payload_mass*g/2
T = 2*s/actuator_speed
E = 2*(Wloaded+Wempty)*h/actuator_efficiency + 2*actuator_idle_power*T
Ppeak = 2*required_actuator_force*actuator_speed/actuator_efficiency
        + 2*actuator_idle_power
```

The first leg is fully loaded, the return empty. Absolute gravitational work
retains the existing conservative no-regeneration/lowering assumption. Peak
uses the existing full-load capability-sweep gravity force; extraction resistance
and dynamic loads are excluded. This is conservative for deposition, not a measured trajectory.
Two actuators move in parallel in each deposition leg. Schema 9 also makes
normal front/rear excavation positioning parallel at their actual digging
commands; see [MOVING_EXCAVATION.md](MOVING_EXCAVATION.md). Deposition costs
remain separate and are charged once.

PA-04 uses its existing chart helper: raising speed/current at the conservative
force envelope, lowering at 0.39 inch/s and 14 A, and electrical energy
`2*12*(Iraise*traise + 14*tlower)`. Efficiency and reference idle losses are not
applied again. Unsupported loads remain rejected by existing force/curve
margins; guarded finite energies are diagnostics only. Equal commands produce
exactly zero extra duration, energy and peak. The zero-motion boundary is
nonsmooth; derivative checks use unequal, smooth poses.

Mission adds deposition duration and energy once, with electronics over the
whole added duration. Peak power takes the maximum over distinct phases;
deposition arm peak occurs separately from chassis motion and dumping.
Existing excavation arm/work envelopes are retained.

## Evidence and limits

No coefficients were fitted and no hardware or soil data were added. Tests
check independent gravity-work balance, CG moment balance, height-invariant yaw
inertia, phase isolation, zero-motion limits, electrical accounting, failure
domains, connections and smooth derivatives for both actuator/frame profiles.
These are software and physical-limit checks, not predictive validation.
Dynamic arm reactions, terrain motion during dumping, soil slosh, actual
actuator packaging and controller behavior remain unverified. The HTML report
adds a deposition view, SVG export, mass-property comparison and transition
costs; schema 8 snapshots require reevaluation of historical results.
