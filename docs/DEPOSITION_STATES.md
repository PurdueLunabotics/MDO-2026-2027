# Separate travel and deposition poses (schema 8)

`travel_actuator_length` retains its metre-valued command for loaded outbound
travel and empty return. New design variable `deposition_actuator_length` has
the same 0.403098–0.606298 m current search bounds. Files omitting it initialize it to
their resolved travel length when the two configured initial guesses match;
an explicitly distinct configured deposition guess is respected. The optimizer may vary them
independently. The reference and plate-frame inventories use the same four
drums, two drum drives and two actuators in both poses, without adding mass.

The assumed cycle is loaded travel, both pairs moving together into deposition
pose, first pair dumping, half-loaded 180-degree turn, second pair dumping,
then both pairs returning empty to travel pose. Both pairs remain mirrored at
the common deposition command throughout the two dumps and turn. Half-loaded
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
remains diagnostic, as previously requested.

Unprefixed `empty_*`, `loaded_*`, `half_*` mass-property outputs continue to
mean travel pose. New `deposition_{empty,loaded,half}_{cg_x,cg_z,yaw_inertia}`
outputs use deposition pose. All CG coordinates are metres and yaw inertias are
kg m² about the chassis-center vertical axis, with `Izz=sum(Ilocal+m(x²+y²))`.
Height alone does not change Izz. A higher CG can reduce dynamic stability.
Travel slope/rock/route checks use travel mass properties. The half-loaded
deposition turn uses deposition CG and inertia, including its offset-force and
dynamic stability screens. Calculated traversal timing still optimizes only
the outbound/return route profiles.

The default static asymmetric dump screen now uses the actual deposition CG:

```
full_pair_payload = payload_mass / 2
CGx = full_pair_payload * deposition_x / (dry_mass + full_pair_payload)
tip_stability_margin = min(wheelbase/2 - abs(CGx), track_width/2)
```

The inventories calculate this CG, so a replacement inventory can supply a
different mass distribution. The same physical wheelbase must satisfy travel
and deposition. This gravity-only level-ground screen has no height term;
the separate turn screen does. Explicit legacy excavation preserves its
maximum-extension `TipStability` component. Custom tip factories are preserved.

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
uses the existing full-load capability-sweep force (including breakout
allowance); this is conservative for deposition, not a measured trajectory.
Two actuators move in parallel in each deposition leg. The existing sequential
front/rear excavation timing is unchanged.

PA-04 uses its existing chart helper: raising speed/current at the conservative
force envelope, lowering at 0.39 inch/s and 14 A, and electrical energy
`2*12*(Iraise*traise + 14*tlower)`. Efficiency and reference idle losses are not
applied again. Unsupported loads remain rejected by existing force/curve
margins; guarded finite energies are diagnostics only. Equal commands produce
exactly zero extra duration, energy and peak. The zero-motion boundary is
nonsmooth; derivative checks use unequal, smooth poses.

Mission adds deposition duration and energy once, with electronics over the
whole added duration. Peak power takes the maximum over distinct phases;
deposition arm peak is not simultaneous with the deposition turn or dumping.
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
