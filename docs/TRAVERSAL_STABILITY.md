# Fully loaded slope and one-wheel articulation stability

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

Finite sampling is a limitation. Optimized results are separately checked at
801 heights per wheel (0.5 mm spacing), recorded in VERIFICATION.md. Agreement
is numerical evidence, not a continuous-height proof or a real crossing test.
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
