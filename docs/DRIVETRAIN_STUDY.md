# Drivetrain study (rev E)

Fixed evaluation of `design_drivetrain.json` against `design_baseline.json`
(main). Not reoptimized and not hardware validation. Report:
[design_reports/drivetrain.html](design_reports/drivetrain.html).

```text
python main.py --inputs design_drivetrain.json --output results/drivetrain.json
```

## Changed inputs

All other inputs are identical to `design_baseline.json`. The ex-dep linear
actuator (lengths, speed, force) is locked and unchanged.

| Input | Main | Rev E | Reason |
|---|---|---|---|
| `wheel_diameter` | 0.359 m | 0.3556 m | 14 in wheel |
| `wheel_width` | 0.18 m | 0.1524 m | 6 in wheel |
| `wheelbase` | 0.60 m | 0.515 m | 20.3 in, front-to-rear wheel centers |
| `track_width` | 0.57 m | 0.515 m | 20.3 in, left-to-right wheel centers |
| `rocker_pivot_offset` | 0.0508 m | 0.1016 m | 4 in, wheel axle to rocker axle |
| `axle_outer_diameter` | 0.033 m | 0.03175 m | 1.25 x 0.125 in rocker axle |
| `axle_wall` | 0.001 m | 0.003175 m | 1.25 x 0.125 in rocker axle |
| `drive_gearbox_mass` | 0.697 kg | 1.0245 kg | +0.3275 kg: REV MAX 90 REV-21-2120 (312.5 g) + 15 g adapter |
| `obstacle_height` | 0.4 m | 0.2 m | 20 cm rock, **pending chief engineer sign-off** |
| `pivot_x` | 0.1524 m | 0.1099 m | ex-dep moved inward with the axles |
| `actuator_pivot_x` | 0.0762 m | 0.0337 m | ex-dep moved inward with the axles |

### Ex-dep inward shift

The axles each moved inward by (0.60 - 0.515)/2 = 0.0425 m. The whole ex-dep
linkage (arm pivot and actuator pivot) moved inward by the same 0.0425 m, with
arm length, pivot heights and attachment unchanged. The stowed, lifted (travel)
and dig drum positions are therefore identical to main relative to the wheels.

This needs `actuator_pivot_x` below its previous 0.05 m lower bound, so the
bound in `rover/config.py` is now 0.03 m (user decision). The front and rear
actuator top pivots end up 67 mm apart center-to-center (152 mm on main);
**confirm the clevises package in CAD.**

Holding the 0.05 m bound instead was checked and rejected: with the actuator
attached at the drum (f = 1) the drum cannot reach both main positions (best
fit about 16 mm off, and `slope_tip_margin` fails at -2.6 mm), and an exact
match needs f = 1.028, which the reference model does not allow.

## Results vs main

| Output | Main | Rev E | Change |
|---|---|---|---|
| Construction points | 2204.8 | 2211.4 | +6.6 |
| Mass points | 1260.5 | 1269.2 | +8.7 |
| Energy points | 944.3 | 942.1 | -2.2 |
| Dry mass | 52.65 kg | 52.68 kg | +0.03 kg |
| Cycle time | 147.85 s | 146.74 s | -1.11 s |
| Run energy | 23.96 W*h | 24.20 W*h | +0.24 W*h |
| Whole-cycle check (6 cycles) | 2183.2 | 2178.4 | -4.8 |
| Stowed length | 1.478 m | 1.393 m | -0.085 m |
| Stowed height | 0.742 m | 0.742 m | 0 |

Constraint margins (positive means satisfied):

| Margin | Main | Rev E | Status |
|---|---|---|---|
| `empty_slope_traversal_margin` | -0.340 | -0.340 | FAIL on both |
| `loaded_slope_traversal_margin` | -0.119 | -0.121 | FAIL on both |
| `turn_traction_margin` | -63.9 N | -91.7 N | FAIL on both, worse on rev E |
| `slope_tip_margin` | 29.0 mm | 0.92 mm | pass, nearly zero |
| `tip_stability_margin` | 46.3 mm | 19.6 mm | pass |
| `mobility_stability_margin` | 270.7 mm | 241.6 mm | pass |
| `obstacle_pose_margin` | 149.8 mm | 292.9 mm | pass |
| `obstacle_tip_margin` | 176.4 mm | 198.8 mm | pass |
| `obstacle_drum_wheel_margin` | 0.02 mm | 42.6 mm | pass |
| `wheel_clearance_margin` | 33.4 mm | 35.1 mm | pass |

All other margins pass on both designs. Both designs are INFEASIBLE under
the modeled constraints because of the three slope/turn failures above.

Effect of the ex-dep shift alone (rev E before vs after): `tip_stability_margin`
4.3 -> 19.6 mm, `turn_traction_margin` -128 -> -92 N, `slope_tip_margin`
0.99 -> 0.92 mm.

## Notes

- `slope_tip_margin` is set by CG height and the 20.3 in track. With a square
  footprint and mirrored drums, pitch and roll margins are equal:
  `0.2575 m - cg_z * tan(30 deg)`. Moving the drums fore/aft does not change
  it; only a lower CG or wider track does.
- The ex-dep shift shortens the robot but does not lower it. Stowed height is
  set by the stowed drum top (0.563 + 0.180 m).
- The obstacle margins improve mainly because `obstacle_height` dropped from
  0.4 m to 0.2 m, pending sign-off.
