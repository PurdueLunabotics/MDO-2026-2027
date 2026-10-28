# Drivetrain study (rev E)

Fixed evaluation of `design_drivetrain.json` against `design_baseline.json`
(main). Not reoptimized and not hardware validation. Report:
[design_reports/drivetrain.html](design_reports/drivetrain.html).

```text
python main.py --inputs design_drivetrain.json --output results/drivetrain.json
```

## Changed inputs

All other inputs are identical to `design_baseline.json`.

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

## Results vs main

| Output | Main | Rev E | Change |
|---|---|---|---|
| Construction points | 2204.8 | 2207.6 | +2.7 |
| Mass points | 1260.5 | 1266.3 | +5.8 |
| Energy points | 944.3 | 941.2 | -3.1 |
| Dry mass | 52.65 kg | 52.80 kg | +0.15 kg |
| Cycle time | 147.85 s | 146.74 s | -1.11 s |
| Cycles | 6.087 | 6.133 | +0.046 |
| Run energy | 23.96 W*h | 24.22 W*h | +0.26 W*h |
| Whole-cycle check (6 cycles) | 2183.2 | 2174.7 | -8.5 |

Constraint margins (positive means satisfied):

| Margin | Main | Rev E | Status |
|---|---|---|---|
| `empty_slope_traversal_margin` | -0.340 | -0.340 | FAIL on both |
| `loaded_slope_traversal_margin` | -0.119 | -0.121 | FAIL on both |
| `turn_traction_margin` | -63.9 N | -128.4 N | FAIL on both, worse on rev E |
| `slope_tip_margin` | 29.0 mm | 0.99 mm | pass, nearly zero |
| `tip_stability_margin` | 46.3 mm | 4.25 mm | pass, small |
| `mobility_stability_margin` | 270.7 mm | 240.3 mm | pass |
| `obstacle_pose_margin` | 149.8 mm | 292.9 mm | pass |
| `obstacle_tip_margin` | 176.4 mm | 198.8 mm | pass |
| `obstacle_drum_wheel_margin` | 0.02 mm | 67.7 mm | pass |
| `wheel_clearance_margin` | 33.4 mm | 77.6 mm | pass |

All other margins pass on both designs. Both designs are INFEASIBLE under
the modeled constraints because of the three slope/turn failures above.

## Notes

- `slope_tip_margin` and `tip_stability_margin` drop to a few millimetres
  with the smaller wheelbase and track, so they have almost no reserve left.
- `turn_traction_margin` roughly doubles its deficit (-64 N to -128 N). The
  cause has not been traced to a specific input yet.
- The obstacle margins improve mainly because `obstacle_height` dropped from
  0.4 m to 0.2 m. That gain depends on the pending sign-off for the 20 cm rock.
- Test suite: `python -m unittest discover -s tests` ran 171 tests, OK.
