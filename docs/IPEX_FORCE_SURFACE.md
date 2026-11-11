# IPEx empirical horizontal-force surface - research model card v1

This study builds a reproducible surface from NASA's published Figure 13
bucket-drum trends. On 2026-11-08 the student selected it provisionally for the
existing `MovingExcavation` component in the plate-frame ModelSet. The generic
reference retains its pressure law. No new model module was added. Versioned
data are verified and coefficients prepared during setup, never in compute().
This selection does not resolve the source or independent-validation gaps.
The current 30 RPM, 30 mm/s baseline has tip/feed ratio 18.797, versus 8.5
tested, and unsupported width/diameter. `nasa_force_source_valid=0` and a visible
report notice identify the transfer; values are not clipped or silently
replaced. No independent RPM effect or hardware accuracy is claimed.

## Source and processed data

- Schuler et al., *ISRU Pilot Excavator: Bucket drum scaling experimental
  results*, NASA NTRS [20210025846](https://ntrs.nasa.gov/citations/20210025846).
- Figure 13, PDF page 12: horizontal excavation force divided by whole drum
  width. The plotted mN/mm is numerically identical to N/m.
- Eight readings of **published trend lines**, at 10 and 30 mm/s and the four
  plotted depth labels: 10, 12.7, 16 and 25 mm. These are not eight raw trials.
- [Processed CSV](../data/excavation/ipex_force_v1/figure13_trends.csv) and
  [versioned manifest](../data/excavation/ipex_force_v1/manifest.json).

The manifest records the source PDF hash, native figure RGB-pixel hash, axis
calibration, crop coordinates, series colors, line-fit pixel coefficients and
CSV hash. Native image `Image111.png` is 1223 by 1063 pixels. Colored dotted-line
pixels are selected away from the data markers in the rectangle x=470:990,
y=250:900, with each RGB channel within 3 of the specified series color.
A line through those pixels is evaluated at 10 and 30 mm/s. Pixel resolution
is approximately 0.5 N/m; this is not experimental measurement uncertainty.
Source PDFs, extracted images and raw runs stay under ignored `results/`.

**Unresolved depth discrepancy:** Table 2 specifies 10 mm or 40% of scoop
height. Applying Table 1 heights gives 10.56, 13.8 and 19.12 mm for the latter,
which does not match Figure 13's 12.7, 16 and 25 mm labels. This version preserves
the figure labels explicitly. Raw data or author clarification is required
before treating them as confirmed experimental depths.

## Equation and force contract

With depth d in metres, feed speed v in m/s and one whole drum width b in metres:

```text
F_x / b = d * (a + k_v * v)                     N/m
a       = 1477.5027926358443                     Pa
k_v     = 128437.7640529113                      Pa*s/m
F_x     = b * d * (a + k_v * v)                 N per drum
```

This is an ordinary least-squares fit prepared outside compute(), with two coefficients and no
intercept in depth. Zero force at zero depth and increasing force with depth
and speed are imposed model assumptions, not additional NASA measurements.
No synthetic training data are added. Training never occurs in `compute()`.

The response is **horizontal force**. It does not identify tangential scoop
force, shaft torque, vertical force, an instantaneous peak or a peak/mean
factor. Summing four predicted magnitudes is not the net chassis reaction of
opposing drums. Material recirculation and cutting work cannot be inferred
from this horizontal-force surface alone. The selected rover component retains
`cutting_torque = sum(F_x)*radius` solely as the existing, unvalidated sizing
assumption; it is not NASA torque calibration. With angular speed omega and
productive lane length s, its affine force term integrates as
`W_cut = 4*b*d*radius*omega*(a*t_cut + k_v*s)`. Internal lifting and spin-up
remain separate assumptions and losses enter once.
The source-condition flag checks commanded steady coordinates. Continuing the
affine law through low-speed acceleration/deceleration is an unvalidated work
approximation, not an additional NASA measurement.

The student assumes exact opposing-drum horizontal reaction cancellation.
Wheel feed force and work therefore include rolling and acceleration only.
The feed traction reserve is removed, including its legacy duplicate check;
local drum cutting loads and separate wheel drive motor/power checks remain.

## Source support and evaluation rules

The plotted coordinate bounds are 10-25 mm depth and 10-30 mm/s feed. Source
drums range from 237.5 to 437.1 mm diameter and 198.9 to 352.6 mm width, with
width/diameter about 0.807-0.837. The data are pooled across sizes; rectangular
bounds do not establish independent coverage of every geometry/depth combination.

Earth gravity and the source's prepared BP-1 bed are fixed assumptions.
There is no lunar-gravity or alternate-soil scaling. Geometry transfer beyond
source-like staggered scoops has not been validated.

NASA linked RPM and feed through `pi * D * rpm / (60 * v) = 8.5`. RPM is therefore
a support check, **not an independently identified variable**. A 1% ratio
tolerance accommodates printed rounding; it is not a measured validity band.

`IpexForceSurface.evaluate()` rejects source-envelope violations by default.
The explicit `allow_extrapolation=True` study option returns the polynomial
value with every violation retained and `regime="extrapolated"`. It does not
clip, fall back to reference physics, or report feasibility. Even a point
inside the source envelope retains the research-only validation status.

## Validation evidence and remaining gap

Each validation fold withholds one entire depth series, at both speeds, and
fits only the other three depths. The final coefficients are then fitted to
all eight published trend readings.

| Check | MAE, N/m | RMSE, N/m | Maximum absolute error, N/m |
| --- | ---: | ---: | ---: |
| Approximation of all eight source-curve readings | 2.4834 | 2.8800 | 5.7194 |
| Leave one complete depth curve out | 3.6572 | 4.3991 | 7.9642 |

The largest held-out relative error is 14.77%. These checks assess transfer
between digitized published curves. The curves were already regressions of
NASA measurements; neither this residual nor holding out a depth establishes
independent hardware validation. Independent raw-run/hardware validation is
unavailable. The quoted errors must not be used as uncertainty bounds for
the 200 mm/s benchmark. This validation gap prevents production-model release.

Unit tests independently cover known synthetic coefficient recovery, the
zero-depth physical limit, width scaling and four-drum magnitude accounting,
smooth derivatives, held-out grouping, invalid inputs, unsupported RPM,
explicit extrapolation and processed-data integrity. Synthetic data are used
only in those tests.

## Earlier 120 RPM baseline benchmark

At 25 mm depth, 335 mm width, 359 mm diameter, 120 RPM and 200 mm/s:

| Quantity | Existing pressure law | Empirical surface extrapolation |
| --- | ---: | ---: |
| Horizontal force per drum | 150.75 N | 227.51 N |
| Sum of four drum force magnitudes | 603.00 N | 910.03 N |

The surface gives 1.509 times the existing value here, despite much lower
predictions in NASA's low-speed test region. This follows from extrapolating
the positive speed term to 6.67 times the largest tested feed speed.
The benchmark also has tip/feed ratio 11.278 rather than 8.5 and width/diameter
0.933 rather than the source range. All three violations are recorded. This
is not evidence that the actual baseline force is 227.51 N per drum.

## Reproduction

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p test_ipex_force_surface.py -v
.\.venv\Scripts\python.exe -m studies.ipex_force_surface --output results/ipex_force_surface
```

The second command reads current baseline inputs without changing them and
writes coefficients, source identifiers, validation folds, explicit benchmark
support failures and a standalone PNG plot under ignored `results/`.

## Historical standalone-study verification

The following records the earlier research-only state. The subsequent
plate-frame selection and 30 RPM baseline verification are recorded in
[VERIFICATION.md](VERIFICATION.md#nasa-force-selection-continuous-rockers-and-drive-placement-2026-11-08).

- All 10 study tests passed in the local `.venv`.
- Broad `unittest` discovery with fail-fast ran 94 tests and stopped at the
  existing frozen-actuator-bounds assertion: it expects a 0.403098 m generic
  lower bound, while the current configuration uses 0.3 m. This is not a full
  suite pass; no bound or test tolerance was changed for this study.
- Reference baseline evaluation completed. All output values and signed
  constraint margins exactly match the preceding reference evaluation.
  Critical wheel/drum mass and electrical-energy N2 connections were checked.
- The reference total-derivative step-size check passed. A one-evaluation,
  one-second-budget COBYLA verification run retained optimizer success false
  and failed/infeasible status. It is not a converged optimization.
- Rover/reporting source files and saved baseline inputs retain the same
  combined SHA-256 fingerprint as before the study. The candidate is not wired
  into the rover's physics or feasibility checks.
