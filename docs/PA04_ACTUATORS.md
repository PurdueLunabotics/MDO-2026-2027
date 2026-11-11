Current attachment interface: [axle-relative side/top offsets, revision 2](ARM_OFFSETS.md). The fraction-based attachment selection described in older sections below is historical; live models use signed metre-valued offsets, with automatic axial tube continuation and an assumed rigid, massless normal bracket.

# PA-04 900 lbf actuator model — revision 1

The student froze the **900 lbf, 12 V PA-04** selection on 2026-10-27.
`design_baseline.json` selects `actuators: "pa04-900"`. Since 2026-11-07 catalog
stroke is a discrete hardware choice: **6, 8, 9 or 10 inches**. `ModelSet()` and the reference
configuration retain generic actuator physics for comparisons. Select the
hardware with `--actuators pa04-900` or
`build_problem(actuators="pa04-900")`. It composes with either structure and
motor profile through `ModelSet.linkage` and `ModelSet.linkage_loads`.

Select 8 inches with `--set actuator_catalog_stroke=0.2032`, or 9 inches with
`--set actuator_catalog_stroke=0.2286`, or 10 inches with
`--set actuator_catalog_stroke=0.254`. All four commanded lengths must lie
inside the selected interval. Existing defaults and saved cases are unchanged.

## Source and domain

Four user-supplied screenshots contain PA-04 current/load and speed/load plots,
a hole-to-hole length table and a stroke/weight table. Transcriptions, units,
source filenames, SHA-256 hashes, approximations and validation errors are in
`data/actuators/manifest.json`. Original manufacturer revision/URL, test
temperature, direction and duty cycle were not supplied. The yellow 900 lbf
curves alone define this profile; other force variants are excluded.

Current is read as 4 A at zero load and 14 A at 900 lbf. Approximate speed is
0.47 inch/s at zero load and 0.39 inch/s at rated load. Graph resolution limits
their precision. These are manufacturer-chart approximations, without
independent prototype measurements. There is no gravity-assisted lowering or
regeneration curve. The operating domain is **0–900 lbf at 12 V**.

The length table spans 1–60 inch strokes, but the supplied mass table spans
only **2–40 inches**. The combined profile uses the narrower 2–40 inch domain;
it does not invent missing mass at 1, 50 or 60 inches. Installed end hardware
and allowance mass were not supplied and are outside the actuator mass fit.

## Catalog geometry, discrete hardware and mass

The student supplied the 6/8-inch endpoints on 2026-11-07 and added the 9/10-inch options from the A/B dimension table on 2026-11-09:

| Catalog stroke | Minimum command | Maximum command |
| --- | --- | --- |
| 6 in (0.1524 m) | 0.352298 m | 0.504698 m |
| 8 in (0.2032 m) | 0.403098 m | 0.606298 m |
| 9 in (0.2286 m) | 0.428498 m | 0.657098 m |
| 10 in (0.254 m) | 0.453898 m | 0.707898 m |

`data/actuators/installed_strokes_2026-11-07.json` records this specification.
`rover/config.py:PA04_STROKES` sets the bounds on all four commands. Stroke is
fixed within a run, selected by the case's `actuator_catalog_stroke`; intermediate
strokes are rejected. The existing geometry study retains its separate 6/8-inch search choices. The old
161.3408 mm installation allowance is not added to these supplied endpoints.

For catalog stroke `s` in metres and configurable installation allowance `a`:

```text
catalog_retracted_length = s + 7.87*0.0254 + a
catalog_extended_length  = 2*s + 7.87*0.0254 + a
a = actuator_installation_allowance = 0 m for the selected installed endpoints
mass_one_actuator = (4.376918958521191 + 0.1574588593327322*s/0.0254)*0.45359237 kg
linkage_mass = 2*(mass_one_actuator + arm_linear_mass*arm_length)
```

The allowance is added **once to each endpoint**. It changes installed span but
does not change available stroke. The two length laws reproduce the supplied
table exactly to its printed precision. They require no nonlinear interpolation.
The mass equation is a global affine least-squares fit, so gradients are smooth.
The 9, 18 and 30 inch rows were withheld from the fit of the other 13 rows.
Held-out MAE is 0.09849 lb (0.04467 kg), RMSE 0.10603 lb (0.04809 kg), and maximum
absolute error 0.15068 lb (0.06835 kg). Across all 16 rows the maximum error is
0.17527 lb (0.07950 kg). These quantify catalog approximation, not manufacturing
tolerance or independent hardware accuracy. Reproduce with:

```powershell
.\.venv\Scripts\python.exe data/actuators/fit_pa04.py
```

`actuator_catalog_stroke` is fixed to one of the two hardware cases. `actuator_force_rating` is frozen at
4003.39945373445 N, and `actuator_speed` is frozen at rated-load lowering speed,
0.009906 m/s. Incompatible overrides fail explicitly. Reference actuator mass,
efficiency and housing coefficients remain recorded but are inactive in this
profile; `actuator_idle_power` still applies to other motor disciplines.

The existing `actuator_retracted_length` port means the **commanded compact
pose**, not necessarily the hardware's minimum length. `travel_actuator_length`
and `actuator_extended_length` remain the commanded raised-travel and digging
poses. All four commands, including deposition, must now remain inside the
selected installed interval. In this profile `actuator_body_margin` remains the minimum signed distance of those four
commands to both hardware travel limits, replacing the generic body-length
surrogate as a diagnostic only. It is absent from optimizer constraints and feasibility decisions; catalog endpoints remain visible. `stroke` retains its meaning as commanded dig-minus-travel distance;
it is distinct from available `actuator_catalog_stroke`.

These command bounds do not verify actuator-body/bracket collisions or joint
packaging. Manufacturer-chart mass and speed/current errors and their validation
limits are unchanged. Generic actuator reference physics remains available.

## Load, time and electrical energy

2026-11-08: the student removed the assumed breakout allowance. Both reference
and PA-04 actuator calculations now use quasi-static gravity loads only, including
arm weight and the carried drum pair/payload. No breakout work is added to energy.
The legacy `breakout_force` input remains accepted but inactive (default zero),
so older saved cases cannot reintroduce the allowance. Extraction resistance and
dynamic loads require a separate physical calculation; none is added here.

The sampled virtual-work leverage is applied to the gravity load:

```text
F = (drum_mass/2 + payload_mass/2 + arm_linear_mass*arm_length/2)*g*actuator_leverage_max
q = F / (900*4.4482216152605)
raising_speed = (0.47 - 0.08*q)*0.0254 m/s
raising_current = 4 + 10*q A
lowering_speed = 0.39*0.0254 m/s
lowering_current = 14 A
lowering_time = max(dig_length - travel_length, 0)/lowering_speed
raising_time = max(stroke, 0)/raising_speed
actuation_time = lowering_time + raising_time
arm_energy = 2*12*(raising_current*raising_time + 14*lowering_time) J
arm_peak_power = 2*12*max(raising_current,14) W
```

Both actuators move in parallel, so their times are not added together. Force,
speed and current are per actuator; energy and peak power include **two**.
Raising applies maximum sampled loaded gravity force, excluding extraction resistance,
throughout the motion. Lowering conservatively assumes rated-load current and
speed throughout because a directional load history is missing. Actual
gravity-assisted behavior, partial filling, controller limits, thermal behavior
and transient/inrush currents remain unverified. These are explicit sizing
assumptions, not a predicted actuator time history.

`12*I` is already electrical power. Applying `actuator_efficiency` or adding
reference actuator idle power again would double-count losses. There is no
regeneration credit. Mission subtracts exactly the lowering overlap from the
sum of cutting and total arm time. The actuator safety factor remains an explicit
input: `actuator_force_margin = rated_force / safety_factor - F`. At the user's
request its default and saved baseline value are now 1.0, so calculated demand
is compared directly with the 4003.399 N catalog rating. This removes the model's
additional actuator reserve; no undocumented manufacturer reserve is assumed.
Structural safety/load factors remain separate and unchanged. Baseline
`fill_fraction` remains 0.9. Changing this force acceptance threshold alone does
not change the predicted force, actuator speed or motion time at fixed geometry.

Pure public helpers reject out-of-domain stroke/load inputs. OpenMDAO adapters
retain finite diagnostic trials for the optimizer, accompanied by negative
stroke-domain/force margins and `actuator_curve_valid=0` for invalid loads.
The load is bounded only for diagnostic curve evaluation; the original force
retains its failing margins. Invalid trials cannot become feasible through
clipping or a silent fallback to the reference actuator model.

## Shared interface integration (schema 3)

To avoid a payload/time circular dependency, `LinkageGeometry` now emits
`lowering_time` in seconds. `Excavation` consumes that explicit port rather than
recalculating motion from config. `LinkageLoads` emits `actuation_time` in
seconds, including raising. The reference load model sets it to twice lowering
time, preserving valid reference-model results. Zero or reversed strokes have
zero diagnostic movement time and retain their failing stroke/pose constraints.
Custom geometry/load replacements must supply these updated native-unit ports.
The reference drum, mass boundaries, overlap sign and mission score remain
unchanged. N2 and declared dependencies include the new connections.

The report saves catalog endpoints, unit mass, raising speed/current/time,
lowering current/time, curve validity, domain checks and profile assumptions.
Case reading, archiving and sensitivity studies preserve `actuators`. A study
now defaults to the baseline start only; `--seed` adds an explicitly chosen
second start with matching hardware. The historical `design_optimized.json`
retains its old profile and is rejected as a PA-04 seed until explicitly migrated.

## Force-versus-stroke report plot

New HTML reports include a gridded actuator force plot before the excavation
feed information. The N/cm and lbf/in buttons switch both axis units. Forces
apply to one actuator; stroke is extension from the catalog minimum installed
length, not the model's separate dig-minus-travel `stroke` output. Generic
actuator reports use the shortest commanded length as zero instead.

The plot uses the existing 81 capability-sweep poses and arm equilibrium
helper. Two curves show empty and full gravity loads,
with no breakout allowance or dynamic load factor. The dashed capacity is rating divided
by actuator safety factor. This is a quasi-static sizing envelope, not a force
history during soil entry. Overloads remain visible; unreachable/toggle poses
produce an explicit unavailable message. The report adds no new fitted physics
or feasibility decisions. Both unit versions are exported as SVG files.

## Historical baseline and verification (before the 2026-11-07 endpoint update)

An **8 inch stroke is a provisional start**, not a final size choice. Its installed
limits are 564.4388–767.6388 mm, while the current baseline travel command is
500 mm after retaining the newer upstream baseline geometry. The installed
travel-limit margin is therefore **−64.4388 mm**. Upstream mounting geometry
and pose commands were retained to expose this
mismatch. The baseline remains failed and requires geometry redesign; the
selection does not establish feasibility or refresh the older published reports.

`tests/test_pa04_actuator.py` checks length-table reproduction, adjustable
allowance, held-out mass errors, curve endpoints/interiors, electrical power
greater than mechanical output, two-actuator mass/energy accounting, zero-motion
energy, overload rejection, packaging diagnostics, fixed ratings, source integrity,
N2/dependency connections and smooth total derivatives. Full reference checks
are listed in [VERIFICATION.md](VERIFICATION.md). Catalog agreement and numerical
convergence do not establish experimental validation. Actual actuator logs,
installation BOM mass and a lowering/current/duty-cycle test are still needed.

## Integration with the pulled traversal and attachment changes

The combined CLI accepts `--actuators`, `--freeze`, `--ignore-constraint`,
`--traversal-mode`, `--optimize-attachment` and `--attachment-max` together.
Frozen inputs keep their original bound checks; ignored constraints remain in
full feasibility. Case files, archived candidates and recorded iterations retain
the actuator profile, traversal mode and optional attachment bound. Catalog
stroke diagnostics remain in reports even when catalog stroke is frozen.

The existing physical arm-extension corrections compose with PA-04 geometry and
electrical curves. Tail mass enters once, and the uniform arm gravity moment
uses the full physical length squared. Reference overhang equations are reused;
the mass and force corrections are tested against independent expressions.
The pivot multistart study retains the selected actuator profile and discrete
stroke, including 6-inch seeds. It does not replace a frozen PA-04 force rating
with the generic 6000 N seed assumption. Generated commands outside the selected
hardware interval are rejected, never clipped into a different pose.

Historically, freezing 500 mm travel and 726 mm dig lengths could not produce valid
PA-04 packaging at any continuous catalog stroke with the current allowance.
The minimum installed length requires `s <= 138.7612 mm`, while the maximum
requires `s >= 182.3806 mm`. These intervals do not overlap. Revise the commanded
lengths or free them during a separate geometry study; the program keeps the
requested frozen values and reports the failure.
