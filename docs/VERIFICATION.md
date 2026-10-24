# Current verification status

Checked locally on **2026-10-23**, using the repository's `.venv`. This page
describes the current implementation; Git history retains earlier run notes.
Software verification is separate from physical calibration and validation.

## Status at a glance

| Check | Result |
| --- | --- |
| Full unit suite | 140 tests passed, 253.347 s |
| Generic reference fixed evaluation | Completed; failed constraints remain visible |
| Generic reference derivative step-size check | Passed at its starting design |
| Generic reference cold-start SLSQP solve | Known failure; saved as failed/infeasible, exit 1 |
| Current saved design, reevaluated with default BWR | Only the new empty/full climb requirements fail |
| Historical optimized candidates | Prior success statements predate BWR climb requirements; reevaluation required |
| Historical sensitivity grid | 96 cells, 192 starts; 88 formerly feasible cells, before BWR climb requirements |
| Hardware/soil predictive validation | Not established |

A baseline command returns 0 after a valid evaluation even when its checks fail.
Optimization returns 0 only when the driver succeeds and its final saved checks
pass. Current BWR run details are recorded below; older verification entries are
historical and do not establish feasibility under the new climb requirements.

## Reproduce the checks

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/verification/reference_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --output results/verification/reference_optimization.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/verification/student_baseline.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --optimize --check-derivatives --output results/verification/student_optimization.json
```

The third command's known optimizer failure must remain visible. Inspect its
derivative message and saved status separately. Numerical derivatives compare
finite-difference step sizes at smooth points; they do not validate equations
against hardware or certify behavior at branch transitions.

The suite covers independent geometry and physical limits, mass/mission-energy
conservation, signed failure domains, explicit dependencies and connections,
motor curve/gear/loss accounting, model replacement, and optimizer regressions.
The reference drum and mass boundaries remain covered. Full-suite local logs
are in ignored `results/pdf_ready_tests.log`; the seven study-specific tests
also passed in `results/traversal_fill_tests.log`. These local logs are not
available in a fresh clone; rerun the commands above.

## Reports and provenance

Only `design_baseline.json` and `design_optimized.json` are maintained design
inputs. Each curated HTML embeds its exact numeric run, native inputs/outputs,
checks, source hash, and model provenance. Publishing rejects stale or unrelated
baseline/candidate pairs and preserves the baseline byte for byte.

The current baseline retains:

- `structure_stress_margin = -655854.8516 Pa`
- `rock_sweep_wheel_load_margin = -0.8535612742 N`

The curated candidate passes its saved checks; this does not establish a global
optimum. Report tests preserve failed statuses, source fidelity, and archive
guards. Browser/PDF review checked first-page scores, section order, all printed
checks regardless of screen filters, top view, three side poses, full native
appendices, restored screen state, and no table overflow. Reviewed PDFs have
27 baseline and 28 candidate pages. Regenerate and re-export after input changes.

## Sensitivity study

The 12 × 8 traversal/fill grid uses unchanged physics, bounds, and solver
settings. Every cell has two recorded starts; only successful, feasible attempts
are eligible. Five 15 s cells retain failed numeric iterates and three retain
exceptions without usable final states. Error reports invent neither score nor
geometry. An independent no-load speed bound rules out the prescribed 15 s
profile with the fixed X44 drive gearing; its optimistic lower limit is 17.42 s.

An independent audit checked all 96 coordinates and 192 attempts, source/model/
data hashes, fixed nonswept assumptions, winner selection, classification, report
links, embedded records, geometry, route timing, payload ceilings, and mass and
energy balances. The reference N2 has 309 explicit connections and no auto-IVC
sources. Resume reused all 96 cells. Browser checks covered cell/keyboard
selection, side poses, successful and failed reports, offline operation, and
mobile layout. Full results and audit stay under ignored
`results/sensitivity/traversal_fill/`.

An additional fixed-hardware evaluation held the 45 s / 0.4 study candidate
unchanged across all 88 cells at 30–180 s. Every current modeled check passed.
At 45 s / 0.9 it scored 2348.03 versus 3694.07 for the reoptimized candidate;
its payload stayed reach-limited at 53.66 kg. This supports modeled feasibility
robustness, not equal performance or physical validation. See
[the study guide](SENSITIVITY_STUDY.md) for findings and reproduction.

## Default parallel-cylinder clearance: 2026-10-23

The user requested parallel finite cylinders as the default for every frame
and motor profile and removal of the former clearance implementation. The
component now derives directly from Discipline, owns its complete dependencies,
and is selected by ModelSet without an override. The removed implementation,
imports and comparison/selection paths are absent from current source.

All 127 current unit tests passed. Retired-algorithm tests were replaced with
independent radial/axial/corner and overlap limits, finite-width separation,
common roll/translation/pitch invariance, all pairs/mirrors, invalid/guarded
poses, scalar connections, smooth derivatives, default selection for all four
frame/motor combinations, and report checks. The motor-interface fixture was
versioned to revision 2: only its three changed clearance margins were updated,
with explicit provenance and source hashes. Its other expected values and all
regression tolerances were retained.

At unchanged design_optimized.json inputs, the cylinder gap is
0.10288576678 m and the margin after the 0.015 m mechanism clearance is
0.08788576678 m. The level-pose gap is 0.21101059004 m; the 0.40 m raised-wheel
endpoint reduces it through relative articulation, despite shared-roll
invariance. Mass, payload, mission energy and construction points at identical
geometry are unchanged by this clearance-only replacement.

Required verification ran the reference baseline and derivative-checked
generic cold-start optimization with cylinders selected by default. Derivatives
passed; the generic cold start remains unsuccessful/infeasible, with failed
structural stress/deflection and rock-path wheel reaction at the saved final
iterate. This failure remains labeled failed. A plate-frame/X44 optimization
from design_optimized.json passed derivative checks, converged and passed all
modeled constraints. Reports and N2 diagrams are in ignored
results/default_cylinder_*.json/.html and the reproducible example writes
results/default_cylinder_example/cylinders.html when selected as its output.
Numerical mass and energy accounting and explicit sources were inspected.

This is ideal rigid-cylinder geometry, without fitted data or hardware
calibration. Independent camber/steering, deformation, other hardware,
contact corrections and the actual climbing path remain outside its scope.
Saved historical reports/PDFs retain their original provenance; rerun them
before using them to assess the new default.

## Experimental rigid-wheel BWR (2026-10-23)

`python -m unittest discover -s tests -v` passes **147 tests**, including nine
new BWR tests for independent adaptive force integration, stress endpoints and
continuity, normal balance, positive power dissipation, load response, invalid
soil/geometry/gravity/load/demand, force-root recovery, smooth sensitivities,
soil-fit transfer flags and ModelSet point-turn invariance. The initial sandbox
run encountered temporary-file permission errors in 14 existing tests; the
normal-permission rerun passes all 147. No assertions were weakened to pass.

Required reference checks were run with the local `.venv`:

```powershell
.\.venv\Scripts\python.exe main.py --baseline --output results/bwr_reference_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --output results/bwr_reference_optimization.json
.\.venv\Scripts\python.exe -m studies.bekker_wong_reece
.\.venv\Scripts\python.exe -m examples.bekker_wong_reece --check-derivatives
```

The reference baseline retains its documented physical failures. Its drum mass,
wheel mass, dig energy, dump energy, dry mass and mobility energy exactly match
the earlier parallel-cylinder reference baseline. The reference derivative
step check passes, but its cold-start optimization still fails with SLSQP exit 8;
final failures include rover width, structural stress/deflection and rock-path
wheel reaction. That iterate remains `failed_or_infeasible`; no constraint was
relaxed or failed result replaced by a successful case.

The optional fixed-design BWR run passes the selected energy/power derivative
step comparison. It preserves six point-turn outputs exactly at identical
inputs. There are 343 explicit connections and no automatic IVC connections.
Dry-mass accounting residual is zero. Mechanical work comes from torque and
rim travel, with chassis acceleration supplied through the net-force solve;
legacy rolling drag and a separate kinetic-energy charge are not added twice.
Electrical conversion remains the existing one-time Gearmotor conversion.
This saved design passes existing modeled mission screens; standalone BWR
30-degree climb diagnostics fail and are not relabeled as a passing climb.

The independent soil-parameter benchmark is numerically complete but fails its
predeclared accuracy gate. Stock 63 kg Husky MAE is 12.1 percentage points
(previous interpolation 50.5 points); predictions are 37.6%, 67.8%, 94.0% at
5, 10, 12.5 degrees versus 20%, 65%, 78% measured. The 384 x 64 mm rigid-wheel
proxy solves two of three means (conditional MAE 10.3 points after explicit
radius-reference conversion) and misses the observed 15-degree climb. The
six-wheel, different-soil SSTB-Lite check is separate: eight of ten points solve,
conditional MAE 11.5 points, two failures. Missing solutions count as failures.
No parameter was fitted to these rover observations. Source ranges, preparation
mismatch and generic stress-shape assumptions remain explicit. Reports and
plots were inspected; reproduction writes `results/bekker_wong_reece/`.

That earlier optional-model verification preceded default selection and the two
new climb constraints; its feasibility statements are historical. Equations, evidence, units,
solver domains, report schema and limitations: BEKKER_WONG_REECE.md.

## Retired straight-driving surrogate

The earlier Sutoh interpolation failed larger-rover transfer checks (stock Husky
MAE 50.5 percentage points). Its model, processed fit data, scripts, example and
tests were removed when the student selected BWR as the default. Historical
ignored result artifacts remain historical; their feasibility statements predate
the new empty/full climb requirements. Measured external rover data are retained
independently for `python -m studies.bekker_wong_reece`.

## CI and remaining uncertainty

[GitHub Actions](../.github/workflows/ci.yml) runs the full unit suite, baseline,
and derivative-checked optimization on Windows and Linux. It checks that the
saved optimizer/feasibility status matches the exit code and that the two BWR
climb decisions obey the strict slip rule. A failed design remains failed; an
exception or missing result is a CI failure. The old feasible-start fixtures
predate the new BWR climbs. Their regression tests now verify rejection under
the new requirements and retain independent derivative/physical checks, rather
than claiming those starts still satisfy the changed problem. Full design-search
outcomes are recorded separately below. No slope was reduced for convergence.

Unresolved physical evidence includes retained soil versus speed/fill, rolling
and turning resistance, motor current/thermal duty, real gearbox/assembly mass,
CF tube/joint properties, and loaded articulation/tilt tests. The plate-frame
structural profile covers mirrored full-load level-ground gravity; cutting,
recovery, obstacle reactions, and connection strength need separate work.
Ditch width/profile and actual climbing capability remain unverified. No
simulation agreement, solver result, or passing software test closes those gaps.

## Default BWR and empty/full climbing constraints (2026-10-23)

The student selected rigid-wheel BWR with published loose GRC-1 as the default
conservative design assumption. The former slip surrogate and its fit-data/code
pipeline are removed. Independent measured rover data remain available; no BWR
soil parameter was refitted. Empty requires 25 degrees, full requires 20 degrees,
with a solved slip strictly below 1. Both angles are editable fixed assumptions.
The separate 30-degree static tipping check and physical dimension/mass caps are
unchanged. Normal failed-demand trials retain a negative force deficit and an
explicit invalid-solution flag; invalid contact inputs still raise errors.

All **140 tests passed** in **253.347 s**, including independent zero-shear force
integration, incline force balance, torque/work conservation, 99.5% slip recovery,
100% slip rejection, empty/full mass accounting, exact point-turn invariance,
strict report decisions, source/contract/dependency checks and smooth derivative
comparisons. Full-suite report tests require ordinary temporary-folder access;
the sandbox-only attempt encountered permission errors and was superseded.
Historical optimizer regression starts no longer satisfy the changed problem;
tests verify their failures rather than reducing the requested slopes. A small
per-evaluation batch shares identical route load/acceleration force solutions,
with no persistent cache or cross-perturbation reuse.

Reproduction artifacts:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/bwr_default_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --output results/bwr_default_optimization_verified.json
.\.venv\Scripts\python.exe main.py --inputs design_optimized.json --output results/bwr_default_current.json
.\.venv\Scripts\python.exe main.py --inputs design_optimized.json --optimize --output results/bwr_default_candidate_optimization_verified.json
```

The current saved design has dry mass 41.9690 kg and full mass 95.6119 kg.
It passes all modeled checks except the two new climbs. At 25 degrees empty and
20 degrees full there is no resolved BWR driving solution below slip 1; limiting
per-wheel net-force shortfalls are 27.0455 N and 41.1808 N, respectively.
There are 350 explicit connections and no automatic IVC sources. Dry-mass sum
residual is zero; cycle-energy accounting residual is below 1e-14 Wh. Reference
drum and motor mechanical regressions pass. Generic baseline derivative checks
pass. Earlier pre-batching optimization attempts were interrupted and superseded
by the `_verified` runs; those partial logs/recorders are not final results.

The independent benchmark remains failed: stock-Husky MAE 12.1382 percentage
points, rigid-wheel Husky 10.3063 points on two solved means with one failed
observed climb, and separate six-wheel SSTB-Lite 11.5404 points on eight solved
points with two failures. Default selection does not change that validation
status or close the soil/prototype calibration gaps.

The saved-design COBYLA search used the unchanged 2,000-evaluation limit
(2,001 recorded evaluations including the initial case), taking 635.06 s.
It ended with optimizer success false and status `failed_or_infeasible`.
The final empty/full climb margins are -0.204463 and -0.100986. The final trial
also fails pressure, several mounting/clearance/envelope checks and design
bounds, including wheel diameter 395.599 mm above 359 mm and wheel width
30.188 mm below 50 mm. Those trial dimensions are **not accepted** or copied
into `design_optimized.json`. The final score is not a feasible design result.
The original saved design remains unchanged; its fixed reevaluation report is
`results/bwr_default_current.html`.

The required generic reference SLSQP run also terminated unsuccessfully, rather
than being stopped early: 654 recorded model evaluations, 115 derivative
evaluations and 1680.5 s driver runtime. The starting total-derivative step-size
check passed. Final status is `failed_or_infeasible`, optimizer success false,
CLI exit 1. Failures remain dry mass (92.9209 kg versus 80 kg), empty/full BWR
climb margins (-0.221034 / -0.119296), wheel-first clearance (-2.116 mm), and the
rock-path wheel reaction (-16.668 N). Both complete optimization attempts are
failed designs. No requested slope, physical bound, acceptance criterion or
maintained design file was changed to obtain convergence.

## Editable climb defaults regression (2026-10-24)

The current configuration uses 30 degrees empty and 20 degrees full. The slope
test still hard-coded the original 25-degree empty default, so editing the
intentionally configurable requirement caused an assertion failure before the
physical checks. The default-propagation test now compares against configuration;
a separate explicit 25/20-degree case retains the original reference-geometry
failure assertions and invalid-angle checks. Current-default documentation is
updated; the 2026-10-23 results above retain their historical 25/20-degree settings.
No physics, calibration, acceptance criteria, configuration or maintained design
file changed in this correction. Baseline/configuration SHA-256 checks confirmed
both files were unchanged during verification.

All **141 tests passed** in **262.679 s**, including the five slope tests. The
initial sandbox suite encountered Windows temporary-folder permission errors
and was interrupted; the complete rerun with normal folder access supersedes it.
The reference baseline evaluation and fixed saved-design evaluation completed;
their physical failures remain reported as failures. The saved-design audit found
350 explicit connections, no automatic IVC sources, and zero dry-mass,
cycle-energy and run-energy accounting residuals.

The required default reference derivative/optimization command passed its total
derivative check, but the slow cold-start optimization was interrupted. A
separate bounded command completed with the same derivative check passing:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/my_change_baseline.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/slope_config_current.json
.\.venv\Scripts\python.exe main.py --check-derivatives --maxiter 1 --output results/slope_config_bounded_optimization.json
```

The bounded run exits 1 with optimizer success false and status
`failed_or_infeasible` after reaching its iteration limit. It verifies derivative
consistency and failure reporting; it is not a convergence result. Full-suite
output is retained locally in `results/slope_config_tests_verified.log`.

## Aluminum box drum arms (2026-10-24)

The plate-frame ModelSet now selects `BoxArmPlateStructureGeometry` and
`BoxArmPlateStructureLoads` from `rover/alternatives/aluminum_box_arm.py`.
The baseline uses 50.8 x 50.8 mm outside with 1.5875 mm wall. Only arm-section
physical inputs changed: round diameter replaced by box width/height and wall
changed from 2 mm. Other baseline inputs, aluminum properties, frame loading,
limits and safety factors were retained. Width/height remain independent design
variables (25–100 mm); wall bounds remain 1–5 mm. Reference `ModelSet()` retains
round arms. Existing port names/units for mass, stress, deflection and margins
are unchanged. The historical candidate and X44 restart have explicit nominal
box-side migrations, without claiming reoptimization or prior feasibility.

All **147 tests passed** in **195.072 s**, with ordinary Windows temporary-folder
access. Six independent box-arm tests cover area/inertia subtraction, cantilever
stress/deflection, rotated rectangles with weaker-axis Euler behavior, two-arm
mass accounting, invalid sections and an undersized buckling failure. Focused
reruns of all six and the legacy section invalid-domain check passed after the
final validation/test cleanup. Strict historical motor/drum regression values
and tolerances remain unchanged, using explicit historical round-arm factories;
current box arms are tested independently. Smooth frame derivative checks use
unequal box sides, avoiding the equal-principal-inertia branch boundary.

The current saved baseline reevaluation gives:

| Quantity | Previous round arm | Requested box arm |
| --- | ---: | ---: |
| Arm linear mass, kg/m | 0.8278724961 | 0.8437483125 |
| Two 560 mm arms, kg | 0.9272171956 | 0.9449981100 |
| Dry rover mass, kg | 52.6613682061 | 52.6791491205 |
| Arm relative tip deflection, m | 7.9101278564e-6 | 5.8372826654e-6 |
| Arm Euler buckling margin, N | 22215.78094 | 31673.33325 |

The saved baseline remains infeasible on the same six checks: empty/full BWR
climbs, turn traction, asymmetric-dump tipping, static slope tipping and rock-path
wheel reaction. No failed check was weakened. There are 352 explicit connections,
no automatic IVC sources, dry-mass accounting residual below 8e-15 kg and zero
cycle-energy accounting residual. Both new section dimensions connect explicitly
to structural geometry and loads. The frame SVG and input tables show the
aluminum box section and wall thickness.

Reproduction:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/my_change_baseline.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/box_arm_baseline.json
.\.venv\Scripts\python.exe main.py --inputs design_optimized.json --output results/box_arm_historical_candidate.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --check-derivatives --set arm_box_width=0.045 --set arm_box_height=0.06 --output results/box_arm_derivatives.json
.\.venv\Scripts\python.exe main.py --check-derivatives --maxiter 1 --output results/my_change_optimization.json
```

The affected-profile and generic-reference total derivative checks passed.
Fixed evaluations completed with their failures retained. The generic reference
optimization was deliberately bounded to one iteration; it exits 1 and remains
`failed_or_infeasible`, not a convergence result. No design was reoptimized as
part of this section change. Test output is `results/box_arm_tests.log`.

These are analytical/numerical checks, not hardware calibration. Sharp-corner
uniform-wall sections omit stock corner radii, certified alloy/temper allowables,
local wall instability, torsion, connections/welds and actual arm-envelope
collisions. The existing sampled, mirrored, full-load gravity scope remains.
