# Current verification status

Checked locally using the repository's `.venv`. This page
describes the current implementation; Git history retains earlier run notes.
Software verification is separate from physical calibration and validation.

## Publication input check (2026-10-29)

The latest student baseline edits were preserved for publication to `main`.
The input edits and the actuator-pivot x lower-bound edit (0.05 to 0.02 m)
postdate the tested source snapshot below. A fresh fixed evaluation with
`main.py --inputs design_baseline.json --output results/prepush_baseline.json`
exits 2 at input validation: `bottom_pivot_height=0.0762 m` is below its existing
`0.09 m` lower bound. No baseline prediction or feasibility result was produced.
The earlier passing suite does not establish that these newer inputs run.
The input-validation failure and the reference slope-domain exception below
remain unresolved; publication is not a claim of passing CI or a feasible design.

## Actuator body constraint retired; diagnostic retained (2026-10-28)

At the user's explicit request, `actuator_body_margin` is no longer an
optimization constraint or a feasibility/pass-fail criterion, for either
actuator profile. Its existing metre-valued calculation and dependencies are
unchanged. Generic body-length and PA-04 installed catalog-interval reserves
remain diagnostic outputs; reports label them diagnostic only. The model does
not thereby establish real actuator packaging. All other constraints, catalog
stroke/load domains, dimensions, safety factors and mission equations remain.
Schema 5 records this acceptance-policy change without removing the output port.

All **185 tests passed in 353.602 s** using the local `.venv`.
The final log is `results/actuator_body_constraint_tests.log`. Tests cover all
four frame/actuator combinations, absence from the actual optimizer constraints,
retention in outputs, and unchanged feasibility/remaining checks after making
the diagnostic negative. Existing actuator, derivative, N2, mass/energy and
reference drum/motor tests pass. The historical motor fixture's retired
constraint expectation is explicitly skipped; all other values and tolerances
are unchanged, and its historical data file is unchanged.

Fixed before/after evaluations used the exact same current student input source.
Every existing numeric output and every remaining constraint is identical.
The diagnostic value is -0.110540800 m, without a pass/fail decision.
The current baseline still fails 8 other checks:
`rover_length`, `empty_slope_traversal_margin`, `loaded_slope_traversal_margin`, `turn_traction_margin`, `wheel_first_margin`, `actuator_force_margin`, `tip_stability_margin`, `obstacle_drum_wheel_margin`.
The updated report is `results/actuator_body_removed_baseline.html`.
Earlier saved reports retain their original acceptance rules and need rerunning.

The required reference fixed evaluation completed; its physical failures remain.
The current student baseline and generic reference derivative step-size checks
passed. The required unbounded reference optimization ended with an analysis exception in `TraversalStability`: `Slope screen needs positive spans/CG height and angle in [0, pi/2).`. It did not return a normal optimizer result or produce a final JSON/HTML report. This is an unresolved domain failure, not convergence or a feasible result. The traceback is retained in `results/actuator_body_removed_optimization.log` and recorded trials remain in its SQL file. No input, constraint or domain check was changed to bypass this failure.
No hardware/soil calibration or external simulator run was performed.

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/actuator_body_removed_reference_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --output results/actuator_body_removed_optimization.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --check-derivatives --output results/actuator_body_removed_baseline.json
```

## Axle-relative actuator offsets (2026-10-28)

The maintained baseline replaces the attachment fraction with signed
`lin_act_side_arm_offset=0.05 m` and `lin_act_top_arm_offset=0.05 m`.
Positive directions are outward past the drum axle and above the arm; rear
geometry is mirrored. Every other baseline input is unchanged. The two new
lengths are ordinary design variables; reference defaults are zero.
Equations, signed domains, schema 4 ports, historical input conversion and
unverified bracket details are in [ARM_OFFSETS.md](ARM_OFFSETS.md).

All **184 tests passed in 520.980 s** using the local `.venv`.
The final log is `results/arm_offsets_tests_verified.log`. Earlier
`arm_offsets_tests.log` and `arm_offsets_tests_final.log` are superseded.
Independent checks cover local coordinates, pin/axle circle closure,
virtual work, gravity conservation, eccentric force/moment balance, the
analytic end-couple beam limit, continuity through zero, actual tail mass/work/
inertia, invalid and unreachable geometry, mirrored drawings, explicit source
migration, declared dependencies and smooth total derivatives. The historical
motor fixture retains its original numbers and tolerances through an explicit
historical round-arm/axle-pin beam factory. The current conservative offset
beam screen is tested separately. Reference drum/motor regressions pass.

The fixed requested report is `results/arm_offsets_baseline.html`, with its
JSON, N2 and seven SVG drawings alongside it. It gives 50.26370105 kg
dry mass, 12.29209524 kg payload, 174.81946390 s cycle time,
32.82664776 Wh continuous mission energy and
337.24857968 construction points. Both offsets are exactly
0.05 m and this is a fixed evaluation, not an optimized design.
Four failures remain: stowed height (787.8916 mm versus
750 mm), empty/full BWR climbing and actuator packaging (âˆ’64.4388 mm).
The old turn-traction failure now passes at this geometry; no check was waived.

The requested baseline and generic reference derivative step-size checks pass.
The required generic reference SLSQP run completed normally with exit mode 8,
optimizer success false and `failed_or_infeasible`: 158 model evaluations,
18 derivative evaluations and 423.64 s driver runtime. Mass, height, both climbs,
straight motor, dig reach/payload, drum/dump torque, obstacle clearance and
rock-path wheel load remain failed in its saved final iterate. It is not a
feasible result and no maintained design was replaced with that iterate.

The current baseline has 356 explicit connections and no automatic IVC sources.
The independent audit found mass/energy/time residuals below 1e-10 in native
units, zero run-energy residual and zero actuator-length closure residual at
all three commanded poses. Uniform tube tails add 0.08437483125
kg across both arms, counted once. Source hashes in the requested report match
the current rover sources. Four drawings were rasterized offline and visually
inspected, including separate axle/pin labels and mirrored bracket offsets.

No actuator/soil hardware test, fitted dataset or external simulator was run.
The normal bracket is assumed rigid and massless; its mass, strength, joints,
full collision envelope and soil interaction remain unverified. Soil transfer,
motor thermal/current capability and other existing model-card limits remain.
Passing software checks is not predictive validation. Side=0 is the physical
tube-length branch boundary; smooth offset derivatives are checked away from it.

Reproduction (ordinary local temporary-folder access; no limits relaxed):

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/arm_offsets_reference_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --output results/arm_offsets_reference_optimization.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --check-derivatives --output results/arm_offsets_baseline.json
```

## Status at a glance

| Check | Result |
| --- | --- |
| Full combined unit suite | 185 tests passed, 353.602 s; actuator body constraint retired |
| Generic reference fixed evaluation | Completed; failed constraints remain visible |
| Generic reference derivative step-size check | Passed at its starting design |
| Generic reference cold-start SLSQP solve | Known failure; saved as failed/infeasible, exit 1 |
| Current saved baseline | 8 remaining failures; actuator packaging reserve is diagnostic only |
| Initial PA-04 baseline derivative check / optimization | Historical pre-update geometry: derivatives passed; COBYLA exhausted 2000 evaluations and retained a failed/infeasible iterate |
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
The reference drum and mass boundaries remain covered. Current full-suite logs
are in ignored `results/conflict_repair_tests_final.log`; initial actuator checks are in
`results/pa04_tests_final.log` and targeted checks are in
`results/pa04_targeted_final.log` and `results/pa04_study_final.log`. Older logs
include `results/pdf_ready_tests.log` and `results/traversal_fill_tests.log`. These local logs are not
available in a fresh clone; rerun the commands above.

## PA-04 actuator integration (2026-10-27)

This section records the initial actuator integration before the newer upstream
500/508/726 mm commanded lengths and revised pivots were combined. Its saved
outputs and numerical comparisons remain historical; current integration
checks are recorded in the next section.

The frozen 900 lbf / 12 V profile uses chart endpoints, exact affine catalog
lengths with the adjustable 161.3408 mm allowance, and an affine mass fit with
three withheld catalog rows. The model card, domain and missing hardware
validation are in [PA04_ACTUATORS.md](PA04_ACTUATORS.md). No physical actuator or
external simulator was run.

Full-suite checks passed (156 tests). The final actuator tests additionally
reject tiny out-of-domain excursions that normal constraint report tolerances
might otherwise accept. N2 connections show geometry `lowering_time` feeding
excavation/loads, and loads `actuation_time` feeding mission. Mass is summed
once, electrical losses are not applied twice, and lowering overlap is removed
once. Reference drum, reference mass/energy regressions and smooth derivatives
passed. Initial sandbox errors were temporary-directory permissions; the suite
passed when rerun with approved access outside the sandbox.

Fixed evaluations at identical mounts and pose commands compare the prior
generic actuator assumptions with the selected PA-04 profile:

| Quantity | Prior actuator assumptions | PA-04, provisional 8-inch stroke |
| --- | ---: | ---: |
| Dry mass | 52.6791 kg | 50.2783 kg |
| Linkage mass, both arms and actuators | 8.4593 kg | 6.0584 kg |
| Actuation time per cycle | 9.5758 s | 61.1611 s |
| Arm electrical energy per cycle | 1742.16 J | 17462.89 J |
| Arm peak electrical power, both actuators | 459.60 W | 336.00 W |
| Mission cycle time | 159.6393 s | 193.6728 s |
| Continuous mission energy | 37.6485 Wh | 57.6964 Wh |

These are conditional diagnostics, not feasible hardware predictions. The
initial baseline retained seven failures, including a Ã¢Ë†â€™189.4388 mm installed
actuator travel-limit margin. The 564.4388 mm hardware minimum is incompatible
with the 375 mm travel command. A mounting/pose redesign is required.

Both required derivative-checked optimizations were actually run with their
normal settings. Both initial derivative checks passed. Reference SLSQP ended
failed/infeasible (114 model evaluations; 312.52 s driver runtime), retaining
mass, climbing, motor and obstacle/path failures. PA-04 plate-frame COBYLA
exhausted 2000 objective evaluations (2001 recorded model evaluations; 325.80 s
driver runtime). Its failed final iterate retains height, climbing, ground
pressure, bottom-mount, actuator packaging (Ã¢Ë†â€™153.35 mm), obstacle, slope,
wheel-reaction and design-bound failures. Its continuous stroke of 170.977 mm
is not a recommended catalog selection. No limits or tests were relaxed.

Ignored local outputs are `results/pa04_reference_baseline.json`,
`results/pa04_reference_optimization.json`, `results/pa04_before_baseline.json`,
`results/pa04_selected_baseline.json` and
`results/pa04_selected_optimization.json`, with HTML/N2 reports and matching logs.
The maintained baseline records the new actuator choice, while its mounting
geometry remains pending. The historical optimized case and published report
snapshots were not promoted to the new hardware profile.

## Traversal/actuator conflict repair

Seven stash-application conflicts were resolved in the CLI, problem builder,
baseline, case reader, report snapshot, archive and traversal/fill study. The
working files contain no conflict markers, and Git's index contains no unresolved
paths. The stash backup remains intact; no commit or push was performed.

The final combined suite passed all **181 tests in 479.634 s**. The initial pass
caught an out-of-bounds seed in a new integration test; a valid inward-pin seed
was used for its rerun without altering model bounds or the assertion.
All 74 Python sources parse successfully; staged and working diffs pass
`git diff --check`.

The combined CLI accepts the PA-04 selection alongside frozen design variables,
calculated traversal, ignored optimizer constraints and the optional attachment
search. The existing arm-overhang corrections compose with both reference and
PA-04 actuator models. Tests verify physical mass/force expressions, preservation
of frozen ratings in multistart seeds, recorded outputs when catalog stroke is
frozen, and propagation of all profile choices through CLI and report metadata.

The newer upstream baseline mounting geometry and commanded lengths are
preserved. Fixed evaluation of the maintained case has four failed checks:
empty/full climbing, turn traction and actuator packaging. The actuator reserve
is Ã¢Ë†â€™64.4388 mm because the travel command is 500 mm and the provisional 8-inch
unit's installed minimum is 564.4388 mm. With climb angles explicitly set to zero
and calculated traversal selected, only turn traction and packaging fail.
The turn-traction exclusion remains visible as a failed, unenforced check.

The user's command was exercised as a bounded smoke run using `--maxiter 1`.
SciPy raises that COBYLA budget to its minimum of 29 objective evaluations
(30 recorded model evaluations); the saved final status correctly remains
failed/infeasible. This verifies command parsing and model/report execution,
not convergence. The normal 2000-evaluation fixed-hardware search was not rerun.
Freezing 500/726 mm travel/dig lengths is geometrically incompatible with every
PA-04 catalog stroke at the current allowance; see [the interval proof](PA04_ACTUATORS.md).

The generic reference fixed evaluation completed, and its initial total
derivative check passed. Generic reference cold-start optimization completed
114 model evaluations in 758.37 s and retains its documented failed/infeasible
status, including mass, climb, motor and obstacle/path failures.
Software/connection/conservation tests do not add
actuator, soil or thermal calibration. Local outputs and logs use the ignored
`results/conflict_repair_*` prefix.

## Reports and provenance

Only `design_baseline.json` and `design_optimized.json` are maintained design
inputs. Each curated HTML embeds its exact numeric run, native inputs/outputs,
checks, source hash, and model provenance. Publishing rejects stale or unrelated
baseline/candidate pairs and preserves the baseline byte for byte.

The published baseline snapshot is the fixed aluminum-box-arm design evaluated on
2026-10-24: dry mass 52.67914912 kg and six failed checks, including
`rock_sweep_wheel_load_margin = -48.36401627 N`. Structural stress passes the
current screen. Full current metrics and inputs are embedded in the published
baseline HTML. The curated candidate is a historical report from the earlier
model; its saved pass status does not establish current feasibility or an
optimum. Report tests preserve failed statuses, source fidelity and archive
guards. Regenerate and re-export after input changes. Earlier page counts and
validation runs below are historical snapshots; see the latest publication note.

## Sensitivity study

The 12 Ãƒâ€” 8 traversal/fill grid uses unchanged physics, bounds, and solver
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
unchanged across all 88 cells at 30Ã¢â‚¬â€œ180 s. Every current modeled check passed.
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
variables (25Ã¢â‚¬â€œ100 mm); wall bounds remain 1Ã¢â‚¬â€œ5 mm. Reference `ModelSet()` retains
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

## Report member counts and camera depth (2026-10-24)

Presentation-only changes add the central rocker axle, two box rockers, plate
footprints, four transverse CF rods, and two centered box drum arms to the plan
view. Each front/rear shaft-linked drum pair has one arm and one actuator,
matching the two-arm mass ownership. Arm widths, CF diameters/spans, rocker
bearing stations and axle diameter come from the existing inputs. Upper/lower
CF projections can overlap; the figure labels that instead of offsetting them.
`reporting/frame_members.py` shares the centered linkage/CF drawing stations
between plan and rock views; it does not participate in model computation.

The fixed rock-view projection has screen-right (.88,.45,0) and screen-up
(-.12,.20,1). Their cross product (.45,-.88,.23) points toward the camera.
Rover bodies and members are painted by increasing camera depth. Visible cap
and barrel faces are opaque; hidden outlines no longer show through nearer
wheels/drums. Wheel labels sit beside the contact annotations.

All **148 tests passed** in **178.670 s**. The report regressions check member
counts, nominal projected widths/spans, centerline linkages, independent
camera-normal body ordering, opaque cylinder caps, four wheel contacts,
unreachable-pose labeling and unchanged snapshot data. Top and rock SVGs were
rendered to PNG and visually inspected at 1600-pixel width.

The full saved-baseline numerical snapshot is identical to the preceding
box-arm report, including all constraints, mass/energy values and model
connections. The fixed baseline evaluations completed. The reference derivative
check passed; its optimizer smoke test used `--maxiter 1` and correctly retained
`failed_or_infeasible` with exit 1. No full reoptimization or physics change was
part of this graphics task. Calibration and unresolved physical failures remain
as documented above.

Reproduce the current graphics with:

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/report_graphics.json
```

Open `results/report_graphics.html`; standalone drawings are
`results/report_graphics_top.svg` and `results/report_graphics_obstacle.svg`.
Full test output is retained locally in `results/report_graphics_tests.log`.

## Published baseline refresh (2026-10-24)

The curated `docs/design_reports/baseline.html`, its seven SVG drawings and
`output/pdf/baseline.pdf` were regenerated from the unchanged
`design_baseline.json`. The published HTML embeds the exact fixed-evaluation
record; its source SHA-256 and every supplied input were checked against the
saved baseline. The report gives 52.67914912 kg dry mass and 3230.488448
construction points, with all six current failures retained. The old candidate
reports were preserved and explicitly labeled historical/outdated in the README.

The printable baseline has 27 nonempty Letter pages. All pages were rendered
with Poppler and reviewed as a contact sheet, with full-size inspection of the
overview, plan drawing, digging/rock drawings and first requirements page.
The source revision, box-arm ports and wheel-load failure field were checked
in extracted PDF text. No clipped tables, missing graphics or blank pages were
observed. PDF export used headless Edge with all engineering details expanded.
Windows sandbox profile creation failed on the first export attempt; the
successful export with ordinary temporary-folder access supersedes that attempt.

The graphics implementation retains the 148-test passing result and derivative
check above. Publication introduced no further model changes or optimization;
the fixed-evaluation and source/embedded-record checks passed before release.
Physical calibration and unresolved feasibility are unchanged.
