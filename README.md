# Lunabotics rover MDO starter

A modular OpenMDAO sizing model for a four-wheel, side-rocker, skid-steer rover with front/rear bucket-drum pairs. It is intended for students improving one discipline at a time, including with coding agents. The reference physics is **uncalibrated**; passing constraints is not hardware validation.

## Design baseline reports

For a fixed-design evaluation, use `--baseline`, or load a named SSOT case with
`--inputs baseline.json`. Input-file runs hold values fixed unless `--optimize`
is explicit. Failed constraints remain visible and do not prevent a baseline
report from being generated.

The PDF-ready report separates inputs, performance, loads, checks and geometry.
See [the report guide and input format](docs/DESIGN_REPORT.md).

`design_baseline.json` is the only baseline input file to edit.
Edit its `inputs` and revision, then regenerate the fixed report:

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/baseline/evaluation.json
```

Open `results/baseline/evaluation.html`. Reporting code lives in `reporting/`,
outside the physics package. The guide also describes club-only PDF sharing
through Drive; viewing a generated report does not require Python.

There are two design JSON files: `design_baseline.json` (your chosen values) and
`design_optimized.json` (the optimizer's candidate). Reports and geometry are kept
in [docs/design_reports](docs/design_reports/README.md). Full calculated results
are embedded in the HTML reports; intermediate run files stay under ignored `results/`.
Ready-to-share PDFs are in [output/pdf](output/pdf). Printing includes all checks,
the top-down view, all three side views and the effective input/output appendices.
The edited baseline is preserved even when a modeled check fails.

## Start here

For the purchased standard Kraken X44 motors, add `--motors x44-standard` to
your plate-frame command. This freezes drive gearing at 100:1 and both motor
models at 12 V, uses catalog motor/gear inventory masses and the official curve,
and leaves drum ratio exploratory. See [motor equations and limitations](docs/X44_MOTORS.md).
The [motor interface/file map](docs/MOTOR_INTERFACE.md) explains the cleanup:
the regular disciplines supply mechanical work and peak power to one shared
motor/gearbox helper. Electrical estimates divide by provisional combined
efficiencies (drive 0.72, drums 0.70); torque-speed limits are separate. There
are no electrical time histories or X44-specific discipline copies.
Current/thermal limits are undecided; a passing result does not verify continuous
operation or 30° climbing. Omitting this option retains generic reference motors.

To run the simplified motor profile from the saved feasible numerical start:

```powershell
$motorStart = Get-Content tests/fixtures/x44_standard_feasible_start.json | ConvertFrom-Json
$motorOverrides = @()
$motorStart.inputs.psobject.Properties | ForEach-Object {
    $motorOverrides += '--set'
    $motorOverrides += "$($_.Name)=$($_.Value)"
}
.\.venv\Scripts\python.exe main.py --structure plate-frame --motors x44-standard --optimizer COBYLA --cobyla-radius 0.01 --maxiter 1500 --check-derivatives @motorOverrides --output results/motor_simple/optimization.json
```

The plate-frame profile now defaults to COBYLA with a 0.02 initial trust
radius, up to 2000 evaluations and 1e-5 stopping accuracy. X44 profiles also use
1e-5 accuracy. The unchanged plate-frame and X44 optimizer assertions pass with
these numerical settings. All physical constraints and starting designs are
retained. The historical failed solves remain documented in VERIFICATION.md;
these results do not establish convergence from every start or a global optimum.

Use `--solver-tol`, `--optimizer`, `--maxiter` and `--cobyla-radius` to
override numerical settings explicitly. Reports record the resolved settings.

1. Read [MODEL.md](MODEL.md) for physics, units, assumptions and sources.
2. Read [model cards](docs/MODEL_CARDS.md) to see what each component can and cannot predict.
3. Follow [model development](docs/MODEL_DEVELOPMENT.md) for interfaces, a student-agent prompt and branch/PR workflow.
4. Use [experiment plans](docs/EXPERIMENTS.md) for bench tests, Altair EDEM and Project Chrono.
5. Coding agents should read [AGENTS.md](AGENTS.md) before edits.
6. Read [implementation verification](docs/VERIFICATION.md) for the latest checks.

This folder can be the root of a standalone GitHub repository. Its
`.github/workflows/ci.yml` runs the tests, baseline, and derivative-checked
optimization on pushes and pull requests once the folder is published as a repo.
While it remains a subdirectory of the all-years repository, GitHub will not
discover that nested workflow; a workflow in the parent repository root would
be needed to test it there.

The CI optimization uses the recorded feasible start in
`tests/fixtures/traversal_stability_feasible_start.json`, matching the optimizer regression
test. It still requires passing derivative checks, optimizer success and final
feasibility. The baseline step evaluates the default design separately and can
report infeasibility. Default-start optimization failures remain documented in
`docs/VERIFICATION.md`; this workflow does not establish convergence from every
starting design or experimentally validate the rover.

The reference CI fixture selects COBYLA with a 0.02 initial trust radius in
scaled design coordinates. The generic reference profile still defaults to
SLSQP and its cold-start failure remains reported. The plate-frame regression
uses its profile's COBYLA defaults from the same original starting design.
These are numerical settings; they do not change physical safety factors or
constraints.

## Install and run

Python 3.12+; Windows commands below (on macOS/Linux use `.venv/bin/python`). The existing local .venv can be reused.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe main.py --baseline
.\.venv\Scripts\python.exe main.py
.\.venv\Scripts\python.exe main.py --check-derivatives
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m examples.empirical_excavation
```

The last command demonstrates a replaceable response using explicitly synthetic data. It is NOT calibration and is never selected by default. No EDEM/Chrono installation is needed to run the reference model. Those simulators should generate validated data offline, outside optimization evaluations.

Outputs include self-contained HTML, JSON, three side-view SVGs, a top-down SVG, optional frame/stability SVGs, a sparse N2 diagram and optimizer SQLite history. Baseline mode has no optimizer recording. Default names are results/baseline.* or results/optimization.*; `--output results/my_design.json` sets a separate stem. The CLI returns nonzero if optimization fails or the final design is infeasible. Reports retain failed final iterates and show constraint margins. Derivative checks are numerical checks, not physical validation.

## Change assumptions

For the proposed CF-tube / aluminum-plate / box-rocker frame, use
`--structure plate-frame`. The reference model remains the default for comparisons.
For example:

```powershell
.\.venv\Scripts\python.exe main.py --structure plate-frame --baseline --output results/plate_frame/baseline.json
.\.venv\Scripts\python.exe main.py --structure plate-frame --check-derivatives --output results/plate_frame/optimization.json
```

The fixed sheet thickness is 6.35 mm and CF tube ID is 25.4 mm. Profile inputs
live in `PLATE_FRAME_DESIGN` and `PLATE_FRAME_ASSUMPTIONS` in `rover/config.py`;
the reported design/assumption lists reflect the selected profile. Spoke width,
CF wall/span, box rocker and axle sections, and centered axle height can vary.
CF material properties and the plate topology are provisional. See
[the frame equations, load scope and limitations](docs/PLATE_FRAME.md).

Plate revision 2 has seven strips and three cutouts, with a lower boundary
through the suspension hub. Hub/lower mounts may share a height. The legacy
filled-chassis collision screen is inactive in this profile; actual frame and
equipment clearance needs separate verification. Both profiles now use Froude
as a diagnostic only, with constant capture/lift and unverified speed response.
Torque, power, startup energy and drum-wheel clearance constraints remain.

```powershell
.\.venv\Scripts\python.exe main.py --baseline --set soil_density=1800 --output results/dense_soil.json
.\.venv\Scripts\python.exe main.py --optimizer COBYLA --cobyla-radius 0.02 --maxiter 5000 --output results/cobyla.json
```

`rover/config.py` contains DESIGN `(initial, lower, upper, units)` and ASSUMPTIONS `(value, units)`. `--set` changes a design's starting value, not whether it is optimized. To freeze a design variable, move it into ASSUMPTIONS. Preserve the explicit competition/user dimension limits.

## What version 2 models

- Stationary excavation is limited by reachable fresh soil and drum storage; partial fill is allowed. Optimized scoop cut depth is distinct from trench depth.
- A provisional traverse screen requires wheel diameter at least as large as trench depth.
- Travel drums clear a wheel-diameter step while loaded-route stability screens excessive height; both are provisional.
- Full, empty and one-pair-loaded CG/yaw inertia using actual drum positions.
- Static tip margin with one drum pair full, the other empty, and both arms fully extended; other half-loaded poses require a software interlock.
- Fully loaded travel-pose gravity support and suspension reach with each wheel
  raised 0.40 m. The average-angle differential and centered pivots use an editable
  `rocker_pivot_offset=0.10 m` estimate. The original 0.40 m wheelbase baseline is
  now infeasible for this added requirement; a 0.46 m wheelbase baseline passes.
  See [obstacle geometry and limitations](docs/OBSTACLE_TRAVERSAL.md). The 0.50 m
  deep, circle-like ditch remains unverified until its width/radius is known.
- Full-load travel-pose static support on a 30-degree planar slope across all
  headings. An additional 41-height, one-wheel articulation screen checks both
  the hull and ideal differential wheel reactions through the 40 cm raise.
  Negative normals reject the prescribed four-contact pose. These checks do
  not verify slipping or actual climbing; see
  [TRAVERSAL_STABILITY.md](docs/TRAVERSAL_STABILITY.md).
- Adjustable wheelbase and track width, both screened against the stowed envelope, structure, and mobility constraints.
- Independent horizontal placement of the actuator and arm pivots.
- Selectable actuator pin position along the arm: `actuator_attachment_fraction`
  is measured from the chassis pivot; 1.0 retains the drum-tip connection and 0.5
  connects halfway along the arm. Its domain is 0 < fraction <= 1; the drum stays
  at full arm length. Use `--set actuator_attachment_fraction=0.8` to select a fixed
  pin position; actuator lengths may need to change for closure. It is fixed in
  ASSUMPTIONS by default to preserve the reference optimizer. To optimize the pin
  position too, move it to DESIGN as `(1.0, 0.001, 1.0, None)`; SLSQP can fail with
  that additional variable, so inspect final margins and use multiple starts.
  JSON reports include the fraction and `actuator_attachment_distance` in metres,
  and geometry drawings show the pin as D when it is inside the arm.
- Separate dig/dump RPM, diagnostic Froude numbers, independent gearing, torque-speed screens and startup energy; no calibrated high-speed granular-loss model.
- Tube-frame/arm mass and elementary stress, stiffness and buckling constraints.
- Explicit cutting/lowering overlap and power, with fresh-site relocation and finite excavation area.

The route remains 2/3/3/1 m with 90/45/45-degree turns in exactly 45 s each way. The separate 180-degree deposition turn carries one loaded pair. Electronics, actuator motion, excavation, dumping and fresh-site relocation are included in mission energy and time. Score uses berm volume, dry inspection mass and consumed Wh:

```text
mass_points   = 4.4*V_cm3/(t_min*dry_mass_kg)
energy_points = 1.5*V_cm3/(t_min*consumed_energy_Wh)
objective     = mass_score_weight*mass_points + energy_score_weight*energy_points
```

Whole-cycle estimates are reported alongside continuous throughput. Other competition points and detailed inspection remain outside this objective. Old result files were generated under different physics; regenerate them before drawing conclusions. Do not compare the old unlimited-soil optimum against a v2 result as if only the optimizer changed.

## Files and ownership

| File | Responsibility |
| --- | --- |
| rover/config.py | Design bounds, fixed assumptions, route/limits |
| rover/models.py | ModelSet component factory injection |
| rover/discipline.py / contracts.py | Explicit interfaces and checks for missing ports, sources and wrong units |
| rover/linkage.py | Kinematics, swept clearance, actuator loads |
| rover/wheels.py | Four-wheel reference mass; independent replacement slot |
| rover/drums.py | Four-drum reference assembly mass; independent replacement slot |
| rover/excavation.py | Reachable soil, storage, cutting, granular response, drum drive |
| rover/structure.py | Tube mass and structural load screens |
| rover/mass_properties.py | Three loading states, CG and yaw inertia |
| rover/tip_stability.py | Static uneven-load tip screen at full arm extension |
| rover/relocation.py | Fresh-site relocation |
| rover/trench.py | Provisional trench traversal screen, replaceable by an SCM surrogate |
| rover/mobility.py | Fixed-time route force, power and energy |
| rover/components.py | Shared adapter, drive/battery/fixed mass accounting and mission |
| rover/alternatives/ | Replaceable team models; obstacle stability and drum–wheel clearance selected by default |
| rover/dependencies.py | Legacy component dependency declarations |
| rover/calibration.py | Strict response-table validation; no extrapolation |
| rover/problem.py | Assembly, signed constraints, optimizer |
| reporting/report.py / html_report.py | Result schema, reporting, geometry/replay |
| examples/ | Executable optional model and synthetic fixture |
| tests/ | Independent physical limits, conservation, interfaces and regression |

Use ModelSet to replace one discipline instead of modifying the mission to fit a simulator. Update dependencies, model cards, data provenance, meaningful tests and docs together. Keep simulator launch/training/file writes out of compute(). Report failed-domain evaluations rather than silently falling back. A single locally converged design is not a global optimum or an experimentally validated design.

