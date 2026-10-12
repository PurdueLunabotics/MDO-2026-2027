# Lunabotics rover MDO starter

A modular OpenMDAO sizing model for a four-wheel, side-rocker, skid-steer rover with front/rear bucket-drum pairs. It is intended for students improving one discipline at a time, including with coding agents. The reference physics is **uncalibrated**; passing constraints is not hardware validation.

## Start here

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

Outputs include self-contained HTML, JSON, three geometry SVGs, a sparse N2 diagram and optimizer SQLite history. Baseline mode has no optimizer recording. Default names are results/baseline.* or results/optimization.*; `--output results/my_design.json` sets a separate stem. The CLI returns nonzero if optimization fails or the final design is infeasible. Reports retain failed final iterates and show constraint margins. Derivative checks are numerical checks, not physical validation.

## Change assumptions

```powershell
.\.venv\Scripts\python.exe main.py --baseline --set soil_density=1800 --output results/dense_soil.json
.\.venv\Scripts\python.exe main.py --optimizer COBYLA --maxiter 5000 --output results/cobyla.json
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
- Separate dig/dump RPM, provisional granular-flow validity limits, independent gearing, torque-speed screens and startup energy.
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
| rover/alternatives/ | Optional team models; none selected by default |
| rover/dependencies.py | Legacy component dependency declarations |
| rover/calibration.py | Strict response-table validation; no extrapolation |
| rover/problem.py | Assembly, signed constraints, optimizer |
| rover/report.py / html_report.py | Result schema, reporting, geometry/replay |
| examples/ | Executable optional model and synthetic fixture |
| tests/ | Independent physical limits, conservation, interfaces and regression |

Use ModelSet to replace one discipline instead of modifying the mission to fit a simulator. Update dependencies, model cards, data provenance, meaningful tests and docs together. Keep simulator launch/training/file writes out of compute(). Report failed-domain evaluations rather than silently falling back. A single locally converged design is not a global optimum or an experimentally validated design.

