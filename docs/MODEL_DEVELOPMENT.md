# Developing the rover model

Start with [the README](../README.md) to run a saved design. Read
[MODEL.md](../MODEL.md), the relevant [model card](MODEL_CARDS.md), and
[AGENTS.md](../AGENTS.md) before changing physics. Choose one question and one
discipline per change. Preserve the reference model for comparisons.

## Where to work

| Question | Main files |
| --- | --- |
| Defaults, design bounds, fixed assumptions | `rover/config.py` |
| Six-inch level-ground clearance after wheel/bar deductions | `rover/ground_clearance.py`, `ModelSet.ground_clearance` |
| Model assembly, constraints, optimization | `rover/problem.py` |
| Select or replace a discipline | `rover/models.py` (`ModelSet`) |
| Kinematics and actuator loads | `rover/linkage.py` |
| Frozen PA-04 900 lbf actuator profile/data | `rover/alternatives/pa04_actuator.py`, `data/actuators/`, `docs/PA04_ACTUATORS.md` |
| Drum mass, soil capture, torque, time | `rover/drums.py`, `rover/excavation.py` |
| Wheel mass, route, fresh-site motion | `rover/wheels.py`, `rover/mobility.py`, `rover/relocation.py` |
| Frame loads and mass properties | `rover/structure.py`, `rover/mass_properties.py` |
| Optional frame, motor, and traversal models | `rover/alternatives/` |
| Motor losses and hardware inventory | `rover/motors.py`, `rover/motor_inventory.py` |
| Reports, input files, archive | `reporting/` |
| Parameter sweeps | `studies/` |

Ground-clearance outputs include each axle's radius and underside clearance.
The bottom radius follows `cf_tube_inner_diameter/2 + cf_tube_wall`; the rocker
radius follows `axle_outer_diameter/2`. These existing plate-frame dimensions
remain active design variables where applicable. The reference configuration
supplies their nominal defaults as fixed clearance-only inputs. They do not
add mass or change the existing absolute bottom-pivot/relative rocker coordinates.

## Replace one component

Factories in `ModelSet` create fresh components. For example, after implementing
and importing a real `MyExcavation` class:

```python
from rover.models import ModelSet
from rover.problem import build_problem

problem = build_problem(models=ModelSet(excavation=MyExcavation))
problem.run_model()
problem.cleanup()
```

Other slots cover structure geometry/loads, linkage geometry/loads, wheel and
drum mass, total geometry/mass, mass properties, relocation, trench traversal,
tip stability, obstacle stability/clearance, traversal stability, mobility, and
`slope_traversal`. BWR straight driving and separate empty/full BWR climbs are
the defaults. The climb degree settings are fixed assumptions in `rover/config.py`;
changing them does not change the separate static tipping angle.
See the actual fields in `rover/models.py` before selecting one.

1. Put the optional implementation in its own module under `rover/alternatives/`.
2. Keep physics in pure helpers; let OpenMDAO components adapt their inputs/outputs.
3. Preserve required scalar ports, native units, loading states, and margin signs.
4. Declare every dependency, including inactive branches. New `Discipline`
   subclasses own `DEPENDENCIES`; older components use `rover/dependencies.py`.
5. Use `rover/contracts.py` checks to catch missing sources and incompatible units.
6. Add meaningful physical-limit, conservation, and invalid-domain tests.
7. Update equations, the model card, provenance, report fields, and reproduction
   commands together. Coordinate shared port/config changes in a separate PR.

Never launch simulators, train models, write files, or make network calls inside
`compute()`. Generate data offline. The example below demonstrates injection and
strict interpolation using **synthetic data**, not a calibrated drum:

```powershell
.\.venv\Scripts\python.exe -m examples.empirical_excavation
```

## Contracts that must remain explicit

Schema 5 retires `actuator_body_margin` from optimizer constraints and report pass/fail decisions at the user’s request. Its metre-valued output, equations, dependencies and catalog endpoint diagnostics remain unchanged. It is listed under diagnostic-only screens and does not affect `feasible` or CLI acceptance. Catalog stroke/load domains and all other constraints remain.

Schema 4 introduces axle-relative metre-valued side/top actuator offsets and explicit tail mass/envelope ports; see [ARM_OFFSETS.md](ARM_OFFSETS.md). The earlier schema 3 actuator integration moves `actuation_time` (total cycle arm motion,
seconds) from linkage geometry to loads. Geometry now supplies `lowering_time`
to excavation and loads. Custom replacements must implement those ports; valid
reference behavior is unchanged. [PA-04 documentation](PA04_ACTUATORS.md) explains
the source change, hardware-specific body margin and phase assumptions.

| Quantity | Convention |
| --- | --- |
| Coordinates | Global x forward, y left, z up; legacy linkage side-view y means global z |
| Units | SI, except explicitly declared rpm, Wh, degrees, etc. |
| `wheel_mass` | FOUR bare wheels, excluding drive assemblies |
| `drum_mass` | FOUR drums plus TWO drum motor/gear assemblies |
| `GeometryMass` | Adds each assembly once; `rocker_mass` is already part of structure mass |
| `drum_length` / `drum_diameter` | Length of ONE drum / OUTER cutting diameter |
| `cut_depth` | Scoop protrusion / maximum bite, distinct from total trench depth |
| `payload_mass` | Retained soil across all FOUR drums |
| `fill_fraction` | Storage ceiling; actual payload also depends on reachable soil |
| `lin_act_side_arm_offset` / `lin_act_top_arm_offset` | Signed metres from drum axle along arm / upward normal; positive side is outward. Tail mass included, normal bracket mass/strength unverified. See ARM_OFFSETS.md |
| `dig_energy`, `dump_energy` | Electrical phase energy in J, including startup/idle |
| `dig_power`, `dump_power` | Peak electrical W; peak × duration is not phase energy |
| `*_yaw_inertia` | About the chassis-center yaw axis, not the displaced CG |
| `*_margin` | Signed reserve in declared units; >= 0 passes, subject to saved tolerances |
| Cycle accounting | Subtract simultaneous lowering/cutting overlap ONCE; relocation and deposition turn are separate from the timed route |

Structural dimensions are explicit design variables. Automatic resizing based
on downstream loads would introduce a coupled solve; design that integration
explicitly. Do not hide a sizing loop or double-count masses in a replacement.

`--structure plate-frame` selects coordinated geometry, load, mass-property, and
traversal factories; `--motors x44-standard` supplies shared motor and inventory
providers. Both preserve the reference options. Detailed contracts and guarded
failure diagnostics live in [plate frame](PLATE_FRAME.md),
[motors](X44_MOTORS.md), and [traversal](OBSTACLE_TRAVERSAL.md).

## Data and validation

Use [the experiment plan](EXPERIMENTS.md) to collect data. A fitted response
needs a versioned manifest, units, conditions, validity range, processing script,
and independent validation runs. Split by complete test/soil preparation, not
adjacent samples. Report quantitative errors and choose acceptance thresholds
before comparing models. Reject extrapolation explicitly.

A one-dimensional RPM table at fixed geometry cannot support optimization over
all geometry variables: collect a covering dataset or hold those dimensions
fixed. Compare models at the **same design first**, then reoptimize and explain
the changed active constraints. Solver convergence and agreement with another
simulation are numerical evidence, not experimental validation.

## Solver settings

`build_problem(traversal_mode="calculated")` or `--traversal-mode calculated`
selects the optional calculated route component through `ModelSet`; default
`fixed` retains the prescribed schedule. Required scalar outputs and Mission's
time/energy interface are preserved. Custom mobility factories are not silently
replaced. Dependency declarations include the selected hardware and timing
physics; the fixed-time input has no calculated-mode dependencies. Reports use
actual leg times, and saved cases/archives preserve the mode. The traversal/fill
study remains explicitly fixed. See [calculated traversal](CALCULATED_TRAVERSAL.md)
for search policy, equations, optional diagnostics and calibration limitations.

| Profile | Default solver | Stopping accuracy | Limit | COBYLA radius |
| --- | --- | --- | --- | --- |
| Generic reference | SLSQP | 1e-3 | 200 iterations | n/a |
| Plate frame | COBYLA | 1e-5 | 2000 evaluations | 0.02 |
| X44 on either frame | Selected by frame | 1e-5 | Selected by frame | Selected by frame |

Override with `--optimizer`, `--solver-tol`, `--maxiter`, and `--cobyla-radius`.
Reports save the resolved settings. Numerical accuracy, derivative step size,
report pass tolerances, and physical safety factors are different quantities.
Do not relax physical constraints to make a solver converge. Use multiple
starts for design decisions; keep failed final iterates labeled failed.

In `rover/config.py`, design entries are `(initial, lower, upper, units)`;
assumption entries are `(value, units)`. Profile-specific dictionaries determine
which inputs are active. To hold a design variable fixed for one run, use
`--freeze NAME [NAME ...]` (repeatable), or pass `freeze=["wheel_diameter"]`
to `build_problem`. Values resolve from defaults, the input file, then `--set`.
The selected inputs are validated against their original bounds, removed from
the optimizer's design variables, and recorded as assumptions. JSON metadata
`configuration.frozen_design` retains their values, original bounds and units;
reports still audit those bounds. Other constraints remain enforced and
derivative checks use only the remaining active variables. Unknown names and
inputs already fixed by the selected profile are errors. Use `--baseline`
instead of freezing every variable. `--set` alone does not freeze a variable.
For a permanent profile change, move it into the selected assumptions
configuration and verify its consumers. Preserve competition limits and
documented physical constraints when changing the search space.

An explicitly relaxed study can use `--ignore-constraint MARGIN [MARGIN ...]`
(repeatable), or `build_problem(ignore_constraints=["turn_traction_margin"])`.
Only active margin names are accepted, not mass/dimension limits or variable
bounds. The margin is omitted from the driver, while its component, equations,
dependencies, numerical derivative checks and report pass/fail calculation remain
unchanged. `configuration.ignored_optimizer_constraints` records the selection;
excluded snapshot entries carry `enforced_in_optimization: false`. Full
`feasible`, saved status and CLI success still include excluded checks. An
optimizer success with any failed check remains `failed_or_infeasible` and
returns exit 1. Such a study is not evidence of full design feasibility.

## Verify and submit a change

From the repository root, using the local virtual environment:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/my_change_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --output results/my_change_optimization.json
```

Inspect constraint margins, mass/energy accounting, and explicit N2 connections.
The generic reference cold-start optimization is a documented failure; derivative
checks should still pass. Do not hide that failure or substitute an unrelated
successful run. Also evaluate/optimize the affected profile. See
[current verification and CI](VERIFICATION.md).

For Git beginners, `origin` names the GitHub remote; it is not a branch:

```powershell
git switch main
git pull --ff-only origin main
git switch -c codex/describe-your-change
# Edit and run checks, then inspect exactly what will be committed
git status
git diff
git add path/to/changed_file.py
git diff --cached
git commit -m "Describe the resulting change"
git push -u origin codex/describe-your-change
```

Replace the branch name and `git add` paths with your own. Do not force-add
ignored results, raw runs, or `.venv`. On GitHub, open a pull request with
**base: main** and **compare: your branch**. Describe the problem, equations/data
or interfaces changed, verification, and remaining limitations. Have another
student review it and check CI before merging. After the PR is merged:

```powershell
git switch main
git pull --ff-only origin main
```

GitHub Actions is configured at this repository's root for Windows and Linux.
It runs tests, a reference baseline, and derivative-checked optimization from a
recorded feasible start. That controlled restart does not establish convergence
from arbitrary designs or validate the physics.

An agent prompt students can adapt:

> Read AGENTS.md, MODEL.md, and the relevant model card. Improve only [discipline]
> using [dataset/manifest]. Preserve mission limits and contracts. Identify the
> domain and missing data, implement an optional ModelSet component, update its
> dependencies/card, and add physical-limit and failure-domain tests. Compare
> identical designs before optimization. Report validation errors, assumptions,
> checks, and unresolved failures. Do not invent measurements.
