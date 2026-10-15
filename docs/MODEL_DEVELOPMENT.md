# Developing the rover model

Start with [the README](../README.md) to run a saved design. Read
[MODEL.md](../MODEL.md), the relevant [model card](MODEL_CARDS.md), and
[AGENTS.md](../AGENTS.md) before changing physics. Choose one question and one
discipline per change. Preserve the reference model for comparisons.

## Where to work

| Question | Main files |
| --- | --- |
| Defaults, design bounds, fixed assumptions | `rover/config.py` |
| Model assembly, constraints, optimization | `rover/problem.py` |
| Select or replace a discipline | `rover/models.py` (`ModelSet`) |
| Kinematics and actuator loads | `rover/linkage.py` |
| Drum mass, soil capture, torque, time | `rover/drums.py`, `rover/excavation.py` |
| Wheel mass, route, fresh-site motion | `rover/wheels.py`, `rover/mobility.py`, `rover/relocation.py` |
| Frame loads and mass properties | `rover/structure.py`, `rover/mass_properties.py` |
| Optional frame, motor, and traversal models | `rover/alternatives/` |
| Motor losses and hardware inventory | `rover/motors.py`, `rover/motor_inventory.py` |
| Reports, input files, archive | `reporting/` |
| Parameter sweeps | `studies/` |

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
tip stability, obstacle stability/clearance, traversal stability, and mobility.
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
| `actuator_attachment_fraction` | Chassis-pivot-to-pin distance / arm length; 0 < f <= 1 |
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
which inputs are active. To hold a design variable fixed during optimization,
move it into the selected assumptions configuration and verify its consumers;
`--set` alone does not freeze it. Preserve competition limits and documented
physical constraints when changing the search space.

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
