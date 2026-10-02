# Improve one discipline without rewriting the rover

Start with the default model, run the tests, and open a generated HTML report.
Choose ONE question, for example: "Does this drum retain less soil at high RPM?"
Work on a branch (`codex/…` for agent-created branches). Keep the reference model
available so the club can compare your replacement against the same mission.

## Component contracts

`rover/models.py` is the replacement entry point. `build_problem(models=ModelSet(
excavation=MyExcavation))` replaces only excavation. Factories must create a fresh
component each time. Other replaceable slots are structure_geometry,
linkage, wheel_mass, drum_mass, geometry, linkage_loads, structure_loads,
mass_properties, relocation, trench_traversal, tip_stability, and mobility.

Read CONFIG_INPUTS, COUPLED_INPUTS, OUTPUT_UNITS and DEPENDENCIES on the component.
For older components read setup() and dependencies.py. Preserve required outputs
even when your internal calculation changes. Add new diagnostic outputs freely;
wire/report them explicitly. Add new coupled inputs only with a documented source.
Avoid cycles: structural dimensions are design variables; structure mass precedes
payload, and load constraints follow payload. Automatic structural resizing would
introduce a coupled solve and must be designed and tested as such.

`rover/contracts.py` checks every required output, scalar shape, native unit and
input source before execution. This prevents OpenMDAO from silently supplying an
automatic default for a forgotten connection. Convert EDEM/Chrono/sensor units
inside the adapter. Replacements may attach a JSON-serializable `model_evidence`
dictionary; it is saved with the factories and source-file hashes.
The empirical example records its dataset hash, conditions and domain this way.

## Parallel team ownership

Keep alternative implementations in separate files under `rover/alternatives/`.
The default model stays available for an identical-design comparison. Teams can
work on these slots independently and combine factories in one `ModelSet`:

| Team | Primary slots | Reference files | Expected evidence |
| --- | --- | --- | --- |
| Drum architecture | `drum_mass`, `excavation` | `rover/drums.py`, `rover/excavation.py` | CAD/BOM mass; retained mass, torque, power and time over the architecture's operating domain |
| Wheels | `wheel_mass`, `mobility`, eventually `trench_traversal` | `rover/wheels.py`, `rover/mobility.py`, `rover/trench.py` | CAD/BOM mass; wheel loads, slip, drawbar/turning data and trench-exit tests or validated Chrono runs |
| Suspension/frame | `structure_geometry`, `structure_loads`, `linkage`, `linkage_loads` | `rover/structure.py`, `rover/linkage.py` | CAD sweep and mass, load/deflection tests, actuator force |

For example, one selected drum architecture can supply both its mass and soil
response while a wheel team supplies its mass and mobility response:

```python
models = ModelSet(drum_mass=MyDrumMass, excavation=MyExcavation,
                  wheel_mass=MyWheelMass, mobility=MyMobility)
problem = build_problem(models=models)
```

Each factory above must be imported from a real optional model module; these
names are placeholders. No alternative is selected by default. Wheel/drum mass
outputs are totals for FOUR wheels/drums; `drum_mass` includes the TWO reference
motor/gear assemblies unless a replacement explicitly preserves that boundary.
`GeometryMass` adds those outputs once to the dry mass. Do not add the same motor,
payload, shaft or hub mass in two components.

Prefer a new optional module and its own tests to simultaneous edits of
`config.py`, `problem.py`, `models.py`, `components.py`, or the report schema. A
new architecture may need extra parameters or may make a reference output
inapplicable. Propose that shared contract change in a separate, reviewed PR;
do not silently repurpose an existing port or relax a constraint. Several drum
architectures can coexist as separate modules, with one selected per run.

| Convention | Meaning |
| --- | --- |
| SI | m, kg, s, N, Pa, J; explicit rpm and W*h conversions; route-yaw diagnostics use deg |
| x/y/z | forward/left/up for mass properties and external simulators |
| linkage x/y | legacy side-elevation y means global z; adapt it once at the boundary |
| drum_length | axial length of ONE drum; four drums, two driven pairs |
| wheel_mass | total dry mass of FOUR wheels, excluding drive motors/gears |
| drum_mass | total dry mass of FOUR drums plus TWO drum motor/gear assemblies in the reference accounting |
| drum_diameter | OUTER cutting envelope, not the storage shell diameter |
| cut_depth | maximum bite / radial scoop protrusion; not total trench depth |
| payload_mass | retained mass across FOUR drums at end of digging |
| dig_time | drum spinup + cutting; includes dig_overlap_time within lowering |
| actuation_time | two nominal actuator strokes; subtract overlap ONCE in mission |
| dig_energy/dump_energy | electrical phase energy, J, including start-up/idle |
| dig_power/dump_power | peak electrical power, W; never multiply peak by duration for energy |
| cut_force | total instantaneous cutting-force envelope across all four drums |
| *_yaw_inertia | kg*m^2 about chassis-center yaw axis, not about displaced CG |
| *_margin | signed: >=0 feasible; retain units and meaning |
| relocation_time/energy | move empty to a fresh lane, outside the 45 s route; trench exit physics is not modeled |
| trench_traverse_margin | wheel diameter minus trench depth, m; must be >= 0 |
| wheel_first_margin | travel drum bottom minus wheel diameter, m; provisional step clearance |
| tip_stability_margin | static CG distance to the closest wheel-center support edge with one drum pair full, the other empty, and both fully extended, m |
| tip_worst_mask | retained diagnostic: 3 means front pair full and rear pair empty; the reverse (12) is symmetric |
| pivot_x / actuator_pivot_x | separate longitudinal positions of arm and actuator chassis pivots; front/rear mirrored |

## Your first replacement

Run `python -m examples.empirical_excavation`. It injects an alternate excavation
component and interpolates a tiny **synthetic** table. Default runs never use it.
The table has explicit units, provenance and fixed conditions. It rejects requests
outside its domain. It demonstrates software plumbing, not validated physics.

1. Copy the example into a clearly named model module. Keep the default unchanged.
2. Write the experimental question and input/output contract in a model card.
3. Collect or simulate data using docs/EXPERIMENTS.md. Store immutable raw data
   externally; check in a small processed table, manifest and processing script.
4. Separate fitting and validation cases by complete test run/soil preparation,
   not random adjacent time samples. Report error and scatter, including failure
   conditions. Choose acceptance thresholds before comparing models.
5. Replace only the relevant response or component. A one-dimensional speed table
   at fixed geometry is NOT valid for full geometry optimization. Freeze those
   design variables or collect multidimensional data covering them.
6. Test exact limits, monotonic trends where physically justified, conservation,
   invalid inputs, out-of-domain handling, and a complete mission.
7. Update dependencies, equations, model card, report metadata and example commands.
8. Compare identical designs first; then optimize both models and explain changed
   active constraints. Keep failed solves visible. Run several initial guesses for
   a design decision: a single local optimum is not a global result.

## Agent prompt students can adapt

> Read AGENTS.md and the model-development documents. Improve only [discipline].
> Use [specific dataset/manifest]. Preserve the documented mission and contracts.
> First identify its domain, calibration/validation split and missing information.
> Implement an optional model through ModelSet, update its card/dependencies, and
> add independent physical and invalid-domain tests. Do not invent measurements
> or change limits to obtain convergence. Compare reference and replacement on
> the same design; report validation error, remaining assumptions and test results.

## Merge workflow for inexperienced contributors

Pull main before starting, create a small topic branch, and give each student a
discipline/module to avoid simultaneous edits of config/problem/mission. A PR
should state: question, equations, data provenance, domain, changed interface,
validation errors, tests, baseline comparison and limitations. Ask another student
to run the commands from a clean environment. Resolve conflicts by checking the
physical contract, not blindly accepting one file version. Never commit .venv,
licensed software, large EDEM/Chrono runs, secrets or credentials.

This folder is presently a subdirectory of the all-years Git repository. Its
`.github/workflows/ci.yml` activates when the folder becomes a standalone repo.
GitHub does not run nested workflows from the parent repository; testing this
folder within that parent would require a separate parent-root workflow. CI is a
software regression check, not empirical validation. The portable local command
is `python -m unittest discover -s tests -v`.
