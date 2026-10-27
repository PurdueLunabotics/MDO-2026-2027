# Optional calculated traversal timing — revision 1

`--traversal-mode fixed` is the default and preserves the original prescribed
schedule. `--traversal-mode calculated` selects `CalculatedTraversalMobility`
through `ModelSet.mobility`. The route, mass boundaries, mission score, Earth
gravity, constraints, dumping and excavation equations are unchanged.

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --traversal-mode fixed --output results/timing_fixed.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --traversal-mode calculated --output results/timing_calculated.json
```

Add `--optimize`, `--freeze ...`, and other existing study options as needed.
A named case may save `"traversal_mode": "calculated"` beside `structure` and
`motors`; the CLI overrides it. Older cases without this field remain fixed.
The traversal/fill sensitivity study explicitly retains fixed timing.

## Scope and contract

The optional implementation lives in `rover/alternatives/calculated_traversal.py`.
It predicts separate loaded outbound and empty return candidate profiles.
`travel_speed` and `acceleration` are upper bounds in this mode, not guaranteed
achieved values. `traversal_time_one_way` remains a positive legacy input but is
inactive; changing it has no effect on any calculated-mode output.

The existing output contracts remain:

| Output | Meaning |
| --- | --- |
| `travel_time` | Outbound plus return, including all route turns, seconds |
| `route_translation_time` / `route_turn_time` | Translation and turn portions of that same round trip |
| `turn_time` | Separate half-loaded deposition turn; reference timing retained |
| `relocation_time` | Fresh-site motion; reference component/timing retained |
| `mobility_energy` | Route plus deposition electrical energy, joules, including actuator idle power |
| `actual_travel_speed` | Highest straight ground speed in the selected profiles |

Mission already consumes `travel_time`, so cycle duration, electronics energy,
cycle count, delivered volume and score use the selected duration without a
separate scoring branch. Route distance and requested turn angles are unchanged.
The half-loaded deposition turn and fresh-site relocation are not automatically
made motor-limited by selecting this mode. Their existing failure screens remain.
Loaded and empty leg times can coincide when both reach the same speed and
acceleration upper bounds; their BWR slip, motor demand and energy can still differ.

## Profile selection

Each route segment starts and ends at rest. For distance `d`, allowed peak ground
speed `v` and acceleration `a`:

```text
peak = min(v, sqrt(d*a))
duration = d/peak + peak/a
```

For each loading state, the pure helper searches one common straight acceleration
`0 < a <= acceleration`. Route turn acceleration is
`a*turn_accel_fraction`. Straight and turn peak speeds have separate limits;
the requested turn speed upper bound is `travel_speed*turn_speed_fraction`.
Return profiles reverse the route sequence and turn directions as before.

Straight profiles reuse the existing equal-normal-load BWR solves at steady
motion and the candidate acceleration. For slip `s`, per-wheel torque `T`, wheel
radius `r`, and the selected gearmotor's wheel-shaft speed limit:

```text
aggregate rim force = 4*T/r
ground speed <= (1-s)*r*motor_angular_speed_limit
ground speed <= (1-s)*available_mechanical_power/aggregate_rim_force
```

Both accelerating and steady operating points constrain speed. Supported BWR
solutions must have slip strictly below one. Wheel-shaft speed, torque and power
use the selected provider, including a configured current limit. No wheel-size
speed bonus is inserted: radius enters the actual motor/soil equations.

Turns retain the original rolling resistance, assumed scrub coefficient and yaw
inertia law. Their torque-speed and mechanical-power envelopes constrain the
candidate turn speed. This is not a BWR lateral/combined-slip turning model.
Legacy Coulomb traction, equal-torque axle-load, pressure and stability screens
remain evaluated and enforced unless explicitly excluded from the optimizer.
Profile selection does not silently waive them; a selected candidate can still
be an infeasible rover design.

Speed selection includes the existing driver's inward reserves of 0.3 N aggregate
motor force and 0.4 W mechanical power, plus 1e-7 in the respective units for
roundoff. These are numerical reserves, not new physical calibration factors.

The deterministic acceleration search samples fractions 1/16, 1/8, 1/4, 1/2 and
1 of the acceleration upper bound, then refines the best neighboring bracket
with bounded scalar minimization (`xatol=1e-9`, 64 iterations maximum). If no
sample supports motion, it explicitly halves the lowest fraction up to 20 times.
The shortest valid evaluated candidate is retained, including endpoints. A failed
refinement does not invalidate an already supported sample, nor establish an
optimum. This policy is a search for a fast common-acceleration profile; it is not
globally time-optimal control. All reuse is local to one helper call; there is
no persistent cache, training, network access or file I/O in model evaluations.
When maximum acceleration supports both requested speed upper bounds, the
analytic minimum-time profile in this family is returned directly; no scalar
search is needed and its numerical result is unchanged.

An unsupported steady operating point, unusable motor, invalid contact geometry
or absence of positive supported motion raises an explicit analysis error.
There is no fixed-time fallback or clipping of an invalid result into feasibility.

## Energy, diagnostics and limitations

The same selected profiles feed the shared mobility force/work accounting.
Straight work reuses BWR shaft torque and slip; legacy compaction/rolling work
is not added a second time. Turn work retains resistance work plus startup
kinetic energy. Nonregenerative braking uses the existing symmetric rest-to-rest
closure; braking torque/slip, wheel rotational inertia and detailed controller
dynamics are not simulated. Motor/gear electrical efficiency is applied once.
Four actuator idle loads are charged over actual route plus deposition duration.

Additional report outputs give outbound/return time, selected acceleration,
straight speed limit, maximum steady/accelerating BWR slip and sinkage, and
source-range transfer flag counts. HTML and console reports explicitly identify
calculated mode and the inactive fixed-time input. Schema version 2 gains these
optional diagnostics and `model_metadata.traversal_mode` /
`configuration.traversal_mode`; existing fixed reports retain their old values.

This is **uncalibrated conceptual timing**, using the existing versioned GRC-1
soil manifest `data/mobility/bekker_wong_reece/grc1_loose.json` and selected motor
provider. No data are fitted and no coefficients are changed. Soil source-window
violations are reported, not hidden. The existing independent BWR accuracy-gate
failure remains; see [BWR provenance and errors](BEKKER_WONG_REECE.md).
Current limit zero still means undecided: the voltage/peak-curve envelope alone
does not establish continuous, battery-sag or thermal capability.

Tests cover an independent unlimited-motor timing limit, distance closure,
torque/speed/power bounds with slip, weaker-motor and wheel-radius effects at the
same mass, idle and mission energy conservation, invalid/no-drive failures,
explicit dependencies, smooth total derivatives, fixed-mode regressions and
CLI/case/report mode selection. These are software/physical-limit checks, not
experimental validation. Loaded/empty timed route and skid-turn tests with wheel
speeds, motor current, yaw motion and soil preparation are needed for validation.

## Local verification commands

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/calculated_change_reference_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --maxiter 1 --output results/calculated_change_reference_bounded.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --traversal-mode calculated --check-derivatives --output results/calculated_timing_baseline.json
```

The reference optimization command is deliberately bounded to one iteration;
its unsuccessful final iterate remains failed/infeasible. This is not a new
converged reference design. Fixed-mode outputs, margins and whole-cycle results
were also compared exactly against numerical snapshots captured before the
shared route-accounting refactor. Calculated-mode checks inspect explicit N2
connections, phase/mission energy conservation and preservation of failed
constraints. No hardware calibration or full feasible reoptimization is claimed.
