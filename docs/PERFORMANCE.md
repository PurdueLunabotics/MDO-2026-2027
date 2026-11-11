# Profiling and bounded development runs

The normal reference and plate-frame defaults are unchanged. Development options
use the same physics, declared dependencies, validity guards, requirements,
constraint margins and score. No soil or hardware calibration is introduced.

## Profile one design

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --profile-model --output results/model_profile.json
```

This evaluates the model exactly once, saves the normal numeric snapshot and a
`model_profile_profile.txt` cumulative CPU profile, and exits without optimizing.
The timing includes OpenMDAO's first-evaluation setup and profiler overhead. Use
identical resolved inputs for before/after comparisons; do not compare timings
from baseline files edited between runs. A profile is software evidence, not
experimental validation.

## Keep development searches short

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --quick --vary wheelbase arm_length pivot_x deposition_actuator_length --output results/geometry_quick.json
```

`--quick` implies optimization and defaults to:

- COBYLA, at most 80 objective evaluations and 60 seconds of driver time;
- initial radius 0.1 in normalized design coordinates;
- the existing profile's stopping tolerance and all physical checks;
- numeric JSON and a compact console summary, without SQL history, HTML, SVGs
  or N2 generation.

`--vary` holds every unlisted active variable fixed at its resolved input value.
Original bounds remain checked and the frozen inputs are identified in the
saved configuration. There is no automatic choice of a variable subset. A
restricted study may be infeasible even when a broader design space is feasible.
Conflicting `--vary`/`--freeze` selections and unknown or inactive names fail
explicitly.

Override budgets with `--maxiter` and `--time-limit`. Explicit numerical settings
take precedence over the preset. Budgets are checked before the next complete
objective evaluation, including when SciPy raises its minimum COBYLA budget.
One model evaluation cannot be interrupted; the starting and final fixed checks
also add time. These are bounded development searches, not hard process deadlines.
Gradient optimizers are rejected for timed runs because a gradient can require
many additional model evaluations.

An interrupted run preserves consistent final inputs/outputs and remains
`failed_or_infeasible`, with optimizer success false and exit code 1. A feasible
snapshot alone is not optimizer convergence. Numerical stopping accuracy never
changes a physical acceptance threshold.

## Choose reporting cost separately

`--report full` retains the original SQL history and complete reports.
`--report final` creates final HTML/geometry/N2 without recording every trial.
`--report json` saves numeric results only. The default remains `full`, except
for quick/profile runs. `--n2` is incompatible with JSON-only output. JSON gains
an optional `performance` object with timing, report mode and budget metadata;
existing physical output names, units and schema remain unchanged.

## Repeated work removed

Calculated traversal now passes its selected steady and accelerating wheel
operating points explicitly into energy accounting, instead of solving those
same points again. Both 64-node soil-integration panels are evaluated together;
panel order, integration nodes, root tolerances and failure domains are retained.
All reuse is local to one evaluation; no persistent or approximate cache crosses
optimizer trials or finite-difference perturbations. Fixed/custom route planners
retain their existing responders.

Compliant tipping is still a major cost. Limiting-angle searches currently feed
the active equilibrium-validity margin, so they have deliberately not been
removed or deferred as report-only work. The existing uncommitted tipping
implementation is preserved.

Development checks should start with targeted physical/domain tests and a short
driver run. Record whether full verification completed; a time-limited or
interrupted suite must never be reported as a complete pass.

## Verification of this change (2026-11-07)

- 30 focused tests passed: eight development-control/reuse tests, nine BWR
  physical/domain tests, nine calculated-traversal tests (including smooth total
  derivatives), and four existing optimizer-setting tests.
- At identical frozen resolved inputs, all 320 saved outputs matched within
  `rtol=1e-8, atol=1e-7`, and all constraint pass/fail decisions were unchanged.
- A paired profile reduced soil operating-point solves from 10 to 6. Overall
  profiled elapsed times were 1.76 and 2.00 seconds; this single pair does not
  establish a whole-model runtime improvement. Compliant tipping remains the
  dominant cost. The quick preset primarily bounds development time and avoids
  recording/report overhead.
- The reference fixed baseline completed with its N2/report. A reference
  wheel-diameter/width derivative check passed before a two-evaluation bounded
  search. The student-profile four-variable search stopped at four evaluations.
  Both searches correctly retained failed/infeasible status; no convergence or
  new feasible rover baseline is claimed.
- Full test discovery was interrupted after 77 completed passing tests, with no
  failures logged before interruption. This is not a complete-suite pass. The
  unrestricted reference derivative/optimization attempt was also interrupted;
  the completed bounded derivative check covers the two wheel variables above.

The tests include independent force integration, work conservation, invalid
domains, custom component preservation, exact objective budgets and consistent
final inputs/outputs after a timed interruption. No hardware or soil validation
data were introduced. Existing uncommitted physics and baseline changes were
preserved.
