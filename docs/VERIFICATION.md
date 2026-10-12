# Current simulator verification

Checked 2026-10-11 after activating the requested 0.40 m raised-wheel gravity
screen. The user confirmed centered rocker pivots, an average-angle differential
and an estimated +0.10 m offset above the wheel-center line. This offset is
editable. The 0.50 m deep ditch is circle-like but lacks width/radius, and remains
explicitly unverified in documentation, report metadata and the HTML report.
See [OBSTACLE_TRAVERSAL.md](OBSTACLE_TRAVERSAL.md) for equations/domain.

- Local .venv full suite: all 51 tests passed. Thirteen obstacle checks cover
  independent rectangle/triangle limits and moment balance, rigid rocker/shaft
  geometry and platform heights, mirrored cases, mounted/suspension mass
  conservation, zero-height limits, high-CG failure, below-axis first-branch
  maximum, invalid domains, smooth valid-pose derivatives and N2 connections.
- Original baseline: `results/obstacle_traversal/baseline.json` is now infeasible.
  Its maximum wheel-height reach is 0.39331 m, 0.00669 m below the new requirement.
  The flagged guarded tip value is not a prediction at 0.40 m. All pre-existing
  baseline outputs are unchanged exactly; old constraints/bounds are unchanged.
- Changing only starting wheelbase to 0.46 m gives a feasible baseline, saved at
  `results/obstacle_traversal/reachable_start.json`, with 0.04722 m reach reserve
  and 0.06795 m tip reserve.
- Default-start derivative check passed, but SLSQP failed. The final iterate
  `results/obstacle_traversal/optimization.json` remains infeasible with width,
  drive/load/torque and structural failures. A solve from wheelbase 0.46 m also
  failed: `reachable_optimization.json`, frame deflection about -5e-5 m. A
  stiffer initial tube (0.065 m diameter) failed stress and deflection:
  `stiff_start_optimization.json`. None is an accepted design.
- The previous reference optimum already passes the added screens. A restart
  from it with starting frame diameter increased by 0.001 m passed its derivative
  check and converged feasible: `results/obstacle_traversal/reference_restart.json`,
  2060.4052 modeled points, 0.07221 m reach reserve and 0.10741 m tip reserve.
  Its fully loaded travel pose has about 29.3 degree pitch and 15.6 degree roll.
  The numerical test fixture `tests/fixtures/obstacle_feasible_start.json`
  records this full starting vector and provenance. The optimizer regression
  now uses that feasible start, retaining its derivative, success, feasibility
  and objective-improvement assertions. Default config values remain unchanged
  apart from adding the two requested geometry/requirement inputs.

Energy accounting matches cycles*cycle_energy to 2e-15 Wh in the restart. The
reference drum and existing port/dependency tests pass. Rocker mass is a subset
of structure_mass; new mounted-body ports exclude it and the wheel/drive masses.
These are analytical/software checks without hardware calibration. The optimizer
results are local, and the cold-start failures remain unresolved. Passing the
added necessary static screens does not verify joint load equilibrium/stops,
climbing path, torque/traction, collisions, soil or dynamic traversal.

Checked 2026-10-10 after adding a selectable actuator pin along the arm.
All 38 tests passed using the local .venv. New checks cover pin/drum circle
closure, numerical virtual work, unchanged gravity work at matched drum poses,
invalid fractions/triangles, arm-length conservation in guarded trials,
derivatives at an interior pin position, N2 dependencies, and SVG pin endpoints.
The default fraction is fixed at 1 in ASSUMPTIONS; moving it into DESIGN enables
optimization without changing its physical meaning.

- Reference baseline: `results/arm_attachment/baseline/design.json`, feasible.
  Pre-change versus post-change reference outputs and constraint margins differ
  by at most 1.2e-16 in their native units (floating-point roundoff).
- Default derivative-checked optimization:
  `results/arm_attachment/reference_optimization/design.json`, converged feasible
  at 2060.8794 modeled points, with the reference drum and tip attachment.
- Matched-pose interior baseline:
  `results/arm_attachment/interior_baseline_matched/design.json`, feasible at
  fraction 0.95, with actuator travel/start/dig lengths of
  0.352987/0.424022/0.653223 m and a 0.570 m B-to-pin distance.
- Optimizing the fraction as an additional design variable from the original
  start failed SLSQP's line search; the saved final iterate in
  `results/arm_attachment/final_optimization/design.json` remains infeasible,
  with approximately -8.2e-5 m frame-deflection margin. An additional start at
  fraction 0.9 also failed. Those failures are not accepted designs.
- A separate restart from the failed adjustable-pin design, increasing only
  the starting frame diameter by 0.001 m, passed its derivative check and
  converged feasible at `results/arm_attachment/warm_restart/design.json`:
  2057.5359 points and fraction 0.9121. This demonstrates local start sensitivity,
  not a global optimum or experimentally validated attachment position.

These are analytical/software checks, with no new calibration data. Actuator
body/arm interference and pin-bracket strength/mass remain outside the model;
the elementary structural envelopes and equivalent linkage mass distribution
are still approximations. Constraint limits, energy accounting and test
acceptance thresholds were not relaxed.

Checked 2026-10-02 for the collaboration handoff: 32 tests passed, baseline
feasible, and derivative-checked SLSQP converged feasible. Separating wheel and
drum mass into replaceable components changed no baseline or optimized design
values or reported physics outputs at the reference inputs. Combined wheel,
drum, frame and linkage replacements were exercised in one test. The workflow
under `.github/workflows/` is ready for a standalone repository, but has not
run on GitHub while this folder remains nested in the all-years repository.

Checked 2026-10-02 after making track width a design variable and narrowing
TipStability to one full pair versus one empty pair at full extension. All 31
tests passed. The baseline was feasible. The derivative-checked default SLSQP
solve converged feasible at `results/extended_tip_optimization.json`: 2060.9
modeled construction points, 0.574 m track width, 0.487 m wheelbase and 0.0523 m
static tip margin for that prescribed state. A solve started from the prior
width-variable optimum converged feasible at 2043.3 points, confirming that
these are local optima. The 2060.9-point design would also pass the former
all-pattern tip screen with about 0.0314 m margin; its score gain is therefore
from finding a different local design, not evidence that relaxing the tip screen
was necessary. Other half-loaded arm positions need a software interlock;
the present model does not verify the interlock or dynamic dumping stability.

## Earlier verification under broader tip screens

Checked 2026-09-30. These are software and numerical checks, not experimental
validation. The reference physics coefficients are uncalibrated.

- `python -m unittest discover -s tests -v`: all 30 tests passed. They include
  independent circle closure for offset actuator pivots, a 16-pattern uneven-load
  tip-envelope check with a failure case, reachable-soil and wheel/trench checks,
  dependency audit, energy accounting and a complete optimization.
- `python main.py --baseline --output results/asymmetric_tip/baseline.json`:
  feasible, with a 0.108 m static uneven-load tip margin.
- `python main.py --check-derivatives --output
  results/asymmetric_tip/optimized.json`: derivative step-size check passed.
  SLSQP converged with all modeled constraints passing in 27 derivative
  evaluations and 77 driver/model evaluations. The result has 2016.0
  construction points, 42.41 kg total retained payload and a 0.0133 m static
  tip margin. The worst pattern is both front drums full and both rear drums
  empty (mask 3), with front/rear arm positions allowed to differ.
- A second SLSQP solve started at wheelbase 0.380 m and drum diameter 0.359 m,
  saved at `results/wheelbase_probe/optimization.json`. Its derivative check
  passed and it converged feasible in 16 derivative evaluations and 40 driver
  evaluations. The final design has 2041.8 points, 0.453 m wheelbase,
  0.321 m drum diameter and 0.0320 m static tip margin. This higher-scoring
  result confirms start-point sensitivity; it does not establish a global optimum.

The static tip result is only a level-ground CG/support-polygon screen. It does
not validate a moving dump, dump reaction, slopes, wheel unloading or changing
terrain support. The mission still deposits at the raised travel pose; the
stability screen also checks extended positions. The wheel-diameter screens
remain placeholders for trench traversal and travel obstacle clearance. They do
not assess slip, sinkage, contact dynamics or whether the rover can exit a
disturbed-soil pit. The reported SLSQP result is local, not a certified global
optimum.

Generated `results/` files are ignored local artifacts; earlier reports use
different equations and are not directly comparable.
