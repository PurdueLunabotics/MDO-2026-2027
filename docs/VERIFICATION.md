# Current simulator verification

## Optional plate-frame profile, 2026-10-14

Implemented `--structure plate-frame` after the student's frame/load discussion.
This is a preliminary analytical model, not plate FEA or experimental validation.
The reference configuration/factories remain available. See PLATE_FRAME.md for
the exact model, material assumptions, scope and additive/profile-specific ports.

- Local .venv full suite: **78 tests passed in 50.405 s**, including 14 new frame
  tests. Independent actuator force/moment balance, truss nodal equilibrium and
  elastic-work equality, box-section subtraction, mass/energy conservation,
  geometry/packing/mechanism failure cases, width/span trends, N2 sources,
  smooth total derivatives and a complete frame optimizer regression all pass.
- `main.py --structure plate-frame --baseline --output
  results/plate_frame/baseline.json`: the new structural screens pass. The rover
  remains infeasible at the original 0.40 m wheelbase: requested obstacle reach
  margin -0.006687198 m, posed tip margin -0.022280099 m and clearance margin
  -0.006787198 m. No obstacle requirement was relaxed.
- `main.py --structure plate-frame --check-derivatives --output
  results/plate_frame/optimization.json`: derivative step-size check passed;
  SLSQP converged with all modeled constraints passing. Score 341.8832 ->
  **2282.8164**, dry mass 38.2183 kg, payload 51.2393 kg.
  Driver recorded 52 model evaluations and 22 derivative evaluations.
  Final aluminum stress envelope 119.308 MPa, deflection envelope 3.6724 mm,
  CF effective bending stress 121.528 MPa. Plate-strip buckling reserve
  1223.213 N; rocker Euler reserve 5594.519 N; arm Euler reserve 10.0064 N.
  Plate force-balance residual 5.68e-13 N. The 5 mm limit, metal allowable,
  strength factor and structural load multiplier were not loosened.
- Fixed user dimensions remain plate thickness 6.35 mm and CF ID 25.4 mm.
  Several final section widths/walls/spans meet their exploratory lower bounds;
  actual connector dimensions, local wall behavior and CF product properties
  can change the result. The model does not certify the selected sections.
- The reference drum, route, score and mission energy boundaries remain intact;
  no rocker/drum/wheel mass is counted twice. Frame CG and sloping-rocker obstacle
  CG are profile-specific rather than using the old frame rectangle lump.
- Reference CLI baseline and derivative-checked default optimization were also
  run: their scores/failures reproduce the earlier cold-start reference result.
  Reference optimization remains **failed_or_infeasible** (2294.9805 points,
  failed stress/deflection and five other constraints), not an accepted design.
  Its saved report is results/plate_frame/reference_optimization.json.

CF properties and the four-radial/four-perimeter plate topology are provisional.
Two-force strips omit plate-junction bending and whole-plate buckling. Tube/axle
flexure screens are not assembled-frame FEA; connections, local box/CF wall
buckling and recovery/cutting/obstacle structural reactions require separate
verification. No hardware or simulator calibration was performed.

Per-output N2 files now use `<output_stem>_n2.html`, preventing reports of different
profiles in one directory from linking to whichever diagram was written last.

## Obstacle drum-wheel clearance integration

Checked 2026-10-14 after completing the student's obstacle drum–wheel clearance
component and its integration. This is an analytical software/envelope check,
not simulator agreement, hardware calibration or experimental traversal validation.

- Local .venv full suite: **64 tests passed**. The 13 clearance tests include
  zero-pose and hand-calculated pitch/roll limits, rigid-distance conservation,
  cross-pair minimum selection, sphere touching/overlap signs, an explicit case
  of separated cylinders with overlapping enclosing spheres, finite/valid input
  domains, unreachable/boundary rejection, smooth derivatives, actual linkage
  sources and active/reportable constraints. Existing drum/mass, interface and
  N2 dependency checks also pass.
- The active ModelSet.obstacle_clearance factory compares 64 pairs at four
  prescribed single-wheel-raised travel poses. The required buffer reuses
  mechanism_clearance=0.015 m. Wheel/drum envelopes are rigid outer cylinders
  enclosed in spheres; a negative sphere bound is inconclusive about actual
  cylinder collision and is conservatively rejected. Default physical inputs,
  old factories, constraints, solver tolerance, route and score are unchanged.
- `results/obstacle_clearance_baseline.json`: original baseline remains
  infeasible. Reach margin -0.0066872 m, guarded tip margin -0.0274500 m and new
  clearance margin -0.0067872 m. Validity is zero; its gap bound is a guarded
  diagnostic at 0.3933124 m, not a prediction at the requested 0.40 m height.
- With only wheelbase changed to 0.46 m, baseline reach/support/clearance pass;
  conservative gap bound 0.0888998 m and clearance margin 0.0738998 m.
- Required default-start derivative check passed, but SLSQP failed with a line
  search exit in `results/obstacle_clearance_optimization.json`. The final iterate
  is retained as failed/infeasible (motor, battery, linkage/actuator, drum torque
  and structure failures). A run from wheelbase 0.46 m likewise passed derivatives
  but failed battery power, actuator force, drum torque and stress; see
  `results/obstacle_clearance_reachable_optimization.json`. Neither is accepted.
- The original obstacle fixture has a new margin of -0.1598726 m. It is retained
  unchanged as a historical pre-clearance starting vector. A new explicit
  numerical fixture, tests/fixtures/obstacle_clearance_feasible_start.json, is
  verified feasible and uses additional initial motor/battery/structural reserves,
  interior cut/RPM settings and shifted pivots. The historical reference wheel
  coefficient 8 kg/m² is explicit. These are synthetic numerical inputs, not
  measured hardware. CI and the optimizer regression now use this fixture with
  the same derivative, optimizer-success, final-feasibility and objective-
  improvement checks; the test additionally checks starting feasibility.
- Derivative-checked optimization from that interior start converged feasible:
  `results/obstacle_clearance_interior/optimization.json`, 2173.4109 → 2177.8442
  modeled points. Final reach reserve 0.0571379 m, gravity tip reserve 0.0019441 m,
  conservative gap bound 0.1077071 m and clearance margin 0.0927071 m. Energy
  accounting residual run_energy-cycles*cycle_energy is 0.0 Wh at stored precision.
  This locally converged result is not a global-optimum or hardware-performance
  claim. The full regression suite reproduces a successful feasible improvement.

JSON and HTML include the additive clearance fields, validity/evaluated height
and model evidence. HTML distinguishes guarded diagnostics and level-ground
schematics from the four posed checks. The alternatives package description now
reflects its two active obstacle factories. No fitting dataset exists or is
implied. Other hardware interference, intermediate climbing poses, flexible
wheel/frame deformation, terrain dynamics and the underspecified 0.50 m ditch
remain outside the screen. The default-start solver failure remains unresolved.

Historical verification records follow.

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
