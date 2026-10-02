# Current simulator verification

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
