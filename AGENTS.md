# Instructions for students' coding agents

Read README.md, MODEL.md, and docs/MODEL_DEVELOPMENT.md before changing physics.
Read the relevant card in docs/MODEL_CARDS.md. Ask the student about actual
hardware/soil data when it is required; do not invent calibration or claim a
simulation was run when only a placeholder was written.

## Working agreement

- Make the smallest discipline-level change that addresses the task. Keep the
  stable input/output contract (names, SI units, per-drum versus total conventions,
  loading state, energy versus peak power, margin sign). Explain contract changes.
- Pure helpers calculate physics; OpenMDAO components adapt those helpers. Use
  ModelSet factories for alternate components. Do not put simulator launches,
  network calls, stochastic training, file writes, or hidden caches in compute().
- Put each optional drum, wheel or suspension model in its own module under
  rover/alternatives/. Select it through ModelSet; keep reference defaults intact.
  Mass boundaries are wheel_mass = FOUR wheels and drum_mass = FOUR drums plus
  TWO drum motor/gear assemblies. GeometryMass sums them once. Coordinate any
  shared port/config/constraint change in a separate integration PR.
- Every fitted relationship needs a model card, versioned data manifest, validity
  range and independent validation. Label assumptions and synthetic data. Do not
  silently extrapolate, clip an invalid prediction into feasibility, or substitute
  missing data with the default physics without reporting it.
- Maintain explicit dependencies, including inactive branches. New Discipline
  subclasses own DEPENDENCIES; older components use rover/dependencies.py. Update
  equations, documentation, report schema and meaningful tests together.
- Keep Earth gravity, the dimension caps, mass limit, route and score semantics
  unless the user specifically changes them. Do not weaken constraints or loosen
  tests just to make optimization converge. A failed final iterate must stay failed.
- Do not treat optimizer convergence or agreement with another simulation as
  experimental validation. Report quantitative errors and the data used to test.
- Do not edit another student's uncommitted work, commit/push/merge automatically,
  or add simulator installers, secrets, large binaries or raw runs to Git.
- This is a standalone repository. Do not modify parent directories without
  checking their instructions and scope. Confirm the root with git rev-parse --show-toplevel.

## Verification before handing off

Use the local .venv when available. Run:

```text
python -m unittest discover -s tests -v
python main.py --baseline --output results/my_change_baseline.json
python main.py --check-derivatives --output results/my_change_optimization.json
```

Add an independent physical limit or conservation test and a failure-domain test
for a new model. Check the reference drum after shared interface changes.
Derivative tests cover smooth points, not arbitrary branch boundaries. Inspect
constraint margins, energy accounting, and N2 connections. Summarize changes,
tests, calibration status, limitations, and unresolved failures for the student.
