# Current verification status

Checked locally on **2026-10-15**, using the repository's `.venv`. This page
describes the current implementation; Git history retains earlier run notes.
Software verification is separate from physical calibration and validation.

## Status at a glance

| Check | Result |
| --- | --- |
| Full unit suite | 130 tests passed |
| Generic reference fixed evaluation | Completed; failed constraints remain visible |
| Generic reference derivative step-size check | Passed at its starting design |
| Generic reference cold-start SLSQP solve | Known failure; saved as failed/infeasible, exit 1 |
| Curated student baseline | Retains structural-stress and articulated-wheel-load failures |
| Curated optimized candidate | Solver succeeded; all saved modeled checks pass |
| Sensitivity grid | 96 cells, 192 actual starts; 88 converged feasible selected designs |
| Hardware/soil predictive validation | Not established |

The reference cold-start final iterate fails width, battery peak power, actuator
force, drum torque, and structural deflection checks. A baseline command returns
0 after a valid evaluation even when its checks fail. Optimization returns 0
only when the driver succeeds and its final saved checks pass.

## Reproduce the checks

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/verification/reference_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --output results/verification/reference_optimization.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/verification/student_baseline.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --optimize --check-derivatives --output results/verification/student_optimization.json
```

The third command's known optimizer failure must remain visible. Inspect its
derivative message and saved status separately. Numerical derivatives compare
finite-difference step sizes at smooth points; they do not validate equations
against hardware or certify behavior at branch transitions.

The suite covers independent geometry and physical limits, mass/mission-energy
conservation, signed failure domains, explicit dependencies and connections,
motor curve/gear/loss accounting, model replacement, and optimizer regressions.
The reference drum and mass boundaries remain covered. Full-suite local logs
are in ignored `results/pdf_ready_tests.log`; the seven study-specific tests
also passed in `results/traversal_fill_tests.log`. These local logs are not
available in a fresh clone; rerun the commands above.

## Reports and provenance

Only `design_baseline.json` and `design_optimized.json` are maintained design
inputs. Each curated HTML embeds its exact numeric run, native inputs/outputs,
checks, source hash, and model provenance. Publishing rejects stale or unrelated
baseline/candidate pairs and preserves the baseline byte for byte.

The current baseline retains:

- `structure_stress_margin = -655854.8516 Pa`
- `rock_sweep_wheel_load_margin = -0.8535612742 N`

The curated candidate passes its saved checks; this does not establish a global
optimum. Report tests preserve failed statuses, source fidelity, and archive
guards. Browser/PDF review checked first-page scores, section order, all printed
checks regardless of screen filters, top view, three side poses, full native
appendices, restored screen state, and no table overflow. Reviewed PDFs have
27 baseline and 28 candidate pages. Regenerate and re-export after input changes.

## Sensitivity study

The 12 × 8 traversal/fill grid uses unchanged physics, bounds, and solver
settings. Every cell has two recorded starts; only successful, feasible attempts
are eligible. Five 15 s cells retain failed numeric iterates and three retain
exceptions without usable final states. Error reports invent neither score nor
geometry. An independent no-load speed bound rules out the prescribed 15 s
profile with the fixed X44 drive gearing; its optimistic lower limit is 17.42 s.

An independent audit checked all 96 coordinates and 192 attempts, source/model/
data hashes, fixed nonswept assumptions, winner selection, classification, report
links, embedded records, geometry, route timing, payload ceilings, and mass and
energy balances. The reference N2 has 309 explicit connections and no auto-IVC
sources. Resume reused all 96 cells. Browser checks covered cell/keyboard
selection, side poses, successful and failed reports, offline operation, and
mobile layout. Full results and audit stay under ignored
`results/sensitivity/traversal_fill/`.

An additional fixed-hardware evaluation held the 45 s / 0.4 study candidate
unchanged across all 88 cells at 30–180 s. Every current modeled check passed.
At 45 s / 0.9 it scored 2348.03 versus 3694.07 for the reoptimized candidate;
its payload stayed reach-limited at 53.66 kg. This supports modeled feasibility
robustness, not equal performance or physical validation. See
[the study guide](SENSITIVITY_STUDY.md) for findings and reproduction.

## CI and remaining uncertainty

[GitHub Actions](../.github/workflows/ci.yml) runs on Windows and Linux with
Python 3.12. It executes the full unit suite, a generic reference baseline, and
derivative-checked optimization from
`tests/fixtures/traversal_stability_feasible_start.json` using that fixture's
COBYLA settings. It requires solver success and final feasibility; it does not
claim the generic cold start works. Motor CSVs retain LF bytes across platforms
so strict manifest checksums remain reproducible.

Unresolved physical evidence includes retained soil versus speed/fill, rolling
and turning resistance, motor current/thermal duty, real gearbox/assembly mass,
CF tube/joint properties, and loaded articulation/tilt tests. The plate-frame
structural profile covers mirrored full-load level-ground gravity; cutting,
recovery, obstacle reactions, and connection strength need separate work.
Ditch width/profile and actual climbing capability remain unverified. No
simulation agreement, solver result, or passing software test closes those gaps.
