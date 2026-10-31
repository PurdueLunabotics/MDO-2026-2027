# Report release - 2026-10-30

The baseline report is a fresh fixed evaluation of `design_baseline.json`,
with its current values preserved. It describes the baseline configuration,
conditional on 13 failed modeled checks. No physical constraint was removed
or weakened to publish these reports.

The candidate report uses the final geometry and operating settings from the
existing `results/fixed_hardware_optimization.json` run. No new optimization
was performed to create this report. A fresh model evaluation reproduced all
288 saved output values within 1e-8 relative/absolute tolerance. The saved
optimizer status remains successful, and overall status remains
`failed_or_infeasible` because turning traction fails by 261.119 N.

These are separate cases, not a matched baseline/optimization pair:

| Setting | Current baseline | Saved candidate |
| --- | --- | --- |
| Route timing | Fixed, 45 seconds each way | Calculated |
| Empty/full climbing checks | 30 / 20 degrees | 0 / 0 degrees |
| Required ground clearance | 0.1524 m | 0.1524 m |
| Feed speed | 0.2 m/s | 0.0340718092 m/s |
| Drum RPM | 120 | 120 |
| Excluded optimizer constraints | None | `dig_traction_margin`, `turn_traction_margin` |
| Frozen hardware during saved optimization | Not applicable | Drum diameter, wheel diameter, wheel width |

The excluded checks remain in the candidate's feasibility assessment. All
failed checks and unassessed capabilities are retained in HTML and PDF.
Installed actuator length is diagnostic only in both cases. Soil-force,
capture, efficiency and structural assumptions remain uncalibrated; no new
soil/hardware measurements or experimental validation are claimed.

## Files and provenance

- Baseline inputs: `design_baseline.json`.
- Candidate effective inputs and original run context: `design_optimized.json`.
- Self-contained HTML and geometry: `docs/design_reports/`.
- PDFs: `output/pdf/baseline.pdf`, `output/pdf/optimized.pdf`.
- Full run records, optimizer SQL and verification logs remain in ignored
  `results/`; they are not added to Git. Full numeric records are embedded in
  the published HTML reports.
- Saved candidate analysis SHA-256:
  `20f632be555d0eca9741c624219619e6f3d4be23495d01826826d1534a0463f9`.

The baseline was evaluated with:

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/report_release/baseline.json
```

The PDFs are printed from the published HTML with all noninteractive detail
sections expanded. Geometry, table pagination, headings and page numbers are
checked using rendered page previews. The report heading now preserves the
case name when the deposition mass-properties table is present.

## Verification

- All 218 distinct tests have passing coverage across the full suite and
  focused reruns. The 218-test run completed in 359.647 s and exposed stale
  fixtures in the freeze and PA-04 modules. After updating the fixture's
  extension command to the current bounds and making the profile-mismatch
  fixture explicit, all five freeze tests and all ten PA-04 tests passed.
  No assertion was removed and no production constraint was loosened.
- Earlier temporary-file permission errors were resolved by setting `TMP`
  and `TEMP` to a writable folder under ignored `results/report_release/`.
- The reference `--baseline` evaluation completed. The required default
  `--check-derivatives` step-size check passed; its optimization was stopped
  after 27 driver records without claiming convergence. The reference rocker
  clearance is bounded above by 0.1907 m, below its 0.2032 m requirement.
- A separate `--check-derivatives --maxiter 1` run completed, passed the
  derivative check and retained `failed_or_infeasible`, `optimizer_success=false`
  and CLI exit 1. It is a bounded verification run, not an optimized design.
- Dependency and N2 tests passed, including pose separation and one-time
  deposition energy/time accounting. Published HTML numeric records match
  their source runs, local download/SVG links resolve, and the evaluated
  baseline SHA-256 still matches the unchanged input file.
- Both final PDFs were rendered and inspected: 37 baseline pages and 39
  candidate pages, including the full technical appendices.

## Baseline presentation update - 2026-10-31

The baseline PDF is now titled **Design Baseline**. Named fixed-input baseline
exports use the design audience: the report starts with the design metrics and
omits opening model notices, orange warning boxes, and the methodology section.
Climbing descriptions use design terminology rather than soil-model names.
Detailed analysis reports and embedded numeric/provenance records remain available.
The optimized report is unchanged.

All design inputs, calculated outputs, constraint margins and check statuses
match the previous baseline. The updated 34-page baseline PDF was rendered and
visually inspected. Nine report tests passed; browser checks confirmed that the
opening notices and methodology are absent from print and all 13 failed checks
remain visible in the requirements table. PDF extraction confirmed that the
removed student/BWR wording is absent from every page.
