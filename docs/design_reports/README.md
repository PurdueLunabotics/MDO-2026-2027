# Baseline and optimized reports

Maintain just two design files, both in the repository root:

- [design_baseline.json](../../design_baseline.json): your chosen values; edit this file.
- [design_optimized.json](../../design_optimized.json): the optimizer's candidate.

| Case | PDF | Browser report |
| --- | --- | --- |
| Your baseline | [PDF](../../output/pdf/baseline.pdf) | [HTML](baseline.html) |
| Optimized candidate | [PDF](../../output/pdf/optimized.pdf) | [HTML](optimized.html) |

Each HTML report contains the complete calculated results and provenance.
There are no duplicate input JSONs, analysis JSONs or manifest in this folder.
Your baseline file is never overwritten when publishing reports.

These reports were refreshed after the student's latest baseline edits.
The baseline retains two failed screens: structural stress and articulated
wheel normal load. The optimized candidate converged and passes modeled checks.
These remain uncalibrated conceptual estimates, not hardware validation.

Both reports include a top-down travel projection, three side views, frame and
stability figures, performance, loads, all checks and native-unit data tables.
Projected overlap alone does not establish physical interference.

Page one includes construction, mass and energy points (continuous estimates).
Rover views come next, followed by performance and the model checks. The final
appendix lists all effective inputs/assumptions, design variables/bounds, then
all output ports. The grouped "What was evaluated" section is removed.

See [the report guide](../DESIGN_REPORT.md) for regeneration and PDF export.
After changing inputs, regenerate the reports and export both PDFs again.
