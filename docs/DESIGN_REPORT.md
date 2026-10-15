# Design inputs, reports and PDF export

There are two design JSON files in the repository root:

| File | Purpose |
| --- | --- |
| `design_baseline.json` | Your chosen design. Edit the values under `inputs`. |
| `design_optimized.json` | The optimizer's candidate. Regenerated when publishing reports. |

There are no separate input copies or analysis JSONs in `docs/design_reports/`.
The calculated outputs, checks, units and provenance are stored inside each
self-contained HTML report. Temporary run data under ignored `results/` is
generated automatically; it is not another baseline to maintain.

## Report order

The first page shows mass, payload, deliveries and the saved construction, mass
and energy points. These score cards use the continuous mission estimate and
the model's configured score weights; the complete-cycle score remains clearly
labeled in Performance & mass.

Rover views follow the overview, then Performance & mass, Requirements & model
checks, loads, model notes and (for the candidate) the optimization comparison.
The final engineering appendix contains, in order:

1. All effective inputs & model assumptions.
2. Design variables and bounds.
3. All output ports.

The earlier grouped "What was evaluated" section is removed. All native-unit
input and output values remain available in the appendix and embedded record.

## Update both reports

From the repository root, run:

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/report_pair/baseline.json
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --optimize --check-derivatives --design-name "Optimized design candidate" --output results/report_pair/optimized.json
.\.venv\Scripts\python.exe -m reporting.archive results/report_pair/baseline.json results/report_pair/optimized.json
```

The first command evaluates your baseline with its values fixed. The second
optimizes from that baseline. The third updates `design_optimized.json` and
both reports in `docs/design_reports/`. It never changes `design_baseline.json`.
It rejects stale reports if the source baseline has changed since evaluation.
No extra input JSONs or report manifest are generated.

A failed optimization still writes its result and returns a nonzero exit code.
It can be published with the failure visible. Passing checks or convergence
does not establish hardware validation. Physics and constraints are unchanged.

To update only a working baseline preview, run the first command and open
`results/report_pair/baseline.html`. Editing HTML does not change model inputs.

## Make the PDFs yourself

1. Open `docs/design_reports/baseline.html` or `optimized.html` in Chrome or Edge.
2. Press **Ctrl+P**, or click **Print / save PDF** near the bottom of the report.
3. Choose **Save as PDF**, **Letter**, **Portrait**, and enable background graphics.
4. Check the preview and save over the corresponding file in `output/pdf/`.
5. Upload that PDF to the club Drive folder. Replace its existing Drive version
   if you want to preserve its sharing link.

Printing includes all checks regardless of screen filters, all effective inputs
and output ports, the top-down travel projection, all three side poses, and the
frame/stability figures. Table headers repeat across pages. Print closes with
the reader's expanded/collapsed state restored. Full provenance remains in the
interactive HTML; bulky provenance JSON and playback controls are omitted from
print. No upload, commit or push happens automatically.

## Input and result conventions

- Inputs use native model units: metres, kilograms, newtons, rpm, Wh, etc.
  Display conversions to mm, MPa, degrees or litres do not change saved values.
- Priority is model defaults, then input JSON, then explicit CLI `--set` values.
  Missing inputs use identified defaults. Source inputs are never overwritten.
- With `--inputs`, fixed evaluation is the default; `--optimize` must be explicit.
- Failed checks remain failed. Unassessed capabilities remain labeled unassessed.
- Dry mass counts four bare wheels and four drums plus two drum drive assemblies
  once. Complete-cycle energy includes idle electronics for remaining mission time;
  continuous mission energy and score are labeled separately.
- Soil, material and efficiency assumptions remain uncalibrated. Actual obstacle
  traversal, motor continuous operation, connections and soil response still
  require independent validation. Top-view overlap is not a collision test.

The HTML contains the exact numeric run record in the `analysis-data` JSON
script element for reproducibility. Intermediate N2 and optimizer recordings
stay in `results/`. Reporting code is in `reporting/`, outside the physics model.
