# Design inputs, reports, and PDF export

Edit **`design_baseline.json`** in the repository root. It contains your chosen
design and assumptions under `inputs`; update `revision` when values change.
**`design_optimized.json`** is a separate saved optimizer candidate. Intermediate
analysis files in ignored `results/` are generated data, not extra baselines.

| Saved design | HTML | PDF |
| --- | --- | --- |
| Baseline | [Browser report](design_reports/baseline.html) | [PDF](../output/pdf/baseline.pdf) |
| Candidate | [Browser report](design_reports/optimized.html) | [PDF](../output/pdf/optimized.pdf) |

Download/clone the files before opening HTML in a browser. GitHub displays its
source rather than the interactive report. Every HTML embeds its full numeric
run and provenance; the curated folder needs no duplicate analysis JSON.

## Evaluate and refresh

Run these commands from the repository root, in order:

```powershell
# Fixed evaluation: preserves all supplied values
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/report_pair/baseline.json

# Separate optimization, starting from the same baseline
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --optimize --check-derivatives --design-name "Optimized design candidate" --output results/report_pair/optimized.json

# Publish the pair and update design_optimized.json
.\.venv\Scripts\python.exe -m reporting.archive results/report_pair/baseline.json results/report_pair/optimized.json
```

For a preview, stop after the first command and open
`results/report_pair/baseline.html`. The second command changes design variables
within their bounds while keeping assumptions fixed. Neither run edits the
baseline. The third command updates both curated HTML reports/SVGs and the root
candidate JSON; it rejects stale sources or unrelated report pairs. It does
**not** export PDFs. Editing HTML does not change physics or model inputs.

An optimization failure returns a nonzero exit code and still saves its final
result when evaluation completes. Inspect that result before publishing; the
archive retains its failed status. A fixed evaluation may report failures while
returning 0 because the evaluation itself completed.

## Export and share a PDF

1. Open the refreshed HTML in Chrome or Edge.
2. Press **Ctrl+P**, or select **Print / save PDF** in the report.
3. Choose **Save as PDF**, **Letter**, **Portrait**, with background graphics on.
4. Check the preview and save to `output/pdf/baseline.pdf` or `optimized.pdf`.
5. Upload the PDF to the club Drive folder. Use a new version of an existing
   Drive file if you want to preserve its sharing link.

Printing includes all checks regardless of screen filters, all three side poses,
top view, and input/output appendices. It restores the screen's expanded state
afterward. Upload, commit, and push are manual; regenerate and re-export after
changing inputs. To share the interactive curated reports, keep their companion
SVGs and relative file links together or share the repository.

## Read the report correctly

Page one shows the design overview and continuous construction, mass, and energy
points. Rover views follow, then performance/mass, requirements/checks, loads,
model notes, and any optimization comparison. The final appendix lists effective
inputs/assumptions, design variables/bounds, and all output ports, in that order.
Complete-cycle results are labeled separately from continuous estimates.

Input precedence is **model defaults → input JSON → CLI `--set`**. Native units
are retained in JSON and appendix tables; display conversions to mm, MPa, degrees,
or litres do not change saved inputs. With `--inputs`, fixed evaluation is the
default; optimization needs `--optimize`.

Failed checks stay failed and unassessed capabilities stay unassessed. A top
projection is not a collision check. Calibration and physical validation remain
open; see [model cards](MODEL_CARDS.md) and [current verification](VERIFICATION.md).
For advanced extraction, the exact run record is in the HTML's `analysis-data`
JSON element. Working N2 diagrams and optimizer histories remain in `results/`.
