# Traversal-time and drum-fill sensitivity

Optimize the hardware independently at each combination of two fixed operating
assumptions, or evaluate the same baseline hardware throughout the grid, then
inspect scores and geometry in an offline heat map. Neither
`design_baseline.json` nor `design_optimized.json` is changed.

## Fixed baseline hardware, without optimization

```powershell
.\.venv\Scripts\python.exe -m studies.traversal_fill --baseline --workers 6
```

This evaluates `design_baseline.json` once per grid cell. All hardware design
variables remain fixed; only traversal time and fill ceiling change.
`design_optimized.json` and `--seed` are ignored, and no optimizer runs.
Use `--inputs another_baseline.json` to evaluate a different saved rover.

The heat map and both line charts show predicted construction points even when
BWR or other checks fail. A **!** on a colored cell and orange line-plot markers
flag failed checks; select a cell for their names and margins. Status is
`baseline`, feasibility remains explicit, and optimizer success is unset.
These are conditional score predictions, not accepted feasible designs.
Gray × indicates an evaluation error with no usable numeric result.
The BWR equations and constraint thresholds are unchanged.

Fixed-design and optimization runs use separate fingerprints and output folders.
`--maxiter` has no effect on baseline evaluation or its fingerprint. The default
command without `--baseline` still reoptimizes each cell.

For a single baseline report without the sensitivity grid:

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --baseline --output results/baseline_check.json
```

## Run, view, or resume

```powershell
.\.venv\Scripts\python.exe -m studies.traversal_fill --workers 6
```

The runner prints **`Study output:`** and **`Open/refresh:`** before optimization
starts. Open that `index.html` in Chrome or Edge. Default outputs now live in
**`results/sensitivity/traversal_fill/runs/<fingerprint>/index.html`**, where the
folder name is the first 16 characters of the run's fingerprint.
Refresh during a run to see completed cells. Click a cell for its top view,
travel/compact/digging side views, dimensions, both solver-start outcomes,
storage utilization, and a detailed PDF-ready report. Color represents
construction points; x is one-way time and y is fill ceiling. The two line plots
show slices at the source baseline's other assumption.

Use fewer workers on a smaller computer. Repeating the command reuses completed
cells only when source inputs, starts, source/data hashes, package versions, and
solver settings match. Changing those selects a separate default run folder;
old results are preserved. Worker count does not change the fingerprint. Grid
axes are also excluded, so a small trial can later be expanded in the same run.
For a small trial before the full run:

```powershell
.\.venv\Scripts\python.exe -m studies.traversal_fill --times 30 45 90 --fills 0.2 0.4 0.9 --workers 2
```

Run the full command afterward to retain those cells and compute the rest.
Changing `--maxiter` creates a different run because it changes solver settings.

### If you see `Stale case` or `Stale study/cache`

This means the folder contains results from different inputs, code/data, or
solver settings; it does not mean the output folder moved. Earlier versions
always used `results/sensitivity/traversal_fill/` and stopped when that cache
became outdated. With the updated runner, repeat the default command above:
it selects a separate matching run automatically. The original
`results/sensitivity/traversal_fill/index.html` remains the historical study.

`--output` still selects an **exact folder**, for example:

```powershell
.\.venv\Scripts\python.exe -m studies.traversal_fill --workers 6 --output results/sensitivity/traversal_fill_current
```

An explicit folder must be empty or contain a matching study. If it is stale,
omit `--output` or choose another unused folder. The runner checks its manifest
and all cached cells before writing, including cases outside the requested grid.
It never combines stale and current predictions or deletes the old study.

All outputs are ignored by Git. A fresh clone must run the study or receive a
shared copy. To share detailed results, copy the **complete output folder**;
the heat map itself is self-contained, but its detailed report links need
`cases/`. Print any case through the [report guide](DESIGN_REPORT.md).

The following paths are relative to the printed study output folder:

| Output | Contents |
| --- | --- |
| `index.html` | Offline interactive heat map and geometry |
| `summary.csv` | One row per cell, scores, status, and design variables |
| `study.json` | Full grid, source inputs/hashes, settings, versions, and summaries |
| `cases/t45_f0p4/` | Example: actual attempt records, logs, selected analysis, report, and SVGs |

## Method and meaning

The default grid has 12 times (15, 30, …, 180 s) × 8 fill ceilings
(0.2, 0.3, …, 0.9), or **96 combinations**. Both outbound and return routes use
the chosen time; route distances and turns stay fixed. Deposition and relocation
remain separate. `fill_fraction` scales usable storage capacity: actual payload
is the smaller of that storage target and reachable captured soil.

Each cell has two deterministic starts: baseline design variables and saved
candidate design variables. Both inherit every nonswept assumption from the
baseline. The runner uses the source hardware profiles and their default solver
(COBYLA for plate-frame), with 2000 iterations/evaluations per start by default;
the exact settings are saved in each attempt's model metadata. The historical
completed study used plate-frame / standard-X44, COBYLA, 2000 evaluations per
start, stopping accuracy 1e-5, and initial radius 0.02. Bounds,
physical constraints, inward numerical reserves, score weights, Earth gravity,
and 900 s mission are unchanged.

Select the highest continuous score among **successful and modeled-feasible**
attempts. A failed solver is never eligible, even if its last iterate passes.
If neither succeeds, retain the least violated numeric final iterate as a failed
diagnostic; exceptions without usable final states get an error report without
invented geometry or scores. Gray cells mean no accepted solution was found,
not a general infeasibility proof. There is no interpolation across failures.
These are local optimization results from two starts, not proven global optima.

## Historical completed study: 2026-10-15

The findings below belong to the earlier model, before the current BWR mobility
and aluminum box-arm changes. They are retained as historical results and do
**not** describe a rerun of today's baseline. The updated runner requires a new
run for the current model; this output-management fix does not rerun all 96 cells.

All 96 cells and 192 actual starts completed. **88 cells have accepted designs.**
All eight 15 s cells fail: five retain failed numeric iterates and three retain
exceptions without usable final states. An independent optimistic bound rules
out this row for the prescribed drive profile: at minimum track width it needs
1.693 m/s peak speed, while the largest permitted wheel with fixed 100:1 X44
gearing reaches 1.458 m/s at no load. That implies at least **17.42 s one way**
before drive load. Different gearing or a different controller/profile would
need a separate analysis.

The highest sampled accepted result is **4348.57 points at 30 s / 0.9 fill**:
41.88 kg dry mass, 100.31 kg modeled payload, 123.36 s cycle, 34.50 Wh continuous
mission energy. These are predictions under uncalibrated assumptions, not
measured carrying capacity. Selected slice values:

| One-way time | Fill ceiling | Reoptimized points |
| --- | --- | ---: |
| 30 s | 0.4 | 2765.96 |
| 45 s | 0.4 | 2348.03 |
| 180 s | 0.4 | 1058.92 |
| 45 s | 0.2 | 1430.50 |
| 45 s | 0.9 | 3694.07 |

### Similar drawings do not mean identical hardware

Traversal time mainly changes throughput in this study. At fill 0.4, the 30 s
and 180 s designs differ by about 5 mm in arm length and less than 1 mm in
track/wheelbase. Fill has a larger sizing effect. At 45 s, moving from 0.2 to
0.9 changes battery capacity from 46.3 to 71.7 Wh, actuator rating from 2.00 to
2.87 kN, track from 570 to 611 mm, and arm length from 594 to 545 mm.

Every accepted design's wheelbase is within 1% of its upper bound. The two
starts differ in score by roughly 0.7% at the median and up to 5%; bounded
geometry and local optimizer behavior both contribute to similar layouts.

A separate **fixed-hardware evaluation** held the 45 s / 0.4 study candidate
unchanged across all 88 cells at 30–180 s. Every modeled check passed. However,
at 45 s / 0.9 it scored 2348.03 versus the reoptimized design's 3694.07, about
36% lower. Its payload remained reach-limited at 53.66 kg as fill ceiling rose.
That is encouraging modeled feasibility robustness, with a performance tradeoff.
It is not proof that fill uncertainty has no design impact.

See [verification](VERIFICATION.md) for unit tests, full-grid audit, conservation,
resume, and browser checks. Retention versus RPM, soil response, efficiencies,
motor continuous operation, and actual slope/obstacle traversal still need
physical evidence; the study does not calibrate or validate them.
