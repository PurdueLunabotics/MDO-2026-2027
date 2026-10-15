# Traversal-time and drum-fill sensitivity

Optimize the hardware independently at each combination of two fixed operating
assumptions, then inspect scores and geometry in an offline heat map. Neither
`design_baseline.json` nor `design_optimized.json` is changed.

## Run, view, or resume

```powershell
.\.venv\Scripts\python.exe -m studies.traversal_fill --workers 6
```

Open **`results/sensitivity/traversal_fill/index.html`** in Chrome or Edge.
Refresh during a run to see completed cells. Click a cell for its top view,
travel/compact/digging side views, dimensions, both solver-start outcomes,
storage utilization, and a detailed PDF-ready report. Color represents
construction points; x is one-way time and y is fill ceiling. The two line plots
show slices at the source baseline's other assumption (45 s and 0.4 here).

Use fewer workers on a smaller computer. Repeating the command reuses completed
cells only when source inputs, starts, source/data hashes, package versions, and
solver settings match. If those change, select a new output folder. For a small
trial before the full run:

```powershell
.\.venv\Scripts\python.exe -m studies.traversal_fill --times 30 45 90 --fills 0.2 0.4 0.9 --workers 2 --output results/sensitivity/trial
```

All outputs are ignored by Git. A fresh clone must run the study or receive a
shared copy. To share detailed results, copy the **complete output folder**;
the heat map itself is self-contained, but its detailed report links need
`cases/`. Print any case through the [report guide](DESIGN_REPORT.md).

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
baseline. The completed study uses plate-frame / standard-X44, COBYLA, 2000
evaluations per start, stopping accuracy 1e-5, and initial radius 0.02. Bounds,
physical constraints, inward numerical reserves, score weights, Earth gravity,
and 900 s mission are unchanged.

Select the highest continuous score among **successful and modeled-feasible**
attempts. A failed solver is never eligible, even if its last iterate passes.
If neither succeeds, retain the least violated numeric final iterate as a failed
diagnostic; exceptions without usable final states get an error report without
invented geometry or scores. Gray cells mean no accepted solution was found,
not a general infeasibility proof. There is no interpolation across failures.
These are local optimization results from two starts, not proven global optima.

## Completed study: 2026-10-15

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
