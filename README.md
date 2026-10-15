# Lunabotics rover design model

Compare rover designs, estimate mass and mission performance, and find promising
dimensions with multidisciplinary design optimization (MDO). The model connects
drums, arms, frame, wheels, motors, battery, stability, and mission timing using
[OpenMDAO](https://openmdao.org/).

**A passing result means the modeled checks pass. It does not prove the rover
will work in soil or meet every competition requirement.** Soil response,
efficiencies, and several structural and mobility assumptions need testing.

## Just want to see the design?

No Python is needed to read the saved reports. Download or clone the repository,
then open an HTML report in Chrome or Edge. GitHub shows HTML source rather than
the interactive report; PDFs can be read directly on GitHub.

| Design | Printable PDF | Interactive report |
| --- | --- | --- |
| Student-selected baseline | [Baseline PDF](output/pdf/baseline.pdf) | [Baseline HTML](docs/design_reports/baseline.html) |
| Optimizer candidate | [Candidate PDF](output/pdf/optimized.pdf) | [Candidate HTML](docs/design_reports/optimized.html) |

Reports include scores, mass and energy, top and side views, failed checks, and
complete input/output tables. The saved baseline currently has structural-stress
and articulated-wheel-load failures; the candidate passes the current modeled
checks. These are saved snapshots: changing inputs does not refresh them.

## First run

Install **Python 3.12 or newer** and [Git](https://git-scm.com/), then open a
terminal in the repository folder. Windows PowerShell commands:

```powershell
git clone https://github.com/PurdueLunabotics/MDO-2026-2027.git
cd MDO-2026-2027
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/my_baseline.json
```

If you already have this repository, skip `git clone` and `cd` into your existing
copy. Reuse its `.venv` if installed. On macOS/Linux, replace
`.\.venv\Scripts\python.exe` with `.venv/bin/python`.
An editor such as VS Code is useful for editing JSON and Python. No EDEM, Chrono,
or other external simulator is required.

Open **`results/my_baseline.html`** in your browser. The command evaluates the
saved design without changing it. A fixed evaluation can finish successfully
while reporting failed checks: inspect the report's feasibility status.

## Edit, evaluate, optimize

There are **two maintained design JSON files**, both in the repository root:

| File | Purpose |
| --- | --- |
| `design_baseline.json` | Your chosen design and assumptions. Edit `inputs` and update its revision. |
| `design_optimized.json` | A saved optimizer candidate, updated when publishing the report pair. |

Inputs use the units in the report appendix: lengths in **metres**, mass in kg,
force in N, speed in rpm, battery capacity in Wh, and angles in their stated
units. The saved cases select the plate frame and standard X44 motors.

```powershell
# Evaluate an edit while keeping every input fixed
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/my_baseline.json

# Optimize design variables, starting from that same baseline
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --optimize --output results/my_optimization.json

# Try an assumption without editing the saved baseline
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --set fill_fraction=0.6 --output results/fill_check.json
```

Optimization varies the selected profile's **design variables** within their
bounds; **assumptions** remain fixed. `--set` changes an input or starting value,
not whether it is optimized. These runs leave both maintained design files unchanged.
The report-publishing command deliberately updates only the candidate JSON.

With `--inputs`, evaluation is the default and optimization requires `--optimize`.
Without `--inputs`, `--baseline` evaluates the generic reference defaults, while
plain `main.py` optimizes those defaults. That reference cold start is known to
fail; use the saved baseline for the current rover. See `main.py --help` for all
options and [verification](docs/VERIFICATION.md) for known failures.

Positive constraint margins generally mean a check passes; negative margins
identify failures. An optimizer candidate is accepted only when the solver
succeeds **and** all final modeled checks pass. Scores are mass and energy
productivity points, with continuous and complete-cycle estimates labeled
separately. A high score alone is insufficient.

## Reports and PDF export

For a quick PDF, open your generated HTML, press **Ctrl+P**, and choose
**Save as PDF → Letter → Portrait**, with background graphics enabled.
Printing includes every check and all rover poses, even if the screen is filtered.

To refresh both reports in the repository and the saved candidate, follow
[the report guide](docs/DESIGN_REPORT.md). Re-export the PDFs after refreshing
the HTML. Drive upload is a separate manual step.

## Traversal-time and fill study

The study independently optimizes 96 combinations: one-way traversal times
15–180 s in 15 s steps, crossed with fill ceilings 0.2–0.9 in 0.1 steps.

```powershell
.\.venv\Scripts\python.exe -m studies.traversal_fill --workers 6
```

Open **`results/sensitivity/traversal_fill/index.html`**. Click a heat-map cell
for its top/side geometry, dimensions, solver outcomes, and detailed report.
The study takes longer than a single optimization; reduce `--workers` on a
smaller computer. Repeating the command resumes matching completed cells.

The completed study found 88 converged feasible cells; the 15 s row is beyond
the current drive profile's speed capability. Fill fraction affects component
sizing as well as score; similar drawings do not establish hardware robustness.
See [method, findings, and limitations](docs/SENSITIVITY_STUDY.md).

## For contributors and advanced users

| Need | Read or edit |
| --- | --- |
| Equations, route, score, units | [MODEL.md](MODEL.md) |
| What each model can predict and what evidence is missing | [Model cards](docs/MODEL_CARDS.md) |
| Add a model, understand interfaces, solver settings, or submit a PR | [Development guide](docs/MODEL_DEVELOPMENT.md) |
| Plate-frame geometry and structural load scope | [Plate frame](docs/PLATE_FRAME.md) |
| X44 curve, gearing, energy accounting, replacement API | [Motors](docs/X44_MOTORS.md) |
| Obstacle, slope, wheel unloading, clearance, and unverified ditch | [Traversal](docs/OBSTACLE_TRAVERSAL.md) |
| Plan physical tests or offline simulation | [Experiments](docs/EXPERIMENTS.md) |
| Latest checks and reproducible commands | [Verification](docs/VERIFICATION.md) |

`rover/config.py` defines defaults and bounds; `rover/problem.py` assembles the
model. `rover/models.py` selects replaceable disciplines. Keep optional models
under `rover/alternatives/`; `reporting/` handles reports and `studies/` handles
sweeps. Coding agents must follow [AGENTS.md](AGENTS.md).

Run the checks before submitting changes:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

The development guide includes baseline and derivative checks and the Git
workflow. GitHub Actions runs the repository's checks on Windows and Linux.
Keep `.venv`, temporary results, raw simulator runs, and credentials out of Git.
`results/` is ignored: a fresh clone must regenerate study outputs. The two
curated reports and PDFs above are versioned; hundreds of individual study
reports are kept locally or shared separately as a complete folder.
