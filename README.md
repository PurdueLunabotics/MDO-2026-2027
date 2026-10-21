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

These steps are for **Windows 10 or 11** and assume you have not used Python,
Git, or PowerShell before. Python runs the model. Git downloads a working copy
of the project, called a **repository**, and tracks changes to its files.
Both are free. You need an internet connection to download the tools, project,
and Python packages; after setup, the model runs locally.

### 1. Install Python 3.12

Use **Python 3.12** to match the version used by this project's automated checks.
If it is already installed, skip the installer and check its version in step 3.

1. Open the [Python 3.12.10 release page](https://www.python.org/downloads/release/python-31210/).
   This release provides a traditional Windows installer for Python 3.12.
2. Scroll to **Files** and choose **Windows installer (64-bit)** for a typical
   Intel or AMD Windows computer. If your computer has an ARM processor, choose
   **Windows installer (ARM64)** instead. You can check **Settings → System →
   About → System type** if you are unsure.
3. Open the downloaded `.exe` file from your browser's downloads list or your
   **Downloads** folder in File Explorer.
4. On the first installer screen, check **Add python.exe to PATH** and leave the
   Python launcher (`py`) option enabled. These let PowerShell find Python.
5. Click **Install Now**, wait for installation to finish, and click **Close**.

The [Python Windows guide](https://docs.python.org/3.12/using/windows.html#installation-steps)
has more detail about the installer options.

### 2. Install Git

If you plan to use **Download ZIP** in step 4, Git is optional. Install it if you
want to clone the repository or contribute code.

1. Open the [Git for Windows download page](https://git-scm.com/install/windows).
2. Choose **Git for Windows/x64 Setup** for an Intel or AMD computer, or
   **Git for Windows/ARM64 Setup** for an ARM computer.
3. Open the downloaded `.exe` file. Keep the default installation folder and
   components, and click **Next** through the setup screens.
4. On the screen about adjusting your PATH, keep **Git from the command line and
   also from 3rd-party software** selected so Git works in PowerShell.
5. Click **Install**, wait for it to finish, and click **Finish**. You do not
   need to launch Git Bash for these instructions.

### 3. Open PowerShell and check the installations

PowerShell is a Windows terminal: you type commands into it and press **Enter**
to run them. It is already included with Windows.

1. Close any PowerShell or Terminal windows that were open during installation.
   A new window will pick up the newly installed tools.
2. Open the Windows **Start** menu, type **PowerShell**, and open **Windows
   PowerShell**. A normal window is sufficient; administrator mode is not needed
   to run the project.
3. Copy and run each line below separately. Press **Enter** after each line:

```powershell
py -3.12 --version
git --version
```

The first command should print `Python 3.12.x` (for example, `Python 3.12.10`);
the second should print `git version ...`. Skip the Git check if you chose the
ZIP option. Do not copy the `PS C:\...>` prompt or the surrounding code-block
markers into PowerShell.

If `py` or `git` is **not recognized**, close PowerShell, reopen it, and try
again. If it still fails, rerun the corresponding installer and check the
launcher/PATH options above. If `py` is unavailable but `python --version`
prints `Python 3.12.x`, you can use `python` wherever these setup steps say
`py -3.12`.

### 4. Get a copy of the repository

Choose **one** option below. Cloning is useful for ongoing team work; a ZIP
download is enough to run the model or view reports. If you already have a local
copy, skip both options and open that folder as described in step 5.

**Option A: Clone with Git**

1. Open **File Explorer** with **Windows+E**. Go to a folder where you want to
   keep the project, such as **Documents**, and create a folder named **GitHub**
   if you want a place for your coding projects.
2. Open that **GitHub** folder. Click File Explorer's address bar (the folder
   path at the top), type `powershell`, and press **Enter**. This opens PowerShell
   in that folder.
3. Run this command and wait for the download to finish:

```powershell
git clone https://github.com/PurdueLunabotics/MDO-2026-2027.git
```

4. Git creates a new folder named `MDO-2026-2027`. Enter it:

```powershell
cd .\MDO-2026-2027
```

`cd` means **change directory**: it changes the folder where your commands run.
If Git says the destination folder already exists and is not empty, open your
existing copy instead of cloning into it again.

**Option B: Download a ZIP without Git**

1. Open the [repository on GitHub](https://github.com/PurdueLunabotics/MDO-2026-2027).
2. Above the file list, click the green **Code** button, then **Download ZIP**.
3. Find the downloaded ZIP in File Explorer, right-click it, choose **Extract
   All...**, select a destination such as **Documents**, and click **Extract**.
4. Open the extracted project folder. Its name may include the branch name,
   such as `MDO-2026-2027-main`. Keep opening folders until you can see
   `README.md`, `main.py`, and `requirements.txt` together. Work from this
   extracted folder, rather than inside the ZIP.
5. Click the address bar, type `powershell`, and press **Enter** to open
   PowerShell in the project folder.

A ZIP contains a snapshot of the files, without Git's change history. It will
not support Git commands for updating or contributing changes. GitHub's guides
explain [cloning](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository)
and [ZIP downloads](https://docs.github.com/en/repositories/working-with-files/using-files/downloading-source-code-archives).

### 5. Check that PowerShell is in the project folder

All remaining commands must run in the folder containing `main.py`. To open a
new PowerShell window there at any time, find that folder in File Explorer,
click the address bar, type `powershell`, and press **Enter**.

Run:

```powershell
Get-Location
dir
```

`Get-Location` shows your current folder; `dir` lists its contents. Check that
the list includes `main.py`, `requirements.txt`, and `design_baseline.json`.
If those files are missing, return to File Explorer and open PowerShell in the
folder that contains them before continuing.

### 6. Create the Python environment and install packages

A **virtual environment** is a folder containing a separate Python environment
for this project. It keeps this project's packages separate from other projects.
Create it once, from the project folder:

```powershell
py -3.12 -m venv .venv
```

The command creates a folder named `.venv`; it may finish without printing
anything. Now install the packages listed in `requirements.txt`:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Wait until installation finishes and the PowerShell prompt returns. The
`pip` command downloads the required packages, including OpenMDAO. If it ends
with an error, resolve that error before continuing. The commands below call
the environment's Python directly, so you do not need to activate the environment
or change PowerShell's script execution policy.

Check the environment's Python version:

```powershell
.\.venv\Scripts\python.exe --version
```

It should print `Python 3.12.x`. If this project already has a working `.venv`
using Python 3.12, reuse it; you do not need to create another environment.

### 7. Run the saved baseline and open the report

In the same PowerShell window, run:

```powershell
.\.venv\Scripts\python.exe main.py --inputs design_baseline.json --output results/my_baseline.json
```

Wait for the command to finish. It evaluates the saved design without changing
it, creates the `results` folder if needed, and writes both numeric results
(`my_baseline.json`) and a browser report (`my_baseline.html`).

Open the report with:

```powershell
Start-Process .\results\my_baseline.html
```

Or open the project's **results** folder in File Explorer, right-click
**my_baseline.html**, and choose **Open with → Microsoft Edge** or **Google Chrome**.
A fixed evaluation can finish successfully while reporting failed checks:
inspect the report's feasibility status. Rerunning this command replaces your
`my_baseline` output files; choose a different output filename to keep an earlier
run.

### Returning to the project later

Open PowerShell in the project folder as in step 5, then run the command in
step 7. You do not need to reinstall Python or Git, clone again, or recreate
`.venv` each time. Rerun the package-install command if `requirements.txt` changes.

An editor such as [VS Code](https://code.visualstudio.com/) is useful for editing
JSON and Python, but is optional for this first run. No EDEM, Chrono, or other
external simulator is required.

**macOS/Linux:** the Windows installer and PowerShell steps above do not apply.
With Python 3.12 and Git installed, clone the repository and use
`python3.12 -m venv .venv` to create the environment. Replace
`.\.venv\Scripts\python.exe` in the remaining Python commands with
`.venv/bin/python`, and open the generated HTML report in your browser.

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
| First model change: branch, replace a tube load model, test, and open a PR | [Training exercise](Training.md) |
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
