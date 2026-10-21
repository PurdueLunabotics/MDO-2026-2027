# First model change: a circular-tube load and sizing exercise

Work through this example on your own branch, starting from `main`. You will
add an optional structural model, check it against hand calculations, compare
it with the reference model, and submit a pull request for review.

The worked example adds **transverse shear** to the reference tube stress
screen. You will also compare candidate tube sizes under specified example
loads. The numbers in that small sizing exercise are **synthetic teaching
inputs**, not measured rover loads or verified material allowables.

Complete the Windows setup in [README.md](README.md#first-run) first. You need
Python 3.12, Git, the project's `.venv`, and a **Git clone** of this repository.
A downloaded ZIP can run Python, but does not contain the Git history needed
for this branch and pull-request exercise.

## 1. Understand where the model belongs

Read [AGENTS.md](AGENTS.md), [MODEL.md](MODEL.md#structure-and-mass), the
`StructureGeometry` and `StructureLoads` cards in
[docs/MODEL_CARDS.md](docs/MODEL_CARDS.md), and the replacement instructions in
[docs/MODEL_DEVELOPMENT.md](docs/MODEL_DEVELOPMENT.md#replace-one-component).

The existing reference tube model is a suitable starting point because its
loads, section equations, outputs, and replacement interface already exist:

| File or component | Role in this exercise |
| --- | --- |
| `rover/structure.py: tube_section()` | Calculates circular-tube area and moments of area. |
| `StructureGeometry` | Calculates frame mass and arm mass per unit length from the chosen tube dimensions. |
| `StructureLoads` | Uses downstream rover loads to check stress, deflection, and Euler buckling for four elementary load envelopes. |
| `rover/models.py: ModelSet.structure_loads` | Selects a different load component without changing the reference default. |
| `rover/config.py` | Already declares frame and arm outside diameters and wall thicknesses as reference design variables. |
| `tests/test_physics.py` | Contains tube-section and structural load-scaling checks to read before adding your own tests. |

The flow is:

```text
chosen tube dimensions -> structure/arm mass -> rover loads
          |                                      |
          +------------> StructureLoads <--------+
                              |
                    signed structural margins
                              |
                optimizer chooses new dimensions
```

**Keep tube dimensions as explicit inputs.** A component that automatically
resizes a tube from downstream loads would change its mass, which changes the
loads again. That needs an explicitly designed coupled solve. This exercise
checks a proposed section; the optimizer or a separate candidate search does
the sizing outside `compute()`.

Use the **generic reference profile** for this exercise. The saved
`design_baseline.json` selects `plate-frame`, whose `PlateStructureLoads`
component has different plate, rocker, and CF-tube assumptions. Do not replace
that coordinated profile with the reference tube component.

## 2. Open PowerShell and start a branch from main

1. Open your repository folder in File Explorer. Confirm you can see `main.py`,
   `README.md`, and `rover`.
2. Click File Explorer's address bar, type `powershell`, and press **Enter**.
3. Run these commands one line at a time:

```powershell
git rev-parse --show-toplevel
git status
```

The first command must identify this repository. The second should say
**nothing to commit, working tree clean**. If it lists existing changes, stop
and identify their owner before switching branches. Keep your earlier work on
its own branch or use a fresh clone for this exercise; do not discard it.

Now update `main` and create your training branch:

```powershell
git switch main
git pull --ff-only origin main
git switch -c codex/tube-training-yourname
git branch --show-current
```

Replace `yourname` with your name, without spaces, in every command that uses
the branch name. The final command should print your new branch name.
`origin` is the GitHub remote. `--ff-only` updates `main` only when Git can do
so without a merge commit; if it refuses, ask a teammate to help inspect the
history. `-c` creates the new branch. See the official
[git switch reference](https://git-scm.com/docs/git-switch).

You will edit files in your usual project folder. Git records your work on
this branch; creating a branch does not make another copy of the folder.

## 3. Record the unchanged reference result

Run the generic reference model before adding anything:

```powershell
.\.venv\Scripts\python.exe main.py --baseline --output results/training/reference_before.json
Start-Process .\results\training\reference_before.html
```

Record `structure_stress`, `structure_stress_margin`, `structure_deflection`,
`structure_buckling_margin`, `dry_mass`, `cycle_energy`, and `construction_points`.
Open the report's complete input/output tables to find these quantities. Keep
its failed checks visible. A successful fixed evaluation does not mean every
constraint passes.

## 4. Define one change and preserve the contract

The current reference stress calculation includes axial load, bending, and
torsional shear. We will add an elementary estimate of transverse shear from
the existing transverse-force envelope. We will retain the four load cases,
load factors, safety factors, deflection equation, and buckling equation.

The replacement must keep these scalar outputs:

| Output | Units | Meaning retained |
| --- | --- | --- |
| `structure_stress` | Pa | Conservative aggregate equivalent-stress screen. |
| `structure_stress_margin` | Pa | `tube_yield_strength / structure_safety_factor - structure_stress`. |
| `structure_deflection` | m | Aggregate of the existing cantilever deflection envelopes. |
| `structure_deflection_margin` | m | `allowable_deflection - structure_deflection`. |
| `structure_buckling_margin` | N | Minimum factored Euler compression reserve. |

Positive margins mean reserve; negative margins mean failure. Adding shear
should increase or preserve the stress and decrease or preserve its margin at
the **same design**. It should not change mass, mission energy, deflection, or
buckling at that same design. Those quantities can change after optimization
chooses different dimensions.

### Equations and limits

For outside diameter `D`, wall `t`, inside diameter `d = D - 2t`, outside radius
`R`, and inside radius `r`:

```text
A = pi * (D^2 - d^2) / 4                [m^2]
I = pi * (D^4 - d^4) / 64               [m^4]
J = 2I                                [m^4]
sigma_bound = N/A + abs(M)*R/I         [Pa]
tau_torsion = abs(T)*R/J               [Pa]
tau_transverse = V*(R^2 + R*r + r^2)/(3I) [Pa]
stress_bound = sqrt(sigma_bound^2 + 3*(tau_torsion + tau_transverse)^2)
deflection = V*L^3/(3*E*I)             [m]
buckling_margin = pi^2*E*I/(2L)^2/SF - N [N]
```

Here `N >= 0` is the compressive-force envelope, `M` is bending moment, `T` is
torque, `V >= 0` is the transverse-force envelope, and `L` is member length.
Moments and torques are in N m. The adapter applies the existing structural
load factor once before calling this helper. `SF` is the existing structural
safety factor; the effective buckling length remains `2L`.

The added shear expression follows the elementary `VQ/(Ib)` beam estimate at
the neutral axis of an annulus: `Q = 2*(R^3-r^3)/3` and `b = 2*(R-r)`.
It approaches `4V/(3A)` for a solid circle and `2V/A` for a thin circular tube.
For the underlying shear-flow equilibrium, see
[MIT's structural-mechanics beam notes](https://ocw.mit.edu/courses/16-20-structural-mechanics-fall-2002/13881c48c7c4654e4b5b689cb6987803_unit14.pdf).
The annulus expression above is our application of that elementary equation.

Adding the separate peak shear magnitudes and peak normal stress is a
**conservative combination within this simplified screen**. Their peaks need
not occur at the same point. It is not an exact three-dimensional stress
solution or an assembled-frame analysis. The existing force used for each
cantilever deflection envelope is also used for its shear estimate; we are not
claiming to have derived new frame reactions. Keep these assumptions in your
model card. Deflection remains bending-only; this change does not add shear
deformation to the deflection model.

Use only straight, prismatic, homogeneous, isotropic circular tubes in the
small-deflection elastic regime. Local wall buckling, joints, fatigue,
stress concentrations, and detailed load sharing remain omitted. In
particular, do not apply this isotropic example to the plate profile's CF rods.

## 5. Write the physics as a pure helper

Open the project in your editor. Create
`rover/alternatives/training_tubes.py` and save the following code. Use the
exact filename so the imports in later steps work. These are files you create
on your training branch; they are not installed merely by reading this guide.

```python
import math

import openmdao.api as om

from rover.components import scalar
from rover.structure import StructureLoads, tube_section


def tube_response(diameter, wall, length, axial, moment, torque, force,
                  youngs_modulus, safety_factor, *, include_shear=True):
    """Elementary elastic tube screen. SI units; axial is compression >= 0."""
    values = (diameter, wall, length, axial, moment, torque, force,
              youngs_modulus, safety_factor)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Tube inputs must be finite.")
    if diameter <= 0 or not 0 < wall < diameter / 2:
        raise ValueError("Require D > 0 and 0 < wall < D/2.")
    if length <= 0 or youngs_modulus <= 0 or safety_factor <= 0:
        raise ValueError("Length, E, and safety factor must be positive.")
    if axial < 0 or force < 0:
        raise ValueError("Compression and transverse envelopes must be >= 0.")

    area, inertia, polar = tube_section(diameter, wall)
    outer = diameter / 2
    inner = outer - wall
    sigma = axial / area + abs(moment) * outer / inertia
    tau = abs(torque) * outer / polar
    transverse = force * (outer**2 + outer*inner + inner**2) / (3*inertia)
    if include_shear:
        tau += transverse
    return {
        "stress": math.sqrt(sigma**2 + 3*tau**2),
        "deflection": force * length**3 / (3*youngs_modulus*inertia),
        "buckling_margin": (
            math.pi**2 * youngs_modulus * inertia / (2*length)**2
            / safety_factor - axial
        ),
    }
```

`include_shear=False` reproduces the original per-case equations for these
nonnegative load envelopes. It is useful for a test, not a fallback for invalid
inputs. Invalid inputs raise an error rather than becoming feasible.

**Pause and explain:** why are `I` and `J` different? Why does doubling `L`
multiply this deflection by eight but divide Euler capacity by four? Why is
`wall = D/2` outside this helper's hollow-tube domain?

## 6. Adapt the helper to the existing component

Append this class to the **same file**, below the helper:

```python
class TrainingTubeLoads(StructureLoads):
    # This class owns its dependency sets, including inactive load cases.
    DEPENDENCIES = {
        name: set(inputs)
        for name, inputs in StructureLoads.DEPENDENCIES.items()
    }

    def setup(self):
        self.model_evidence = {
            "revision": "training-tubes-v1",
            "status": "analytical teaching example; no hardware validation",
            "change": "add annular transverse-shear stress envelope",
            "domain": "isotropic elastic hollow tubes; reference load envelopes",
            "omissions": "joints, fatigue, local buckling, assembled load sharing",
        }
        super().setup()

    def compute(self, inputs, outputs):
        v = lambda name: scalar(inputs, name)
        factor = v("structure_load_factor")
        weight = (v("dry_mass") + v("payload_mass"))*v("gravity")*factor
        pair = (v("drum_mass") + v("payload_mass"))/2*v("gravity")*factor
        actuator = v("required_actuator_force")*factor
        cut = v("cut_force")/2*factor
        wb, track, arm = v("wheelbase"), v("track_width"), v("arm_length")
        tower = max(math.hypot(
            v("actuator_pivot_x") - v("pivot_x"),
            v("top_pivot_height") - v("bottom_pivot_height")), 1e-3)

        # Same four load envelopes and tower-length guard as the reference.
        # family, length, compression, bending moment, torque, transverse force
        cases = [
            ("frame", wb, weight/2, weight*wb/8, pair*track/2, weight/2),
            ("frame", track, weight/2, weight*track/4, pair*arm, weight/2),
            ("frame", tower, actuator, actuator*tower, cut*arm, actuator),
            ("arm", arm, actuator+cut, pair*arm, cut*arm, pair),
        ]
        responses = []
        for family, length, axial, moment, torque, force in cases:
            try:
                response = tube_response(
                    v(family + "_tube_diameter"), v(family + "_wall"),
                    length, axial, moment, torque, force,
                    v("tube_youngs_modulus"), v("structure_safety_factor"))
            except ValueError as exc:
                raise om.AnalysisError(str(exc)) from exc
            responses.append(response)

        stress = sum(item["stress"]**8 for item in responses)**(1/8)
        deflection = sum(item["deflection"]**8 for item in responses)**(1/8)
        outputs["structure_stress"] = stress
        outputs["structure_stress_margin"] = (
            v("tube_yield_strength")/v("structure_safety_factor") - stress)
        outputs["structure_deflection"] = deflection
        outputs["structure_deflection_margin"] = v("allowable_deflection") - deflection
        outputs["structure_buckling_margin"] = min(
            item["buckling_margin"] for item in responses)
```

Inheritance reuses the reference input/output declarations; the class owns a
copy of the dependency map. No new ports are needed because the transverse
force already depends on the existing inputs listed for stress. If you later
introduce a new input, update the dependencies and explicit source as well.

The pure helper does the section calculation. `compute()` only translates the
existing rover inputs into load cases and aggregates results. It must not
start another optimizer, write files, or launch a simulator. Leave
`rover/structure.py`, `rover/config.py`, and the defaults in `rover/models.py`
unchanged for this exercise.

## 7. Add independent limits and failure tests

Create `tests/test_training_tubes.py`:

```python
import math
import unittest

from rover.alternatives.training_tubes import TrainingTubeLoads, tube_response
from rover.models import ModelSet
from rover.problem import build_problem


class TrainingTubeTests(unittest.TestCase):
    def response(self, **changes):
        # SYNTHETIC loads and illustrative material inputs, not measurements.
        values = dict(diameter=.05, wall=.002, length=.6, axial=1000.,
                      moment=80., torque=10., force=200.,
                      youngs_modulus=69e9, safety_factor=2.)
        values.update(changes)
        return tube_response(**values)

    def test_pure_bending_and_zero_load(self):
        # Independent section integration for a circular annulus.
        inertia = math.pi*(.025**4 - .023**4)/4
        bending = self.response(axial=0., torque=0., force=0.)
        self.assertAlmostEqual(bending["stress"], 80*.025/inertia)
        empty = self.response(axial=0., moment=0., torque=0., force=0.)
        self.assertEqual(empty["stress"], 0.)
        self.assertEqual(empty["deflection"], 0.)
        capacity = math.pi**2*69e9*inertia/(1.2**2)/2
        self.assertAlmostEqual(empty["buckling_margin"], capacity)

    def test_solid_and_thin_tube_shear_limits(self):
        # Independent known limits: 4V/(3A) for solid, 2V/A for thin tube.
        for wall, coefficient in ((.025-1e-9, 4/3), (1e-8, 2.)):
            area = math.pi*wall*(.05-wall)
            result = self.response(wall=wall, axial=0., moment=0., torque=0.)
            expected = math.sqrt(3)*coefficient*200/area
            self.assertAlmostEqual(result["stress"]/expected, 1., places=5)

    def test_scaling_and_failed_buckling(self):
        first = self.response()
        longer = self.response(length=1.2)
        self.assertAlmostEqual(longer["deflection"], 8*first["deflection"])
        self.assertAlmostEqual(longer["buckling_margin"] + 1000.,
                               (first["buckling_margin"] + 1000.)/4)
        self.assertLess(self.response(axial=1e9)["buckling_margin"], 0.)

    def test_invalid_domain_is_rejected(self):
        for changes in (dict(diameter=0.), dict(wall=0.), dict(wall=.025),
                        dict(wall=.03), dict(length=0.), dict(axial=-1.),
                        dict(force=-1.), dict(youngs_modulus=0.),
                        dict(safety_factor=0.), dict(torque=math.nan)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.response(**changes)

    def test_no_shear_recovers_original_per_case_stress(self):
        area = math.pi*(.05**2 - .046**2)/4
        inertia = math.pi*(.05**4 - .046**4)/64
        sigma = 1000/area + 80*.025/inertia
        tau = 10*.025/(2*inertia)
        original = self.response(include_shear=False)
        self.assertAlmostEqual(original["stress"], math.sqrt(sigma**2+3*tau**2))
        self.assertGreater(self.response()["stress"], original["stress"])

    def test_replacement_preserves_fixed_design_accounting(self):
        reference = build_problem(structure="reference", motors="reference")
        self.addCleanup(reference.cleanup)
        replacement = build_problem(
            structure="reference", motors="reference",
            models=ModelSet(structure_loads=TrainingTubeLoads))
        self.addCleanup(replacement.cleanup)
        reference.run_model()
        replacement.run_model()
        get = lambda problem, name: problem.get_val(name).item()
        self.assertGreater(get(replacement, "structure_stress"),
                           get(reference, "structure_stress"))
        self.assertLess(get(replacement, "structure_stress_margin"),
                        get(reference, "structure_stress_margin"))
        for name in ("dry_mass", "payload_mass", "cycle_energy",
                     "construction_points", "structure_deflection",
                     "structure_buckling_margin"):
            self.assertAlmostEqual(get(replacement, name), get(reference, name))
```

Run only your new tests first:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p test_training_tubes.py -v
```

For the code above, a successful run ends with `Ran 6 tests` and `OK`. If it
prints `FAIL` or `ERROR`, read the named test and traceback before continuing.

Explain how the two shear limits test the physics independently of the
implementation. A test that merely copies the full new equation is not enough.
The deliberately extreme thin/solid limits are mathematical checks, not
recommended manufactured sections.

## 8. Size a tube for specified teaching loads

Create `examples/training_tubes.py`. Start with this small candidate search:

```python
import argparse
import json
from pathlib import Path

import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_totals

from reporting.report import snapshot
from rover.alternatives.training_tubes import TrainingTubeLoads, tube_response
from rover.models import ModelSet
from rover.problem import build_problem


def size_teaching_tube():
    # SYNTHETIC already-factored loads. No hardware selection is implied.
    passing = []
    for diameter, wall in ((.025, .001), (.05, .002), (.06, .003)):
        response = tube_response(diameter, wall, .6, 1000., 80., 10., 200.,
                                 69e9, 2.)
        area = 3.141592653589793*wall*(diameter-wall)
        mass = 2700.*area*.6
        stress_margin = 240e6/2 - response["stress"]
        section_margin = diameter/4 - wall
        passed = (stress_margin >= 0 and response["deflection"] <= .005
                  and response["buckling_margin"] >= 0 and section_margin >= 0)
        print(f"D={diameter:.3f} m, t={wall:.3f} m, mass={mass:.3f} kg, "
              f"stress={response['stress']/1e6:.2f} MPa, "
              f"deflection={response['deflection']*1000:.2f} mm, pass={passed}")
        if passed:
            passing.append((mass, diameter, wall))
    if passing:
        print("Lowest mass among these three passing candidates:", min(passing))
    else:
        print("No candidate passed; retain that failure.")
```

These are one-member masses, not the rover's frame mass. The search tests just
three candidates; it does not establish a continuous optimum or catalogue
availability. Before running it, predict which tube will have the least mass
and which is most likely to fail deflection. All dimensions in the code are
metres: `.002 m` is a 2 mm wall.

## 9. Select the replacement explicitly and compare the same design

Append the following functions to `examples/training_tubes.py`:

```python
def evaluate_rover(label, models, *, optimize=False):
    output = Path("results/training")
    output.mkdir(parents=True, exist_ok=True)
    problem = build_problem(structure="reference", motors="reference",
                            models=models, display=True)
    try:
        problem.run_model()
        before = snapshot(problem)
        success = None
        message = "Fixed design only"
        if optimize:
            checks = problem.check_totals(
                of=["construction_points", *problem.margin_specs],
                wrt=list(problem.design_specs), method="fd", form="central",
                step=3e-5, step_calc="rel_avg", out_stream=None)
            assert_check_totals(checks, atol=1e-3, rtol=2e-3)
            print("Replacement derivative step-size check passed.")
            problem.model.approx_totals(method="fd", form="central",
                                        step=1e-5, step_calc="rel_avg")
            result = problem.run_driver()
            success = bool(result.success)
            message = str(problem.driver.result)
            problem.run_model()
        final = snapshot(problem)
        status = ("baseline" if not optimize else
                  "converged_feasible" if success and final["feasible"] else
                  "failed_or_infeasible")
        record = {"schema_version": 2, "status": status,
                  "optimizer_success": success, "optimizer_result": message,
                  "model_metadata": problem.model_metadata,
                  "baseline": before, "final": final}
        (output/f"{label}.json").write_text(
            json.dumps(record, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        om.n2(problem, outfile=str(output/f"{label}_n2.html"), show_browser=False)
        print(label, status, "modeled feasible:", final["feasible"])
        return final, (not optimize or bool(success and final["feasible"]))
    finally:
        problem.cleanup()


def main():
    parser = argparse.ArgumentParser(description="Synthetic tube training example")
    parser.add_argument("--optimize", action="store_true")
    args = parser.parse_args()
    size_teaching_tube()
    reference, _ = evaluate_rover("reference", ModelSet())
    training_models = ModelSet(structure_loads=TrainingTubeLoads)
    replacement, _ = evaluate_rover("replacement", training_models)
    for name in ("structure_stress", "structure_stress_margin", "dry_mass",
                 "cycle_energy", "structure_deflection", "structure_buckling_margin"):
        old = reference["outputs"][name]["value"]
        new = replacement["outputs"][name]["value"]
        print(f"{name}: reference={old:.8g}, replacement={new:.8g}")
    if args.optimize:
        _, accepted = evaluate_rover("replacement_optimization", training_models,
                                    optimize=True)
        return 0 if accepted else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

Run it **from the repository root**, using `-m` so Python resolves the packages:

```powershell
.\.venv\Scripts\python.exe -m examples.training_tubes
```

The runner saves `reference.json`, `replacement.json`, and their N2 diagrams in
`results/training/`. These JSON files contain the existing snapshot fields and
model provenance; the small teaching runner does not produce the full rover
HTML report. Open the JSON files in your editor to compare numerical values.
No new report ports or schema fields are introduced by the tube component.

The worked candidate search should give approximately:

| Outside diameter | Wall | One-member mass | Stress screen | Deflection | Passes all four teaching checks? |
| --- | --- | --- | --- | --- | --- |
| 25 mm | 1 mm | 0.122 kg | 199.28 MPa | 38.38 mm | No |
| 50 mm | 2 mm | 0.489 kg | 26.73 MPa | 2.40 mm | Yes |
| 60 mm | 3 mm | 0.870 kg | 13.07 MPa | 0.95 mm | Yes |

Thus the 50 mm outside-diameter, 2 mm wall tube is the lightest passing option
**among these three synthetic candidates**. The 25 mm candidate fails both the
120 MPa allowable-stress screen and the 5 mm deflection limit.

At the current default reference design, the worked replacement increases the
aggregate stress from approximately **49.328 MPa to 50.313 MPa**. Dry mass stays
about **45.251 kg** and cycle energy stays about **2.08734 Wh**. These are
numerical checkpoints for this exercise, not measurements; rerun them if the
reference defaults change. Both fixed rover evaluations retain failed checks.

Check that the metadata names
`rover.alternatives.training_tubes.TrainingTubeLoads` for `structure_loads` in
the replacement. The `main.py` command does **not** discover this alternative
automatically; this runner's `ModelSet(...)` call is what selects it.

Inspect the connection diagram:

```powershell
Start-Process .\results\training\replacement_n2.html
```

Find `structure_loads` and confirm its loads and material/section inputs have
explicit sources. Compare the reference and replacement JSON at the same
dimensions first. A higher stress must not mysteriously change `dry_mass` or
`cycle_energy` during that fixed evaluation.

## 10. Verify the default and the alternative separately

Run the required reference checks:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe main.py --baseline --output results/training/check_reference_baseline.json
.\.venv\Scripts\python.exe main.py --check-derivatives --output results/training/check_reference_optimization.json
```

The generic reference cold-start optimization has a
[documented failure](docs/VERIFICATION.md). Preserve the failed status and report
it separately from the derivative check. Do not relax constraints, loosen test
tolerances, or claim the last iterate succeeded.

Then try sizing through the **full rover optimization with your replacement**:

```powershell
.\.venv\Scripts\python.exe -m examples.training_tubes --optimize
```

This varies the reference profile's design variables, including frame and arm
diameters and walls; it is different from the three-candidate one-tube search.
It first checks the replacement's total derivatives at the starting point,
then uses the normal reference SLSQP setup. It may fail from this cold start.
During verification of this worked example, the replacement derivative check
passed, but SLSQP stopped with `Positive directional derivative for linesearch`
and the saved candidate was `failed_or_infeasible`. Reproducing and explaining
that failure is an acceptable training outcome; it is not an accepted design.
The runner saves the final status and returns exit code 1 if solver success and
final feasibility do not both hold. Check `$LASTEXITCODE` immediately after the
command. If an exception prevents a usable final state, retain the traceback
and describe it; do not invent a result. Repeated runs overwrite these local
training outputs, so copy any results you want to compare before rerunning.

The defaults must still use the original `StructureLoads`. Since this exercise
does not change the shared interface, the existing drum and mass-accounting
tests should remain unchanged and pass. Derivative agreement checks numerical
consistency at smooth points, not physical calibration or branch boundaries.

## 11. Document what the new model can and cannot predict

Add a short `TrainingTubeLoads, revision 1` card to `docs/MODEL_CARDS.md` and
describe the optional stress equation in `MODEL.md`, without changing the
reference equations. Include:

- Replacement slot, all five output units, compression convention, load factor,
  safety factor, and the reference loading states retained.
- The added transverse-shear equation and conservative combination of separate
  peaks; the unchanged deflection, buckling, and mass conventions.
- The hollow-tube domain and explicit invalid-input rejection. Actual sections
  must also pass the existing `D/4 - wall` margin.
- The synthetic load/material inputs in the example and tests. No fitted
  relationship or experimental dataset is used, so there is no calibration
  manifest to invent. Any later fitted model requires a versioned data manifest
  and independent validation.
- Quantitative errors against the bending, Euler, and shear-limit benchmarks;
  dependency/connection checks and the fixed-design comparison results.
- Missing evidence: selected material properties, actual load paths, joints,
  local wall buckling, and measured load/deflection data. Ask for hardware data
  before recommending a real tube. Passing software tests or optimizer
  convergence does not establish hardware validity.

Summarize your results in a small table in the pull request:

| Check | Record your result |
| --- | --- |
| Lowest-mass passing teaching candidate | D, wall, mass; only among the three candidates |
| Independent limits | Relative errors and tolerances used |
| Fixed-design comparison | Stress change, margin change, and unchanged mass/energy |
| Full test suite and derivative checks | Pass/fail, including unresolved failures |
| Reference optimization | Actual solver/status and failed constraints |
| Replacement optimization | Actual solver/status and failed constraints |
| Calibration | Analytical/synthetic only; physical validation still missing |

## 12. Review, commit, push, and open a pull request

These commands are for you to run after reviewing your work. Writing this
guide does not create a branch, commit, or push on your behalf.

```powershell
git status
git diff
```

New files appear as untracked in `git status`; ordinary `git diff` does not show
their contents until they are staged. Open them in your editor, then stage only
the exercise files and documentation changes:

```powershell
git add rover/alternatives/training_tubes.py
git add examples/training_tubes.py
git add tests/test_training_tubes.py
git add MODEL.md docs/MODEL_CARDS.md
git diff --cached
```

Confirm that `.venv`, `results`, raw runs, and unrelated student work are absent
from the staged diff. Check that the reference defaults and constraints are
unchanged. When the staged result is ready:

```powershell
git commit -m "Add optional circular tube transverse shear screen"
git push -u origin codex/tube-training-yourname
```

If Git asks for your identity, configure your own name and email in this clone
with `git config user.name "Your Name"` and
`git config user.email "YOUR_EMAIL"`, then retry the commit. Use your own GitHub
account if pushing prompts for sign-in. If you do not have permission to push
to the team repository, ask a maintainer for access or the team's fork workflow.

Open the repository on GitHub. Choose **Compare & pull request**, or
**Pull requests → New pull request**, with **base: main** and
**compare: codex/tube-training-yourname**. Describe the physical change, attach
the results table, and identify the limitations. Ask another student to review
the equations, tests, and interface; check CI before a maintainer merges it.

After the pull request has been merged, return to and update `main`:

```powershell
git switch main
git pull --ff-only origin main
```

You have completed the exercise when another student can reproduce the new
model selection, explain the tube-sizing tradeoff, see the failed cases, and
confirm that the reference model still works as before.
