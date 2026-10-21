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