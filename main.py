r"""Run with .venv\Scripts\python.exe main.py; use --baseline to analyze only."""

import argparse
import json
from pathlib import Path
import openmdao
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_totals
from rover.config import DESIGN
from rover.problem import build_problem, MARGINS
from rover.report import snapshot, print_report
from rover.html_report import write_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", action="store_true", help="Analyze without optimization")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="Override a config value in its stated units")
    parser.add_argument("--optimizer", choices=["SLSQP", "COBYLA"], default="SLSQP")
    parser.add_argument("--maxiter", type=int, default=200)
    parser.add_argument("--check-derivatives", action="store_true", help="Compare total derivatives at two FD step sizes")
    parser.add_argument("--n2", action="store_true", help="Compatibility flag; N2 and HTML reports are now always saved")
    parser.add_argument("--output", type=Path, help="JSON path; defaults to results/baseline.json or results/optimization.json")
    args = parser.parse_args()
    if args.maxiter < 1:
        parser.error("--maxiter must be positive")
    try:
        overrides = dict(item.split("=", 1) for item in args.set)
        problem = build_problem(overrides, optimizer=args.optimizer, maxiter=args.maxiter, display=True)
    except (ValueError, TypeError) as exc:
        parser.error(str(exc))
    output = args.output or Path("results") / ("baseline.json" if args.baseline else "optimization.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    if not args.baseline:
        recorder_path = output.with_suffix(".sql").resolve()
        problem.driver.add_recorder(om.SqliteRecorder(str(recorder_path)))
        problem.driver.recording_options["includes"] = ["*"]
    try:
        problem.run_model()
        baseline = snapshot(problem)
        print_report("Starting design", baseline)
        if args.check_derivatives:
            checks = problem.check_totals(
                of=["construction_points", *MARGINS], wrt=list(DESIGN),
                method="fd", form="central", step=3e-5, step_calc="rel_avg", out_stream=None)
            assert_check_totals(checks, atol=1e-3, rtol=2e-3)
            print("Total derivative step-size check passed (not a physics validation).")
            # check_totals changes OpenMDAO's model-wide approximation settings.
            # Restore the optimizer's intended finite-difference step/form.
            problem.model.approx_totals(method="fd", form="central", step=1e-5, step_calc="rel_avg")
        success, message, final = None, "Baseline analysis only", baseline
        if not args.baseline:
            result = problem.run_driver()
            success = bool(result.success)
            message = str(problem.driver.result)
            problem.run_model()
            final = snapshot(problem)
            print_report("Final optimizer iterate", final)
            print(f"Optimizer success: {success}")
        om.n2(problem, outfile=str(output.parent / "n2.html"), show_browser=False)
        record = {"schema_version": 2, "model_metadata": problem.model_metadata,
                  "openmdao_version": openmdao.__version__, "optimizer": args.optimizer,
                  "optimizer_success": success, "optimizer_result": message,
                  "status": "baseline" if args.baseline else ("converged_feasible" if success and final["feasible"] else "failed_or_infeasible"),
                  "notice": "Uncalibrated conceptual estimates; points include only mass and energy productivity.",
                  "baseline": baseline, "final": final}
        output.write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        iterations = []
        if not args.baseline:
            reader = om.CaseReader(str(recorder_path))
            for i, case in enumerate(reader.get_cases("driver", recurse=False)):
                iterations.append({"label": f"Driver evaluation {i + 1}", "snapshot": snapshot(case)})
        write_report(record, output, iterations)
        print(f"\nSaved {output.resolve()}")
        print(f"Report and geometry: {output.with_suffix('.html').resolve()}")
        return 0 if args.baseline or (success and final["feasible"]) else 1
    finally:
        problem.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
