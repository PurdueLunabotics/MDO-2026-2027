r"""Run with .venv\Scripts\python.exe main.py; use --baseline to analyze only."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import openmdao
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_totals
from rover.problem import build_problem
from reporting.report import snapshot, print_report
from reporting.html_report import write_report
from reporting.case_inputs import read_case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--baseline", action="store_true", help="Evaluate fixed inputs without optimization")
    mode.add_argument("--optimize", action="store_true", help="Run a separate optimization study, including when using --inputs")
    parser.add_argument("--inputs", type=Path, help="Design baseline JSON; evaluates fixed values unless --optimize is explicit")
    parser.add_argument("--design-name", help="Human-readable case name")
    parser.add_argument("--revision", help="Baseline/source revision shown in the report")
    parser.add_argument("--structure", choices=["reference", "plate-frame"],
                        help="Select reference tubes or the preliminary aluminum cross-tube/plate/box-rocker frame")
    parser.add_argument("--motors", choices=["reference", "x44-standard"],
                        help="Generic scalable motors or fixed standard X44s at 12 V, drive gearing 100:1")
    parser.add_argument("--traversal-mode", choices=["fixed", "calculated"],
                        help="Fixed one-way schedule (default) or optional BWR/motor-limited route timing")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="Override a config value in its stated units")
    parser.add_argument("--freeze", action="extend", nargs="+", default=[], metavar="NAME",
                        help="Hold design variables at their resolved input values; accepts multiple names and may be repeated")
    parser.add_argument("--optimize-attachment", action="store_true",
                        help="Compatibility flag: both actuator offsets are now ordinary design variables; use --freeze to hold them")
    parser.add_argument("--attachment-max",type=float,
                        help="Legacy study compatibility only; use signed offsets in metres (tube continuation is automatic)")
    parser.add_argument("--ignore-constraint", action="extend", nargs="+", default=[], metavar="MARGIN",
                        help="Exclude named margin constraints from the optimizer only; keep full feasibility checks and reports (repeatable)")
    parser.add_argument("--actuators", choices=["reference", "pa04-900"], help="Reference sizing or frozen PA-04 900 lbf / 12 V hardware")
    parser.add_argument("--optimizer", choices=["SLSQP", "COBYLA"],
                        help="Default: COBYLA for plate-frame, SLSQP for reference")
    parser.add_argument("--maxiter", type=int,
                        help="Default: 2000 evaluations for plate-frame COBYLA; 200 otherwise")
    parser.add_argument("--solver-tol", type=float,
                        help="Numerical stopping accuracy; default 1e-5 for plate-frame/X44, 1e-3 otherwise")
    parser.add_argument("--cobyla-radius", type=float, help="Initial COBYLA trust radius in scaled design coordinates")
    parser.add_argument("--check-derivatives", action="store_true", help="Compare total derivatives at two FD step sizes")
    parser.add_argument("--n2", action="store_true", help="Compatibility flag; N2 and HTML reports are now always saved")
    parser.add_argument("--output", type=Path, help="JSON path; defaults to results/baseline.json or results/optimization.json")
    args = parser.parse_args()
    if args.maxiter is not None and args.maxiter < 1:
        parser.error("--maxiter must be positive")
    try:
        case_values, source = read_case(args.inputs) if args.inputs else ({}, {"source": "Configured defaults"})
        cli_values = dict(item.split("=", 1) for item in args.set)
        overrides = {**case_values, **cli_values}
        structure = args.structure or source.get("structure", "reference")
        motors = args.motors or source.get("motors", "reference")
        traversal_mode = args.traversal_mode or source.get("traversal_mode", "fixed")
        attachment_max = args.attachment_max if args.attachment_max is not None else source.get('attachment_max',1.)
        actuators = args.actuators or source.get("actuators", "reference")
        baseline_mode = args.baseline or (args.inputs is not None and not args.optimize)
        output = args.output or Path("results") / ("baseline.json" if baseline_mode else "optimization.json")
        artifacts = [output, output.with_suffix(".html"), output.with_suffix(".sql"),
                     output.with_name(output.stem+"_n2.html")]
        artifacts += [output.with_name(output.stem+"_"+part+".svg")
                      for part in ("retracted", "travel", "extended", "frame", "stability", "top", "obstacle")]
        if args.inputs and args.inputs.resolve() in {path.resolve() for path in artifacts}:
            raise ValueError("Output artifacts would overwrite the source baseline. Choose --output with a different filename.")
        problem = build_problem(overrides, optimizer=args.optimizer, maxiter=args.maxiter, display=True, structure=structure, cobyla_radius=args.cobyla_radius, motors=motors, actuators=actuators, solver_tol=args.solver_tol, freeze=args.freeze, ignore_constraints=args.ignore_constraint, traversal_mode=traversal_mode, optimize_attachment=args.optimize_attachment, attachment_max=attachment_max)
        context = {
            "name": args.design_name or source.get("name", "Configured design — example"),
            "revision": args.revision or source.get("revision", "Not specified"),
            "source": source["source"], "source_sha256": source.get("sha256"),
            "source_notes": source.get("notes"),
            "input_file_names": sorted(case_values), "cli_override_names": sorted(cli_values),
            "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "mode": "fixed_design" if baseline_mode else "optimization",
        }
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    output.parent.mkdir(parents=True, exist_ok=True)
    if not baseline_mode:
        recorder_path = output.with_suffix(".sql").resolve()
        problem.driver.add_recorder(om.SqliteRecorder(str(recorder_path)))
        problem.driver.recording_options["includes"] = ["*"]
    try:
        problem.run_model()
        baseline = snapshot(problem)
        print_report("Design baseline evaluation" if baseline_mode else "Optimization starting design", baseline)
        if args.check_derivatives:
            checks = problem.check_totals(
                of=["construction_points", *problem.margin_specs], wrt=list(problem.design_specs),
                method="fd", form="central", step=3e-5, step_calc="rel_avg", out_stream=None)
            assert_check_totals(checks, atol=1e-3, rtol=2e-3)
            print("Total derivative step-size check passed (not a physics validation).")
            # check_totals changes OpenMDAO's model-wide approximation settings.
            # Restore the optimizer's intended finite-difference step/form.
            problem.model.approx_totals(method="fd", form="central", step=1e-5, step_calc="rel_avg")
        success, message, final = None, "Baseline analysis only", baseline
        if not baseline_mode:
            result = problem.run_driver()
            success = bool(result.success)
            message = str(problem.driver.result)
            problem.run_model()
            final = snapshot(problem)
            print_report("Final optimizer iterate", final)
            print(f"Optimizer success: {success}")
        om.n2(problem, outfile=str(output.parent / f"{output.stem}_n2.html"), show_browser=False)
        record = {"schema_version": 7, "model_metadata": problem.model_metadata, "report_context": context,
                  "openmdao_version": openmdao.__version__, "optimizer": problem.model_metadata["optimizer_settings"]["optimizer"],
                  "optimizer_success": success, "optimizer_result": message,
                  "status": "baseline" if baseline_mode else ("converged_feasible" if success and final["feasible"] else "failed_or_infeasible"),
                  "notice": "Uncalibrated conceptual estimates; points include only mass and energy productivity.",
                  "baseline": baseline, "final": final}
        output.write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        iterations = []
        if not baseline_mode:
            reader = om.CaseReader(str(recorder_path))
            for i, case in enumerate(reader.get_cases("driver", recurse=False)):
                iterations.append({"label": f"Driver evaluation {i + 1}", "snapshot": snapshot(case, configuration=problem.model_metadata["configuration"])})
        input_link = (Path(os.path.relpath(args.inputs.resolve(), output.parent.resolve())).as_posix()
                      if args.inputs and baseline_mode else None)
        write_report(record, output, iterations, download_links={"inputs": input_link})
        print(f"\nSaved {output.resolve()}")
        print(f"Report and geometry: {output.with_suffix('.html').resolve()}")
        return 0 if baseline_mode or (success and final["feasible"]) else 1
    finally:
        problem.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
