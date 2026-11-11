r"""Run with .venv\Scripts\python.exe main.py; use --baseline to analyze only."""

import argparse
from datetime import datetime, timezone
from functools import partial
import json
import os
from pathlib import Path
import time
import openmdao
import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_totals
from rover.problem import build_problem
from rover.config import configuration
from rover.development import BudgetedScipyDriver, RunBudgetExceeded, profile_model, selected_freeze
from reporting.report import snapshot, print_report
from reporting.html_report import write_report
from reporting.case_inputs import read_case


def print_evaluation(title, state, *, compact=False):
    if not compact:
        print_report(title, state)
        return
    failures = [name for name, check in state["constraints"].items() if not check["passed"]]
    print(f"{title}: feasible={state['feasible']}; {len(failures)} failed modeled checks")
    if failures:
        print("Failed checks: " + ", ".join(failures))


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
                        help="Calculated motor/soil-limited route timing (default) or a fixed one-way schedule")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="Override a config value in its stated units")
    parser.add_argument("--freeze", action="extend", nargs="+", default=[], metavar="NAME",
                        help="Hold design variables at their resolved input values; accepts multiple names and may be repeated")
    parser.add_argument("--vary", action="extend", nargs="+", metavar="NAME",
                        help="Optimize only these active design variables; hold the rest fixed with their bound checks")
    parser.add_argument("--quick", action="store_true",
                        help="Bounded development optimization: COBYLA, 80 evaluations, 60 s, radius 0.1 and JSON only; explicit settings override")
    parser.add_argument("--time-limit", type=float, metavar="SECONDS",
                        help="COBYLA time budget checked between complete evaluations; interrupted runs remain failed")
    parser.add_argument("--profile-model", action="store_true",
                        help="Profile exactly one fixed model evaluation and save cumulative timings; never optimize")
    parser.add_argument("--report", choices=["full", "final", "json"],
                        help="Full history (default), final HTML/N2 only, or numeric JSON only; quick/profile default to JSON")
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
    parser.add_argument("--n2", action="store_true", help="Compatibility flag; N2 is included with full/final reports")
    parser.add_argument("--output", type=Path, help="JSON path; defaults to results/baseline.json or results/optimization.json")
    args = parser.parse_args()
    if args.maxiter is not None and args.maxiter < 1:
        parser.error("--maxiter must be positive")
    if args.profile_model and (args.optimize or args.quick or args.check_derivatives):
        parser.error("--profile-model performs one fixed evaluation; omit optimization and derivative-check flags")
    if args.quick and args.baseline:
        parser.error("--quick is an optimization; omit --baseline")
    report_mode = args.report or ("json" if args.quick or args.profile_model else "full")
    if args.n2 and report_mode == "json":
        parser.error("--n2 requires --report final or --report full")
    try:
        case_values, source = read_case(args.inputs) if args.inputs else ({}, {"source": "Configured defaults"})
        cli_values = dict(item.split("=", 1) for item in args.set)
        overrides = {**case_values, **cli_values}
        structure = args.structure or source.get("structure", "reference")
        motors = args.motors or source.get("motors", "reference")
        traversal_mode = args.traversal_mode or source.get("traversal_mode", "calculated")
        attachment_max = args.attachment_max if args.attachment_max is not None else source.get('attachment_max',1.)
        actuators = args.actuators or source.get("actuators", "reference")
        baseline_mode = args.baseline or args.profile_model or (args.inputs is not None and not (args.optimize or args.quick))
        if baseline_mode and (args.vary is not None or args.time_limit is not None):
            raise ValueError("--vary and --time-limit require optimization")
        optimizer = args.optimizer or ("COBYLA" if args.quick else None)
        maxiter = args.maxiter if args.maxiter is not None else (80 if args.quick else None)
        radius = args.cobyla_radius if args.cobyla_radius is not None else (.1 if args.quick else None)
        time_limit = args.time_limit if args.time_limit is not None else (60. if args.quick else None)
        design, _ = configuration(structure, motors, actuators,
                                  actuator_stroke=overrides.get("actuator_catalog_stroke"))
        freeze = selected_freeze(design, vary=args.vary, freeze=args.freeze)
        driver_factory = None
        if time_limit is not None:
            effective_optimizer = optimizer or ("COBYLA" if structure == "plate-frame" else "SLSQP")
            if effective_optimizer != "COBYLA":
                raise ValueError("--quick/--time-limit require --optimizer COBYLA; gradient evaluations are not time-bounded")
            driver_factory = partial(BudgetedScipyDriver, evaluation_limit=maxiter or 2000, time_limit=time_limit)
        filename = ("model_profile.json" if args.profile_model else "quick_optimization.json" if args.quick
                    else "baseline.json" if baseline_mode else "optimization.json")
        output = args.output or Path("results") / filename
        artifacts = [output, output.with_suffix(".html"), output.with_suffix(".sql"), output.with_name(output.stem+"_profile.txt"),
                     output.with_name(output.stem+"_n2.html")]
        artifacts += [output.with_name(output.stem+"_"+part+".svg")
                      for part in ("retracted", "travel", "deposition", "extended", "frame", "stability", "top", "obstacle")]
        if args.inputs and args.inputs.resolve() in {path.resolve() for path in artifacts}:
            raise ValueError("Output artifacts would overwrite the source baseline. Choose --output with a different filename.")
        problem = build_problem(overrides, optimizer=optimizer, maxiter=maxiter, display=True, structure=structure, cobyla_radius=radius, motors=motors, actuators=actuators, solver_tol=args.solver_tol, freeze=freeze, ignore_constraints=args.ignore_constraint, traversal_mode=traversal_mode, optimize_attachment=args.optimize_attachment, attachment_max=attachment_max, driver_factory=driver_factory)
        context = {
            "name": args.design_name or source.get("name", "Configured design — example"),
            "revision": args.revision or source.get("revision", "Not specified"),
            "source": source["source"], "source_sha256": source.get("sha256"),
            "source_notes": source.get("notes"),
            "input_file_names": sorted(case_values), "cli_override_names": sorted(cli_values),
            "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "mode": "fixed_design" if baseline_mode else "optimization",
            "audience": "design" if baseline_mode and args.inputs else "analysis",
        }
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    output.parent.mkdir(parents=True, exist_ok=True)
    if not baseline_mode and report_mode == "full":
        recorder_path = output.with_suffix(".sql").resolve()
        problem.driver.add_recorder(om.SqliteRecorder(str(recorder_path)))
        problem.driver.recording_options["includes"] = ["*"]
    try:
        timings = {}
        if args.profile_model:
            timings["baseline_seconds"], profile_text = profile_model(problem)
            profile_path = output.with_name(output.stem+"_profile.txt")
            profile_path.write_text(profile_text, encoding="utf-8")
            print(profile_text)
        else:
            started = time.perf_counter()
            problem.run_model()
            timings["baseline_seconds"] = time.perf_counter()-started
        print(f"Model evaluation: {timings['baseline_seconds']:.3f} s; active design variables: {len(problem.design_specs)}")
        baseline = snapshot(problem)
        print_evaluation("Design baseline evaluation" if baseline_mode else "Optimization starting design", baseline,
                         compact=args.quick or args.profile_model)
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
            started = time.perf_counter()
            try:
                result = problem.run_driver()
                success = bool(result.success)
                message = str(problem.driver.result)
            except RunBudgetExceeded as exc:
                success, message = False, str(exc)
                print(message)
            timings["driver_seconds"] = time.perf_counter()-started
            problem.run_model()
            final = snapshot(problem)
            print_evaluation("Final optimizer iterate", final, compact=args.quick)
            print(f"Optimizer success: {success}")
        if report_mode != "json":
            om.n2(problem, outfile=str(output.parent / f"{output.stem}_n2.html"), show_browser=False)
        record = {"schema_version": problem.model_metadata["schema_version"], "model_metadata": problem.model_metadata, "report_context": context,
                  "openmdao_version": openmdao.__version__, "optimizer": problem.model_metadata["optimizer_settings"]["optimizer"],
                  "optimizer_success": success, "optimizer_result": message,
                  "status": "baseline" if baseline_mode else ("converged_feasible" if success and final["feasible"] else "failed_or_infeasible"),
                  "notice": "Uncalibrated conceptual estimates; points include only mass and energy productivity.",
                  "baseline": baseline, "final": final,
                  "performance": {"timings": timings, "report_mode": report_mode,
                                  "quick": args.quick, "time_limit_seconds": time_limit,
                                  "objective_evaluations": getattr(problem.driver, "objective_evaluations", None)}}
        output.write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        iterations = []
        if not baseline_mode and report_mode == "full":
            reader = om.CaseReader(str(recorder_path))
            for i, case in enumerate(reader.get_cases("driver", recurse=False)):
                iterations.append({"label": f"Driver evaluation {i + 1}", "snapshot": snapshot(case, configuration=problem.model_metadata["configuration"])})
        input_link = (Path(os.path.relpath(args.inputs.resolve(), output.parent.resolve())).as_posix()
                      if args.inputs and baseline_mode else None)
        if report_mode != "json":
            write_report(record, output, iterations, download_links={"inputs": input_link})
        print(f"\nSaved {output.resolve()}")
        if report_mode != "json":
            print(f"Report and geometry: {output.with_suffix('.html').resolve()}")
        return 0 if baseline_mode or (success and final["feasible"]) else 1
    finally:
        problem.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
