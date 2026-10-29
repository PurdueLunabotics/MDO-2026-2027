"""Publish two reports with one canonical input JSON for each design.

Run after main.py: python -m reporting.archive BASELINE.json OPTIMIZED.json
"""
import argparse
import hashlib
import json
import os
from pathlib import Path

from .html_report import write_report
from .case_inputs import read_case


def archive_cases(baseline, optimized, destination, *, baseline_inputs, optimized_inputs):
    """Keep the baseline read-only; write the candidate inputs and both reports.

    Full numeric run records stay in the working run directory and are also
    embedded in each self-contained HTML report. No duplicate JSON is published.
    """
    destination = Path(destination)
    sources = [("baseline", Path(baseline)), ("optimized", Path(optimized))]
    cases = [(name, path, json.loads(path.read_text(encoding="utf-8")))
             for name, path in sources]
    if cases[0][2].get("status") != "baseline":
        raise ValueError("The baseline must be a fixed-design evaluation.")
    if cases[1][2].get("status") == "baseline":
        raise ValueError("The optimized case must be an optimizer result, including a failed result.")
    # Prevent archiving stale or unrelated studies as a baseline/optimization pair.
    if cases[0][2]["final"]["design"] != cases[1][2]["baseline"]["design"] or cases[0][2]["final"]["assumptions"] != cases[1][2]["baseline"]["assumptions"]:
        raise ValueError("The optimization must start from the evaluated baseline inputs.")
    if cases[0][2]["model_metadata"].get("traversal_mode", "fixed") != cases[1][2]["model_metadata"].get("traversal_mode", "fixed"):
        raise ValueError("The archived baseline and optimization must use the same traversal mode.")
    baseline_inputs, optimized_inputs = Path(baseline_inputs), Path(optimized_inputs)
    supplied, source = read_case(baseline_inputs)
    actual = {n: item['value'] for group in ('design', 'assumptions')
              for n, item in cases[0][2]['final'][group].items()}
    if any(n not in actual or actual[n] != value for n, value in supplied.items()):
        raise ValueError("The baseline source has edits that are not in the evaluated report.")
    source_hash = cases[0][2].get('report_context', {}).get('source_sha256')
    if source_hash and source_hash != source['sha256']:
        raise ValueError("The baseline source changed after evaluation; regenerate the reports.")
    targets = [destination/f"{name}.html" for name, _, _ in cases] + [optimized_inputs]
    if any(path.resolve() in {target.resolve() for target in targets} for _, path, _ in cases):
        raise ValueError("Choose an archive directory separate from the source runs.")
    if baseline_inputs.resolve() in {target.resolve() for target in targets}:
        raise ValueError("Published files must not overwrite the source baseline.")
    destination.mkdir(parents=True, exist_ok=True)
    for name, path, record in cases:
        output = destination/f"{name}.json"
        context, metadata = record.get("report_context", {}), record["model_metadata"]
        inputs = {n: item["value"] for group in ("design", "assumptions")
                  for n, item in record["final"][group].items()}
        effective = {"name": context.get("name", name), "revision": context.get("revision", "Not specified"),
                     "structure": metadata["structure_profile"], "motors": metadata["motor_profile"],
                     "traversal_mode": metadata.get("traversal_mode", "fixed"),
                     "attachment_max": metadata.get("attachment_max",1.),
                     "actuators": metadata.get("actuator_profile", "reference"),
                     "inputs": inputs,
                     "notes": {"source_baseline_sha256": source['sha256'],
                               "analysis_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                               "status": record["status"], "optimizer_success": record["optimizer_success"],
                               "modeled_feasible": record["final"]["feasible"],
                               "origin": "Effective model inputs; not measured hardware validation."}}
        input_path = baseline_inputs if name == 'baseline' else optimized_inputs
        if name == 'optimized':
            input_path.parent.mkdir(parents=True, exist_ok=True)
            input_path.write_text(json.dumps(effective, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        input_link = Path(os.path.relpath(input_path.resolve(), destination.resolve())).as_posix()
        write_report(record, output, download_links={"analysis": None, "inputs": input_link, "n2": None})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("optimized", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("docs/design_reports"))
    parser.add_argument("--baseline-inputs", type=Path, default=Path("design_baseline.json"))
    parser.add_argument("--optimized-inputs", type=Path, default=Path("design_optimized.json"))
    args = parser.parse_args()
    archive_cases(args.baseline, args.optimized, args.output_dir,
                  baseline_inputs=args.baseline_inputs, optimized_inputs=args.optimized_inputs)
    print(f"Published reports in {args.output_dir.resolve()}; inputs: {args.baseline_inputs}, {args.optimized_inputs}")


if __name__ == "__main__":
    main()
