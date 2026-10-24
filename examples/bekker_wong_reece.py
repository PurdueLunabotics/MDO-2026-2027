"""Fixed-design BWR versus legacy route comparison; no optimization or changed bounds.

python -m examples.bekker_wong_reece --check-derivatives
The published GRC-1 preset is an exploratory proxy, not a BP-1 calibration.
"""
import argparse
from dataclasses import replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from openmdao.utils.assert_utils import assert_check_totals

from reporting.case_inputs import read_case
from reporting.html_report import write_report
from reporting.report import snapshot
from rover.alternatives.bekker_wong_reece import rover_on_slope, transfer_flags, WheelDomainError
from rover.models import ModelSet, plate_frame_models
from rover.mobility import Mobility
from rover.problem import build_problem


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs",type=Path,default=Path("design_optimized.json"))
    parser.add_argument("--output",type=Path,default=Path("results/bwr_current_rover.json"))
    parser.add_argument("--check-derivatives",action="store_true")
    args = parser.parse_args()
    values, source = read_case(args.inputs)
    structure, motors = source.get("structure","reference"), source.get("motors","reference")
    models = plate_frame_models() if structure == "plate-frame" else ModelSet()
    reference = build_problem(values,models=replace(models,mobility=Mobility),structure=structure,motors=motors)
    experiment = build_problem(values,models=models,structure=structure,motors=motors)
    try:
        reference.run_model()
        old = snapshot(reference)
        experiment.run_model()
        new = snapshot(experiment)
        turns = ("turn_time","route_turn_time","turn_peak_power","turn_motor_margin","turn_traction_margin","turn_power_margin")
        assert all(reference.get_val(n).item()==experiment.get_val(n).item() for n in turns)
        derivatives = "not requested"
        if args.check_derivatives:
            checked = experiment.check_totals(of=["mobility_energy","drive_peak_power"],wrt=["wheel_diameter","wheel_width"],
                                             method="fd",form="central",step=3e-5,step_calc="rel_avg",out_stream=None)
            assert_check_totals(checked,atol=1e-3,rtol=2e-3)
            derivatives = "passed total derivative step comparison at this smooth fixed design"
        v = lambda name: float(experiment.get_val(name).item())
        points = []
        for state,mass in (("empty",v("dry_mass")),("loaded",v("dry_mass")+v("payload_mass"))):
            for angle in (0,5,10,12.5,15,20,30):
                row = {"state":state,"mass_kg":mass,"slope_deg":angle}
                try:
                    p = rover_on_slope(experiment.model.mobility.soil,v("wheel_diameter"),v("wheel_width"),mass,math.radians(angle))
                    row.update(status="solved_exploratory",**p,soil_fit_transfer_flags=transfer_flags(p))
                except WheelDomainError as exc:
                    row.update(status="failed",reason=str(exc))
                points.append(row)
        record = {"schema_version":2,"model_metadata":experiment.model_metadata,
            "report_context":{"name":"Default rigid-wheel BWR - fixed design",
                              "source":str(args.inputs),"revision":experiment.model_metadata["evidence"]["mobility"]["revision"],
                              "generated_utc":datetime.now(timezone.utc).isoformat(timespec="seconds"),
                              "input_file_names":sorted(values),"cli_override_names":[],"mode":"fixed_design"},
            "status":"baseline","optimizer_success":None,"optimizer_result":"Fixed design only; no optimization attempted",
            "baseline":old,"final":new,"bwr_slope_diagnostics":points,
            "bwr_diagnostic_scope":"Additional equal-load slope sweep; default empty/full requirements are also enforced separately. Failed solutions remain failed.",
            "comparison":{"point_turn_outputs_identical":True,"derivatives":derivatives,
                          "connection_count":len(experiment.model._conn_global_abs_in2out),
                          "automatic_ivc_connections":[n for n in experiment.model._conn_global_abs_in2out.values() if n.startswith('_auto_ivc')]}}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(record,indent=2)+"\n",encoding="utf-8")
        write_report(record,args.output)
        print(json.dumps({"reference_mobility_energy_J":v_from(old,"mobility_energy"),
                          "bwr_mobility_energy_J":v("mobility_energy"),"derivatives":derivatives,
                          "modeled_constraints_feasible":new["feasible"],"slope_diagnostics":points},indent=2))
        print(args.output.with_suffix('.html'))
    finally:
        reference.cleanup()
        experiment.cleanup()


def v_from(state,name):
    return state["outputs"][name]["value"]


if __name__ == "__main__":
    main()
