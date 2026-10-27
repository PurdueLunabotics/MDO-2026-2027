"""Offline actuator/pivot multistart study. Never rewrites maintained designs.

python -m studies.linkage_multistart --workers 3
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import redirect_stdout, redirect_stderr
from datetime import datetime, timezone
from html import escape
from importlib.metadata import version
import json
import math
from pathlib import Path
import time
import traceback

from .traversal_fill import atomic_json, digest

ROOT = Path(__file__).resolve().parents[1]
FROZEN = ["wheel_diameter", "wheel_width"]
IGNORED = ["turn_traction_margin"]
LAYOUTS = {
    "matched": (0., 0., 0., 0., 1.),
    "rear_high": (-.02, .02, -.035, .08, 1.),
    "rear_low": (-.05, -.015, -.025, -.04, 1.05),
    "forward_high": (.04, .035, .035, .09, .95),
}


def now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def selected_failures(state):
    return {n:c for n,c in state["constraints"].items() if not c["passed"] and n not in IGNORED}


def normalized_violation(state, metadata):
    from rover.config import REQUIREMENTS
    config = metadata["configuration"]
    total = 0.
    for name,check in selected_failures(state).items():
        if name in REQUIREMENTS:
            scale = REQUIREMENTS[name][0]
        elif name.endswith("_bounds"):
            spec = {**config["design"], **config.get("frozen_design", {})}[name[:-7]]
            scale = spec[2]-spec[1]
        else:
            scale = config["margins"][name][0]
        total += max(0., -check["margin"])/scale
    return total


def geometry_seed(reference, fraction, layout, rotation):
    """Construct actual pin lengths from varied pivots and matched arm angles."""
    state = reference["final"]
    v = {n:i["value"] for group in ("design", "assumptions", "outputs") for n,i in state[group].items()}
    bx,bz,ax,az,scale = LAYOUTS[layout]
    values = {n:i["value"] for group in ("design", "assumptions") for n,i in state[group].items()}
    values.update(pivot_x=v["pivot_x"]+bx, bottom_pivot_height=v["bottom_pivot_height"]+bz,
                  actuator_pivot_x=v["actuator_pivot_x"]+ax, top_pivot_height=v["top_pivot_height"]+az,
                  arm_length=v["arm_length"]*scale, actuator_attachment_fraction=fraction)
    for pose,length in (("travel", "travel_actuator_length"), ("stowed", "actuator_retracted_length"),
                         ("extended", "actuator_extended_length")):
        angle = math.atan2(v[pose+"_y"]-v["bottom_pivot_height"], v[pose+"_x"]-v["pivot_x"])+rotation
        pin_x = values["pivot_x"]+fraction*values["arm_length"]*math.cos(angle)
        pin_z = values["bottom_pivot_height"]+fraction*values["arm_length"]*math.sin(angle)
        values[length] = math.hypot(pin_x-values["actuator_pivot_x"], pin_z-values["top_pivot_height"])
    if fraction < 1:
        values["actuator_force_rating"] = 6000.  # Explicit starting value, not automatic resizing in compute.
    values.update(empty_slope_degrees=0., loaded_slope_degrees=0.)
    return values


def make_problem(values, **options):
    from rover.problem import build_problem
    return build_problem(values, structure="plate-frame", motors="x44-standard", traversal_mode="calculated",
                         freeze=FROZEN, ignore_constraints=IGNORED, **options)


def make_row(name, record, folder):
    state = record.get("final")
    row = {"id": name, "status": record["status"], "optimizer_success": record.get("optimizer_success", False),
           "report": f"cases/{name}/analysis.html" if state else None,
           "wall_seconds": record["study_attempt"]["wall_seconds"], "error": record.get("error")}
    if state:
        values = {n:i["value"] for group in ("design", "assumptions", "outputs") for n,i in state[group].items()}
        row.update(score=values["construction_points"], full_model_feasible=state["feasible"],
                   selected_constraints_pass=not selected_failures(state),
                   selected_failed_checks=selected_failures(state),
                   full_failed_checks={n:c for n,c in state["constraints"].items() if not c["passed"]},
                   values=values)
        row["eligible"] = bool(row["optimizer_success"] and row["selected_constraints_pass"])
    else:
        row["eligible"] = False
    return row


def run_attempt(job):
    from rover.problem import build_problem
    from reporting.report import snapshot
    from reporting.html_report import write_report
    import openmdao
    name = job["name"]
    folder = Path(job["output"])/"cases"/name
    folder.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    p = None
    record = {"schema_version": 2, "openmdao_version": openmdao.__version__, "optimizer_success": False,
              "status": "error", "notice": "Uncalibrated conceptual estimates; explicit turn-traction exclusion retained.",
              "report_context": {"name": "Actuator/pivot study: "+name, "revision": "2026-10-27",
                  "source": "Immutable study seed inputs", "generated_utc": now(), "mode": "optimization",
                  "source_notes": {"study_fingerprint": job["fingerprint"], "parent_attempt": job.get("parent")}}}
    with (folder/"solver.log").open("w", encoding="utf-8") as log, redirect_stdout(log), redirect_stderr(log):
        print("Starting", name, flush=True)
        try:
            p = make_problem(job["values"], optimize_attachment=job.get("optimize_attachment", True),
                             maxiter=job["maxiter"], cobyla_radius=job["radius"], solver_tol=job["tol"],attachment_max=job.get('attachment_max',1.))
            record["model_metadata"] = p.model_metadata
            p.run_model()
            record["baseline"] = snapshot(p)
            result = p.run_driver()
            record["optimizer_success"] = bool(result.success)
            record["optimizer_result"] = str(p.driver.result)
            record["optimizer"] = "COBYLA"
            p.run_model()
            record["final"] = snapshot(p)
            record["status"] = "converged_feasible" if result.success and record["final"]["feasible"] else "failed_or_infeasible"
        except Exception:
            record["error"] = traceback.format_exc()
            print(record["error"], flush=True)
            # A partially computed state after an exception is never a candidate.
            record.pop("final", None)
        finally:
            if p is not None:
                p.cleanup()
        record["study_attempt"] = {"name": name, "parent": job.get("parent"), "wall_seconds": time.perf_counter()-started,
                                   "maxiter": job["maxiter"], "radius": job["radius"], "tol": job["tol"],
                                   "seed_inputs": job["values"], "optimize_attachment": job.get("optimize_attachment", True)}
        atomic_json(folder/"analysis.json", record)
        if "final" in record:
            effective = {n:i["value"] for group in ("design", "assumptions") for n,i in record["final"][group].items()}
            atomic_json(folder/"candidate_inputs.json", {"name": name, "revision": "Actuator/pivot study 2026-10-27",
                "structure": "plate-frame", "motors": "x44-standard", "traversal_mode": "calculated", "inputs": effective,
                "attachment_max":job.get('attachment_max',1.),
                "notes": {"full_status": record["status"], "optimizer_success": record["optimizer_success"],
                          "selected_constraints_pass": not selected_failures(record["final"]), "ignored_optimizer_constraints": IGNORED,
                          "frozen_for_study": FROZEN, "not_measured_hardware": True}})
            write_report(record, folder/"analysis.json", download_links={"inputs": "candidate_inputs.json", "n2": None})
    return make_row(name, record, folder)


def publish(manifest, output):
    atomic_json(output/"study.json", manifest)
    rows = sorted(manifest["rows"], key=lambda r:(not r.get("eligible"), -r.get("score", -1e100)))
    body = []
    reference_score = manifest["reference_score"]
    for row in rows:
        v = row.get("values", {})
        cell = lambda n,scale=1: f"{v[n]*scale:.2f}" if n in v else "—"
        title = escape(row["id"])
        if row.get("report"):
            title = f'<a href="{row["report"]}">{title}</a>'
        label = "Converged under selected constraints" if row.get("eligible") else "Failed / diagnostic"
        gain = f'{100*(row["score"]/reference_score-1):+.2f}%' if "score" in row else "—"
        body.append('<tr>'+''.join('<td>'+str(x)+'</td>' for x in [title,label,cell("construction_points"),gain,
            cell("actuator_attachment_fraction"),cell("arm_length",1000),
            cell("actuator_pivot_x",1000)+", "+cell("top_pivot_height",1000),
            cell("pivot_x",1000)+", "+cell("bottom_pivot_height",1000),
            cell("travel_actuator_length",1000)+" / "+cell("actuator_retracted_length",1000)+" / "+cell("actuator_extended_length",1000),
            cell("actuator_force_rating"),cell("turn_traction_margin")])+'</tr>')
    html = '<!doctype html><meta charset="utf-8"><title>Actuator and pivot multistart study</title><style>body{font:15px system-ui;margin:32px;color:#20323e}table{border-collapse:collapse;width:100%}th,td{padding:9px;border-bottom:1px solid #ddd;text-align:left}th{background:#edf4f4}a{color:#176a67}p{max-width:1100px}</style>'
    html += '<h1>Actuator and pivot multistart study</h1><p>Reference score: '+f'{reference_score:.2f}'+f'. Wheels fixed; actuator lengths and pivots adjustable. Attachment is optimized in [0.001, {manifest["settings"].get("attachment_max",1.):g}] unless held at the reference value for a control run. Climb angles are zero; turn traction is excluded from the optimizer only.</p>'
    if manifest['settings'].get('attachment_max',1.)>1:
        html+='<p>Fractions above 1 continue the uniform arm tube beyond the drum shaft. Extra mass, inertia, structural loads and physical arm envelopes are included; brackets, arm/actuator interference and soil contact still need CAD/testing.</p>'
    html += '<p><b>Eligibility means solver success and every selected constraint passes.</b> Full-model pass/fail remains unchanged in each report. An excluded turn-traction failure is not a verified feasible rover. Continuous actuator dimensions are exploratory; no catalog selection or physical validation is claimed.</p>'
    html += f'<p>{len(rows)} / {manifest["planned_attempts"]} planned attempts saved. <a href="study.json">Study manifest and numerical results</a>. Best found local results; no global optimality claim.</p>'
    html += '<div style="overflow:auto"><table><thead><tr>'+''.join('<th>'+x+'</th>' for x in ["Attempt/report","Study status","Score","Gain","Pin fraction","Arm mm","Actuator pivot x,z mm","Arm pivot x,z mm","Actuator travel/retracted/extended mm","Actuator rating N","Turn traction N"])+ '</tr></thead><tbody>'+''.join(body)+'</tbody></table></div>'
    (output/"index.html").write_text(html, encoding="utf-8")


def screen_seeds(reference, count, attachment_max=1.,fractions=(1.,.85,.7,.55)):
    from rover.config import configuration
    from reporting.report import snapshot
    design,_ = configuration("plate-frame", "x44-standard")
    design["actuator_attachment_fraction"] = (1., .001, attachment_max, None)
    screened = []
    for fraction in fractions:
        pool = []
        for layout in LAYOUTS:
            for rotation in (-.06, 0., .06):
                values = geometry_seed(reference, fraction, layout, rotation)
                if any(not lo <= values[n] <= hi for n,(_,lo,hi,_) in design.items()):
                    continue
                p = None
                try:
                    p = make_problem(values, optimize_attachment=True, maxiter=1,attachment_max=attachment_max)
                    p.run_model(); state = snapshot(p)
                    pool.append({"name": f"f{fraction:g}_{layout}_r{rotation:+.2f}".replace('.','p').replace('+','plus').replace('-','minus'),
                                 "values": values, "fraction": fraction, "layout": layout,
                                 "violation": normalized_violation(state,p.model_metadata),
                                 "seed_score": state["outputs"]["construction_points"]["value"]})
                except Exception:
                    continue  # Rejected screening geometry, not a solver result.
                finally:
                    if p is not None:p.cleanup()
        pool.sort(key=lambda s:(s["violation"],-s["seed_score"]))
        used = set()
        for seed in pool:
            if seed["layout"] not in used:
                screened.append(seed);used.add(seed["layout"])
            if len(used) >= count:break
    return screened


def execute(jobs, manifest, output, workers):
    cached = {r["id"] for r in manifest["rows"]}
    jobs = [j for j in jobs if j["name"] not in cached]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_attempt,job):job for job in jobs}
        for future in as_completed(futures):
            row = future.result()
            manifest["rows"].append(row);manifest["updated_utc"] = now();publish(manifest,output)
            print(json.dumps({"completed":len(manifest["rows"]),"id":row["id"],"score":row.get("score"),
                              "eligible":row["eligible"],"seconds":row["wall_seconds"]}),flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference",type=Path,default=Path("results/fixed_hardware_optimization.json"))
    parser.add_argument("--output",type=Path,default=Path("results/actuator_pivot_study_2026_10_27"))
    parser.add_argument("--workers",type=int,default=3)
    parser.add_argument("--seeds-per-fraction",type=int,default=2)
    parser.add_argument("--maxiter",type=int,default=900)
    parser.add_argument("--refine",type=int,default=3)
    parser.add_argument("--refine-maxiter",type=int,default=2000)
    parser.add_argument('--attachment-max',type=float,default=1.)
    parser.add_argument('--fractions',type=float,nargs='+')
    args = parser.parse_args()
    fractions=args.fractions or ([1.,1.15,1.3,1.5] if args.attachment_max>1 else [1.,.85,.7,.55])
    if not 1<=args.attachment_max<=1.5 or any(not 0<f<=args.attachment_max for f in fractions):
        parser.error('Attachment bound must be in [1,1.5], and starting fractions must fit the bound')
    if min(args.workers,args.seeds_per_fraction,args.maxiter,args.refine_maxiter)<1 or args.refine<0:
        parser.error("Worker, seed and evaluation limits must be positive; refine must be nonnegative")
    raw = args.reference.read_bytes();reference = json.loads(raw)
    if reference["model_metadata"]["structure_profile"]!="plate-frame" or reference["model_metadata"]["motor_profile"]!="x44-standard":
        parser.error("This study uses the selected plate-frame/X44 hardware profiles")
    output = args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    source_inputs = {n:i["value"] for group in ("design","assumptions") for n,i in reference["final"][group].items()}
    source_inputs.update(empty_slope_degrees=0.,loaded_slope_degrees=0.)
    import hashlib
    files = sorted((ROOT/"rover").rglob("*.py"))+sorted((ROOT/"data/motors").glob("*.csv"))+[ROOT/"data/motors/manifest.json",ROOT/"data/mobility/bekker_wong_reece/grc1_loose.json"]
    settings = {"reference_sha256":hashlib.sha256(raw).hexdigest(),"model_hashes":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                "runner_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "maxiter":args.maxiter,"refine":args.refine,"refine_maxiter":args.refine_maxiter,"seeds_per_fraction":args.seeds_per_fraction,
                "attachment_max":args.attachment_max,"fractions":fractions,
                "versions":{n:version(n) for n in ("openmdao","numpy","scipy")},"frozen":FROZEN,"ignored":IGNORED}
    fingerprint = digest(settings)
    path = output/"study.json"
    if path.exists():
        manifest=json.loads(path.read_text())
        if manifest["fingerprint"]!=fingerprint:parser.error("Existing study fingerprint differs; choose a new output folder")
        seeds=manifest["seeds"]
    else:
        print("Screening pivot/pin seed geometries",flush=True)
        seeds=screen_seeds(reference,args.seeds_per_fraction,args.attachment_max,fractions)
        manifest={"schema_version":1,"created_utc":now(),"fingerprint":fingerprint,"settings":settings,"seeds":seeds,
                  "reference_score":reference["final"]["outputs"]["construction_points"]["value"],"rows":[],"planned_attempts":len(seeds)+2+args.refine}
        (output/"reference_report.json").write_bytes(raw)
        publish(manifest,output)
    controls=[{"name":"control_fixed_attachment","values":source_inputs,"optimize_attachment":False},
              {"name":"incumbent_free_fraction","values":source_inputs,"optimize_attachment":True}]
    common={"output":str(output),"fingerprint":fingerprint,"maxiter":args.maxiter,"radius":.02,"tol":1e-5,"attachment_max":args.attachment_max}
    jobs=[{**common,**s} for s in controls+seeds]
    execute(jobs,manifest,output,args.workers)
    ranked=sorted((r for r in manifest["rows"] if not r["id"].startswith("refine_") and r.get("selected_constraints_pass")),key=lambda r:r["score"],reverse=True)
    refinements=[]
    for index,row in enumerate(ranked[:args.refine]):
        record=json.loads((output/"cases"/row["id"]/"analysis.json").read_text())
        values={n:i["value"] for group in ("design","assumptions") for n,i in record["final"][group].items()}
        refinements.append({**common,"name":f"refine_{index+1}_{row['id']}","values":values,"parent":row["id"],
                            "maxiter":args.refine_maxiter,"radius":.003,"tol":1e-6,"optimize_attachment":True})
    manifest["planned_attempts"]=len(jobs)+len(refinements)
    execute(refinements,manifest,output,args.workers)
    manifest["complete"]=True;publish(manifest,output)
    print("Study complete:",output/"index.html",flush=True)


if __name__=="__main__":
    main()
