"""Independent larger-rover check of the unfitted rigid-wheel BWR model.

python -m studies.bekker_wong_reece
Published parameters and the acceptance gate are versioned before comparison.
Sensitivity scenarios are not fitted parameters, error bars or confidence limits.
"""
from dataclasses import replace
import html
import json
import math
from pathlib import Path

import numpy as np

from rover.alternatives.bekker_wong_reece import load_preset, rover_on_slope, transfer_flags, WheelDomainError
from studies.rover_validation_data import measurements

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "results/bekker_wong_reece"


def prediction(soil, row, diameter=None):
    try:
        p = rover_on_slope(soil, diameter or row["diameter_m"], row["width_m"], row["mass_kg"],
                           math.radians(row["slope_deg"]), wheel_count=row["wheel_count"])
        return {"status": "solved_exploratory", **p, "soil_fit_transfer_flags": transfer_flags(p)}
    except WheelDomainError as exc:
        return {"status": "failed", "reason": str(exc), "slip": None}


def assess():
    soil, manifest = load_preset()
    previous = measurements()
    rows = []
    sd = manifest["source_conditions"]["K_standard_deviation_m"]
    variants = {
        "K_minus_published_SD": replace(soil, shear_modulus_m=soil.shear_modulus_m-sd),
        "K_plus_published_SD": replace(soil, shear_modulus_m=soil.shear_modulus_m+sd),
        "generic_compact_sand_stress_shape": replace(soil, stress_c1=.43, stress_c2=.32),
    }
    for point in previous["results"]:
        predicted = prediction(soil, point)
        measured = point["measured_outer_reference_slip"]
        row = {**point, "bwr": predicted,
               "bwr_error_pp": 100*(predicted["slip"]-measured) if predicted["slip"] is not None else None,
               "sensitivities": {name: prediction(s, point) for name, s in variants.items()}}
        if point["case"] == "husky_grouser":
            # Alternate physical cylinder geometry, not a change to the measured
            # slip reference; express its prediction on the SAME 384 mm basis.
            alternate = prediction(soil, point, diameter=.34304)
            if alternate["slip"] is not None:
                alternate["slip_on_384mm_reference"] = 1-(1-alternate["slip"])*.34304/.384
            row["nominal_cylinder_sensitivity"] = alternate
        rows.append(row)
    summaries = {}
    gate = manifest["predeclared_benchmark_gate"]
    for case in previous["manifest"]["cases"]:
        group = [r for r in rows if r["case"] == case["id"]]
        valid = [r for r in group if r["bwr_error_pp"] is not None]
        errors = np.array([r["bwr_error_pp"] for r in valid])
        summaries[case["id"]] = {
            "total_observations": len(group), "solved": len(valid), "failed": len(group)-len(valid),
            "mae_pp_on_solved_points_only": float(np.mean(abs(errors))) if len(valid) else None,
            "rmse_pp_on_solved_points_only": float(np.sqrt(np.mean(errors**2))) if len(valid) else None,
            "max_abs_error_pp_on_solved_points_only": float(max(abs(errors))) if len(valid) else None,
            "gate_passed": bool(len(valid) == len(group) and np.mean(abs(errors)) <= 100*gate["maximum_group_mae_slip"]
                                and max(abs(errors)) <= 100*gate["maximum_point_absolute_error_slip"]),
            "gate_applicable": case["wheel_count"] == 4,
            "scope": "Four-wheel GRC-1 preparation transfer" if case["wheel_count"] == 4 else "Secondary six-wheel, different natural sand; not matched-soil validation",
        }
    return {"model_revision": manifest["revision"], "soil_manifest": manifest,
            "external_manifest": previous["manifest"], "external_manifest_sha256": previous["manifest_sha256"],
            "parameters_refitted": False,
            "benchmark_passed": all(summaries[c]["gate_passed"] for c in ("husky_stock", "husky_grouser")),
            "summary": summaries, "results": rows,
            "interpretation": "Default conservative design assumption selected by student; failed independent accuracy gate retained. Rigid smooth proxies and soil preparation mismatch remain. Missing solutions remain failures."}


def main():
    result = assess()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "comparison.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cases = result["external_manifest"]["cases"]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.8), constrained_layout=True, sharey=True)
    for ax, case in zip(axes, cases):
        rows = [r for r in result["results"] if r["case"] == case["id"]]
        x = [r["slope_deg"] for r in rows]
        ax.plot(x, [100*r["measured_outer_reference_slip"] for r in rows], "o-", color="#217b8c", label="Measured, common radius reference")
        ax.plot(x, [100*r["bwr"]["slip"] if r["bwr"]["slip"] is not None else np.nan for r in rows], "s-", color="#b85329", label="Rigid BWR, independent soil data")
        for r in rows:
            if r["bwr"]["slip"] is None:
                ax.plot(r["slope_deg"], 101, "v", color="#b85329")
        failed = sum(r["bwr"]["slip"] is None for r in rows)
        if failed:
            ax.text(.45,.22,f"{failed} observation(s): no solution\nat slip < 1 (triangles above)",
                    transform=ax.transAxes,fontsize=7,color="#993c1c")
        ax.set(title=f"{case['label']}\n{case['mass_kg']:g} kg; {case['wheel_count']} wheels", xlabel="Uphill slope (degrees)", ylim=(-7,110))
        ax.grid(alpha=.2)
        ax.legend(loc="upper left", fontsize=7)
    axes[0].set_ylabel("Slip (%)")
    fig.suptitle("Unfitted rigid-wheel BWR benchmark - GRC-1 coefficients transferred from independent soil tests")
    fig.savefig(OUTPUT / "comparison.png", dpi=170)
    plt.close(fig)

    # Show prescribed sensitivity cases separately; do not disguise them as
    # experimental scatter or use best-agreeing settings to replace the preset.
    fig, axes = plt.subplots(1,2,figsize=(10,4.5),constrained_layout=True,sharey=True)
    for ax, caseid in zip(axes, ("husky_stock","husky_grouser")):
        rows = [r for r in result["results"] if r["case"] == caseid]
        x = [r["slope_deg"] for r in rows]
        ax.plot(x,[100*r["measured_outer_reference_slip"] for r in rows],"ko-",label="Measured, common reference")
        ax.plot(x,[100*r["bwr"]["slip"] if r["bwr"]["slip"] is not None else np.nan for r in rows],"s-",label="Preset")
        for name in rows[0]["sensitivities"]:
            ax.plot(x,[100*r["sensitivities"][name]["slip"] if r["sensitivities"][name]["slip"] is not None else np.nan for r in rows],"x--",label=name.replace('_',' '))
        ax.set(title=rows[0]["label"],xlabel="Slope (degrees)",ylim=(0,105))
        ax.grid(alpha=.2)
        ax.legend(fontsize=7)
    axes[0].set_ylabel("Slip (%)")
    fig.suptitle("Sensitivity only: published K +/- 1 SD and an alternative generic stress shape; no refit")
    fig.savefig(OUTPUT / "sensitivity.png",dpi=170)
    plt.close(fig)
    tables = []
    for case in cases:
        rows = [r for r in result["results"] if r["case"] == case["id"]]
        body = ""
        for r in rows:
            p = r["bwr"]
            if p["slip"] is None:
                predicted, error, sinkage = "FAILED: no solution", "--", "--"
                notes = p["reason"]
            else:
                predicted, error, sinkage = f"{100*p['slip']:.1f}%", f"{r['bwr_error_pp']:+.1f}", f"{1000*p['sinkage_m']:.1f}"
                notes = "; ".join(p["soil_fit_transfer_flags"]) or "No plotted-range exceedance; soil/geometry transfer still provisional"
            body += f"<tr><td>{r['slope_deg']:.1f}</td><td>{100*r['measured_slip']:.1f}%</td><td>{100*r['measured_outer_reference_slip']:.1f}%</td><td>{predicted}</td><td>{error}</td><td>{sinkage}</td><td>{html.escape(notes)}</td></tr>"
        summary = result["summary"][case["id"]]
        tables.append(f"<h2>{html.escape(case['label'])}</h2><p>{html.escape(summary['scope'])}. Solved {summary['solved']}/{summary['total_observations']}; failed {summary['failed']}. MAE on solved points only: {summary['mae_pp_on_solved_points_only']:.1f} percentage points. Gate: {'PASS' if summary['gate_passed'] else 'FAIL'}{' (secondary case, not a GRC-1 validation gate)' if not summary['gate_applicable'] else ''}.</p><table><tr><th>Slope</th><th>Published slip</th><th>Common radius reference</th><th>BWR slip</th><th>Error, pp</th><th>Sinkage, mm</th><th>Transfer flags</th></tr>{body}</table>")
    sources = "".join(f"<li><a href='{html.escape(s['url'])}'>{html.escape(s['title'])}</a></li>" for s in result["soil_manifest"]["sources"])
    passed = result["benchmark_passed"]
    doc = f"""<!doctype html><meta charset="utf-8"><title>Rigid-wheel BWR benchmark</title>
<style>body{{font:16px system-ui;line-height:1.5;max-width:1250px;margin:35px auto;padding:0 20px;color:#172735}}h1,h2{{line-height:1.2}}img{{width:100%}}table{{width:100%;border-collapse:collapse;font-size:14px}}td,th{{padding:9px;text-align:left;border-bottom:1px solid #d4dce2}}.notice{{background:#fff0d4;border-left:5px solid #bd5c2f;padding:18px}}li{{margin:9px 0}}</style>
<h1>Rigid-wheel Bekker-Wong-Reece benchmark</h1>
<p class="notice"><strong>Benchmark gate: {'PASS' if passed else 'FAIL'}.</strong> The model is implemented and numerically checked. No parameters were fitted to these rover observations. The student selected BWR as the default conservative design model. Empty/full climb constraints are 25/20 degrees with solved slip below 1; the 30-degree static tipping check remains separate.</p>
<p>Bekker pressure-sinkage, continuous Wong-Reece front/rear normal stress, Janosi-Hanamoto shear, and integration of normal force, thrust, compaction resistance and torque. Entry angle is solved from normal equilibrium; slip is solved from net force demand. The wheel is a rigid smooth cylinder. No tire compliance, explicit grousers, bulldozing, rate law, rear-wheel multipass or wheel spin inertia is included. Rigidity is an assumption, not a demonstrated conservative bound.</p>
<img src="comparison.png" alt="Independent BWR predictions compared to measured larger-rover slip">
<h2>Independent soil coefficients and predeclared gate</h2>
<p>Oravec (2009), Tables 5.4 and 5.5, GRC-1 at 1640 kg/m3 and 14% relative density: n=0.95, kc=221260 N/m^(n+1), kphi=1957510 N/m^(n+2), c=10 Pa, phi=33.3 degrees, K=25.5 mm. Generic loose-sand stress-peak coefficients c1=0.18 and c2=0.32 are from NASA/TM-20250006958, Table 2; they are not measured GRC-1 coefficients. Exit angle zero is an explicit no-rebound assumption. No use is made of negative cohesion fits in other density rows.</p>
<p>The Husky tests used GRC-1 at 28-31% relative density, so this is an imperfect preparation transfer. Numerical existence inside s<1 and z/R<=0.95 is separate from source-data validity. Soil-fit pressure/sinkage extrapolations are listed per point. The source ranges are plate pressures up to 50 kPa and sinkage up to 10 mm, and shear tests at 5-30 kPa. They do not validate a wheel within those bounds.</p>
<p>Before the first BWR benchmark, the versioned manifest specified group MAE <=10 percentage points, each error <=20 points, and a valid solution at every observed Husky point. Failed solutions are not dropped to claim a pass. The six-wheel natural-dune example is secondary and excluded from that GRC-1 gate. Three means from one wheel configuration are not independent campaigns.</p>
{''.join(tables)}
<h2>Radius conventions and geometry assumptions</h2>
<p>Stock Husky is assumed 330 mm diameter / 125 mm width from Fiset Table 3.2; Demishkevich does not repeat absolute stock dimensions. Rigid Husky uses the reported 384 mm grouser-tip diameter and 64 mm width as a smooth-cylinder proxy. Its commanded speed is consistent with 343.04 mm nominal diameter. The conditional conversion to a common 384 mm slip reference is s_tip=1-(1-s_reported)*343.04/384; both values are retained. Alternate 343.04 mm cylinder predictions, converted onto that same reference, are included in the JSON. No grouser benefit is fabricated.</p>
<p>SSTB-Lite has six wheels; only its rear-left wheel slip is digitized, from ten resolved measured diamond markers in Zhou Figure 13. Its natural dune sand is not GRC-1. Digitization and geometry provenance are retained in the external manifest.</p>
<h2>Parameter sensitivity, not fitting</h2><img src="sensitivity.png" alt="Slip sensitivity to published shear modulus variation and generic stress-peak shape">
<p>K=16.3 and 34.7 mm are the published mean plus/minus one reported standard deviation. Missing markers indicate failed operating-point solutions, not missing measurements. The alternative c1=0.43,c2=0.32 is the NASA generic compact-sand row, not a claim that this test soil was compact. Every scenario and failure is saved; none replaces the preset based on agreement. These curves are not confidence bounds.</p>
<h2>Sources and reproducibility</h2><ul>{sources}</ul>
<p><a href="comparison.json">Full benchmark, failures, provenance and sensitivities</a>. Reproduce with <code>.\\.venv\\Scripts\\python.exe -m studies.bekker_wong_reece</code>. For the current saved rover, run <code>python -m examples.bekker_wong_reece --check-derivatives</code>. BWR is now the default straight-driving model; point-turn physics is unchanged.</p>"""
    (OUTPUT / "index.html").write_text(doc,encoding="utf-8")
    print(json.dumps({"benchmark_passed":passed, "summary":result["summary"]},indent=2))
    print(OUTPUT / "index.html")


if __name__ == "__main__":
    main()
