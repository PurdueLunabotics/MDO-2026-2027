"""Presentation only: a fixed-design review and a separate optimization study.

All values and pass/fail decisions come from the saved snapshot. No physics or
feasibility is recalculated here. Display conversions never alter the JSON.
"""
from html import escape
import json
import math

from rover.config import REQUIREMENTS


LABELS = {
    "wheel_diameter": "Wheel diameter", "wheel_width": "Wheel width",
    "wheelbase": "Wheelbase", "track_width": "Wheel-center track",
    "drum_diameter": "Drum outer diameter", "drum_length": "Width of each drum",
    "cut_depth": "Scoop protrusion / cut depth", "drum_rpm": "Excavation speed",
    "dump_rpm": "Dumping speed", "drum_gear_ratio": "Drum reduction ratio",
    "drive_gear_ratio": "Drive reduction ratio", "arm_length": "Arm length",
    "travel_actuator_length": "Travel actuator length", "actuator_retracted_length": "Compact actuator length",
    "actuator_extended_length": "Digging actuator length", "actuator_force_rating": "Actuator rating, each",
    "actuator_speed": "Actuator speed", "battery_capacity": "Battery capacity",
    "top_pivot_height": "Actuator pivot height", "bottom_pivot_height": "Arm pivot height",
    "pivot_x": "Arm pivot longitudinal position", "actuator_pivot_x": "Actuator pivot longitudinal position",
    "rocker_pivot_offset": "Rocker pivot above wheel centers", "cf_tube_span": "CF tube span / plate spacing",
    "cf_tube_wall": "CF wall thickness", "plate_spoke_width": "Plate strip width",
    "plate_thickness": "Plate sheet thickness", "wheel_rocker_gap": "Wheel-to-rocker gap",
    "dry_mass": "Dry rover mass", "rover_length": "Compact overall length",
    "rover_width": "Overall width", "rover_height": "Compact overall height",
    "slope_tip_margin": "30° slope: static tip reserve",
    "rock_sweep_tip_margin": "Rock path: global tip reserve",
    "rock_sweep_wheel_load_margin": "Rock path: smallest wheel reaction",
    "obstacle_pose_margin": "Rock pose: geometric reach reserve",
    "obstacle_tip_margin": "Rock endpoint: global tip reserve",
    "obstacle_drum_wheel_margin": "Obstacle drum–wheel clearance",
    "structure_stress_margin": "Structural stress envelope",
    "structure_deflection_margin": "Structural deflection envelope",
    "structure_buckling_margin": "Structural buckling reserve",
    "plate_stress_margin": "Plate strip stress", "plate_buckling_margin": "Plate strip buckling reserve",
    "cf_bending_margin": "CF tube bending stress", "frame_packing_margin": "Frame packaging reserve",
    "plate_geometry_margin": "Plate geometry reserve", "actuator_force_margin": "Actuator force capacity",
    "straight_motor_margin": "Straight travel: motor-force reserve",
    "turn_motor_margin": "Turning: motor-force reserve",
    "straight_traction_margin": "Straight travel: traction reserve",
    "turn_traction_margin": "Turning: traction reserve",
    "straight_power_margin": "Straight travel: shaft-power reserve",
    "turn_power_margin": "Turning: shaft-power reserve",
    "battery_energy_margin": "Battery usable energy", "battery_power_margin": "Battery peak-power capacity",
    "drum_torque_margin": "Excavation torque reserve", "dump_torque_margin": "Dumping torque reserve",
    "drum_power_margin": "Excavation shaft-power reserve", "ground_pressure_margin": "Ground contact pressure",
    "mobility_stability_margin": "Level-route dynamic tip reserve", "tip_stability_margin": "Asymmetric dump tip reserve",
    "dig_traction_margin": "Excavation: traction reserve", "penetration_margin": "Excavation: penetration reserve",
    "site_drive_margin": "Fresh-site move: drive/traction reserve", "trench_traverse_margin": "Excavated trench diameter reserve",
    "wheel_first_margin": "Travel drum height above wheel-diameter step",
    "complete_cycle_margin": "Time remaining after one complete cycle", "payload_margin": "Minimum payload reserve",
    "excavation_area_margin": "Available excavation area", "dig_reach_margin": "Minimum digging-depth reserve",
}

UNASSESSED = [
    ("Slope motor limits / sideways sliding", "The separate BWR climb checks accept slip<1 only. Motor capability on slopes, lateral sliding and controller behavior are not evaluated."),
    ("Real 40 cm rock traversal", "Static poses and clearance screens do not establish the driving forces or actual terrain-contact sequence."),
    ("50 cm ditch crossing", "Ditch width/radius and contact sequence have not been defined."),
    ("Continuous motor operation", "Stator-current limits and thermal duty remain undecided."),
    ("Connections & material validation", "Bonded inserts, bushings, bolts, local failure and selected CF tube properties need separate checks."),
    ("Soil and skid-steer predictions", "Rolling, traction, capture and scrub relationships remain uncalibrated."),
]


def label(name):
    return LABELS.get(name, name.replace("_", " ").capitalize())


def quantity(value, units=None, name=""):
    """Friendly display units; native values are retained in engineering data."""
    if value is None:
        return "Not reported"
    if units == "m":
        value, units = value*1000, "mm"
    elif units == "m**3":
        value, units = value*1000, "L"
    elif units == "rad":
        value, units = math.degrees(value), "°"
    elif units == "Pa":
        scale, units = (1000, "kPa") if "pressure" in name else (1e6, "MPa")
        value /= scale
    units = {"W*h": "Wh", "N*m": "N·m", "m**2": "m²", "kg*m**2": "kg·m²"}.get(units, units)
    digits = 0 if abs(value) >= 1000 else 1 if abs(value) >= 10 else 2
    number = f"{value:,.{digits}f}"
    if value and abs(value) < .01:
        number = f"{value:.3g}"
    return number+(f" {units}" if units else "")


def table(headers, rows, cls=""):
    return f'<div class="table-wrap"><table class="{cls}"><thead><tr>'+''.join(f'<th>{escape(h)}</th>' for h in headers)+'</tr></thead><tbody>'+''.join(rows)+'</tbody></table></div>'


def row(cells, attrs=""):
    return f'<tr {attrs}>'+''.join(f'<td>{c}</td>' for c in cells)+'</tr>'


def badge(status):
    return f'<span class="status {status.lower().replace(" ", "-")}">{escape(status)}</span>'


def render_report(record, output, drawings, frame_drawing, stability_drawing, route_drawing, frames, top_drawing="", download_links=None, obstacle_drawing=""):
    final, start = record["final"], record["baseline"]
    metadata = record.get("model_metadata", final.get("model_metadata", {}))
    context = record.get("report_context", {})
    fixed = record.get("status") == "baseline"
    inputs = {**final["design"], **final["assumptions"]}
    outputs = final["outputs"]
    values = {n: i["value"] for group in (inputs, outputs) for n, i in group.items()}
    val = lambda n, default=0: values.get(n, default)
    q = lambda n: escape(quantity(values.get(n), (outputs.get(n) or inputs.get(n) or {}).get("units"), n))
    prefix = escape(output.stem, quote=True)
    name = context.get("name", "Configured design — example" if fixed else "Rover optimization study")
    mode = "Design baseline evaluation" if fixed else "Optimization study"
    checks = final["constraints"]
    failed = {n: c for n, c in checks.items() if not c["passed"]}
    physical = {n: c for n, c in checks.items() if not n.endswith("_bounds")}
    failures = len(failed)
    revision = context.get("revision", "Not specified")
    default_names = set(inputs)-set(context.get("input_file_names", []))-set(context.get("cli_override_names", []))
    source = context.get("source", "Saved model inputs")
    example = source == "Configured defaults" or not context
    title_status = f"{failures} modeled checks need attention" if failures else "All modeled checks pass"
    overall = "attention" if failures else "clear"
    if not fixed and not record.get("optimizer_success"):
        title_status, overall = "Optimizer did not converge successfully", "attention"
    conditional = "Predictions are conditional on the failed checks below." if failed else "Passing modeled checks does not establish hardware performance."
    guarded = (val("obstacle_pose_valid", 1) < .5 or val("rock_sweep_pose_valid", 1) < .5 or
               val("frame_load_solution_valid", 1) < .5)
    guard_note = '<p class="callout">Some geometry or load calculations use a guarded trial. Obstacle diagnostics at a reduced height do not verify the requested height; invalid frame loads are not design loads.</p>' if guarded else ""

    # Human-readable actual/limit pairs where the model provides both. Remaining
    # checks are explicitly reserves, rather than invented applied loads.
    measured = {n: (val(n), u, upper, u) for n, (upper, u) in REQUIREMENTS.items()}
    for check, actual, limit in (
        ("structure_stress_margin", "structure_stress", val("tube_yield_strength")/val("structure_safety_factor", 1)),
        ("structure_deflection_margin", "structure_deflection", val("allowable_deflection")),
        ("plate_stress_margin", "plate_stress", val("tube_yield_strength")/val("structure_safety_factor", 1)),
        ("cf_bending_margin", "cf_bending_stress", val("cf_flexural_strength")/val("structure_safety_factor", 1)),
        ("ground_pressure_margin", "ground_pressure", val("allowable_ground_pressure")),
        ("actuator_force_margin", "required_actuator_force", val("actuator_force_rating")/val("actuator_safety_factor", 1)),
        ("obstacle_drum_wheel_margin", "obstacle_drum_wheel_gap_bound", val("mechanism_clearance")),
        ("battery_energy_margin", "run_energy", val("battery_capacity")*val("battery_usable_fraction")),
        ("battery_power_margin", "peak_power", val("battery_capacity")*val("battery_c_rate")),
    ):
        if actual in outputs:
            measured[check] = (val(actual), outputs[actual]["units"], limit, outputs[actual]["units"])

    check_rows = []
    for check, item in sorted(checks.items(), key=lambda entry: entry[1]["passed"]):
        bounds = check.endswith("_bounds")
        actual = "Reserve: "+escape(quantity(item["margin"], item["units"], check))
        limit = "Reserve ≥ 0"
        if check in measured:
            a, au, b, bu = measured[check]
            actual = escape(quantity(a, au, check))
            op = "≥" if check == "obstacle_drum_wheel_margin" else "≤"
            limit = op+" "+escape(quantity(b, bu, check))
        if bounds:
            variable = final["design"][check[:-7]]
            actual = escape(quantity(variable["value"], variable["units"], check))
            limit = escape(quantity(variable["lower"], variable["units"], check))+" to "+escape(quantity(variable["upper"], variable["units"], check))
        display_name = label(check[:-7])+" search range" if bounds else label(check)
        if check in ("empty_slope_traversal_margin", "loaded_slope_traversal_margin"):
            state = check.split("_")[0]
            display_name = f'{"Empty" if state == "empty" else "Full"} rover: {val(state+"_slope_degrees"):g}° BWR climb'
            actual = (f'{val(state+"_slope_slip"):.6g}' if val(state+"_slope_solution_valid") == 1 else
                      'No climbing solution<small>Slip = 1 is a failure sentinel</small>')
            limit = "Slip &lt; 1 (strict)"
        status = "Pass" if item["passed"] else "Fail"
        check_rows.append(row([f'<strong>{escape(display_name)}</strong><small>{escape(check)}</small>', actual, limit,
                               escape(quantity(item["margin"], item["units"], check)), badge(status)],
                              f'data-port="{escape(check, quote=True)}" data-check="{status.lower()}" data-kind="{"bounds" if bounds else "physical"}"'))
    for title, note in UNASSESSED:
        check_rows.append(row([f'<strong>{escape(title)}</strong>', f'<span class="muted">{escape(note)}</span>', "—", "—", badge("Not assessed")], 'data-check="not-assessed" data-kind="unknown"'))

    def metric(title, value, note):
        return f'<article class="metric"><span>{escape(title)}</span><strong>{value}</strong><small>{escape(note)}</small></article>'

    whole = final["whole_cycles"]
    metrics = ''.join([
        metric("Dry rover mass", q("dry_mass"), "Before adding regolith"),
        metric("Payload per cycle", q("payload_mass"), "Total carried by four drums"),
        metric("Complete cycles", str(whole["completed"]), f'Within {val("run_time")/60:g} minutes; no partial delivery credit'),
        metric("Delivered volume", escape(quantity(whole["berm_volume_m3"], "m**3")), "Complete-cycle mission estimate"),
    ])
    score_metrics = ''.join([
        metric("Construction points", f'{val("construction_points"):,.2f}', "Weighted mass + energy points"),
        metric("Mass points", f'{val("mass_points"):,.2f}', "Dry-mass productivity component"),
        metric("Energy points", f'{val("energy_points"):,.2f}', "Consumed-energy productivity component"),
    ])
    performance_rows = [
        row(["Full cycle duration", q("cycle_time"), "Includes digging, actuation, travel, dumping and relocation; overlap removed once."]),
        row(["Energy per cycle", q("cycle_energy"), "Electrical energy including auxiliaries."]),
        row(["Mission energy · complete cycles", escape(quantity(whole["run_energy_Wh"], "W*h")), "Includes idle electronics for remaining mission time."]),
        row(["Mission energy · continuous estimate", q("run_energy"), "Used by the existing battery constraint and MDO objective."]),
        row(["Peak electrical power", q("peak_power"), "Largest modeled simultaneous demand; not an average power rating."]),
        row(["Loaded mass", escape(quantity(val("dry_mass")+val("payload_mass"), "kg")), "Dry mass + all four drums' regolith payload."]),
        row(["Actual route peak speed", q("actual_travel_speed"), "Prescribed motion, not measured vehicle performance."]),
        row(["Score · complete-cycle estimate", f'{whole["construction_points"]:,.2f} points', "Uses complete deliveries and idle remainder; separate from the continuous MDO objective."]),
    ]
    for state in ("empty", "loaded"):
        if state+"_slope_solution_valid" in outputs:
            solved = val(state+"_slope_solution_valid") == 1
            performance_rows.append(row([
                f'{"Empty" if state == "empty" else "Full"} rover · {val(state+"_slope_degrees"):g}° climb',
                f'Slip {val(state+"_slope_slip"):.4f}' if solved else 'No climbing solution',
                f'BWR, GRC-1, equal mean wheel loads. Force shortfall per wheel: {val(state+"_slope_force_shortfall"):.2f} N. '
                f'Soil-fit range flags: {val(state+"_slope_transfer_flag_count"):g}. Acceptance: solved slip strictly below 1.']))
    mass_names = [("fixed_mass", "Equipment allowance"), ("structure_mass", "Frame"), ("wheel_mass", "Four bare wheels"),
                  ("drive_mass", "Four drive assemblies"), ("drum_mass", "Four drums + two drive assemblies"),
                  ("battery_mass", "Battery"), ("linkage_mass", "Arms + actuators")]
    def bars(items, total, units):
        lines = []
        for n, title, amount in items:
            percent = max(0, min(100, amount/max(total, 1e-12)*100))
            lines.append(f'<div class="bar-row"><div><span>{escape(title)}</span><b>{escape(quantity(amount, units, n))}</b></div><div class="bar-track"><i style="width:{percent:.3f}%"></i></div></div>')
        return ''.join(lines)
    mass_bars = bars([(n, title, val(n)) for n, title in mass_names], val("dry_mass"), "kg")
    energy_items = [(n, title, val(n)/3600) for n, title in [("dig_energy", "Digging"), ("dump_energy", "Dumping"),
                    ("mobility_energy", "Travel + deposition turn"), ("arm_energy", "Actuators"), ("relocation_energy", "Fresh-site move")]]
    energy_items.append(("electronics", "Electronics", val("electronics_power")*val("cycle_time")/3600))
    energy_bars = bars(energy_items, val("cycle_energy"), "W*h")

    load_specs = [
        ("required_actuator_force", "Required force per actuator", "Loaded raising/extraction envelope, including the model's breakout allowance."),
        ("frame_gravity_actuator_force", "Frame gravity reaction at each actuator", "Mirrored loaded gravity sweep; includes the structural load factor. Separate from actuator sizing load."),
        ("frame_nominal_wheel_reaction", "Nominal force per wheel", "Equal-share level-ground weight. Not a rock or slope wheel-load envelope."),
        ("cut_force", "Total cutting force", "Across four drums at the selected bite and engagement."),
        ("ground_pressure", "Worst modeled contact pressure", "Route pressure estimate, including load transfer and rocker factor."),
        ("structure_stress", "Structural stress envelope", "Envelope of local member stresses; not one assembled-frame FEA result."),
        ("structure_deflection", "Structural deflection envelope", "Conservative combination of local/relative deflections."),
        ("frame_mount_relative_deflection", "Upper/lower mount relative displacement", "Frame gravity sweep."),
        ("rocker_deflection", "Rocker-end flexure", "Local beam-screen displacement."),
        ("axle_deflection", "Suspension-axle flexure", "Axle-overhang screen."),
        ("arm_deflection", "Arm-tip flexure", "Relative arm beam displacement."),
        ("loaded_cg_z", "Loaded CG height", "Fully loaded travel pose, above level ground."),
    ]
    load_rows = [row([f'{escape(title)}<small>{escape(n)}</small>', q(n), escape(note)]) for n, title, note in load_specs if n in outputs]
    frame_valid = val("frame_load_solution_valid", 1) > .5
    load_note = ("Frame gravity solution is valid for its declared load case." if frame_valid else
                 "Frame gravity solution is invalid: structural trial diagnostics must not be used as design loads.")
    if "frame_load_solution_valid" not in values:
        load_note = "Reference beam screens; assembled-frame connection equilibrium is not resolved."

    raw_inputs = table(["Input port", "Effective value", "Native units"], [row([escape(n), f'{i["value"]:.8g}', escape(i["units"] or "—")]) for n, i in inputs.items()])
    raw_outputs = table(["Output port", "Value", "Native units"], [row([escape(n), f'{i["value"]:.8g}', escape(i["units"] or "—")]) for n, i in outputs.items()])
    bounds_rows = [row([escape(n), f'{i["lower"]:.6g}', f'{i["value"]:.6g}', f'{i["upper"]:.6g}', escape(i["units"] or "—")]) for n, i in final["design"].items()]
    diagnostics = metadata.get("diagnostic_only_screens", {})
    diagnostic_list = ''.join(f'<li><code>{escape(n)}</code>: {escape(reason)}</li>' for n, reason in diagnostics.items())
    motor_note = ""
    if metadata.get("motor_profile") == "x44-standard":
        motor_note = f'<details><summary>Standard X44 motors · assumptions</summary><p>Four wheel motors at 100:1; two drum motors at {val("drum_gear_ratio"):.2f}:1 (exploratory gearing).</p><p>Electrical energy = output-shaft mechanical work / combined efficiency. Drive: {val("drive_efficiency"):.0%}; drums: {val("drum_efficiency"):.0%}. Both efficiencies are provisional.</p><p>Continuous operation unverified. Current-limit inputs of zero mean undecided. Shaft torque-speed capability is enforced separately from energy accounting.</p></details>'
    obstacle_note = "Guarded pose: requested height is unreachable. Endpoint and path diagnostics are evaluated at a guarded height, not the requested rock height." if val("obstacle_pose_valid", 1) < .5 else "Requested endpoint rock pose is geometrically reachable; this alone does not establish traversal."
    clearance_note = "Guarded clearance: requested pose is unreachable. Clearance diagnostics are not predictions at the requested height." if val("obstacle_clearance_pose_valid", 1) < .5 else "Requested-height obstacle clearance screen."
    if metadata.get('models', {}).get('obstacle_clearance', '').endswith('ParallelCylinderClearance'):
        clearance_note += ' Parallel finite cylinders: shared roll is removed, then circular-plane and axial-width gaps are combined. Negative gap means overlap of the ideal solids.'
    else:
        clearance_note += ' This saved record uses an older or custom clearance model; consult its recorded model evidence and regenerate it with the current default.'
    clearance_note += ' A common roll rotation preserves relative clearance in both models; rocker articulation still changes relative geometry.'

    comparison = ""
    if not fixed:
        changed = []
        for n, i in final["design"].items():
            before = start["design"][n]["value"]
            if abs(i["value"]-before) > 1e-9*max(1, abs(before)):
                changed.append(row([escape(label(n)), escape(quantity(before, i["units"], n)), q(n)]))
        comparison = f'<section id="mdo"><div class="section-heading"><span>06 / MDO only</span><h2>Optimization study</h2></div><article class="panel"><p>The candidate is separate from a released design baseline. Optimizer status: <strong>{escape(str(record.get("optimizer_success")))}</strong>. Saved status: {escape(record.get("status", "not recorded"))}.</p><p>Continuous-score objective: {start["outputs"]["construction_points"]["value"]:,.2f} → {outputs["construction_points"]["value"]:,.2f} points. Complete-cycle estimate: {whole["construction_points"]:,.2f} points.</p>'+table(["Changed input", "Starting value", "Candidate value"], changed)+'</article>'
        comparison += f'<details class="panel screen-only"><summary>Recorded optimizer evaluations</summary><p>Actual driver evaluations, including failed trials. Finite-difference perturbations are excluded.</p><input id="iteration" aria-label="Recorded evaluation" type="range" min="0" max="{max(0, len(frames)-1)}" value="{max(0, len(frames)-1)}"><p id="iteration-label"></p><div class="two-col"><div id="history-retracted"></div><div id="history-extended"></div></div></details></section>'

    nav = [("overview", "Overview"), ("geometry", "Rover views"), ("performance", "Performance"),
           ("requirements", "Requirements"), ("loads", "Loads"), ("notes", "Model notes")]
    if not fixed:
        nav.append(("mdo", "Optimization study"))
    nav.append(("engineering-data", "Inputs & outputs"))
    nav_html = ''.join(f'<a href="#{target}">{title}</a>' for target, title in nav)
    frames_json = json.dumps(frames, ensure_ascii=True).replace("<", "\\u003c")
    warning = '<span class="tag">Example · no SSOT baseline supplied</span>' if example else '<span class="tag">Named input case · '+escape(source)+'</span>'
    mobility_evidence = metadata.get("evidence", {}).get("mobility", {})
    mobility_notice = ('<article class="panel"><strong>'+escape(mobility_evidence["notice"])+
                       '</strong></article>') if mobility_evidence.get("exploratory") and mobility_evidence.get("notice") else ''
    input_hint = f'{len(context.get("input_file_names", []))} inputs supplied by file; {len(context.get("cli_override_names", []))} CLI overrides; {len(default_names)} remaining inputs use model defaults.' if context else 'Effective values are captured below. Original input source was not recorded.'
    if not fixed:
        input_hint += ' Optimizer-selected design values appear in the bounds table.'
    extra_geometry = ''
    if obstacle_drawing:
        extra_geometry += '<article class="panel geometry-extra">'+obstacle_drawing+f'<p class="screen-only"><a href="{prefix}_obstacle.svg">Wheel-on-rock SVG</a></p></article>'
    if frame_drawing:
        extra_geometry += '<details class="panel geometry-extra"><summary>Plate-frame member schematic</summary>'+frame_drawing+'</details>'
    if stability_drawing:
        extra_geometry += '<details class="panel geometry-extra"><summary>Static slope and articulation curves</summary>'+stability_drawing+'</details>'
    source_hash = context.get("source_sha256") or "No source file; evaluated inputs are listed below."
    links = {"analysis": output.name, "inputs": None,
             "n2": output.stem+"_n2.html" if output.with_name(output.stem+"_n2.html").exists() else None}
    if download_links is not None:
        links.update(download_links)
    downloads = ''.join(f'<a class="button secondary" href="{escape(links[key], quote=True)}">{caption}</a>'
                        for key, caption in (("analysis", "Analysis JSON"), ("inputs", "Design inputs JSON"),
                                             ("n2", "Model connections / N2")) if links[key])
    analysis_json = json.dumps(record, ensure_ascii=True, allow_nan=False).replace("<", "\\u003c")
    html = f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>{escape(name)} · rover design review</title><style>{CSS}{PRINT_CSS}</style></head><body>
<aside class="sidebar"><div class="brand">LUNABOTICS<span>Engineering design report</span></div><nav aria-label="Report sections">{nav_html}</nav><div class="sidebar-foot">Fixed inputs → model predictions<br>Constraints remain visible<br><b>Uncalibrated conceptual model</b></div></aside>
<main><header><div class="eyebrow">{mode}</div><h1>{escape(name)}</h1><p class="subtitle">Revision {escape(revision)} · {escape(metadata.get("structure_profile", "reference"))} frame · {escape(metadata.get("motor_profile", "reference"))} motors</p><div class="header-bottom">{warning}<span class="muted">{escape(context.get("generated_utc", "Saved analysis"))}</span></div></header>{mobility_notice}
<section id="overview"><div class="verdict {overall}"><div><strong>{title_status}</strong><p>{len(physical)-sum(not c["passed"] for c in physical.values())} / {len(physical)} physical model checks pass · {len(UNASSESSED)} capabilities not assessed</p></div><a class="button" href="#requirements">Review checks ↓</a></div><p class="muted">{conditional} These are predictions from the stated inputs, not measured performance.</p>{guard_note}<div class="metrics">{metrics}</div><div class="score-summary"><h2>Construction score</h2><p class="muted">Continuous mission estimate using the configured mass and energy weights. The complete-cycle estimate is reported in Performance &amp; mass.</p><div class="metrics score-metrics">{score_metrics}</div></div></section>
<section id="geometry"><div class="section-heading"><span>01 / Rover configuration</span><h2>Geometry at the evaluated inputs</h2><p>Level-ground schematics and one prescribed obstacle pose. Check statuses below determine whether envelopes and clearance requirements are satisfied.</p></div><article class="panel geometry-top">{top_drawing}<p class="muted">Travel-pose plan view. Four wheels and four drums are shown at their model stations. <a href="{prefix}_top.svg">Top-down SVG</a></p></article><article class="panel geometry-sides"><div class="pose-controls" role="group" aria-label="Linkage pose"><button data-pose="travel" aria-pressed="true">Travel</button><button data-pose="retracted" aria-pressed="false">Compact</button><button data-pose="extended" aria-pressed="false">Digging</button></div><div id="pose-travel" class="pose">{drawings[1]}</div><div id="pose-retracted" class="pose" hidden>{drawings[0]}</div><div id="pose-extended" class="pose" hidden>{drawings[2]}</div><p class="muted">Blue: arm · orange: actuator. Each visible drum represents a shaft-linked pair. Drawings are not CAD.</p><p class="screen-only"><a href="{prefix}_travel.svg">Travel SVG</a> · <a href="{prefix}_retracted.svg">Compact SVG</a> · <a href="{prefix}_extended.svg">Digging SVG</a></p></article>{extra_geometry}</section>
<section id="performance"><div class="section-heading"><span>02 / Predicted output</span><h2>Performance & mass</h2><p>Complete-cycle estimates and continuous MDO estimates are labeled separately.</p></div><article class="panel">{table(["Prediction", "Value", "Meaning"], performance_rows)}</article><div class="two-col"><article class="panel"><h3>Dry mass breakdown</h3><p class="muted">Each assembly is counted once.</p>{mass_bars}</article><article class="panel"><h3>Electrical energy per cycle</h3><p class="muted">Work / provisional efficiency, plus auxiliaries.</p>{energy_bars}</article></div><details class="panel"><summary>Route & cycle accounting</summary><div class="two-col">{route_drawing}<div><h3>{val("traversal_time_one_way"):g} s outbound + {val("traversal_time_one_way"):g} s return</h3><p>Dig and lower concurrently → raise loaded drums → outbound → dump one pair → turn → dump second pair → return → relocate to fresh ground.</p><p>Lowering overlap is subtracted once. The deposition turn and fresh-site relocation are additional to the timed route. Modeled full cycle: {q("cycle_time")}.</p></div></div></details></section>
<section id="requirements"><div class="section-heading"><span>03 / Gaps to resolve</span><h2>Requirements & model checks</h2><p>Failures are listed first. “Not assessed” means the model does not verify that capability.</p></div><article class="panel"><div class="toolbar"><label>Show <select id="check-filter"><option value="attention">Needs attention</option><option value="all">All checks & gaps</option><option value="fail">Failed checks</option><option value="pass">Passed checks</option><option value="not-assessed">Not assessed</option><option value="bounds">Optimizer search ranges</option></select></label><label class="search-label">Search checks <input id="check-search" type="search" placeholder="e.g. actuator, slope, clearance"></label><span id="check-count" aria-live="polite"></span></div>{table(["Check", "Prediction / reserve", "Limit / criterion", "Signed margin", "Status"], check_rows, "checks")}<p class="muted">Positive margins satisfy the modeled criterion. Status uses the saved model's numerical tolerance. Search ranges are hidden by default and available in the filter; they remain part of saved feasibility. Margins use their own units and cannot be ranked directly across unlike checks.</p><p id="empty-checks" hidden>No matching checks.</p></article></section>
<section id="loads"><div class="section-heading"><span>04 / Engineering loads</span><h2>Loads & structural response</h2><p>Values identify their load case and whether they apply per component or to the whole rover.</p></div><article class="panel"><p class="muted">{escape(load_note)} Structural load factor: {val("structure_load_factor"):g}; structural safety factor: {val("structure_safety_factor"):g}.</p>{table(["Quantity / scope", "Prediction", "Load case / interpretation"], load_rows)}</article></section>
<section id="notes"><div class="section-heading"><span>05 / Scope & traceability</span><h2>Model notes & engineering data</h2><p>Keep assumptions and validity limits attached to the predictions.</p></div><article class="panel"><details open><summary>Obstacle and structural interpretation</summary><p>{escape(obstacle_note)}</p><h3>Obstacle drum–wheel clearance</h3><p>{escape(clearance_note)}</p><p>Ditch requirement remains unverified. Static slope tipping assumes adequate friction; it does not verify climbing. The {val("allowable_deflection")*1000:g} mm structural limit applies to the modeled envelope of local deflections, not one assembled-frame displacement.</p><p>For the plate frame, CF tube stiffness/strength are PROVISIONAL. Connections, dynamic obstacle loads, cutting/recovery frame cases and whole-plate buckling need separate verification.</p></details>{motor_note}<details><summary>Removed screens: diagnostics only</summary><ul>{diagnostic_list or '<li>No removed screens recorded.</li>'}</ul></details><details class="screen-only"><summary>Baseline source & revision</summary><pre>{escape(json.dumps(context, indent=2))}</pre></details><details class="screen-only"><summary>Model provenance and evidence</summary><pre>{escape(json.dumps(metadata, indent=2))}</pre></details><div class="downloads">{downloads}<button class="print-button" onclick="window.print()">Print / save PDF</button></div></article></section>
{comparison}
<section id="engineering-data"><div class="section-heading"><span>Appendix / Native units</span><h2>Design inputs &amp; output data</h2><p>{escape(input_hint)} Values below retain their native model units.</p></div><details id="effective-inputs" class="panel raw-data" open><summary>All effective inputs &amp; model assumptions</summary><p>Includes mission, soil, material properties and inactive legacy inputs. Source SHA-256: <code>{escape(source_hash)}</code>.</p>{raw_inputs}</details><details id="design-bounds" class="panel raw-data" open><summary>Design variables and bounds</summary><p>Optimizer search ranges and evaluated design variables. These ranges guide MDO and input validation; they are not all hardware requirements.</p>{table(["Port", "Lower", "Evaluated", "Upper", "Native units"], bounds_rows)}</details><details id="output-ports" class="panel raw-data" open><summary>All output ports</summary>{raw_outputs}</details></section>
<footer>Engineering design report · Input values and predictions are separate from baseline approval. OpenMDAO {escape(record.get("openmdao_version", "not recorded"))}. Failed checks remain visible.</footer>
<script id="analysis-data" type="application/json">{analysis_json}</script><script id="iteration-data" type="application/json">{frames_json}</script><script>{JS}</script></main></body></html>'''
    return html


CSS = '''
:root{--ink:#20323e;--muted:#62717b;--line:#e2e7e9;--teal:#176a67;--bg:#f4f6f6}*{box-sizing:border-box}html{scroll-behavior:smooth;scroll-padding-top:24px}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.55 system-ui,-apple-system,sans-serif}.sidebar{position:fixed;inset:0 auto 0 0;width:236px;padding:34px 22px;background:#172d36;color:#dce9ed;display:flex;flex-direction:column}.brand{font-weight:800;font-size:19px;letter-spacing:.1em}.brand span{display:block;font-size:11px;font-weight:500;letter-spacing:.03em;color:#a9bfc6;margin-top:7px}.sidebar nav{display:grid;gap:7px;margin-top:42px}.sidebar nav a{padding:10px 12px;color:#c7d8dd;text-decoration:none;border-radius:7px}.sidebar nav a:hover,.sidebar nav a:focus{background:#29444d;color:white}.sidebar-foot{margin-top:auto;font-size:12px;line-height:1.9;color:#9fb6be}.sidebar-foot b{color:#d2e0e5;font-size:11px}main{margin-left:236px;max-width:1660px;padding:42px 48px}header{padding-bottom:28px;border-bottom:1px solid var(--line);margin-bottom:28px}.eyebrow,.section-heading>span{text-transform:uppercase;font-size:11px;letter-spacing:.14em;color:var(--teal);font-weight:800}h1{font-size:clamp(27px,3vw,39px);letter-spacing:-.035em;line-height:1.2;margin:12px 0}h2{font-size:25px;letter-spacing:-.025em;margin:7px 0}h3{font-size:16px;margin:0 0 12px}.subtitle,.muted{color:var(--muted)}p{margin:10px 0}.header-bottom{display:flex;align-items:center;justify-content:space-between;gap:12px;font-size:12px;margin-top:22px}.tag{background:#e7eeee;padding:5px 10px;border-radius:5px;font-weight:600}.verdict{border:1px solid #ecd2ac;background:#fff7e9;border-left:4px solid #bf791d;border-radius:9px;padding:19px 22px;display:flex;justify-content:space-between;align-items:center;gap:18px}.verdict strong{font-size:20px}.verdict p{font-size:13px;margin:4px 0 0;color:#6c604b}.verdict.clear{border-color:#bbd8cf;background:#edf7f2;border-left-color:#24715b}.button,button{font:inherit;cursor:pointer}.button{display:inline-block;text-decoration:none;border-radius:6px;background:var(--teal);color:white;padding:9px 14px;font-size:13px;white-space:nowrap;font-weight:650}.button.secondary{background:#eaf1f1;color:var(--teal)}a{color:var(--teal)}button:focus-visible,a:focus-visible,input:focus-visible,select:focus-visible,summary:focus-visible{outline:3px solid #4f9bb7;outline-offset:3px}.callout{background:#fff1e7;padding:12px 16px;border-left:3px solid #b76433;font-size:13px}.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:15px;margin-top:24px}.metric{background:white;border:1px solid var(--line);border-radius:9px;padding:20px}.metric>span{font-size:12px;color:var(--muted);font-weight:650}.metric strong{display:block;font-size:29px;letter-spacing:-.035em;margin:7px 0}.metric small{font-size:11px;color:var(--muted);display:block}section{margin:0 0 38px}.section-heading{margin:36px 0 18px}.section-heading p{font-size:14px;color:var(--muted);max-width:850px}.panel{background:white;border:1px solid var(--line);border-radius:9px;padding:22px;margin-bottom:16px;min-width:0}.input-grid,.two-col{display:grid;grid-template-columns:1fr 1fr;gap:17px}.input-grid>.panel{padding:19px 15px}.input-grid h3{padding-left:8px}.table-wrap{overflow-x:auto}table{width:100%;border-collapse:collapse;font-size:13px;font-variant-numeric:tabular-nums;text-align:left}thead{font-size:11px;color:var(--muted);background:#f5f8f8}th,td{padding:12px 11px;border-bottom:1px solid var(--line);vertical-align:top}th{font-weight:650}td:first-child{min-width:160px}td strong{font-weight:600}td small{display:block;font:10px/1.5 ui-monospace,monospace;color:#7a8890;margin-top:3px;overflow-wrap:anywhere}td:nth-child(2){white-space:nowrap}.input-grid td:first-child{min-width:0}.input-grid table{font-size:12px}.origin{font-size:10px;color:var(--muted);white-space:nowrap}.bar-row{margin:17px 0}.bar-row>div:first-child{display:flex;justify-content:space-between;gap:12px;font-size:12px}.bar-row b{font-weight:600;white-space:nowrap}.bar-track{height:6px;background:#edf2f2;border-radius:3px;margin-top:7px;overflow:hidden}.bar-track i{height:100%;display:block;background:#498b88;border-radius:3px}.status{display:inline-block;padding:4px 8px;font-size:11px;border-radius:5px;font-weight:750;white-space:nowrap}.pass{background:#e6f2ec;color:#24694c}.fail{background:#fbe9e4;color:#a13e25}.not-assessed{background:#fff3dc;color:#8a651f}.toolbar{display:flex;gap:16px;align-items:flex-end;flex-wrap:wrap;margin-bottom:18px}.toolbar label{font-size:11px;color:var(--muted);display:grid;gap:5px}.toolbar select,.toolbar input{font:13px system-ui;border:1px solid #cdd8dc;border-radius:5px;padding:9px 10px;background:white;color:var(--ink)}.search-label{flex:1;min-width:200px}#check-count{font-size:12px;color:var(--muted);padding:10px 0}.checks td:nth-child(2){white-space:normal;min-width:145px}.checks td:nth-child(3),.checks td:nth-child(4){min-width:110px}.checks tr[data-check=fail]{background:#fffaf7}.checks tr[data-check=not-assessed] td:nth-child(2){min-width:240px;font-size:12px}summary{cursor:pointer;font-weight:650;font-size:14px}details[open]>summary{margin-bottom:18px}details>p{font-size:13px;color:var(--muted)}svg{display:block;width:100%;height:auto}.pose-controls{display:flex;gap:5px;margin-bottom:18px;border-bottom:1px solid var(--line);padding-bottom:13px}.pose-controls button,.print-button{border:1px solid #cdd8dc;color:var(--ink);background:white;border-radius:5px;padding:7px 15px;font-size:12px}.pose-controls button[aria-pressed=true]{background:var(--teal);color:white;border-color:var(--teal)}[hidden]{display:none!important}pre{font:11px/1.6 ui-monospace,monospace;white-space:pre-wrap;overflow-wrap:anywhere;color:#4c606a}code{font-size:11px;overflow-wrap:anywhere}.downloads{display:flex;flex-wrap:wrap;gap:9px;margin-top:24px}footer{font-size:11px;color:var(--muted);padding:20px 0;border-top:1px solid var(--line)}input[type=range]{width:100%}#iteration-label{font-size:13px}.panel>.muted{font-size:12px}@media(min-width:1650px){main{padding-left:65px;padding-right:65px}}@media(max-width:1120px){.sidebar{width:195px;padding:28px 15px}main{margin-left:195px;padding:30px 24px}.metrics{grid-template-columns:repeat(2,1fr)}.input-grid{grid-template-columns:1fr}}@media(max-width:760px){.sidebar{position:static;width:auto;padding:18px 20px}.sidebar nav{display:flex;overflow-x:auto;gap:4px;margin-top:15px}.sidebar nav a{white-space:nowrap;font-size:12px;padding:7px 10px}.sidebar-foot{display:none}.brand{font-size:15px}.brand span{display:none}main{margin-left:0;padding:24px 15px}.two-col{grid-template-columns:1fr}.header-bottom{align-items:flex-start;flex-direction:column}.verdict{align-items:flex-start;flex-direction:column}.panel{padding:17px 12px}.metric{padding:16px}.metric strong{font-size:25px}.verdict strong{font-size:18px}}@media print{body{background:white}.sidebar,.toolbar,.pose-controls,.downloads,.verdict .button{display:none}main{margin:0;padding:0;max-width:none}section{margin-bottom:25px}.panel,.metric{break-inside:avoid;box-shadow:none}.metrics{grid-template-columns:repeat(4,1fr)}.input-grid,.two-col{grid-template-columns:1fr 1fr}.pose[hidden]{display:block!important}details>*{display:block}details>summary{margin-bottom:12px}tr[hidden]{display:table-row!important}.verdict{background:white}h1{font-size:28px}}
'''

PRINT_CSS = '''
.score-metrics { grid-template-columns: repeat(3, minmax(0, 1fr)); }
.score-summary { margin-top: 36px; }
.score-summary h2 { font-size: 22px; }

@page { size: Letter; margin: 0.6in 0.55in 0.65in; }
@media print {
  html, body { background: white; color: #172d38; font-size: 9pt; }
  body { -webkit-print-color-adjust: exact; print-color-adjust: exact; }
  .sidebar, .toolbar, .pose-controls, .downloads, .print-button,
  .verdict .button, .screen-only, #empty-checks { display: none !important; }
  main { margin: 0; padding: 0; width: auto; max-width: none; }
  header { margin-bottom: 16pt; padding-bottom: 12pt; }
  h1 { font-size: 26pt; } h2 { font-size: 18pt; } h3 { font-size: 11pt; }
  h1, h2, h3, summary, .section-heading { break-after: avoid; }
  p { orphans: 3; widows: 3; }
  section { break-before: page; margin: 0 0 14pt; }
  #overview { break-before: auto; }
  .section-heading { margin-bottom: 12pt; }
  .section-heading p, .subtitle, .muted, .callout, details > p { font-size: 9pt; }
  .panel { padding: 10pt; margin-bottom: 12pt; border-radius: 0; box-shadow: none; break-inside: auto; }
  .metrics { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 6pt; }
  .metric { padding: 9pt; break-inside: avoid; }
  .score-summary { margin-top: 28pt; break-inside: avoid; }
  .score-summary h2 { font-size: 16pt; }
  .score-metrics { grid-template-columns: repeat(3, minmax(0, 1fr)); }
  #effective-inputs { break-before: auto; }
  .metric strong { font-size: 17pt; } .metric small, .metric span { font-size: 8pt; }
  .input-grid { display: block; }
  .input-grid > .panel { break-inside: auto; }
  .two-col { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10pt; }
  .two-col > .panel { break-inside: avoid; }
  .table-wrap { overflow: visible; }
  table { width: 100%; table-layout: fixed; font-size: 8pt; }
  thead { display: table-header-group; }
  tr { break-inside: avoid; }
  th, td { padding: 6pt 5pt; min-width: 0 !important; white-space: normal !important; overflow-wrap: anywhere; }
  th { font-size: 7.5pt; } td small { font-size: 6.8pt; }
  .checks th:nth-child(1) { width: 27%; }
  .checks th:nth-child(2) { width: 23%; }
  .checks th:nth-child(3) { width: 20%; }
  .checks th:nth-child(4) { width: 14%; }
  .checks th:nth-child(5) { width: 16%; }
  .checks tr[data-check=not-assessed] td:nth-child(2) { font-size: 8pt; }
  .status { font-size: 7pt; white-space: normal; padding: 3pt; }
  tr[hidden] { display: table-row !important; }
  details { margin: 10pt 0; }
  details > summary { font-size: 11pt; list-style: none; margin-bottom: 9pt; }
  details > summary::marker { content: ''; }
  .raw-data { break-before: page; }
  .raw-data table { line-height: 1.25; }
  .raw-data th, .raw-data td { padding: 3pt 5pt; }
  .raw-data th:first-child { width: 54%; }
  .raw-data th:last-child { width: 13%; }
  .raw-data table:has(th:nth-child(5)) th:first-child { width: 42%; }
  #mdo table { line-height: 1.25; }
  #mdo th, #mdo td { padding: 2pt 5pt; }
  .pose, .pose[hidden] { display: block !important; break-inside: avoid; break-before: page; }
  .geometry-top, .geometry-extra { break-inside: avoid; }
  .geometry-sides { border: 0; padding: 0; }
  #geometry .pose svg { max-height: 7.2in; }
  .bar-row { break-inside: avoid; }
  .bar-row > div:first-child, .bar-row b { font-size: 8pt; white-space: normal; }
  .verdict { background: #f7faf9; padding: 12pt; }
  .verdict strong { font-size: 16pt; }
  .tag, .header-bottom { font-size: 8pt; }
  footer { display: none; }
}
'''

JS = '''
// Closed details have a browser-managed content box; open them for printing.
// Restore the reader's screen state after the print dialog closes.
let printDetails = [];
window.addEventListener('beforeprint', () => {
  printDetails = [...document.querySelectorAll('details')].filter(d => !d.open);
  printDetails.forEach(d => d.open = true);
});
window.addEventListener('afterprint', () => {
  printDetails.forEach(d => d.open = false);
  printDetails = [];
});
document.querySelectorAll('[data-pose]').forEach(button=>button.addEventListener('click',()=>{
  document.querySelectorAll('[data-pose]').forEach(b=>b.setAttribute('aria-pressed',String(b===button)));
  document.querySelectorAll('.pose').forEach(p=>p.hidden=p.id!=='pose-'+button.dataset.pose);
}));
const filter=document.getElementById('check-filter'), search=document.getElementById('check-search');
function filterChecks(){let shown=0;document.querySelectorAll('[data-check]').forEach(r=>{
  const mode=filter.value;
  const selected=mode==='attention'?(r.dataset.check==='fail'||r.dataset.kind==='unknown'):mode==='all'?r.dataset.kind!=='bounds':mode==='bounds'?r.dataset.kind==='bounds':r.dataset.check===mode;
  r.hidden=!(selected && r.textContent.toLowerCase().includes(search.value.toLowerCase()));
  if(!r.hidden)shown++;
});document.getElementById('check-count').textContent=shown+' shown';document.getElementById('empty-checks').hidden=shown!==0;}
filter.addEventListener('change',filterChecks);search.addEventListener('input',filterChecks);filterChecks();
const slider=document.getElementById('iteration');
if(slider){const frames=JSON.parse(document.getElementById('iteration-data').textContent);function showFrame(){const f=frames[Number(slider.value)];if(!f)return;document.getElementById('iteration-label').textContent=f.label+' · '+(f.feasible?'modeled checks pass':'failed trial')+' · '+f.points.toFixed(1)+' points';document.getElementById('history-retracted').innerHTML=f.retracted;document.getElementById('history-extended').innerHTML=f.extended;}slider.addEventListener('input',showFrame);showFrame();}
'''
