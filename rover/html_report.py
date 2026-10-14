"""Self-contained HTML + SVG drawings. No graphics/browser dependency to generate.

Drawings use exactly the same kinematics as the constraints. Driver evaluations
are replayed, not finite-difference perturbations or invented iteration states.
"""
from html import escape
import json
import math
from pathlib import Path
from .config import ROUTE
from .linkage import sweep_poses, attachment_point


def values_from_snapshot(snapshot):
    return {name: item["value"] for group in ("design", "assumptions", "outputs")
            for name, item in snapshot[group].items()}


def plate_frame_svg(values):
    """Coordinate-driven axial-member schematic, explicitly not a CAD outline."""
    from .alternatives.plate_frame import plate_geometry, PLATE_EDGES
    g = plate_geometry(values)
    nodes = g["nodes"]
    low, high = min(nodes[:, 1])-.07, max(nodes[:, 1])+.07
    scale = min(650., 350./(high-low))
    point = lambda p: (300+p[0]*scale, 470-(p[1]-low)*scale)
    pieces = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1100 550" role="img" aria-label="Plate frame axial-member geometry">',
              '<rect width="1100" height="550" fill="#f7fafc"/>',
              '<g font-family="sans-serif" fill="#203249">',
              '<text x="25" y="35" font-size="23">Plate frame: provisional member topology</text>',
              '<text x="25" y="63" font-size="15">Lengths and directions follow the mounting coordinates; this is not the manufactured cutout outline.</text>']
    for i, j in PLATE_EDGES:
        x1, y1 = point(nodes[i]); x2, y2 = point(nodes[j])
        pieces.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="#7898b0" stroke-width="{values["plate_spoke_width"]*scale:.2f}"/>')
    for p, label in zip(nodes, ("S: suspension axle", "A: front actuator", "B: front arm", "A: rear actuator", "B: rear arm")):
        x, y = point(p)
        anchor = 'end' if p[0] < 0 else 'start'
        offset = -14 if p[0] < 0 else 14
        pieces.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="6" fill="#d77422"/><text x="{x+offset:.2f}" y="{y-12:.2f}" text-anchor="{anchor}" font-size="13">{escape(label)}</text>')
    lines = [f'Plate thickness: {values["plate_thickness"]*1000:.2f} mm (fixed)',
             f'Common strip width: {values["plate_spoke_width"]*1000:.2f} mm',
             f'CF ID / OD: {values["cf_tube_inner_diameter"]*1000:.2f} / {g["cf_od"]*1000:.2f} mm',
             f'Plate spacing / CF span: {values["cf_tube_span"]*1000:.1f} mm',
             f'Rocker bearing spacing: {g["rocker_span"]*1000:.1f} mm',
             f'Wheel-to-rocker gap: {values["wheel_rocker_gap"]*1000:.1f} mm',
             'Central support: pin, balanced mirrored gravity only.',
             'CF material properties are PROVISIONAL.',
             'Spoke buckling is a strip screen,', 'not whole-plate buckling validation.']
    for i, line in enumerate(lines):
        pieces.append(f'<text x="640" y="{145+30*i}" font-size="15">{escape(line)}</text>')
    pieces.append('</g></svg>')
    return ''.join(pieces)


def geometry_svg(values, extended=False, title=None, travel=False):
    v = values.__getitem__
    poses, _ = sweep_poses(values)
    cx, cy = ((v("travel_x"),v("travel_y")) if travel else
              (poses[-1] if extended else (v("stowed_x"),v("stowed_y"))))
    fraction = values.get("actuator_attachment_fraction", 1.0)
    jx, jy = attachment_point(v("pivot_x"), v("bottom_pivot_height"), cx, cy, fraction)
    rd, rw = v("drum_diameter") / 2, v("wheel_diameter") / 2
    xlim = max(1.05, max(p[0] for p in poses) + rd + .12)
    yhi = max(.85, v("top_pivot_height") + .12, max(p[1] for p in poses) + rd + .12)
    ylo = min(-.55, min(p[1] for p in poses) - rd - .12)
    scale = min(1060 / (2 * xlim), 540 / (yhi - ylo))
    ox, oy = 600, 95 + yhi * scale
    def point(x, y):
        return ox + x * scale, oy - y * scale
    def line(x1, y1, x2, y2, color, width=3, dash=""):
        a,b = point(x1,y1); c,d = point(x2,y2)
        return f'<line x1="{a:.3f}" y1="{b:.3f}" x2="{c:.3f}" y2="{d:.3f}" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>'
    def circle(x,y,r,color,fill="none",width=2):
        a,b=point(x,y)
        return f'<circle cx="{a:.3f}" cy="{b:.3f}" r="{r*scale:.3f}" stroke="{color}" fill="{fill}" stroke-width="{width}"/>'
    def label(x,y,text,dx=0,dy=0):
        a,b=point(x,y)
        return f'<text x="{a+dx:.2f}" y="{b+dy:.2f}" font-size="16" fill="#24354b">{escape(text)}</text>'
    def rect(x,y,w,h,fill,stroke="none",dash=""):
        a,b=point(x,y+h)
        return f'<rect x="{a:.3f}" y="{b:.3f}" width="{w*scale:.3f}" height="{h*scale:.3f}" fill="{fill}" stroke="{stroke}" stroke-width="2" stroke-dasharray="{dash}"/>'
    state = "Raised · travel / deposition" if travel else ("Extended · digging" if extended else "Compact · starting envelope")
    title = title or state
    actuator_length = v("travel_actuator_length") if travel else (v("actuator_extended_length") if extended else v("actuator_retracted_length"))
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 770" role="img" aria-label="{escape(title, quote=True)}">',
           '<rect width="1200" height="770" rx="16" fill="#f7fafc"/>',
           f'<text x="35" y="36" font-family="sans-serif" font-size="24" font-weight="700" fill="#142a43">{escape(title)}</text>',
           '<g font-family="sans-serif">',
           '<text x="35" y="67" font-size="16" fill="#546579">Side elevation · mirrored front and rear · each visible drum represents a shaft-linked pair</text>',
           rect(-xlim, ylo, 2*xlim, -ylo, "#eee9e1"),
           rect(-.75, 0, 1.5, .75, "none", "#5282ac", "8 5"),
           line(-xlim,0,xlim,0,"#8c7960",2),
           line(-xlim,-v("pit_depth"),xlim,-v("pit_depth"),"#ac7756",1,"6 5"),
           rect(-v("wheelbase")/2, rw, v("wheelbase"), 2*rw+v("chassis_height")-rw, "#dce7f0", "#67839f")]
    for sign in (-1,1):
        svg += [circle(sign*v("wheelbase")/2,rw,rw,"#354657","#cbd6df",3)]
        path=" ".join(f'{point(sign*x,y)[0]:.2f},{point(sign*x,y)[1]:.2f}' for x,y in poses)
        svg += [f'<polyline points="{path}" fill="none" stroke="#9aafba" stroke-width="2" stroke-dasharray="3 5"/>',
                line(sign*v("pivot_x"),v("bottom_pivot_height"),sign*cx,cy,"#246f99",7),
                line(sign*v("actuator_pivot_x"),v("top_pivot_height"),sign*jx,jy,"#d77422",7),
                circle(sign*cx,cy,rd,"#a54b1e","#e6b68e",3),
                circle(sign*cx,cy,.009,"#172e45","#172e45"),
                circle(sign*jx,jy,.009,"#172e45","#172e45"),
                circle(sign*v("actuator_pivot_x"),v("top_pivot_height"),.009,"#172e45","#172e45"),
                circle(sign*v("pivot_x"),v("bottom_pivot_height"),.009,"#172e45","#172e45")]
    svg += [label(v("actuator_pivot_x"),v("top_pivot_height"),"A",-22,-12),
            label(v("pivot_x"),v("bottom_pivot_height"),"B",-22,22),
            label(cx,cy,"C",12,-10), label(-.74,.75,"1.50 m × 0.75 m stowed envelope",8,-10),
            label(-xlim+.03,0,"Ground level",0,-10),
            label(-xlim+.03,-v("pit_depth"),f'Pit floor: −{v("pit_depth"):.2f} m',0,22)]
    if fraction < 1:
        svg.append(label(jx,jy,"D",12,18))
    top = cy + rd; bottom = cy - rd
    svg += [f'<text x="35" y="672" font-size="18" fill="#24354b">Actuator: {actuator_length*1000:.1f} mm · Fixed arm: {v("arm_length")*1000:.1f} mm · Drum Ø {2*rd*1000:.1f} mm</text>',
            f'<text x="35" y="705" font-size="18" fill="#24354b">Drum top: {top:+.3f} m · Drum bottom: {bottom:+.3f} m · A: {v("top_pivot_height"):.3f} m · B: {v("bottom_pivot_height"):.3f} m</text>',
            f'<text x="35" y="739" font-size="15" fill="#546579">Blue = fixed arm · Orange = actuator · Attachment B-D: {fraction*v("arm_length")*1000:.1f} mm ({fraction:.1%} of arm) · Ground is y = 0</text>',
            '</g></svg>']
    return "".join(svg)


def route_svg(one_way_time=45.0):
    points=[(0.,0.)]; heading=0.
    for d, degrees in ROUTE:
        x,y=points[-1]
        points.append((x+d*math.cos(heading), y+d*math.sin(heading)))
        heading+=math.radians(degrees)
    min_x=min(p[0] for p in points)
    coords=lambda p: (80+(p[0]-min_x)*90, 450-p[1]*70)
    line=" ".join(f'{coords(p)[0]:.1f},{coords(p)[1]:.1f}' for p in points)
    labels=[]
    for i, ((d,deg),p,q) in enumerate(zip(ROUTE,points,points[1:])):
        x,y=coords(((p[0]+q[0])/2,(p[1]+q[1])/2))
        labels.append(f'<text x="{x+12:.1f}" y="{y-12:.1f}" font-size="17">{d:g} m</text>')
        if deg:
            x,y=coords(q)
            labels.append(f'<text x="{x+12:.1f}" y="{y+18:.1f}" font-size="14">{deg:g}° left</text>')
    x,y=coords(points[-1])
    start_x,start_y=coords(points[0])
    return f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 630 550" role="img" aria-label="Traversal route"><rect width="630" height="550" fill="#f7fafc"/><g font-family="sans-serif" fill="#24354b"><text x="25" y="35" font-size="22">{one_way_time:g} s outbound + {one_way_time:g} s return</text><polyline points="{line}" stroke="#246f99" stroke-width="5" fill="none"/><circle cx="{start_x:.1f}" cy="{start_y:.1f}" r="7" fill="#246f99"/><text x="{start_x-15:.1f}" y="{start_y+35:.1f}" font-size="16">Excavate</text><circle cx="{x:.1f}" cy="{y:.1f}" r="7" fill="#d77422"/><text x="{x-45:.1f}" y="{y+40:.1f}" font-size="16">Deposit</text>{"".join(labels)}<text x="25" y="535" font-size="15">Return follows the same path in reverse order; yaw signs reverse.</text></g></svg>'


def write_report(record, output, iterations=()):
    output=Path(output)
    final, baseline=record["final"],record["baseline"]
    values=values_from_snapshot(final)
    one_way_time=values["traversal_time_one_way"]
    route_text=" → ".join(f'{d:g} m' + (f' → {abs(a):g}° {"left" if a > 0 else "right"}' if a else "") for d,a in ROUTE)
    turn_count=sum(a != 0 for _,a in ROUTE)
    prefix=output.stem
    drawings=[]
    for name,extended,travel in (("retracted",False,False),("travel",False,True),("extended",True,False)):
        svg=geometry_svg(values,extended,travel=travel)
        (output.parent/f'{prefix}_{name}.svg').write_text(svg,encoding="utf-8")
        drawings.append(svg)
    status=record["status"].replace("_"," ").upper()
    provenance=record.get("model_metadata",final.get("model_metadata",{}))
    def fmt(n): return f'{n:,.5g}'
    rows=[]
    for name,item in final["design"].items():
        value,low,high=item["value"],item["lower"],item["upper"]
        fraction=(value-low)/(high-low)
        bound="Lower bound" if fraction<.005 else ("Upper bound" if fraction>.995 else "Interior")
        rows.append(f'<tr><th>{escape(name)}</th><td>{fmt(low)}</td><td><b>{fmt(value)}</b></td><td>{fmt(high)}</td><td>{escape(item["units"] or "—")}</td><td><div class="track"><i style="left:{100*max(0,min(1,fraction)):.2f}%"></i></div>{bound}</td></tr>')
    output_rows="".join(f'<tr><th>{escape(name)}</th><td>{fmt(baseline["outputs"][name]["value"])}</td><td><b>{fmt(item["value"])}</b></td><td>{escape(item["units"] or "—")}</td></tr>' for name,item in final["outputs"].items())
    constraint_rows="".join(f'<tr><th>{escape(name)}</th><td>{fmt(item["margin"])}</td><td>{escape(item["units"] or "—")}</td><td class="{"pass" if item["passed"] else "fail"}">{"PASS" if item["passed"] else "FAIL"}</td></tr>' for name,item in final["constraints"].items())
    assumptions="".join(f'<tr><th>{escape(name)}</th><td>{fmt(item["value"])}</td><td>{escape(item["units"] or "—")}</td></tr>' for name,item in final["assumptions"].items())
    history=[{"label":"Starting design", "snapshot":baseline},*iterations,{"label":"Final iterate", "snapshot":final}]
    frames=[]
    for entry in history:
        snap=entry["snapshot"]
        vals=values_from_snapshot(snap)
        frames.append({"label":entry["label"],"feasible":snap["feasible"],
                       "points":snap["outputs"]["construction_points"]["value"],
                       "retracted":geometry_svg(vals),"extended":geometry_svg(vals,True)})
    data=json.dumps(frames,ensure_ascii=True).replace("</","<\\/")
    css='''body{margin:0;background:#eef3f7;color:#203249;font:15px/1.55 system-ui,sans-serif}main{max-width:1240px;margin:auto;padding:35px 24px}h1{font-size:32px;margin:0}h2{margin-top:0;font-size:22px}p{max-width:1000px}.sub{color:#536780}.card{background:white;border-radius:14px;padding:24px;margin:22px 0;box-shadow:0 3px 16px #1e3d5610}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}svg{width:100%;height:auto}.metrics{display:flex;flex-wrap:wrap;gap:18px}.metric{flex:1;min-width:150px;padding:15px;background:#f0f6fa;border-radius:8px}.metric b{display:block;font-size:25px}.scroll{overflow-x:auto}table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;font-size:14px}th,td{text-align:left;border-bottom:1px solid #e4ebf1;padding:10px 12px}thead{background:#eaf1f6}tbody th{font-weight:500}th:first-child{overflow-wrap:anywhere}.track{width:170px;height:9px;background:#dce7ee;border-radius:8px;position:relative;margin:8px 0}.track i{display:block;position:absolute;width:5px;height:17px;background:#176d96;top:-4px;transform:translateX(-50%)}.pass{color:#12694d}.fail{color:#a33323}.badge{display:inline-block;background:#dfeef6;padding:5px 12px;border-radius:30px;font-weight:700}.note{padding:12px 15px;border-left:4px solid #d77422;background:#fff5e8}input[type=range]{width:100%;margin:14px 0}summary{cursor:pointer;font-weight:650}a{color:#12668e}@media(max-width:800px){.grid{grid-template-columns:1fr}main{padding:20px 10px}.card{padding:16px}}@media print{body{background:white}.card{box-shadow:none;break-inside:avoid}.replay{display:none}}'''
    metrics="".join(f'<div class="metric">{label}<b>{fmt(final["outputs"][name]["value"])} {unit}</b></div>' for name,label,unit in
                    [("construction_points","Construction productivity","pts"),("dry_mass","Robot mass","kg"),("cycle_time","Full cycle","s"),("actuation_time","Raise + lower","s"),("tip_stability_margin","Uneven-load tip reserve","m")])
    obstacle_card = ""
    if "obstacle_pose_valid" in values:
        pose_status = ("Requested pose is geometrically reachable." if values["obstacle_pose_valid"] > .5
                       else "Guarded pose: requested height is unreachable. Tip, CG and angle diagnostics use the reported guarded height; they are not predictions at the requested height.")
        obstacle_card = f'''<section class="card"><h2>Loaded obstacle stability</h2>
<p>{escape(pose_status)}</p><p>All four drums loaded, arms fixed in travel position; each wheel raised in turn. Requested height: {values['obstacle_height']:.3f} m. Evaluated height: {values['obstacle_evaluated_height']:.3f} m.</p>
<p>Reach reserve: {values['obstacle_pose_margin']:+.5f} m. Gravity tip reserve: {values['obstacle_tip_margin']:+.5f} m. Centered rocker pivots: {values['rocker_pivot_offset']:+.3f} m above the wheel-center line (editable estimate).</p>
<p>Average-angle differential: chassis pitch {math.degrees(values['obstacle_chassis_pitch']):+.2f}°, roll {math.degrees(values['obstacle_chassis_roll']):+.2f}°. Point-contact platforms are assumed; joint stops/load equilibrium, climbing torque, traction, other hardware interference and terrain dynamics remain unverified.</p>
<p class="note">Ditch requirement remains unverified. The 0.50 m deep ditch is circle-like, but width/radius is unknown. The excavation-trench screen below does not verify this requirement.</p></section>'''
    clearance_card = ""
    if "obstacle_drum_wheel_gap_bound" in values:
        clearance_status = (
            "Requested-height clearance screen." if values["obstacle_clearance_pose_valid"] > .5
            else "Guarded clearance: requested pose is unreachable. The gap is a diagnostic at the guarded height, not a prediction at the requested height; the margin remains failed."
        )
        clearance_card = f'''<section class="card"><h2>Obstacle drum–wheel clearance</h2>
<p>{escape(clearance_status)} Evaluated height: {values['obstacle_clearance_evaluated_height']:.3f} m.</p>
<p>Conservative gap bound: {values['obstacle_drum_wheel_gap_bound']:+.5f} m. Required clearance: {values['mechanism_clearance']:.5f} m. Constraint margin: {values['obstacle_drum_wheel_margin']:+.5f} m.</p>
<p>All 16 drum–wheel pairs are checked in each of four raised-wheel poses. Spheres enclose the rigid cylindrical envelopes, including the drums' outer cutting radius. A positive bound guarantees envelope separation; a negative bound is inconclusive about actual cylinder collision and is conservatively rejected.</p>
<p>Arms remain fixed in travel position. Intermediate crossing poses, wheel/frame deflection, terrain contact and interference with hubs, shafts, arms or the frame are outside this screen.</p></section>'''
    frame_card = ""
    if "plate_spoke_width" in values:
        frame_drawing = plate_frame_svg(values)
        (output.parent/f'{prefix}_frame.svg').write_text(frame_drawing, encoding="utf-8")
        frame_card = f'''<section class="card"><h2>Plate-frame structural screen</h2>{frame_drawing}
<p>Aluminum plates: 6061-T6, fixed 6.35 mm sheet. Four CF rods: fixed 25.4 mm ID, adjustable wall and span. Aluminum box-section rockers and round suspension axle have separate dimensions. Two-force plate strips omit junction bending.</p>
<p>All four drums loaded; mirrored compact and travel-to-dig gravity poses. Cutting, self-recovery, obstacle load redistribution, connections, local tube-wall and whole-plate buckling are outside this structural screen.</p>
<p>Maximum relative upper/lower mount displacement: {values['frame_mount_relative_deflection']*1000:.3f} mm. Rocker-end flexure: {values['rocker_deflection']*1000:.3f} mm. Axle-overhang flexure: {values['axle_deflection']*1000:.3f} mm. Arm-tip flexure: {values['arm_deflection']*1000:.3f} mm.</p>
<p>The retained {values['allowable_deflection']*1000:.2f} mm limit applies to their conservative eighth-norm envelope, not one assembled-frame displacement. Aluminum strength, CF flexural strength and strip buckling have separate margins below.</p>
<p class="note">CF density, bending modulus and flexural strength remain PROVISIONAL until an actual tube/layup is selected. Numerical feasibility is conditional on those inputs. Frame load solution valid: {bool(values['frame_load_solution_valid'])}; plate geometry valid: {bool(values['plate_geometry_valid'])}. Failed arithmetic trials retain failed strength/deflection/buckling margins.</p>
<p><a href="{prefix}_frame.svg">Open frame schematic</a></p></section>'''
    html=f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Rover MDO design report</title><style>{css}</style></head><body><main>
<h1>Rover MDO · design report</h1><p class="sub">Fixed-time route · four bucket drums · mirrored actuator linkages · OpenMDAO {escape(record['openmdao_version'])}</p>
<p>Calibration status: {escape(provenance.get('calibration','not recorded'))}.</p>
<span class="badge">{escape(status)}</span><p>Optimizer success: {escape(str(record['optimizer_success']))}. Modeled feasibility: <b>{final['feasible']}</b>. All dimensions and results below describe the final saved iterate.</p>
<div class="metrics">{metrics}</div>
{obstacle_card}
{clearance_card}
{frame_card}
<p class="note">Uncalibrated conceptual screens. Payload is limited by reachable soil and drum storage; partial fill is allowed. The provisional trench screen requires wheel diameter at least as large as trench depth. Travel drums must clear a wheel-diameter step; loaded-route stability limits raising them. The static tip screen checks one drum pair full and the other empty, with both pairs fully extended. Other half-loaded arm positions require a software interlock; this model does not verify it. The geometry drawings are reference schematics, not CAD of optional drum, wheel or suspension models. These screens need terrain, obstacle and asymmetric-load tests. Tube strength and RPM validity also require physical validation. See MODEL.md and the model cards; 3D packaging and joint strength remain separate checks. Failed iterates remain visible and are labeled as such.</p>
<section class="card"><h2>Final geometry</h2><div class="grid">{drawings[0]}{drawings[1]}{drawings[2]}</div><p>The compact starting pose fits the dashed envelope. After deployment, the drums clear a wheel-diameter step in the travel pose. The 2.50 m deployed height and 0.45 m pit floor are checked separately. The level-ground linkage clearance covers the sampled travel-to-dig arc; the obstacle drum–wheel screen separately checks four raised-wheel travel poses. These drawings show level-ground geometry.</p><p><a href="{prefix}_retracted.svg">Open starting drawing</a> · <a href="{prefix}_travel.svg">Open travel drawing</a> · <a href="{prefix}_extended.svg">Open digging drawing</a> · <a href="{prefix}_n2.html">Open sparse N2 diagram</a></p></section>
<section class="card replay"><h2>Geometry through driver evaluations</h2><p>Recorded driver evaluations, including line-search trials; finite-difference perturbations are excluded. The slider starts at the final iterate.</p><label for="iteration">Evaluation</label><input id="iteration" type="range" min="0" max="{len(frames)-1}" value="{len(frames)-1}" step="1"><p id="iteration-label"></p><div class="grid"><div id="history-retracted"></div><div id="history-extended"></div></div></section>
<section class="card"><h2>Route and cycle accounting</h2><div class="grid"><div>{route_svg(one_way_time)}</div><div><p>Outbound: {route_text}.</p><p><b>{one_way_time:g} s outbound + {one_way_time:g} s return</b>, including {turn_count} route turns per direction. Total translation: {values['route_distance']:g} m; route yaw: {values['route_turn_angle']:g}°. Yaw signs reverse on return.</p><p>Lower and rotate in place → dwell if needed → raise loaded drums → outbound → dump one pair → separate {math.degrees(values['turn_angle']):g}° turn → dump second pair → return along the reversed path.</p><p>Cycle counts the lowering, excavation and raising phases once. The actuator/lowering overlap is removed once; rotation never creates soil beyond the fully extended cutting envelope. The modeled cycle deposits from the raised travel position; the tip screen also covers extended-position deposition.</p><p>Whole-cycle cross-check: {final['whole_cycles']['completed']} completed cycles; {fmt(final['whole_cycles']['berm_volume_m3'])} m³; {fmt(final['whole_cycles']['construction_points'])} points. Remaining time is idle.</p></div></div></section>
<section class="card"><h2>Design variables and bounds</h2><p>Marks show each result within its physical bounds. Bounds other than the 359 mm wheel/drum caps are exploratory choices.</p><div class="scroll"><table><thead><tr><th>Variable</th><th>Lower</th><th>Final</th><th>Upper</th><th>Units</th><th>Position in range</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div></section>
<section class="card"><h2>Outputs</h2><div class="scroll"><table><thead><tr><th>Output</th><th>Starting design</th><th>Final iterate</th><th>Units</th></tr></thead><tbody>{output_rows}</tbody></table></div></section>
<section class="card"><h2>Constraint margins</h2><p>Nonnegative means satisfied, allowing the documented scaled numerical tolerance.</p><div class="scroll"><table><thead><tr><th>Constraint</th><th>Margin</th><th>Units</th><th>Check</th></tr></thead><tbody>{constraint_rows}</tbody></table></div></section>
<section class="card"><details><summary>Fixed assumptions</summary><table><thead><tr><th>Input</th><th>Value</th><th>Units</th></tr></thead><tbody>{assumptions}</tbody></table></details></section>
<section class="card"><details><summary>Model provenance and evidence</summary><pre style="white-space:pre-wrap;overflow-wrap:anywhere">{escape(json.dumps(provenance,indent=2))}</pre></details></section>
<p class="sub">Objective = w_mass × 4.4 V/(t m) + w_energy × 1.5 V/(t E), with V in cm³, t in minutes, m in kg, E in Wh. Energy consumed is used, not motor power ratings.</p>
<script id="iteration-data" type="application/json">{data}</script><script>const frames=JSON.parse(document.getElementById('iteration-data').textContent);const slider=document.getElementById('iteration');function showFrame(){{const f=frames[Number(slider.value)];document.getElementById('iteration-label').textContent=f.label+' · '+(f.feasible?'modeled constraints pass':'infeasible trial')+' · '+f.points.toFixed(1)+' points';document.getElementById('history-retracted').innerHTML=f.retracted;document.getElementById('history-extended').innerHTML=f.extended;}}slider.addEventListener('input',showFrame);showFrame();</script>
</main></body></html>'''
    output.with_suffix('.html').write_text(html,encoding="utf-8")
