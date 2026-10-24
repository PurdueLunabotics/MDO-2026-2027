"""Self-contained HTML + SVG drawings. No graphics/browser dependency to generate.

Drawings use exactly the same kinematics as the constraints. Driver evaluations
are replayed, not finite-difference perturbations or invented iteration states.
"""
from html import escape
import json
import math
from pathlib import Path
from rover.config import ROUTE
from rover.linkage import sweep_poses, attachment_point


def values_from_snapshot(snapshot):
    return {name: item["value"] for group in ("design", "assumptions", "outputs")
            for name, item in snapshot[group].items()}


def plate_frame_svg(values):
    """Coordinate-driven axial-member schematic, explicitly not a CAD outline."""
    from rover.alternatives.plate_frame import plate_geometry, PLATE_EDGES
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
    for index, (p, label) in enumerate(zip(nodes, ("S: suspension axle", "A: front actuator", "B: front arm", "A: rear actuator", "B: rear arm"))):
        x, y = point(p)
        anchor = 'middle' if index == 0 else ('end' if p[0] < 0 else 'start')
        offset = 0 if index == 0 else (-14 if p[0] < 0 else 14)
        label_y = y+28 if index == 0 else y-12
        pieces.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="6" fill="#d77422"/><text x="{x+offset:.2f}" y="{label_y:.2f}" text-anchor="{anchor}" font-size="13">{escape(label)}</text>')
    lines = [f'Plate thickness: {values["plate_thickness"]*1000:.2f} mm (fixed)',
             f'Common strip width: {values["plate_spoke_width"]*1000:.2f} mm',
             f'CF ID / OD: {values["cf_tube_inner_diameter"]*1000:.2f} / {g["cf_od"]*1000:.2f} mm',
             f'Plate spacing / CF span: {values["cf_tube_span"]*1000:.1f} mm',
             f'Rocker bearing spacing: {g["rocker_span"]*1000:.1f} mm',
             f'Wheel-to-rocker gap: {values["wheel_rocker_gap"]*1000:.1f} mm',
             'Central support: pin, balanced mirrored gravity only.',
             'CF material properties are PROVISIONAL.',
             'Spoke buckling is a strip screen,', 'not whole-plate buckling validation.']
    if 'arm_box_width' in values and 'arm_box_height' in values:
        lines[6:6] = [f'Aluminum arm box: {values["arm_box_width"]*1000:.2f} x {values["arm_box_height"]*1000:.2f} mm',
                      f'Arm wall: {values["arm_wall"]*1000:.4f} mm']
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
           line(-xlim,-v("pit_depth"),xlim,-v("pit_depth"),"#ac7756",1,"6 5")]
    if "plate_spoke_width" in values:
        from rover.alternatives.plate_frame import plate_geometry, PLATE_EDGES
        frame = plate_geometry(values)
        for i, j in PLATE_EDGES:
            svg.append(line(*frame["nodes"][i], *frame["nodes"][j], "#7898b0", v("plate_spoke_width")*scale))
        svg.append(circle(*frame["nodes"][0], v("plate_hub_radius"), "#67839f", "#dce7f0"))
        for sign in (-1, 1):
            svg.append(line(0, frame["nodes"][0, 1], sign*v("wheelbase")/2, rw, "#67839f", v("rocker_box_height")*scale))
    else:
        svg.append(rect(-v("wheelbase")/2, rw, v("wheelbase"), 2*rw+v("chassis_height")-rw, "#dce7f0", "#67839f"))
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


def top_view_svg(values):
    """Orthographic travel-pose footprints, using the model's lateral stations.

    Overlap in projection is not a collision: height is absent from this view.
    Equipment and motor packaging is deliberately not inferred.
    """
    v = values.__getitem__
    cx, diameter = v("travel_x"), v("drum_diameter")
    wb, track, ww = v("wheelbase"), v("track_width"), v("wheel_width")
    length, gap = v("drum_length"), v("drum_pair_gap")
    overall_length = max(2*abs(cx)+diameter, wb+v("wheel_diameter"))
    overall_width = max(track+ww, 2*length+gap)
    scale = min(940/max(overall_length+.32, 1.82), 350/max(overall_width, .75))

    def point(x, y):
        return 600+x*scale, 335-y*scale

    def line(x1, y1, x2, y2, color="#567181", width=2, dash="", kind=""):
        a, b = point(x1, y1); c, d = point(x2, y2)
        return f'<line data-part="{kind}" x1="{a:.3f}" y1="{b:.3f}" x2="{c:.3f}" y2="{d:.3f}" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>'

    def box(x, y, dx, dy, fill, stroke, dash="", kind=""):
        a, b = point(x-dx/2, y+dy/2)
        return f'<rect data-part="{kind}" x="{a:.3f}" y="{b:.3f}" width="{dx*scale:.3f}" height="{dy*scale:.3f}" fill="{fill}" stroke="{stroke}" stroke-width="2" stroke-dasharray="{dash}"/>'

    def text(x, y, label, offset=0):
        a, b = point(x, y)
        return f'<text x="{a:.2f}" y="{b+offset:.2f}" text-anchor="middle" font-size="16">{escape(label)}</text>'

    pieces = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 740" role="img" aria-label="Top-down travel geometry: four wheels and four drums">',
              '<rect width="1200" height="740" rx="12" fill="#f7fafc"/>',
              '<g font-family="sans-serif" fill="#24354b">',
              '<text x="35" y="38" font-size="24" font-weight="700">Top-down view / travel pose</text>',
              '<text x="35" y="68" font-size="16">Orthographic footprints; front is +x (right). Dimensions in mm.</text>',
              box(0, 0, 1.5, .75, "none", "#5282ac", "8 5", "stowed-limit"),
              box(0, 0, wb, track, "#edf2f4", "#99aeb8", "4 5", "support-polygon")]
    from .frame_members import chassis_members, rocker_bearing_span
    members = chassis_members(values)
    if 'cf_tube_span' in values:
        span = rocker_bearing_span(values)
        # A top footprint of each sloping box rocker and the central round axle.
        pieces.append(box(0,0,v('axle_outer_diameter'),span,'#526a7b','#304859',kind='rocker-axle'))
        for sy in (-1,1):
            pieces.append(box(0,sy*span/2,wb,v('rocker_box_width'),'#9aafbd','#536c7f',kind='rocker'))
            for sx in (-1,1):
                pieces.append(line(sx*wb/2,sy*span/2,sx*wb/2,sy*track/2,
                                   '#526a7b',3,kind='wheel-axle'))
        plate_half_length = max(v('pivot_x'),v('actuator_pivot_x'))+v('cf_tube_inner_diameter')/2+v('cf_tube_wall')+v('frame_mount_clearance')
        for sy in (-1,1):
            pieces.append(box(0,sy*v('cf_tube_span')/2,2*plate_half_length,v('plate_thickness'),
                              '#bacbd4','#7898b0',kind='frame-plate'))
        for m in members:
            if m['kind'] == 'cf-connector':
                x,y,_ = m['start']; _,end_y,_ = m['end']
                pieces.append(box(x,(y+end_y)/2,m['width'],end_y-y,'#405761','#203b46',kind=m['kind']))
    else:
        for sy in (-1,1):
            pieces.append(line(-wb/2,sy*track/2,wb/2,sy*track/2,'#7898b0',4))
    for m in members:
        x,y,_ = m['start']; ex,ey,_ = m['end']
        if m['kind'] == 'arm' and m['width'] is not None:
            pieces.append(box((x+ex)/2,0,abs(ex-x),m['width'],'#76aac7','#246f99',kind='arm'))
        elif m['kind'] == 'arm':
            pieces.append(line(x,y,ex,ey,'#246f99',5,kind='arm'))
        elif m['kind'] == 'actuator':
            pieces.append(line(x,y,ex,ey,'#d77422',5,kind='actuator'))
    for sx in (-1,1):
        for sy in (-1,1):
            pieces.append(box(sx*wb/2,sy*track/2,v('wheel_diameter'),ww,'#cbd6df','#354657',kind='wheel'))
    for sx in (-1,1):
        for sy in (-1,1):
            pieces.append(box(sx*cx,sy*(length+gap)/2,diameter,length,'#e6b68e','#a54b1e',kind='drum'))
    for sx in (-1, 1):
        pieces.append(line(sx*cx, -(length+gap/2), sx*cx, length+gap/2, "#a54b1e", 2,kind='drum-shaft'))
    dim_y = max(overall_width, .75)/2+.10
    pieces += [line(-wb/2, dim_y, wb/2, dim_y),
               text(0, dim_y, f'Wheelbase {wb*1000:.1f}', -8),
               line(-overall_length/2, -dim_y, overall_length/2, -dim_y),
               text(0, -dim_y, f'Travel length {overall_length*1000:.1f}', 23)]
    for x in (-wb/2, wb/2):
        pieces.append(line(x, dim_y-.015, x, dim_y+.015))
    for x in (-overall_length/2, overall_length/2):
        pieces.append(line(x, -dim_y-.015, x, -dim_y+.015))
    pieces += [f'<text x="35" y="605" font-size="18">Wheel-center track: {track*1000:.1f} | Overall width: {overall_width*1000:.1f} | Wheel width: {ww*1000:.1f}</text>',
               f'<text x="35" y="636" font-size="18">Each drum: {diameter*1000:.1f} diameter x {length*1000:.1f} axial length | Pair gap: {gap*1000:.1f}</text>',
               '<text x="35" y="675" font-size="15">Blue: one arm per drum pair · orange: actuators · charcoal: CF connectors · slate: rockers and axle.</text>',
               '<text x="35" y="705" font-size="15">Four CF tubes: upper/lower footprints can overlap. Dashed blue: 1500 x 750 stowed limit.</text>',
               '<text x="35" y="730" font-size="13">Plate outlines are schematic. Projection overlap does not prove collision. Heights and equipment are not shown.</text>',
               '</g></svg>']
    return ''.join(pieces)


def stability_svg(values):
    """Scientific SVG of the declared gravity screens, not terrain animation."""
    from rover.alternatives.traversal_stability import slope_support, rock_path_screen
    fraction = .5 if "plate_spoke_width" in values else 1.
    rock = rock_path_screen(values, rocker_centroid_fraction=fraction, include_trace=True)
    max_angle = min(89., max(40., math.degrees(values["slope_angle"])+5))
    angles = [max_angle*i/80 for i in range(81)]
    slope = [slope_support(values["wheelbase"], values["track_width"], values["loaded_cg_x"],
                           values["loaded_cg_z"], math.radians(a)) for a in angles]
    height = [r["height"] for r in rock["trace"]]
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 410" role="img" aria-label="Static slope and one-wheel articulation stability curves">',
             '<rect width="1200" height="410" fill="#f7fafc"/><g font-family="sans-serif" fill="#203249">',
             '<text x="22" y="28" font-size="20">Loaded travel pose: gravity stability screens</text>',
             '<text x="22" y="52" font-size="13">One wheel raised at a time on horizontal platforms. Curves are model calculations; no physical rock profile is simulated.</text>']

    def panel(index, title, xs, curves, xmax, xlabel, ylabel, marker=None):
        left, top, width, high = 58+index*398, 110, 315, 220
        ys = [v for _, data, _ in curves for v in data]
        low, upper = min(0., min(ys)), max(0., max(ys))
        pad = max(upper-low, 1e-6)*.1
        low -= pad; upper += pad
        px = lambda x: left+x/max(xmax, 1e-9)*width
        py = lambda y: top+high-(y-low)/(upper-low)*high
        parts.append(f'<text x="{left}" y="82" font-size="16">{escape(title)}</text>')
        parts.append(f'<text x="{left}" y="101" font-size="12">{escape(ylabel)}</text>')
        parts.append(f'<rect x="{left}" y="{py(0):.2f}" width="{width}" height="{top+high-py(0):.2f}" fill="#fce9e5"/>')
        for i in range(5):
            y = low+(upper-low)*i/4
            parts.append(f'<line x1="{left}" y1="{py(y):.2f}" x2="{left+width}" y2="{py(y):.2f}" stroke="#dde5ea"/>')
            parts.append(f'<text x="{left-6}" y="{py(y)+4:.2f}" text-anchor="end" font-size="11">{y:.1f}</text>')
            x = xmax*i/4
            parts.append(f'<text x="{px(x):.2f}" y="{top+high+18}" text-anchor="middle" font-size="11">{x:.2g}</text>')
        parts.append(f'<line x1="{left}" y1="{py(0):.2f}" x2="{left+width}" y2="{py(0):.2f}" stroke="#a33323" stroke-dasharray="4 3"/>')
        if marker is not None:
            parts.append(f'<line x1="{px(marker):.2f}" y1="{top}" x2="{px(marker):.2f}" y2="{top+high}" stroke="#7e6c98" stroke-dasharray="4 3"/>')
        for j, (name, data, color) in enumerate(curves):
            points = ' '.join(f'{px(x):.2f},{py(y):.2f}' for x, y in zip(xs, data))
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2.5"/>')
            parts.append(f'<text x="{left+j*150}" y="{top+high+60}" font-size="12" fill="{color}">{escape(name)}</text>')
        parts.append(f'<text x="{left+width/2}" y="{top+high+39}" text-anchor="middle" font-size="12">{escape(xlabel)}</text>')

    panel(0, "Planar slope support", angles,
          [("Pitch margin", [s["slope_pitch_tip_margin"]*1000 for s in slope], "#246f99"),
           ("Roll margin", [s["slope_roll_tip_margin"]*1000 for s in slope], "#d77422")],
          max_angle, "Slope angle [degrees]", "Gravity-line reserve [mm]", math.degrees(values["slope_angle"]))
    panel(1, "One-wheel support hull", height,
          [("Worst of four raises", [r["tip"]*1000 for r in rock["trace"]], "#246f99")],
          max(height), "Raised-wheel height [m]", "Projected CG edge reserve [mm]")
    panel(2, "Differential wheel reactions", height,
          [("Smallest wheel reaction", [r["load"] for r in rock["trace"]], "#246f99")],
          max(height), "Raised-wheel height [m]", "Normal reaction [N]")
    parts.append('</g></svg>')
    return ''.join(parts)


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


def write_report(record, output, iterations=(), *, download_links=None):
    """Write a design review with unchanged snapshot data and geometry helpers."""
    from .report_layout import render_report
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    final, baseline = record["final"], record["baseline"]
    values = values_from_snapshot(final)
    drawings = []
    for name, extended, travel in (("retracted", False, False), ("travel", False, True), ("extended", True, False)):
        svg = geometry_svg(values, extended, travel=travel)
        (output.parent/f"{output.stem}_{name}.svg").write_text(svg, encoding="utf-8")
        drawings.append(svg)
    frame = plate_frame_svg(values) if "plate_spoke_width" in values else ""
    stability = stability_svg(values) if "slope_tip_margin" in values else ""
    top = top_view_svg(values)
    from .obstacle_drawing import obstacle_pose_svg
    obstacle = obstacle_pose_svg(values) if "obstacle_height" in values else ""
    for name, svg in (("frame", frame), ("stability", stability), ("top", top), ("obstacle", obstacle)):
        if svg:
            (output.parent/f"{output.stem}_{name}.svg").write_text(svg, encoding="utf-8")
    frames = []
    if record.get("status") != "baseline":
        history = [{"label": "Optimization starting design", "snapshot": baseline},
                   *iterations, {"label": "Saved candidate", "snapshot": final}]
        for entry in history:
            snap = entry["snapshot"]
            vals = values_from_snapshot(snap)
            frames.append({"label": entry["label"], "feasible": snap["feasible"],
                           "points": snap["outputs"]["construction_points"]["value"],
                           "retracted": geometry_svg(vals), "extended": geometry_svg(vals, True)})
    html = render_report(record, output, drawings, frame, stability,
                         route_svg(values["traversal_time_one_way"]), frames, top_drawing=top,
                         download_links=download_links, obstacle_drawing=obstacle)
    output.with_suffix(".html").write_text(html, encoding="utf-8")
