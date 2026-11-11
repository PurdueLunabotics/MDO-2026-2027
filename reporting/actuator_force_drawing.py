"""Report-only force sweep using the existing arm equilibrium helper.

Forces are axial magnitudes for ONE actuator. Stroke is extension from the
hardware minimum length (or the shortest command for the generic actuator).
Nothing here changes model outputs, sizing envelopes, or feasibility.
"""
import math

from rover.linkage import offset_values, sweep_poses
from rover.alternatives.offset_attachment import (
    offset_mount_loads, physical_length, pin_radius, attachment_point,
)

LBF = 4.4482216152605
INCH = .0254


def force_stroke_data(values):
    """Resolve the same 81 capability poses and gravity load cases."""
    v = values.__getitem__
    side, top = offset_values(values)
    arm = v("arm_length")
    radius = pin_radius(arm, side, top)
    bottom = (v("pivot_x"), v("bottom_pivot_height"))
    upper = (v("actuator_pivot_x"), v("top_pivot_height"))
    commands = [v("travel_actuator_length"), v("actuator_extended_length"),
                values.get("deposition_actuator_length", v("travel_actuator_length"))]
    separation = math.dist(bottom, upper)
    numbers = [*bottom, *upper, *commands, v("drum_mass"), v("payload_mass"),
               v("arm_linear_mass"), v("gravity"),
               v("actuator_force_rating"), v("actuator_safety_factor")]
    if not all(math.isfinite(n) for n in numbers):
        raise ValueError("Nonfinite actuator geometry or loads")
    if (separation <= 0 or min(commands) <= abs(radius-separation)
            or max(commands) >= radius+separation):
        raise ValueError("The commanded sweep is unreachable; guarded poses do not define a valid force curve")
    if (min(v("drum_mass"), v("payload_mass"), v("arm_linear_mass")) < 0
            or min(v("gravity"), v("actuator_force_rating"), v("actuator_safety_factor")) <= 0):
        raise ValueError("Actuator masses, loads and ratings must be in their physical domains")
    hardware_min = values.get("actuator_catalog_retracted_length")
    origin = hardware_min if hardware_min is not None else min(*commands, v("actuator_retracted_length"))
    if not math.isfinite(origin):
        raise ValueError("Nonfinite stroke origin")
    arm_weight = v("arm_linear_mass")*physical_length(arm, side)*v("gravity")
    empty_weight = v("drum_mass")/2*v("gravity")
    loaded_weight = empty_weight+v("payload_mass")/2*v("gravity")
    points = []
    poses, _ = sweep_poses(values)
    for drum in poses:
        pin = attachment_point(*bottom, *drum, side, top)
        length = math.dist(pin, upper)
        point = {"stroke_m": length-origin, "length_m": length}
        for name, weight in (("empty", empty_weight), ("loaded", loaded_weight)):
            reaction = offset_mount_loads(bottom, upper, drum, weight, arm_weight, side, top)[2]
            point[name+"_force_n"] = math.hypot(*reaction)
        points.append(point)
    return {"points": sorted(points, key=lambda p: p["stroke_m"]),
            "origin_m": origin, "hardware_origin": hardware_min is not None,
            "allowable_force_n": v("actuator_force_rating")/v("actuator_safety_factor")}


def force_stroke_svg(data, units="metric"):
    """Self-contained SVG with both grid directions and no force clipping."""
    if units not in ("metric", "imperial"):
        raise ValueError("Actuator plot units must be metric or imperial")
    imperial = units == "imperial"
    distance_scale, force_scale = (1/INCH, 1/LBF) if imperial else (100., 1.)
    distance_unit, force_unit = ("in", "lbf") if imperial else ("cm", "N")
    points = data["points"]
    xmin = min(0., min(p["stroke_m"] for p in points))*distance_scale
    xmax = max(p["stroke_m"] for p in points)*distance_scale
    xmax = max(xmax, xmin+.01*distance_scale)
    ymax = max(data["allowable_force_n"], *(p[n+"_force_n"] for p in points
                                          for n in ("empty", "loaded")))*force_scale*1.12
    left, top, width, height = 108., 100., 970., 300.
    px = lambda x: left+(x-xmin)/(xmax-xmin)*width
    py = lambda y: top+height-y/ymax*height
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1140 620" role="img" '
        f'aria-label="Actuator force versus stroke in {force_unit} and {distance_unit}" data-force-units="{units}">',
        '<rect width="1140" height="620" rx="14" fill="#f7fafc"/>',
        '<g font-family="sans-serif" fill="#203249">',
        '<text x="30" y="36" font-size="25" font-weight="700">Actuator force versus stroke</text>',
        '<text x="30" y="65" font-size="17">Axial force per actuator · quasi-static capability sweep</text>',
        f'<rect x="{left:g}" y="{top:g}" width="{width:g}" height="{height:g}" fill="white"/>',
    ]
    for i in range(6):
        fraction = i/5
        x, y = xmin+(xmax-xmin)*fraction, ymax*fraction
        parts += [
            f'<line data-grid="x" x1="{px(x):.2f}" y1="{top:g}" x2="{px(x):.2f}" y2="{top+height:g}" stroke="#d9e2e8"/>',
            f'<line data-grid="y" x1="{left:g}" y1="{py(y):.2f}" x2="{left+width:g}" y2="{py(y):.2f}" stroke="#d9e2e8"/>',
            f'<text x="{px(x):.2f}" y="424" text-anchor="middle" font-size="15">{x:.2f}</text>',
            f'<text x="94" y="{py(y)+5:.2f}" text-anchor="end" font-size="15">{y:,.0f}</text>',
        ]
    parts.append(f'<path d="M{left:g} {top:g} V{top+height:g} H{left+width:g}" fill="none" stroke="#536579" stroke-width="2"/>')
    for index, (name, caption, color) in enumerate((
            ("empty", "Empty drums + arm", "#176a67"),
            ("loaded", "Full drums + arm", "#246f99"))):
        coords = ' '.join(f'{px(p["stroke_m"]*distance_scale):.2f},{py(p[name+"_force_n"]*force_scale):.2f}' for p in points)
        parts += [f'<polyline data-load="{name}" points="{coords}" fill="none" stroke="{color}" stroke-width="3"/>',
                  f'<line x1="{108+index*318}" y1="490" x2="{138+index*318}" y2="490" stroke="{color}" stroke-width="3"/>',
                  f'<text x="{146+index*318}" y="495" font-size="16">{caption}</text>']
    capacity = data["allowable_force_n"]*force_scale
    parts += [
        f'<line data-load="capacity" data-capacity="{capacity:.12g}" x1="{left:g}" y1="{py(capacity):.2f}" x2="{left+width:g}" y2="{py(capacity):.2f}" stroke="#755b91" stroke-width="2.5" stroke-dasharray="8 5"/>',
        '<line x1="108" y1="520" x2="138" y2="520" stroke="#755b91" stroke-width="2.5" stroke-dasharray="8 5"/>',
        f'<text x="146" y="525" font-size="16">Allowable force: {capacity:,.1f} {force_unit} (rating / safety factor)</text>',
        f'<text x="593" y="460" text-anchor="middle" font-size="17">Stroke from {"hardware minimum" if data["hardware_origin"] else "shortest command"} ({distance_unit})</text>',
        f'<text x="26" y="250" transform="rotate(-90 26 250)" text-anchor="middle" font-size="17">Force per actuator ({force_unit})</text>',
        f'<text x="30" y="562" font-size="16">Stroke zero = {data["origin_m"]*distance_scale:.3f} {distance_unit} pin-to-pin length.</text>',
        '<text x="30" y="588" font-size="15">Quasi-static gravity loads only. Extraction resistance and dynamic loads are not modeled.</text>',
        '</g></svg>',
    ]
    return ''.join(parts)
