"""Native SVG views of the evaluated moving-dig geometry and feed screen.

These drawings explain the analytic model; they do not simulate soil motion.
"""
import math


def moving_excavation_svg(values):
    """Draw tandem pairs cutting two distinct fresh layers in the same tracks."""
    radius = values["drum_diameter"] / 2
    opening = values["inlet_depth"]
    front = (values["front_dig_x"], values["front_dig_y"])
    rear = (values["rear_dig_x"], values["rear_dig_y"])
    half_span = max(abs(front[0]), abs(rear[0])) + radius + .12
    scale = min(1060 / (2 * half_span), 190 / max(2 * radius, .1))
    origin, grade = 600, 300
    point = lambda x, z: (origin + scale * x, grade - scale * z)
    fx, fy = point(*front)
    rx, ry = point(*rear)
    upper_floor = grade + scale * opening
    lower_floor = grade + scale * 2 * opening
    pieces = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 560" role="img" aria-label="Moving excavation: front and rear fresh soil layers">',
        '<rect width="1200" height="560" rx="14" fill="#f7fafc"/>',
        '<g font-family="sans-serif" fill="#203249">',
        '<text x="30" y="36" font-size="25" font-weight="700">Moving excavation · two fresh layers</text>',
        '<text x="30" y="65" font-size="17">Side elevation · each circle represents a shaft-linked pair · forward is right</text>',
        '<rect x="35" y="300" width="1130" height="125" fill="#d8c3a4"/>',
        f'<rect x="35" y="300" width="{fx-35:.2f}" height="{scale*opening:.2f}" fill="#f7fafc"/>',
        f'<rect x="35" y="{upper_floor:.2f}" width="{rx-35:.2f}" height="{scale*opening:.2f}" fill="#f7fafc"/>',
        f'<path d="M35 {lower_floor:.2f} H{rx:.2f} V{upper_floor:.2f} H{fx:.2f} V300 H1165" fill="none" stroke="#816b50" stroke-width="3"/>',
        '<line x1="35" y1="300" x2="1165" y2="300" stroke="#8b7961" stroke-dasharray="7 5"/>',
        '<text x="1020" y="290" font-size="15">Original grade</text>',
        f'<line x1="{fx+scale*radius+15:.2f}" y1="300" x2="{fx+scale*radius+15:.2f}" y2="{upper_floor:.2f}" stroke="#227a80" stroke-width="3"/>',
        f'<line x1="{rx-scale*radius-15:.2f}" y1="300" x2="{rx-scale*radius-15:.2f}" y2="{lower_floor:.2f}" stroke="#b36624" stroke-width="3"/>',
    ]
    for (x, z), color, name, depth in ((front, "#227a80", "Front pair", opening), (rear, "#b36624", "Rear pair", 2*opening)):
        cx, cy = point(x, z)
        pieces += [
            f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{radius*scale:.2f}" fill="{color}" fill-opacity=".12" stroke="{color}" stroke-width="4"/>',
            f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="5" fill="{color}"/>',
            f'<text x="{cx:.2f}" y="84" text-anchor="middle" font-size="18" font-weight="700">{name}</text>',
            f'<text x="{cx:.2f}" y="104" text-anchor="middle" font-size="15">Depth {depth*1000:.1f} mm = {"d" if name == "Front pair" else "2d"}</text>',
        ]
    pieces += [
        '<path d="M470 165 H715 M700 155 L715 165 L700 175" fill="none" stroke="#203249" stroke-width="3"/>',
        f'<text x="590" y="151" text-anchor="middle" font-size="16">Feed {values["feed_speed"]*1000:.2f} mm/s</text>',
        f'<text x="600" y="385" text-anchor="middle" font-size="17">Each pair removes one inlet-depth layer: d = {opening*1000:.1f} mm, across the full drum width.</text>',
        '<text x="600" y="411" text-anchor="middle" font-size="16">The rear pair does not count the front pair’s removed soil again.</text>',
        '<text x="30" y="452" font-size="16">Staged entry: front starts → rear reaches that start and lowers → front lifts when full → rear finishes.</text>',
        f'<text x="30" y="479" font-size="16">Productive lane: {values["feed_distance"]:.3f} m · total feed travel: {values["feed_drive_distance"]:.3f} m · feed time: {values["feed_time"]:.1f} s</text>',
        '<text x="30" y="509" font-size="15">The maximum-extension view is a separate plunge/recovery reach screen; it is not this operating pose.</text>',
        '<text x="30" y="535" font-size="15">Schematic soil boundary; entry/endcap soil is not credited. Soil flow and recovery driving require validation.</text>',
        '</g></svg>',
    ]
    return ''.join(pieces)


def feed_rpm_svg(values):
    """Plot the analytic feed limit without implying measured granular accuracy."""
    rpm, speed = values["drum_rpm"], values["feed_speed"]
    n, bite = values["scoops_per_revolution"], values["inlet_depth"]
    rpm_max, speed_max = max(120., rpm*1.15), max(.05, speed*1.2)
    left, top, width, height = 100., 100., 940., 330.
    xy = lambda r, v: (left+r/rpm_max*width, top+height-v/speed_max*height)
    slope = bite*n/60
    bound_top_rpm = speed_max/slope
    safe = [(0, 0), (rpm_max, 0), (rpm_max, min(speed_max, slope*rpm_max))]
    if bound_top_rpm < rpm_max:
        safe.append((bound_top_rpm, speed_max))
    safe.append((0, 0))
    polygon = ' '.join(f'{xy(r,v)[0]:.2f},{xy(r,v)[1]:.2f}' for r,v in safe)
    bound_end = xy(min(rpm_max, bound_top_rpm), min(speed_max, slope*rpm_max))
    cx, cy = xy(rpm, speed)
    # Saved signed reserve controls the point color; no feasibility is decided here.
    margin = values.get("feed_margin", math.nan)
    color = "#b74335" if margin < 0 else "#176d71"
    pieces = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1140 650" role="img" aria-label="Drum RPM versus feed speed with no-overfeed boundary">',
        '<rect width="1140" height="650" rx="14" fill="#f7fafc"/>',
        '<g font-family="sans-serif" fill="#203249">',
        '<text x="30" y="36" font-size="25" font-weight="700">Feed speed and drum RPM</text>',
        '<text x="30" y="65" font-size="17">Forward advance per scoop ≤ inlet depth · an uncalibrated feed screen</text>',
        f'<rect x="{left:g}" y="{top:g}" width="{width:g}" height="{height:g}" fill="#f5e4df"/>',
        f'<polygon points="{polygon}" fill="#d9ebe4" data-region="feed-screen-passes"/>',
    ]
    for fraction in (0., .25, .5, .75, 1.):
        x = left+fraction*width
        y = top+height-fraction*height
        pieces += [
            f'<line x1="{x:g}" y1="{top:g}" x2="{x:g}" y2="{top+height:g}" stroke="#ffffff" stroke-width="1"/>',
            f'<line x1="{left:g}" y1="{y:g}" x2="{left+width:g}" y2="{y:g}" stroke="#ffffff" stroke-width="1"/>',
            f'<text x="{x:g}" y="456" text-anchor="middle" font-size="15">{fraction*rpm_max:g}</text>',
            f'<text x="85" y="{y+5:g}" text-anchor="end" font-size="15">{fraction*speed_max*1000:g}</text>',
        ]
    pieces += [
        f'<path d="M{left:g} {top:g} V{top+height:g} H{left+width:g}" fill="none" stroke="#536579" stroke-width="2"/>',
        f'<line x1="{left:g}" y1="{top+height:g}" x2="{bound_end[0]:.2f}" y2="{bound_end[1]:.2f}" stroke="#176d71" stroke-width="4" data-limit="inlet_depth*scoops_per_revolution*rpm/60"/>',
        f'<line x1="{cx:.2f}" y1="{top+height:g}" x2="{cx:.2f}" y2="{cy:.2f}" stroke="{color}" stroke-dasharray="5 4"/>',
        f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="7" fill="{color}" stroke="white" stroke-width="2" data-current-rpm="{rpm:.8g}" data-current-feed="{speed:.8g}"/>',
        '<text x="570" y="484" text-anchor="middle" font-size="17">Drum RPM</text>',
        '<text x="24" y="270" transform="rotate(-90 24 270)" text-anchor="middle" font-size="17">Feed speed (mm/s)</text>',
        f'<text x="30" y="518" font-size="16">Current: {rpm:.2f} rpm, {speed*1000:.2f} mm/s</text>',
        f'<text x="30" y="542" font-size="16">Forward advance: {values["feed_per_scoop"]*1000:.2f} mm/scoop · inlet depth: {bite*1000:.2f} mm</text>',
        f'<text x="30" y="572" font-size="16">Maximum feed = inlet_depth × scoops/revolution × RPM / 60; at this RPM: {values["feed_speed_limit"]*1000:.2f} mm/s.</text>',
        '<text x="30" y="600" font-size="15">Green: passes this feed screen. Red: excessive advance per scoop. Motor, reach and traction checks still apply.</text>',
        '<text x="30" y="627" font-size="15">This volume/bite constraint does not establish measured capture, cutting loads or high-speed soil retention.</text>',
        '</g></svg>',
    ]
    return ''.join(pieces)
