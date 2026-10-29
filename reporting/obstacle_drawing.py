"""One declared wheel-on-rock pose, plus its roll-free clearance section."""
import math
from html import escape
from rover.alternatives.parallel_cylinder_clearance import cylinder_clearance_cases, chassis_point_in_world
from .frame_members import chassis_members, rocker_bearing_span


def obstacle_view_depth(point):
    """Larger is nearer the fixed oblique camera.

    Cross product of screen-right (.88,.45,0) and screen-up (-.12,.20,1)
    gives (.45,-.88,.23), toward the viewer above the negative-y side.
    """
    x,y,z = point
    return .45*x-.88*y+.23*z


def obstacle_pose_svg(values):
    geometry = cylinder_clearance_cases(values)
    case = geometry['cases'][0]  # explicit front-left example; all four are checked
    pose, requested = case['pose'], geometry['requested_pose']
    rw, rd = values['wheel_diameter']/2, values['drum_diameter']/2
    scale = 275/max(1,abs(values['travel_x'])+rd)
    def project(p):
        x,y,z = p
        return 365+scale*(.88*x+.45*y), 475-scale*(z+.20*y-.12*x)
    def points(ps):
        return ' '.join(f'{x:.2f},{y:.2f}' for x,y in map(project,ps))
    def line(a,b,color,width=3,dash=''):
        x,y=project(a); u,v=project(b)
        return f'<line x1="{x:.2f}" y1="{y:.2f}" x2="{u:.2f}" y2="{v:.2f}" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>'
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 680" role="img" aria-label="Front-left wheel on rock and parallel-cylinder clearance section">',
             '<rect width="1200" height="680" rx="12" fill="#f7fafc"/><g font-family="sans-serif" fill="#203249">',
             '<text x="25" y="34" font-size="24" font-weight="700">One wheel on the rock / travel pose</text>',
             f'<text x="25" y="64" font-size="15">Front-left raised: {pose["used_height"]*1000:.1f} mm · chassis pitch {math.degrees(pose["pitch"]):.1f}° · roll {math.degrees(pose["roll"]):.1f}°</text>']
    if not requested['valid']:
        parts.append(f'<text x="25" y="91" font-size="15" fill="#a33323">UNREACHABLE requested {values["obstacle_height"]*1000:.1f} mm — guarded illustration only</text>')
    else:
        parts.append('<text x="25" y="91" font-size="14">Same kinematic pose as the model; arms held in travel position.</text>')
    bound = max(abs(values['travel_x'])+rd+.12,values['wheelbase']/2+rw+.12)
    ground = [(-bound,-.5,0),(bound,-.5,0),(bound,.5,0),(-bound,.5,0)]
    parts.append(f'<polygon points="{points(ground)}" fill="#e9e4dc" stroke="#b5aa98"/>')
    x,y,h = pose['contacts'][0]
    b = max(rw*.75,.06)  # illustrative support width, not a modeled rock dimension
    top = [(x-b,y-b,h),(x+b,y-b,h),(x+b,y+b,h),(x-b,y+b,h)]
    for face,fill in (([top[0],top[1],(x+b,y-b,0),(x-b,y-b,0)],'#a69987'),
                      ([top[1],top[2],(x+b,y+b,0),(x+b,y-b,0)],'#b8ad9d'),(top,'#c8beaf')):
        parts.append(f'<polygon data-part="rock" points="{points(face)}" fill="{fill}" stroke="#897d6b"/>')
    cr,sr = math.cos(pose['roll']),math.sin(pose['roll'])
    origin = pose['shaft_origin']
    layers = []
    def layer(kind, coordinates, drawing, identity=''):
        center = tuple(sum(p[i] for p in coordinates)/len(coordinates) for i in range(3))
        depth = obstacle_view_depth(center)
        layers.append((depth, f'<g data-part="{kind}" data-object="{identity}" data-depth="{depth:.9f}">{drawing}</g>'))
    def mounted(p):
        return chassis_point_in_world(p,pose,rw+values['rocker_pivot_offset'])
    span = rocker_bearing_span(values) if 'rocker_box_width' in values else values['track_width']
    shafts = []
    for side in (-1,1):
        shaft = (origin[0],origin[1]+side*span/2*cr,
                 origin[2]+side*span/2*sr)
        shafts.append(shaft)
        for wi in ([0,2] if side==1 else [1,3]):
            center = pose['centers'][wi]
            overhang = side*(values['track_width']-span)/2
            bearing = (center[0],center[1]-overhang*cr,center[2]-overhang*sr)
            layer('rocker',(shaft,bearing),line(shaft,bearing,'#668095',7))
            layer('wheel-axle',(bearing,center),line(bearing,center,'#526a7b',3))
    layer('rocker-axle',shafts,line(*shafts,'#394f63',max(4,values.get('axle_outer_diameter',0)*scale*.7)))
    plate_y = values.get('cf_tube_span',values['track_width'])/2
    for side in (-1,1):
        a = mounted((-values['pivot_x'],side*plate_y,values['bottom_pivot_height']))
        b = mounted((values['pivot_x'],side*plate_y,values['bottom_pivot_height']))
        layer('frame-plate',(a,b),line(a,b,'#7898b0',6))
    for member in chassis_members(values):
        a,b = mounted(member['start']),mounted(member['end'])
        color,width = {'arm':('#246f99',7),'actuator':('#d77422',4),
                       'cf-connector':('#263f49',5), 'offset-bracket':('#246f99',4)}[member['kind']]
        layer(member['kind'],(a,b),line(a,b,color,width),member['kind']+'-'+member['station'])
    for sign in (-1,1):
        a = mounted((sign*values['travel_x'],-values['drum_length']-values['drum_pair_gap']/2,values['travel_y']))
        b = mounted((sign*values['travel_x'],values['drum_length']+values['drum_pair_gap']/2,values['travel_y']))
        layer('drum-shaft',(a,b),line(a,b,'#a54b1e',3))
    def cylinder(center,radius,length,kind):
        rings=[]
        for side in (-1,1):
            cap=[]
            for i in range(41):
                a=2*math.pi*i/40
                cap.append((center[0]+radius*math.cos(a),
                            center[1]+side*length/2*cr-radius*math.sin(a)*sr,
                            center[2]+side*length/2*sr+radius*math.sin(a)*cr))
            rings.append(cap)
        fill,side_fill,stroke = ('#cbd6df','#9bafbd','#354657') if kind=='wheel' else ('#e6b68e','#c48b54','#a54b1e')
        faces = []
        # Opaque visible caps and barrel faces prevent far outlines bleeding
        # through a nearer wheel/drum. Cull faces pointing away from this POV.
        for sign,cap in zip((-1,1),rings):
            if obstacle_view_depth((0,sign*cr,sign*sr)) > 0:
                cap_center = (center[0],center[1]+sign*length/2*cr,center[2]+sign*length/2*sr)
                faces.append((obstacle_view_depth(cap_center),
                              f'<polygon data-surface="cap" points="{points(cap)}" fill="{fill}" stroke="{stroke}" stroke-width="2"/>'))
        for i in range(40):
            angle = 2*math.pi*(i+.5)/40
            normal = (math.cos(angle),-math.sin(angle)*sr,math.sin(angle)*cr)
            if obstacle_view_depth(normal) <= 0:
                continue
            face = [rings[0][i],rings[0][i+1],rings[1][i+1],rings[1][i]]
            midpoint = tuple(sum(p[j] for p in face)/4 for j in range(3))
            faces.append((obstacle_view_depth(midpoint),
                          f'<polygon data-surface="barrel" points="{points(face)}" fill="{side_fill}" stroke="{side_fill}" stroke-width=".6" stroke-linejoin="round"/>'))
        return ''.join(svg for _,svg in sorted(faces,key=lambda item:item[0]))
    bodies = [(c,rw,values['wheel_width'],'wheel') for c in case['wheels']]
    bodies += [(c,rd,values['drum_length'],'drum') for c in case['drums']]
    for i,body in enumerate(bodies):
        layer(body[3],(body[0],),cylinder(*body),body[3]+'-'+str(i%4))
    # Paint all rover members and bodies from far to near in the actual camera
    # basis, rather than sorting global y in the opposite visibility order.
    parts.extend(svg for _,svg in sorted(layers,key=lambda item:item[0]))
    for wi,(center,contact) in enumerate(zip(case['wheels'],pose['contacts'])):
        px,py=project(contact)
        parts.append(f'<circle data-contact="{wi}" cx="{px:.2f}" cy="{py:.2f}" r="4" fill="#a33323"/>')
        parts.append(f'<text x="{px:.2f}" y="{py+18:.2f}" font-size="13" text-anchor="middle">{("FL","FR","RL","RR")[wi]}</text>')
    parts.append('<line x1="730" y1="118" x2="730" y2="564" stroke="#d8e1e7"/>')
    worst=case['worst']
    d=case['unrolled_drums'][worst['drum']]; w=case['unrolled_wheels'][worst['wheel']]
    lo=min(d[0]-rd,w[0]-rw); hi=max(d[0]+rd,w[0]+rw)
    zl=min(d[2]-rd,w[2]-rw); zh=max(d[2]+rd,w[2]+rw)
    section_scale=min(360/max(hi-lo,.01),230/max(zh-zl,.01))
    for center,radius,fill,stroke in ((w,rw,'#cbd6df','#354657'),(d,rd,'#e6b68e','#a54b1e')):
        px=965+(center[0]-(lo+hi)/2)*section_scale
        py=320-(center[2]-(zl+zh)/2)*section_scale
        parts.append(f'<circle data-part="section" cx="{px:.2f}" cy="{py:.2f}" r="{radius*section_scale:.2f}" fill="{fill}" fill-opacity=".6" stroke="{stroke}" stroke-width="2"/>')
    lines = ['Common roll removed',
             f'Closest cylinder pair: drum {worst["drum"]+1} / {("FL","FR","RL","RR")[worst["wheel"]]}',
             f'Radial circle gap: {worst["radial_gap"]*1000:+.2f} mm',
             f'Axial end gap: {worst["axial_gap"]*1000:+.2f} mm',
             f'Finite-cylinder gap: {worst["gap"]*1000:+.2f} mm',
             f'Required clearance: {values["mechanism_clearance"]*1000:.2f} mm']
    for i,label in enumerate(lines):
        yy=142+i*25 if i<2 else 475+(i-2)*24
        parts.append(f'<text x="755" y="{yy}" font-size="{17 if i==0 else 14}">{escape(label)}</text>')
    parts += ['<text x="25" y="586" font-size="14">One centered arm/actuator per drum pair · blue: arms · orange: actuators · charcoal: CF rods · gray: wheels · tan: drums.</text>',
              '<text x="25" y="612" font-size="14">Support block shows the prescribed height; its shape and width are illustrative. Actual rock climbing is not simulated.</text>',
              '<text x="25" y="638" font-size="14">Contact model omits camber effects. The right section also needs the axial-width check; circles alone can over-reject.</text>',
              '<text x="25" y="663" font-size="14">Parallel-cylinder clearance is the default model. This section shows the closest pair for the illustrated front-left raise.</text>',
              '</g></svg>']
    return ''.join(parts)
