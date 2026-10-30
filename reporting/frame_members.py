"""Drawing stations only; no sizing, mass or collision physics is added."""


def chassis_members(values):
    """One centered arm/actuator per front/rear drum pair, plus four aluminum cross-tubes.

    Coordinates are the existing level-chassis x/y/z stations. Arm width comes
    from the section; actuator line thickness remains a schematic style.
    """
    from rover.linkage import attachment_point, offset_values
    side, top = offset_values(values)
    members = []
    for sign, station in ((-1, 'rear'), (1, 'front')):
        base = (sign*values['pivot_x'], 0., values['bottom_pivot_height'])
        end = (sign*values['travel_x'], 0., values['travel_y'])
        x,z = attachment_point(values['pivot_x'],values['bottom_pivot_height'], values['travel_x'],values['travel_y'],side,top)
        pin = (sign*x,0.,z)
        ex,ez = attachment_point(values['pivot_x'],values['bottom_pivot_height'], values['travel_x'],values['travel_y'],max(side,0.),0.)
        qx,qz = attachment_point(values['pivot_x'],values['bottom_pivot_height'], values['travel_x'],values['travel_y'],side,0.)
        arm_end = (sign*ex,0.,ez)
        upper = (sign*values['actuator_pivot_x'], 0., values['top_pivot_height'])
        members += [dict(kind='arm', station=station, start=base, end=arm_end,
                         width=values.get('arm_box_width', values.get('arm_tube_diameter'))),
                    dict(kind='actuator', station=station, start=upper, end=pin, width=None)]
        if top != 0:
            members.append(dict(kind='offset-bracket',station=station,start=(sign*qx,0.,qz),end=pin,width=None))
        if 'cross_tube_span' in values:
            half = values['cross_tube_span']/2
            od = values['cross_tube_inner_diameter']+2*values['cross_tube_wall']
            for level, point in (('upper', upper), ('lower', base)):
                x,_,z = point
                members.append(dict(kind='cross-tube', station=station+'-'+level,
                                    start=(x,-half,z), end=(x,half,z), width=od))
    return members


def rocker_bearing_span(values):
    """Existing plate-frame bearing-center spacing, not wheel-center track."""
    return (values['track_width']-values['wheel_width']-2*values['wheel_rocker_gap']
            -values['rocker_box_width'])
