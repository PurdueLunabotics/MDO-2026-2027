"""Drawing stations only; no sizing, mass or collision physics is added."""


def chassis_members(values):
    """One centered arm/actuator per front/rear drum pair, plus four CF rods.

    Coordinates are the existing level-chassis x/y/z stations. Arm width comes
    from the section; actuator line thickness remains a schematic style.
    """
    members = []
    for sign, station in ((-1, 'rear'), (1, 'front')):
        base = (sign*values['pivot_x'], 0., values['bottom_pivot_height'])
        end = (sign*values['travel_x'], 0., values['travel_y'])
        fraction = values.get('actuator_attachment_fraction', 1.)
        pin = tuple(a+fraction*(b-a) for a,b in zip(base,end))
        upper = (sign*values['actuator_pivot_x'], 0., values['top_pivot_height'])
        members += [dict(kind='arm', station=station, start=base, end=end,
                         width=values.get('arm_box_width', values.get('arm_tube_diameter'))),
                    dict(kind='actuator', station=station, start=upper, end=pin, width=None)]
        if 'cf_tube_span' in values:
            half = values['cf_tube_span']/2
            od = values['cf_tube_inner_diameter']+2*values['cf_tube_wall']
            for level, point in (('upper', upper), ('lower', base)):
                x,_,z = point
                members.append(dict(kind='cf-connector', station=station+'-'+level,
                                    start=(x,-half,z), end=(x,half,z), width=od))
    return members


def rocker_bearing_span(values):
    """Existing plate-frame bearing-center spacing, not wheel-center track."""
    return (values['track_width']-values['wheel_width']-2*values['wheel_rocker_gap']
            -values['rocker_box_width'])
