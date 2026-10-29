"""Three loading states, all inertias about the chassis-center yaw axis.

The half-loaded state retains the front pair. Positions, cylinder inertias and
parallel-axis terms replace a uniform wheelbase-sized box. CAD can replace this
component without changing mobility or relocation. Coordinate axes: x forward,
y left, z above the original level ground (linkage.py calls z 'y').
"""
from .components import scalar
from .alternatives.offset_attachment import attachment_point
from .discipline import Discipline, names


def mass_properties(parts):
    """parts = (mass, x, y, z, local_Izz); returns mass, CGx, CGz, Izz."""
    total = sum(p[0] for p in parts)
    return (total, sum(m*x for m,x,y,z,i in parts)/total,
            sum(m*z for m,x,y,z,i in parts)/total,
            sum(i+m*(x*x+y*y) for m,x,y,z,i in parts))


def tail_part(v, sign):
    """Actual uniform axial continuation; excluded from the core linkage lump."""
    extra = v("arm_extension_mass")/2
    if extra == 0: return None
    ratio = max(v("lin_act_side_arm_offset"), 0.)/v("arm_length")
    dx = ratio*(v("travel_x")-v("pivot_x"))
    dz = ratio*(v("travel_y")-v("bottom_pivot_height"))
    return (extra, sign*(v("travel_x")+dx/2), 0., v("travel_y")+dz/2, extra*dx*dx/12)


class MassProperties(Discipline):
    CONFIG_INPUTS = tuple(sorted(names("wheelbase track_width wheel_diameter wheel_width drum_diameter drum_length drum_pair_gap fixed_mass chassis_height pivot_x actuator_pivot_x bottom_pivot_height top_pivot_height lin_act_side_arm_offset lin_act_top_arm_offset arm_length")))
    COUPLED_INPUTS = {"wheel_mass": "kg", "drive_mass": "kg", "drum_mass": "kg", "battery_mass": "kg", "structure_mass": "kg", "rocker_mass": "kg", "linkage_mass": "kg", "arm_extension_mass": "kg", "payload_mass": "kg", "travel_x": "m", "travel_y": "m"}
    OUTPUT_UNITS = {f"{state}_{quantity}": units for state in ("empty", "loaded", "half")
                    for quantity,units in (("yaw_inertia","kg*m**2"),("cg_x","m"),("cg_z","m"))}
    OUTPUT_UNITS.update({"loaded_sprung_mass": "kg", "loaded_sprung_cg_x": "m", "loaded_sprung_cg_z": "m"})
    dry = set(CONFIG_INPUTS) | set(COUPLED_INPUTS) - {"payload_mass"}
    DEPENDENCIES = {}
    for state in ("empty", "loaded", "half"):
        for quantity in ("yaw_inertia", "cg_x", "cg_z"):
            DEPENDENCIES[f"{state}_{quantity}"] = dry | ({"payload_mass"} if state != "empty" else set())
    for quantity in ("mass", "cg_x", "cg_z"):
        DEPENDENCIES["loaded_sprung_"+quantity] = set(CONFIG_INPUTS) | set(COUPLED_INPUTS)

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        wb, track, rw, rd, length = v("wheelbase"), v("track_width"), v("wheel_diameter")/2, v("drum_diameter")/2, v("drum_length")
        parts = []
        sprung = []
        for sx in (-1,1):
            for sy in (-1,1):
                mw = v("wheel_mass")/4
                parts.append((mw, sx*wb/2, sy*track/2, rw, mw*(3*rw**2+v("wheel_width")**2)/12))
                parts.append((v("drive_mass")/4, sx*wb/2, sy*track/2, rw, 0))
                md = v("drum_mass")/4
                parts.append((md, sx*v("travel_x"), sy*(length+v("drum_pair_gap"))/2, v("travel_y"), md*(3*rd**2+length**2)/12))
                sprung.append(parts[-1])
            tail = tail_part(v, sx)
            if tail is not None:
                parts.append(tail); sprung.append(tail)
            ml = (v("linkage_mass")-v("arm_extension_mass"))/2
            # Equivalent midpoint of arm and actuator links. Their actual masses
            # differ, so CAD mass properties should replace this approximation.
            pivot_x = (v("pivot_x")+v("actuator_pivot_x"))/2
            pivot_z = (v("bottom_pivot_height")+v("top_pivot_height"))/2
            joint_x, joint_z = attachment_point(v("pivot_x"), v("bottom_pivot_height"), v("travel_x"), v("travel_y"), v("lin_act_side_arm_offset"), v("lin_act_top_arm_offset"))
            # Mean of the arm and actuator endpoints, retaining the reference
            # equivalent-rod approximation and its mass boundary.
            end_x, end_z = (v("travel_x")+joint_x)/2, (v("travel_y")+joint_z)/2
            parts.append((ml, sx*(pivot_x+end_x)/2, 0,
                          (pivot_z+end_z)/2,
                          ml*(end_x-pivot_x)**2/12))
            sprung.append(parts[-1])
        # Frame distributed over support rectangle; central equipment at half height.
        mc = v("fixed_mass")+v("battery_mass")+v("structure_mass")
        parts.append((mc, 0, 0, rw+v("chassis_height")/2, mc*(wb**2+track**2)/12))
        # Existing level-ground CG outputs retain their reference lumped frame.
        # Obstacle posing removes the two uniform rockers from the mounted group.
        # They are placed at their actual wheel-center midpoints by that model.
        if not 0 <= v("rocker_mass") <= v("structure_mass"):
            raise ValueError("rocker_mass must be a nonnegative subset of structure_mass.")
        sprung.append((mc-v("rocker_mass"),0,0,rw+v("chassis_height")/2,0))
        for state in ("empty", "loaded", "half"):
            load = list(parts)
            body_load = list(sprung)
            if state != "empty":
                for sx in ((-1,1) if state == "loaded" else (1,)):
                    for sy in (-1,1):
                        m = v("payload_mass")/4
                        load.append((m, sx*v("travel_x"), sy*(length+v("drum_pair_gap"))/2,
                                     v("travel_y")-rd/3, m*(3*rd**2+length**2)/12))
                        body_load.append(load[-1])
            _, x, z, inertia = mass_properties(load)
            outputs[state+"_cg_x"], outputs[state+"_cg_z"], outputs[state+"_yaw_inertia"] = x, z, inertia
            if state == "loaded":
                m,x,z,_ = mass_properties(body_load)
                outputs["loaded_sprung_mass"],outputs["loaded_sprung_cg_x"],outputs["loaded_sprung_cg_z"] = m,x,z
