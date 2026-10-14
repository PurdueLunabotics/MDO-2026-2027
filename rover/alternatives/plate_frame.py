"""Preliminary loaded-gravity plate/CF/box-rocker frame, selected via ModelSet.

Two monolithic plates are approximated by identical axial trusses: four radial
strips plus four perimeter strips. This is an explicitly provisional topology,
not a reconstruction of CAD cutouts. Central pins release in-plane rotation;
only balanced, mirrored fully loaded gravity cases are supported. Plate bending,
whole-plate buckling, joints, CF local buckling and recovery/cutting are omitted.
Pure helpers below do all physics; adapters only move scalar ports.
"""
import math
import numpy as np

from ..components import scalar
from ..discipline import Discipline, names
from ..structure import StructureGeometry, StructureLoads, tube_section
from ..linkage import sweep_poses, drum_pose
from ..mass_properties import MassProperties, mass_properties
from .obstacle_stability import ObstacleStability, raised_wheel_pose, posed_loaded_cg, projected_support_margin


# S, front upper/lower, rear upper/lower. Connectivity survives moving mounts.
PLATE_EDGES = ((0, 1), (0, 2), (0, 3), (0, 4),
               (1, 2), (3, 4), (1, 3), (2, 4))


def box_section(width, height, wall):
    """Rectangular hollow section: area, strong/weak inertias [m2,m4,m4]."""
    if (not all(math.isfinite(x) and x > 0 for x in (width, height, wall))
            or 2*wall >= min(width, height)):
        raise ValueError("Box section needs positive dimensions and a hollow interior.")
    wi, hi = width-2*wall, height-2*wall
    return (width*height-wi*hi,
            (width*height**3-wi*hi**3)/12,
            (height*width**3-hi*wi**3)/12)


def plate_nodes(values):
    """Mount-center geometry in x/z; axle stays at wheelbase midpoint."""
    v = values.__getitem__
    return np.array([(0, v("wheel_diameter")/2+v("rocker_pivot_offset")),
                     (v("actuator_pivot_x"), v("top_pivot_height")),
                     (v("pivot_x"), v("bottom_pivot_height")),
                     (-v("actuator_pivot_x"), v("top_pivot_height")),
                     (-v("pivot_x"), v("bottom_pivot_height"))], dtype=float)


def plate_geometry(values):
    """Inventory plus signed geometry reserves; no clipping into feasibility."""
    v = values.__getitem__
    for name in GEOMETRY_INPUTS:
        value = v(name)
        if not math.isfinite(value) or (name != "rocker_pivot_offset" and value <= 0):
            raise ValueError(f"Invalid frame geometry/material input {name}.")
    nodes = plate_nodes(values)
    lengths = np.array([np.linalg.norm(nodes[j]-nodes[i]) for i, j in PLATE_EDGES])
    if not np.isfinite(nodes).all() or min(lengths) <= 1e-9:
        raise ValueError("Degenerate plate mounting geometry.")
    w, t, hub = v("plate_spoke_width"), v("plate_thickness"), v("plate_hub_radius")
    od = v("cf_tube_inner_diameter")+2*v("cf_tube_wall")
    mount = od/2+v("frame_mount_clearance")
    # Four triangular cutouts: each remaining ligament has common width w.
    inradii = []
    for a, b in ((1, 2), (2, 4), (4, 3), (3, 1)):
        u, q = nodes[a]-nodes[0], nodes[b]-nodes[0]
        twice_area = abs(u[0]*q[1]-u[1]*q[0])
        perimeter = np.linalg.norm(u)+np.linalg.norm(q)+np.linalg.norm(nodes[a]-nodes[b])
        inradii.append(twice_area/perimeter)
    rocker_span = v("track_width")-v("wheel_width")-2*v("wheel_rocker_gap")-v("rocker_box_width")
    reserves = [min(inradii)-w/2-v("plate_cutout_clearance"),
                nodes[0, 1]-nodes[2, 1]-hub-w/2,
                nodes[1, 1]-nodes[0, 1]-hub-w/2,
                min(lengths[:4])-hub-mount-w/2,
                min(lengths[4:])-2*mount-w,
                min(nodes[2, 1], nodes[4, 1])-mount]
    packing = (rocker_span-v("cf_tube_span")-t)/2-v("minimum_axle_overhang")
    # Sum-of-strips inventory deliberately retains junction overlap; add a hub
    # disk. Conservative mass inventory, not exact boolean-union CAD volume.
    plate_mass = v("tube_density")*t*(w*sum(lengths)+math.pi*hub**2)
    return {"nodes": nodes, "lengths": lengths, "plate_mass": plate_mass,
            "rocker_span": rocker_span, "cf_od": od,
            "geometry_margin": float(min(reserves)), "packing_margin": packing,
            "valid": min(*reserves, packing) >= 0}


def geometry_inventory(values):
    v = values.__getitem__
    g = plate_geometry(values)
    ar, _, _ = box_section(v("rocker_box_width"), v("rocker_box_height"), v("rocker_wall"))
    aa, _, _ = tube_section(v("arm_tube_diameter"), v("arm_wall"))
    ax, _, _ = tube_section(v("axle_outer_diameter"), v("axle_wall"))
    ac, _, _ = tube_section(g["cf_od"], v("cf_tube_wall"))
    half = math.hypot(v("wheelbase")/2, v("rocker_pivot_offset"))
    rocker_mass = 4*half*ar*v("tube_density")
    cf_mass = 4*v("cf_tube_span")*ac*v("cf_density")
    axle_mass = max(g["rocker_span"], 1e-6)*ax*v("tube_density")
    sprung_mass = 2*g["plate_mass"]+cf_mass+axle_mass+v("joint_mass")+v("differential_mass")
    shaft_z = g["nodes"][0, 1]
    strip_mass = v("tube_density")*v("plate_thickness")*v("plate_spoke_width")*g["lengths"]
    strip_mids = np.array([(g["nodes"][i]+g["nodes"][j])/2 for i, j in PLATE_EDGES])
    hub_mass = g["plate_mass"]-sum(strip_mass)
    plate_mz = float(strip_mass @ strip_mids[:, 1])+hub_mass*shaft_z
    cf_z = (v("top_pivot_height")+v("bottom_pivot_height"))/2
    sprung_mz = 2*plate_mz+cf_mass*cf_z+(axle_mass+v("joint_mass")+v("differential_mass"))*shaft_z
    rocker_z = v("wheel_diameter")/2+v("rocker_pivot_offset")/2
    # Line-member mass moments; joints/differential are lumps at shaft origin.
    # Plate widths' in-plane local inertia w2/12 is an inventory approximation.
    plate_izz = 0.0
    for mass, (i, j), length in zip(strip_mass, PLATE_EDGES, g["lengths"]):
        xi, xj = g["nodes"][i, 0], g["nodes"][j, 0]
        plate_izz += mass*((xi*xi+xi*xj+xj*xj)/3+v("cf_tube_span")**2/4+v("plate_spoke_width")**2/12)
    plate_izz += hub_mass*(v("cf_tube_span")**2/4+v("plate_hub_radius")**2/4)
    cf_izz = cf_mass*(v("cf_tube_span")**2/12+(v("actuator_pivot_x")**2+v("pivot_x")**2)/2)
    rocker_izz = rocker_mass*(v("wheelbase")**2/12+g["rocker_span"]**2/4+v("rocker_box_width")**2/12)
    total = sprung_mass+rocker_mass
    return {**g, "structure_mass": total, "rocker_mass": rocker_mass,
            "arm_linear_mass": aa*v("tube_density"),
            "plate_mass_total": 2*g["plate_mass"], "cf_tube_mass_total": cf_mass,
            "suspension_axle_mass": axle_mass, "suspension_bearing_span": g["rocker_span"],
            "cf_tube_outer_diameter": g["cf_od"],
            "plate_geometry_margin": g["geometry_margin"], "frame_packing_margin": g["packing_margin"],
            "plate_geometry_valid": float(g["valid"]),
            "structure_cg_z": (sprung_mz+rocker_mass*rocker_z)/total,
            "structure_sprung_cg_z": sprung_mz/sprung_mass,
            "structure_yaw_inertia": 2*plate_izz+cf_izz+axle_mass*g["rocker_span"]**2/12+rocker_izz,
            "frame_section_margin": min(min(v("rocker_box_width"), v("rocker_box_height"))/4-v("rocker_wall"),
                                         v("axle_outer_diameter")/4-v("axle_wall")),
            "arm_section_margin": v("arm_tube_diameter")/4-v("arm_wall")}


def arm_mount_loads(bottom, top, drum, pair_weight, arm_weight, fraction):
    """Forces ON FRAME at A/B from a gravity-loaded arm; weights are N.

    Pin at D transmits force along D-A. Include uniform arm self-weight at its
    midpoint. No breakout/cutting/recovery forces. Returns force on arm at D
    as well, allowing equilibrium and member-bending checks at the same pose.
    """
    b, a, c = (np.asarray(p, dtype=float) for p in (bottom, top, drum))
    if (any(p.shape != (2,) or not np.isfinite(p).all() for p in (a, b, c))
            or not all(math.isfinite(x) for x in (pair_weight, arm_weight, fraction))
            or pair_weight < 0 or arm_weight < 0 or not 0 < fraction <= 1):
        raise ValueError("Invalid arm load/geometry.")
    r = c-b
    direction = a-(b+fraction*r)
    distance = np.linalg.norm(direction)
    if distance <= 1e-9 or np.linalg.norm(r) <= 1e-9:
        raise ValueError("Coincident arm/actuator joints.")
    u = direction/distance
    lever = fraction*(r[0]*u[1]-r[1]*u[0])
    if abs(lever) <= 1e-9:
        raise ValueError("Actuator toggle cannot balance loaded arm moment.")
    actuator = u*((pair_weight+arm_weight/2)*r[0]/lever)
    weight = np.array([0., -pair_weight-arm_weight])
    return -actuator, actuator+weight, actuator


def solve_plate(nodes, forces, width, thickness, modulus):
    """Axial elastic truss, central PIN, balanced loads only; batch RHS allowed.

    An exact rotation gauge removes the free rigid-body rotation, without adding
    physical stiffness. Nonzero external moment about S is rejected. The gauge
    reaction must be zero. Additional mechanisms are rejected, never stabilized
    with fictitious springs. Returns signed axial forces, translations, reaction.
    """
    p, f = np.asarray(nodes, dtype=float), np.asarray(forces, dtype=float)
    if p.shape != (5, 2) or f.shape[:2] != (5, 2) or f.ndim not in (2, 3):
        raise ValueError("Plate requires five 2D nodes and matching nodal forces.")
    if (not np.isfinite(p).all() or not np.isfinite(f).all()
            or not all(math.isfinite(x) and x > 0 for x in (width, thickness, modulus))):
        raise ValueError("Nonfinite/invalid plate data.")
    batch = f.ndim == 3
    rhs = f.reshape(10, -1)
    rel = p-p[0]
    moment = (-rel[:, 1]) @ f[:, 0]+rel[:, 0] @ f[:, 1]
    if np.max(np.abs(moment)) > 1e-8*max(1., float(np.max(np.abs(f)))):
        raise ValueError("Central pin cannot support an unbalanced plate moment.")
    k = np.zeros((10, 10))
    operators, lengths = [], []
    for i, j in PLATE_EDGES:
        d = p[j]-p[i]
        length = np.linalg.norm(d)
        if length <= 1e-9:
            raise ValueError("Zero-length plate member.")
        row = np.zeros(10)
        row[2*i:2*i+2], row[2*j:2*j+2] = -d/length, d/length
        stiffness = modulus*width*thickness/length
        k += stiffness*np.outer(row, row)
        operators.append(stiffness*row)
        lengths.append(length)
    # Scale by E*A for numerical conditioning; gauge carries no load/stiffness.
    scale = modulus*width*thickness
    rotation = np.column_stack((-rel[1:, 1], rel[1:, 0])).reshape(8)
    norm = np.linalg.norm(rotation)
    if norm <= 1e-9:
        raise ValueError("Collapsed plate.")
    rotation /= norm
    augmented = np.zeros((9, 9))
    augmented[:8, :8] = k[2:, 2:]/scale
    augmented[:8, 8], augmented[8, :8] = rotation, rotation
    if np.linalg.cond(augmented) > 1e10:
        raise ValueError("Plate has an additional mechanism/degenerate geometry.")
    target = np.vstack((rhs[2:]/scale, np.zeros((1, rhs.shape[1]))))
    solution = np.linalg.solve(augmented, target)
    if np.max(np.abs(solution[8]*scale)) > 1e-7*max(1., float(np.max(np.abs(f)))):
        raise ValueError("Rotation gauge acquired a physical reaction.")
    displacement = np.vstack((np.zeros((2, rhs.shape[1])), solution[:8]))
    axial = np.asarray(operators) @ displacement
    reaction = (k @ displacement-rhs)[:2]
    if not batch:
        return axial[:, 0], displacement[:, 0].reshape(5, 2), reaction[:, 0]
    return axial, displacement.reshape(5, 2, -1), reaction


def arm_beam_screen(length, fraction, direction, actuator, pair_weight, arm_weight, area, inertia, diameter, modulus):
    """Fixed-root relative-flexure approximation with actual C/D loads.

    Extremum stations are endpoints, pin sides and interior zero-shear points;
    uniform arm self-weight is included. No hidden stress sampling gaps.
    """
    e = np.asarray(direction)/length
    n = np.array([-e[1], e[0]])
    fc = float(np.array([0., -pair_weight]) @ n)
    fa = float(np.asarray(actuator) @ n)
    q = float(np.array([0., -arm_weight/length]) @ n)
    a = fraction*length
    stations = [0., a, length]
    if abs(q) > 1e-12:
        for lo, hi, pin in ((0., a, fa), (a, length, 0.)):
            s = length+(fc+pin)/q
            if lo < s < hi:
                stations.append(s)
    stresses = []
    moments = []
    compression = []
    for s in stations:
        moment = fc*(length-s)+q*(length-s)**2/2+fa*max(a-s, 0.)
        moments.append(abs(moment))
        # Include both sides of the concentrated actuator force at D.
        for include in ((True, False) if s == a else (s < a,)):
            axial = -pair_weight*e[1]-arm_weight*(1-s/length)*e[1]
            if include:
                axial += float(np.asarray(actuator) @ e)
            stresses.append(abs(axial)/area+abs(moment)*diameter/(2*inertia))
            compression.append(max(-axial, 0.))
    tip = (fc*length**3/3+fa*a*a*(3*length-a)/6+q*length**4/8)/(modulus*inertia)
    return max(stresses), abs(tip), max(moments), max(compression)


def evaluate_frame_loads(values):
    """All four drums fully loaded, equal front/rear poses; gravity cases only."""
    v = values.__getitem__
    g = geometry_inventory(values)
    for name in ("gravity", "structure_safety_factor", "structure_load_factor", "tube_youngs_modulus",
                 "tube_yield_strength", "cf_flexural_modulus", "cf_flexural_strength", "plate_buckling_k", "allowable_deflection"):
        if not math.isfinite(v(name)) or v(name) <= 0:
            raise ValueError(f"Invalid frame load/material input {name}.")
    for name in ("dry_mass", "payload_mass", "drum_mass", "wheel_mass", "drive_mass"):
        if not math.isfinite(v(name)) or v(name) < 0:
            raise ValueError(f"Invalid frame mass input {name}.")
    if v("dry_mass") < v("wheel_mass")+v("drive_mass")+g["rocker_mass"]:
        raise ValueError("Dry mass cannot be smaller than its unsprung subset.")
    sf, factor, grav = v("structure_safety_factor"), v("structure_load_factor"), v("gravity")
    e, strength = v("tube_youngs_modulus"), v("tube_yield_strength")
    ac, ic, _ = tube_section(g["cf_od"], v("cf_tube_wall"))
    aa, ia, _ = tube_section(v("arm_tube_diameter"), v("arm_wall"))
    ar, ir, iw = box_section(v("rocker_box_width"), v("rocker_box_height"), v("rocker_wall"))
    ax, ix, _ = tube_section(v("axle_outer_diameter"), v("axle_wall"))
    poses, _ = sweep_poses(values)
    poses = [drum_pose(v("arm_length"), v("pivot_x"), v("bottom_pivot_height"),
                      v("actuator_pivot_x"), v("top_pivot_height"), v("actuator_retracted_length"),
                      v("actuator_attachment_fraction")), *poses]
    pair_weight = (v("drum_mass")+v("payload_mass"))/2*grav*factor
    arm_weight = g["arm_linear_mass"]*v("arm_length")*grav*factor
    b, a = g["nodes"][2], g["nodes"][1]
    loads, actuator_forces = [], []
    cf_stress, arm_stress, arm_deflection, arm_compression, tube_motion = [], [], [], [], []
    span, half = v("cf_tube_span"), v("cf_tube_span")/2
    distributed = np.array([0., -ac*v("cf_density")*grav*factor])
    for c in poses:
        upper, lower, actuator = arm_mount_loads(b, a, c, pair_weight, arm_weight, v("actuator_attachment_fraction"))
        actuator_forces.append(float(np.linalg.norm(actuator)))
        f = np.zeros((5, 2))
        f[1], f[2] = upper/2+distributed*span/2, lower/2+distributed*span/2
        f[3], f[4] = f[1]*[-1, 1], f[2]*[-1, 1]
        # Member self-weight split equally among its endpoints.
        for (i, j), length in zip(PLATE_EDGES, g["lengths"]):
            member_w = v("tube_density")*v("plate_thickness")*v("plate_spoke_width")*length*grav*factor
            f[i, 1] -= member_w/2
            f[j, 1] -= member_w/2
        loads.append(f)
        displacements = []
        for force in (upper, lower):
            # Two half-span cantilever flexure screens, each receiving F/2.
            moment = force*half/2+distributed*half**2/2
            cf_stress.append(np.linalg.norm(moment)*g["cf_od"]/(2*ic))
            displacements.append((force*half**3/6+distributed*half**4/8)/(v("cf_flexural_modulus")*ic))
        tube_motion.append(displacements)
        sigma, delta, _, compressive = arm_beam_screen(v("arm_length"), v("actuator_attachment_fraction"),
                                        np.asarray(c)-b, actuator, pair_weight, arm_weight,
                                        aa, ia, v("arm_tube_diameter"), e)
        arm_stress.append(sigma)
        arm_deflection.append(delta)
        arm_compression.append(compressive)
    forces = np.stack(loads, axis=2)
    axial, displacements, reactions = solve_plate(g["nodes"], forces, v("plate_spoke_width"), v("plate_thickness"), e)
    area = v("plate_spoke_width")*v("plate_thickness")
    plate_stress = float(np.max(np.abs(axial))/area)
    weak = v("plate_spoke_width")*v("plate_thickness")**3/12
    pcr = math.pi**2*e*weak/(v("plate_buckling_k")*g["lengths"])**2
    compression = np.maximum(-axial, 0.)
    buckling = float(np.min(pcr[:, None]/sf-compression))
    tube_motion = np.asarray(tube_motion)
    mount_difference = displacements[1]-displacements[2]+(tube_motion[:, 0]-tube_motion[:, 1]).T
    mount_delta = float(np.max(np.linalg.norm(mount_difference, axis=0)))
    # Net wheel-to-rocker force excludes wheel/drive mass and half rocker mass;
    # distributed self-weight of each sloping half-rocker is handled separately.
    total_weight = (v("dry_mass")+v("payload_mass"))*grav*factor
    wheel_reaction = total_weight/4
    end_force = wheel_reaction-(v("wheel_mass")+v("drive_mass"))/4*grav*factor
    half_length = math.hypot(v("wheelbase")/2, v("rocker_pivot_offset"))
    cos = v("wheelbase")/(2*half_length)
    sin = -v("rocker_pivot_offset")/half_length
    own_weight = g["rocker_mass"]/4*grav*factor
    rocker_moment = (end_force-own_weight/2)*v("wheelbase")/2
    rocker_axial = abs(end_force*sin)+own_weight*abs(sin)
    rocker_sigma = rocker_axial/ar+abs(rocker_moment)*v("rocker_box_height")/(2*ir)
    rocker_delta = abs(end_force*cos*half_length**3/3-own_weight*cos*half_length**3/8)/(e*ir)
    rocker_buckling = math.pi**2*e*min(ir, iw)/(2*half_length)**2/sf-rocker_axial
    arm_buckling = math.pi**2*e*ia/(2*v("arm_length"))**2/sf-max(arm_compression)
    # Axle overhang screen: plate-to-rocker bearing span, not wheel-center span.
    overhang = max((g["rocker_span"]-span)/2, 0.)
    sprung_side_force = max(total_weight-(v("wheel_mass")+v("drive_mass")+g["rocker_mass"])*grav*factor, 0.)/2
    axle_moment = sprung_side_force*overhang
    axle_sigma = axle_moment*v("axle_outer_diameter")/(2*ix)
    axle_delta = sprung_side_force*overhang**3/(3*e*ix)
    stresses = (rocker_sigma, axle_sigma, plate_stress, max(arm_stress))
    # An upper envelope of DIFFERENT local/relative measures, not one physical tip.
    stress = float(sum(s**8 for s in stresses)**(1/8))
    deflection = float(sum(d**8 for d in (rocker_delta, axle_delta, mount_delta, max(arm_deflection)))**(1/8))
    return {"structure_stress": stress, "structure_stress_margin": strength/sf-stress,
            "structure_deflection": deflection, "structure_deflection_margin": v("allowable_deflection")-deflection,
            "structure_buckling_margin": min(buckling, rocker_buckling, arm_buckling), "plate_stress": plate_stress,
            "plate_stress_margin": strength/sf-plate_stress, "plate_buckling_margin": buckling,
            "cf_bending_stress": float(max(cf_stress)),
            "cf_bending_margin": v("cf_flexural_strength")/sf-float(max(cf_stress)),
            "frame_mount_relative_deflection": mount_delta,
            "rocker_buckling_margin": rocker_buckling, "arm_buckling_margin": arm_buckling,
            "rocker_deflection": rocker_delta, "axle_deflection": axle_delta,
            "arm_deflection": float(max(arm_deflection)),
            "frame_gravity_actuator_force": max(actuator_forces),
            "frame_nominal_wheel_reaction": total_weight/(4*factor),
            "frame_force_balance_error": float(np.max(np.abs(reactions+np.sum(forces, axis=0)))),
            "frame_load_solution_valid": 1.0}


GEOMETRY_INPUTS = names("wheelbase track_width wheel_diameter wheel_width rocker_pivot_offset pivot_x actuator_pivot_x top_pivot_height bottom_pivot_height arm_tube_diameter arm_wall tube_density joint_mass differential_mass cf_tube_span cf_tube_wall cf_tube_inner_diameter cf_density plate_spoke_width plate_thickness plate_hub_radius plate_cutout_clearance frame_mount_clearance minimum_axle_overhang wheel_rocker_gap rocker_box_width rocker_box_height rocker_wall axle_outer_diameter axle_wall")
EXTRA_GEOMETRY_OUTPUTS = {"plate_mass_total": "kg", "cf_tube_mass_total": "kg", "suspension_axle_mass": "kg",
                          "suspension_bearing_span": "m", "cf_tube_outer_diameter": "m",
                          "plate_geometry_margin": "m", "frame_packing_margin": "m", "plate_geometry_valid": None,
                          "structure_cg_z": "m", "structure_sprung_cg_z": "m", "structure_yaw_inertia": "kg*m**2"}
EVIDENCE = {"revision": 1, "status": "analytical preliminary frame; NOT hardware validated",
            "hardware": "user: 6.35 mm 6061-T6 plates, 25.4 mm ID CF rods, aluminum box rockers; bonded inserts/bolts and rotating bushings",
            "topology": "PROVISIONAL: four radial and four perimeter strips per plate; verify against parametric CAD",
            "CF": "density/modulus/flexural strength are explicit provisional assumptions; no product/layup selected",
            "loading": "all four drums loaded, mirrored level-ground gravity, sampled compact/travel-to-dig poses; load factor retained",
            "plate_support": "central pin; exact free-rotation gauge, zero artificial reaction; reject unbalanced moments",
            "mass": "strip inventory + hub disk retains overlapping junction material; joint/differential allowances retained",
            "omissions": "whole-plate and local box/CF wall buckling, plate junction bending, joints/inserts, differential torque, fatigue, dynamic/obstacle load redistribution, cutting and self-recovery",
            "deflection": "eighth-norm envelope of rocker-end, axle-overhang, arm-tip relative flexure and upper/lower mount relative displacement; not assembled-frame displacement",
            "buckling": "pinned weak-axis Euler strips; not validated whole-plate buckling capacity"}


class PlateStructureGeometry(Discipline):
    CONFIG_INPUTS = tuple(sorted(GEOMETRY_INPUTS))
    OUTPUT_UNITS = {**StructureGeometry.OUTPUT_UNITS, **EXTRA_GEOMETRY_OUTPUTS}
    DEPENDENCIES = {n: set(GEOMETRY_INPUTS) for n in OUTPUT_UNITS}
    model_evidence = EVIDENCE

    def compute(self, inputs, outputs):
        result = geometry_inventory({n: scalar(inputs, n) for n in inputs})
        for n in self.OUTPUT_UNITS:
            outputs[n] = result[n]


class PlateStructureLoads(Discipline):
    CONFIG_INPUTS = tuple(sorted(GEOMETRY_INPUTS | names("arm_length gravity structure_load_factor structure_safety_factor tube_youngs_modulus tube_yield_strength cf_flexural_modulus cf_flexural_strength plate_buckling_k allowable_deflection actuator_attachment_fraction actuator_retracted_length travel_actuator_length actuator_extended_length")))
    COUPLED_INPUTS = {"dry_mass": "kg", "payload_mass": "kg", "drum_mass": "kg", "wheel_mass": "kg", "drive_mass": "kg"}
    OUTPUT_UNITS = {**StructureLoads.OUTPUT_UNITS,
                    "plate_stress": "Pa", "plate_stress_margin": "Pa", "plate_buckling_margin": "N",
                    "cf_bending_stress": "Pa", "cf_bending_margin": "Pa",
                    "frame_mount_relative_deflection": "m", "rocker_deflection": "m", "axle_deflection": "m", "arm_deflection": "m",
                    "rocker_buckling_margin": "N", "arm_buckling_margin": "N",
                    "frame_gravity_actuator_force": "N", "frame_nominal_wheel_reaction": "N",
                    "frame_force_balance_error": "N", "frame_load_solution_valid": None}
    DEPENDENCIES = dict.fromkeys(OUTPUT_UNITS, set(CONFIG_INPUTS) | set(COUPLED_INPUTS))
    model_evidence = EVIDENCE

    def compute(self, inputs, outputs):
        values = {n: scalar(inputs, n) for n in inputs}
        try:
            result = evaluate_frame_loads(values)
        except (ValueError, np.linalg.LinAlgError):
            # Explicit failed arithmetic guard for driver trials at mechanisms or
            # toggles. Never substitute a different feasible frame/physics model.
            result = {n: 0. for n in self.OUTPUT_UNITS}
            for n in ("structure_stress", "plate_stress", "cf_bending_stress"):
                result[n] = 1e12
            for n in ("structure_stress_margin", "plate_stress_margin", "cf_bending_margin"):
                result[n] = -1e12
            result.update(structure_deflection=1e3, structure_deflection_margin=-1e3,
                          structure_buckling_margin=-1e9, plate_buckling_margin=-1e9)
        for n in self.OUTPUT_UNITS:
            outputs[n] = result[n]


class PlateMassProperties(Discipline):
    """Reference mass boundaries, with plate-profile frame mass placement."""
    CONFIG_INPUTS = MassProperties.CONFIG_INPUTS
    COUPLED_INPUTS = {**MassProperties.COUPLED_INPUTS,
                      "structure_cg_z": "m", "structure_sprung_cg_z": "m", "structure_yaw_inertia": "kg*m**2"}
    OUTPUT_UNITS = MassProperties.OUTPUT_UNITS
    DEPENDENCIES = dict.fromkeys(OUTPUT_UNITS, set(CONFIG_INPUTS) | set(COUPLED_INPUTS))
    model_evidence = {**EVIDENCE, "placement": "explicit frame inventory; equipment and unequal actuator/arm placement remain reference approximations"}

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        wb, track, rw, rd, length = v("wheelbase"), v("track_width"), v("wheel_diameter")/2, v("drum_diameter")/2, v("drum_length")
        parts, sprung = [], []
        for sx in (-1, 1):
            for sy in (-1, 1):
                mw = v("wheel_mass")/4
                parts.append((mw, sx*wb/2, sy*track/2, rw, mw*(3*rw**2+v("wheel_width")**2)/12))
                parts.append((v("drive_mass")/4, sx*wb/2, sy*track/2, rw, 0))
                md = v("drum_mass")/4
                parts.append((md, sx*v("travel_x"), sy*(length+v("drum_pair_gap"))/2, v("travel_y"), md*(3*rd**2+length**2)/12))
                sprung.append(parts[-1])
            ml = v("linkage_mass")/2
            px = (v("pivot_x")+v("actuator_pivot_x"))/2
            pz = (v("bottom_pivot_height")+v("top_pivot_height"))/2
            jx = v("pivot_x")+v("actuator_attachment_fraction")*(v("travel_x")-v("pivot_x"))
            jz = v("bottom_pivot_height")+v("actuator_attachment_fraction")*(v("travel_y")-v("bottom_pivot_height"))
            ex, ez = (v("travel_x")+jx)/2, (v("travel_y")+jz)/2
            parts.append((ml, sx*(px+ex)/2, 0, (pz+ez)/2, ml*(ex-px)**2/12))
            sprung.append(parts[-1])
        mc = v("fixed_mass")+v("battery_mass")
        equipment = (mc, 0, 0, rw+v("chassis_height")/2, mc*(wb**2+track**2)/12)
        parts.extend((equipment, (v("structure_mass"), 0, 0, v("structure_cg_z"), v("structure_yaw_inertia"))))
        sprung.extend((equipment, (v("structure_mass")-v("rocker_mass"), 0, 0, v("structure_sprung_cg_z"), 0)))
        for state in ("empty", "loaded", "half"):
            load, body = list(parts), list(sprung)
            if state != "empty":
                for sx in ((-1, 1) if state == "loaded" else (1,)):
                    for sy in (-1, 1):
                        m = v("payload_mass")/4
                        item = (m, sx*v("travel_x"), sy*(length+v("drum_pair_gap"))/2,
                                v("travel_y")-rd/3, m*(3*rd**2+length**2)/12)
                        load.append(item)
                        body.append(item)
            _, x, z, inertia = mass_properties(load)
            outputs[state+"_cg_x"], outputs[state+"_cg_z"], outputs[state+"_yaw_inertia"] = x, z, inertia
            if state == "loaded":
                m, x, z, _ = mass_properties(body)
                outputs["loaded_sprung_mass"], outputs["loaded_sprung_cg_x"], outputs["loaded_sprung_cg_z"] = m, x, z


def plate_frame_posed_cg(pose, radius, offset, sprung_mass, sprung_x, sprung_z, wheel_drive_mass, rocker_mass):
    """Separate wheel/drive centers from uniform V-half-rocker centroids."""
    if min(wheel_drive_mass, rocker_mass) < 0:
        raise ValueError("Nonnegative unsprung masses required.")
    body = np.asarray(posed_loaded_cg(pose, radius, offset, sprung_mass, sprung_x, sprung_z, 0))
    wheels = np.mean(pose["centers"], axis=0)
    # Equal two-half rockers: centroid halfway from shaft to wheel midpoint.
    # Symmetric wheel outboard offsets cancel in the two-rocker aggregate CG.
    rocker = (wheels+np.asarray(pose["shaft_origin"]))/2
    return tuple((sprung_mass*body+wheel_drive_mass*wheels+rocker_mass*rocker)/(sprung_mass+wheel_drive_mass+rocker_mass))


class PlateObstacleStability(Discipline):
    CONFIG_INPUTS = ObstacleStability.CONFIG_INPUTS
    COUPLED_INPUTS = ObstacleStability.COUPLED_INPUTS
    OUTPUT_UNITS = ObstacleStability.OUTPUT_UNITS
    DEPENDENCIES = ObstacleStability.DEPENDENCIES
    model_evidence = {**ObstacleStability.model_evidence,
                      "rockers": "uniform sloping box half-arms; distinct wheel/drive and rocker centroids",
                      "shaft": "longitudinally centered; contact-track kinematics include symmetric lateral wheel overhang"}

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        cases = []
        for wheel in range(4):
            pose = raised_wheel_pose(v("wheelbase"), v("track_width"), v("wheel_diameter")/2,
                                     v("rocker_pivot_offset"), v("obstacle_height"), wheel)
            cg = plate_frame_posed_cg(pose, v("wheel_diameter")/2, v("rocker_pivot_offset"),
                                      v("loaded_sprung_mass"), v("loaded_sprung_cg_x"), v("loaded_sprung_cg_z"),
                                      v("wheel_mass")+v("drive_mass"), v("rocker_mass"))
            cases.append((projected_support_margin(pose["contacts"], cg), wheel, pose, cg))
        margin, wheel, pose, cg = min(cases, key=lambda case: case[0])
        outputs["obstacle_tip_margin"] = margin
        outputs["obstacle_pose_margin"] = pose["reach_margin"]
        outputs["obstacle_pose_valid"] = float(pose["valid"])
        outputs["obstacle_evaluated_height"] = pose["used_height"]
        outputs["obstacle_worst_wheel"] = wheel
        outputs["obstacle_chassis_pitch"], outputs["obstacle_chassis_roll"] = pose["pitch"], pose["roll"]
        for axis, value in zip("xyz", cg):
            outputs["obstacle_loaded_cg_"+axis] = value
