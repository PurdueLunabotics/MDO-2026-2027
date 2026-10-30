"""Planar, mirrored fixed-arm/linear-actuator mechanisms; dimensions in metres.

Lower chassis pivot B carries a fixed arm; upper pivot A carries the actuator.
The actuator pin is D=C+s*u+t*n, with u directed from B to drum axle C
and n=(-u_z,u_x). Positive s points past the axle; positive t is above
the arm. The same tube continues to the pin axial station when s>0.
Extending A-D lowers C on the outward B-D branch.
The startup pose is compact; the shorter travel setting raises the drum for
driving after deployment, and the longest setting digs.
"""
import math
import numpy as np
from .components import SimpleComponent, scalar
from .alternatives.offset_attachment import attachment_point, pin_radius, physical_length

SWEEP_SAMPLES = 81


def drum_pose(arm, bottom_x, bottom_z, top_x, top_z, actuator_length,
              side_offset=0.0, top_offset=0.0):
    """Outward B-D circle branch rotated back by the local pin offset angle.

    Guards keep unreachable optimizer trials finite; signed triangle/toggle
    margins reject them. B-C remains exactly arm_length.
    """
    radius = pin_radius(arm, side_offset, top_offset)
    dx, dz = top_x-bottom_x, top_z-bottom_z
    separation = max(math.hypot(dx, dz), 1e-4)
    cosine = (radius**2 + separation**2 - actuator_length**2)/(2*radius*separation)
    cosine = float(np.clip(cosine, -1+1e-9, 1-1e-9))
    sine = math.sqrt(1-cosine**2)
    qx, qz = (cosine*dx+sine*dz)/separation, (cosine*dz-sine*dx)/separation
    cb, sb = (arm+side_offset)/radius, top_offset/radius
    return (bottom_x+arm*(cb*qx+sb*qz), bottom_z+arm*(cb*qz-sb*qx))


def offset_values(values):
    if "actuator_attachment_fraction" in values:
        if "lin_act_side_arm_offset" in values or "lin_act_top_arm_offset" in values:
            raise ValueError("Ambiguous old/new attachment coordinates")
        return ((values["actuator_attachment_fraction"]-1)*values["arm_length"], 0.)
    return values.get("lin_act_side_arm_offset", 0.), values.get("lin_act_top_arm_offset", 0.)


def sweep_poses(values, samples=SWEEP_SAMPLES):
    """Equal-angle capability sweep covering travel, deposition and extension."""
    v = values.__getitem__
    arm, bottom, pivot = v("arm_length"), v("bottom_pivot_height"), v("pivot_x")
    side, top = offset_values(values)
    lengths = (v("travel_actuator_length"), values.get("deposition_actuator_length", v("travel_actuator_length")), v("actuator_extended_length"))
    start = drum_pose(arm, pivot, bottom, v("actuator_pivot_x"), v("top_pivot_height"), min(lengths), side, top)
    end = drum_pose(arm, pivot, bottom, v("actuator_pivot_x"), v("top_pivot_height"), max(lengths), side, top)
    angles = np.linspace(math.atan2(start[1] - bottom, start[0] - pivot),
                         math.atan2(end[1] - bottom, end[0] - pivot), samples)
    return [(pivot + arm * math.cos(a), bottom + arm * math.sin(a)) for a in angles], angles


class LinkageGeometry(SimpleComponent):
    OUTPUT_UNITS = {name: "m" for name in (
        "stowed_x stowed_y travel_x travel_y extended_x extended_y deposition_x deposition_y "
        "stowed_length stowed_height deployed_length deployed_height stroke "
        "pit_depth_margin transport_clearance_margin wheel_first_margin "
        "startup_position_margin dig_position_margin pivot_spacing_margin "
        "pivot_mount_margin pivot_longitudinal_margin bottom_mount_margin "
        "stroke_margin actuator_body_margin triangle_retracted_margin "
        "triangle_extended_margin toggle_margin wheel_clearance_margin "
        "chassis_clearance_margin deployed_height_margin "
        "actuator_pivot_longitudinal_margin sweep_x_min sweep_x_max "
        "actuator_toggle_margin actuator_attachment_distance arm_physical_length "
        "deposition_reach_margin deposition_position_margin deposition_clearance_margin").split()}
    OUTPUT_UNITS.update({"actuator_leverage_max": None, "lowering_time": "s",
                         "linkage_mass": "kg", "arm_extension_mass": "kg"})

    def setup(self):
        self.inputs_from_config(
            "actuator_speed", "travel_actuator_length", "deposition_actuator_length", "actuator_retracted_length", "actuator_extended_length",
            "top_pivot_height", "bottom_pivot_height", "arm_length", "pivot_x", "actuator_pivot_x", "lin_act_side_arm_offset", "lin_act_top_arm_offset",
            "actuator_force_rating", "wheel_diameter", "drum_diameter", "wheelbase",
            "chassis_height", "transport_clearance", "mechanism_clearance",
            "minimum_pivot_spacing", "minimum_stroke", "actuator_housing_allowance",
            "pit_depth", "actuator_efficiency", "actuator_fixed_mass", "actuator_mass_per_force",
            "actuator_mass_per_power", "actuator_mass_per_length")
        self.add_input("arm_linear_mass", units="kg/m")
        self.add_input("arm_envelope_radius", units="m")
        for name, units in self.OUTPUT_UNITS.items():
            self.add_output(name, units=units)

    def compute(self, inputs, outputs):
        values = {n: scalar(inputs, n) for n in inputs}
        v = values.__getitem__
        poses, angles = sweep_poses(values)
        xs, ys = np.array(poses).T
        def command_pose(name):
            return drum_pose(v("arm_length"), v("pivot_x"), v("bottom_pivot_height"),
                v("actuator_pivot_x"), v("top_pivot_height"), v(name),
                v("lin_act_side_arm_offset"), v("lin_act_top_arm_offset"))
        tx, ty = command_pose("travel_actuator_length")
        dep_x, dep_y = command_pose("deposition_actuator_length")
        sx, sy = drum_pose(v("arm_length"), v("pivot_x"), v("bottom_pivot_height"),
                          v("actuator_pivot_x"), v("top_pivot_height"), v("actuator_retracted_length"), v("lin_act_side_arm_offset"), v("lin_act_top_arm_offset"))
        ex, ey = command_pose("actuator_extended_length")
        arm = v("arm_length")
        side, top = v("lin_act_side_arm_offset"), v("lin_act_top_arm_offset")
        attachment_radius = pin_radius(arm, side, top)
        ell = physical_length(arm, side)
        outputs["arm_physical_length"] = ell
        outputs["arm_extension_mass"] = 2*v("arm_linear_mass")*(ell-arm)
        outputs["actuator_attachment_distance"] = attachment_radius
        dx, dz = v("actuator_pivot_x")-v("pivot_x"), v("top_pivot_height")-v("bottom_pivot_height")
        gap = math.hypot(dx, dz)
        retracted, extended = v("actuator_retracted_length"), v("actuator_extended_length")
        rw, rd = v("wheel_diameter") / 2, v("drum_diameter") / 2
        chassis_top = 2 * rw + v("chassis_height")
        chassis_end = v("wheelbase") / 2
        clearance = v("mechanism_clearance")
        # Every intermediate point is at most this arc length from its nearest sample.
        # Subtracting it makes the sampled circular/box-clearance check conservative.
        sample_allowance = arm * abs(angles[-1] - angles[0]) / (2 * (SWEEP_SAMPLES - 1))
        wheel_distance = np.hypot(xs - chassis_end, ys - rw)
        box_distance = np.hypot(np.maximum(xs - chassis_end, 0),
                                np.maximum(np.maximum(rw - ys, ys - chassis_top), 0))
        outputs["stowed_x"], outputs["stowed_y"] = sx, sy
        outputs["travel_x"], outputs["travel_y"] = tx, ty
        outputs["extended_x"], outputs["extended_y"] = ex, ey
        outputs["deposition_x"], outputs["deposition_y"] = dep_x, dep_y
        dep = v("deposition_actuator_length")
        outputs["deposition_reach_margin"] = min(dep-abs(attachment_radius-gap)-clearance,
                                                  attachment_radius+gap-dep-clearance)
        outputs["deposition_position_margin"] = extended-dep
        outputs["deposition_clearance_margin"] = dep_y-rd-v("transport_clearance")
        outputs["stowed_length"] = 2 * max(chassis_end + rw, sx + rd, v("pivot_x"), v("actuator_pivot_x"))
        outputs["stowed_height"] = max(chassis_top, sy + rd, v("top_pivot_height"))
        outputs["deployed_length"] = 2 * max(chassis_end + rw, max(xs) + rd, v("actuator_pivot_x"))
        outputs["deployed_height"] = max(chassis_top, max(ys) + rd, v("top_pivot_height"))
        outputs["deployed_height_margin"] = 2.5 - outputs["deployed_height"]
        travel = v("travel_actuator_length")
        outputs["stroke"] = extended - travel
        outputs["lowering_time"] = max(extended - travel, 0.) / v("actuator_speed")
        outputs["pit_depth_margin"] = v("pit_depth") + ey - rd
        outputs["transport_clearance_margin"] = sy - rd - v("transport_clearance")
        # The front drum leads the wheel: keep its travel envelope above a
        # wheel-diameter step, pending measured obstacle/soil capability.
        outputs["wheel_first_margin"] = ty - rd - v("wheel_diameter")
        outputs["startup_position_margin"] = retracted - travel
        outputs["dig_position_margin"] = extended - retracted
        outputs["pivot_spacing_margin"] = gap - v("minimum_pivot_spacing")
        outputs["pivot_mount_margin"] = chassis_top - v("top_pivot_height")
        outputs["bottom_mount_margin"] = v("bottom_pivot_height") - rw
        outputs["pivot_longitudinal_margin"] = chassis_end - v("pivot_x")
        outputs["actuator_pivot_longitudinal_margin"] = chassis_end - v("actuator_pivot_x")
        outputs["stroke_margin"] = extended - travel - v("minimum_stroke")
        # Diagnostic only: this reserve does not constrain optimization/feasibility.
        outputs["actuator_body_margin"] = 2 * min(travel, dep) - extended - v("actuator_housing_allowance")
        outputs["triangle_retracted_margin"] = travel - abs(attachment_radius - gap) - clearance
        outputs["triangle_extended_margin"] = attachment_radius + gap - extended - clearance
        outputs["toggle_margin"] = min(xs) - v("pivot_x") - clearance
        joint_xs, joint_ys = attachment_point(v("pivot_x"), v("bottom_pivot_height"), xs, ys, side, top)
        cross = (joint_xs-v("pivot_x"))*dz - (joint_ys-v("bottom_pivot_height"))*dx
        outputs["actuator_toggle_margin"] = min(cross)/max(gap, 1e-4) - clearance
        actuator_lengths = np.hypot(joint_xs-v("actuator_pivot_x"), joint_ys-v("top_pivot_height"))
        leverage = np.abs((xs-v("pivot_x"))*actuator_lengths/np.maximum(np.abs(cross), 1e-6))
        outputs["actuator_leverage_max"] = max(leverage)
        x_candidates = [poses[0][0], poses[-1][0]]
        if min(angles) <= 0 <= max(angles):
            x_candidates.append(v("pivot_x")+arm)
        outputs["sweep_x_min"], outputs["sweep_x_max"] = min(x_candidates), max(x_candidates)
        outputs["wheel_clearance_margin"] = min(wheel_distance) - rw - rd - clearance - sample_allowance
        outputs["chassis_clearance_margin"] = min(box_distance) - rd - clearance - sample_allowance
        # Include the physical tube end and rigid normal bracket pin. Exact
        # coordinate extrema avoid gaps between sampled poses for these points.
        if side != 0 or top != 0:
            def extrema(a, b):
                lo, hi = sorted((float(angles[0]), float(angles[-1])))
                critical = [lo, hi]
                phase = math.atan2(b, a)
                for k in range(-4, 5):
                    angle = -phase+k*math.pi/2
                    if lo <= angle <= hi: critical.append(angle)
                return [(v("pivot_x")+a*math.cos(q)-b*math.sin(q),
                         v("bottom_pivot_height")+a*math.sin(q)+b*math.cos(q)) for q in critical]
            # Declared tube section radius encloses the extension; bracket
            # thickness/body is unknown, so only its pin/centerline is screened.
            radius = v("arm_envelope_radius")
            tips = [(x,z,radius) for x,z in extrema(ell, 0.)]+[(x,z,0.) for x,z in extrema(arm+side, top)]
            stowed = [(*attachment_point(v("pivot_x"), v("bottom_pivot_height"), sx, sy, max(side,0.), 0.),radius),
                      (*attachment_point(v("pivot_x"), v("bottom_pivot_height"), sx, sy, side, top),0.)]
            outputs["stowed_length"] = max(outputs["stowed_length"], 2*max(abs(x)+r for x,z,r in stowed))
            outputs["stowed_height"] = max(outputs["stowed_height"], max(z+r for x,z,r in stowed))
            outputs["deployed_length"] = max(outputs["deployed_length"], 2*max(abs(x)+r for x,z,r in tips))
            outputs["deployed_height"] = max(outputs["deployed_height"], max(z+r for x,z,r in tips))
            outputs["deployed_height_margin"] = 2.5-outputs["deployed_height"]
            outputs["transport_clearance_margin"] = min(outputs["transport_clearance_margin"], min(z-r for x,z,r in stowed)-v("transport_clearance"))
            outputs["pit_depth_margin"] = min(outputs["pit_depth_margin"], min(z-r for x,z,r in tips)+v("pit_depth"))
        rated_power = v("actuator_force_rating") * v("actuator_speed") / v("actuator_efficiency")
        outputs["linkage_mass"] = 2 * (
            v("actuator_fixed_mass") + v("actuator_mass_per_force") * v("actuator_force_rating")
            + v("actuator_mass_per_power") * rated_power
            + v("actuator_mass_per_length") * extended + v("arm_linear_mass") * ell)


class LinkageLoads(SimpleComponent):
    OUTPUT_UNITS = {"required_actuator_force": "N", "actuator_force_margin": "N",
                    "arm_energy": "J", "arm_peak_power": "W", "actuation_time": "s"}

    def setup(self):
        self.inputs_from_config("gravity", "actuator_speed",
            "actuator_force_rating",
            "actuator_efficiency", "actuator_safety_factor", "breakout_force", "cut_depth",
            "arm_length", "lin_act_side_arm_offset", "actuator_idle_power")
        self.add_input("arm_linear_mass", units="kg/m")
        for name, units in {"payload_mass": "kg", "drum_mass": "kg", "travel_y": "m", "extended_y": "m", "lowering_time": "s", "actuator_leverage_max": None}.items():
            self.add_input(name, units=units)
        for name, units in self.OUTPUT_UNITS.items():
            self.add_output(name, units=units)

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        # Per pair: both drums + half payload + half the arm's own mass at its tip.
        empty_weight = (v("drum_mass") / 2 + v("arm_linear_mass") * physical_length(v("arm_length"), v("lin_act_side_arm_offset"))**2 / (2*v("arm_length"))) * v("gravity")
        loaded_weight = empty_weight + v("payload_mass") / 2 * v("gravity")
        # Maximum virtual-work leverage across the travel-to-dig arm sweep.
        force = (loaded_weight + v("breakout_force")) * v("actuator_leverage_max")
        lift = max(v("travel_y") - v("extended_y"), 0)
        outputs["actuation_time"] = 2 * v("lowering_time")
        outputs["required_actuator_force"] = force
        outputs["actuator_force_margin"] = v("actuator_force_rating") / v("actuator_safety_factor") - force
        # Both pairs raise loaded and lower empty in parallel; no regeneration credited.
        outputs["arm_energy"] = 2 * ((loaded_weight + empty_weight) * lift + v("breakout_force") * v("cut_depth")) / v("actuator_efficiency")
        outputs["arm_energy"] += 2 * v("actuator_idle_power") * outputs["actuation_time"]
        outputs["arm_peak_power"] = 2 * force * v("actuator_speed") / v("actuator_efficiency") + 2 * v("actuator_idle_power")
