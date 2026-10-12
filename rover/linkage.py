"""Planar, mirrored fixed-arm/linear-actuator mechanisms; dimensions in metres.

Lower chassis pivot B carries a fixed arm; upper pivot A carries the actuator.
The actuator meets the arm at D=B+f(C-B), where C is the drum center and
0<f<=1. Extending A-D lowers C on the outward branch; f=1 is the reference.
The startup pose is compact; the shorter travel setting raises the drum for
driving after deployment, and the longest setting digs.
"""
import math
import numpy as np
from .components import SimpleComponent, scalar

SWEEP_SAMPLES = 81


def attachment_point(bottom_x, bottom_z, drum_x, drum_z, fraction):
    """Actuator pin on the straight arm, measured from chassis pivot B."""
    return (bottom_x + fraction*(drum_x-bottom_x),
            bottom_z + fraction*(drum_z-bottom_z))


def drum_pose(arm, bottom_x, bottom_z, top_x, top_z, actuator_length,
              attachment_fraction=1.0):
    """Circle intersection on the outward branch. Guards keep invalid trials finite.

    Triangle and toggle constraints reject guarded/infeasible geometry; a finite
    drawing or finite output alone does not establish geometric feasibility.
    """
    if not math.isfinite(attachment_fraction) or attachment_fraction <= 0:
        raise ValueError("actuator attachment fraction must be finite and positive")
    radius = attachment_fraction * arm
    dx, dz = top_x-bottom_x, top_z-bottom_z
    separation = max(math.hypot(dx, dz), 1e-4)
    cosine = (radius**2 + separation**2 - actuator_length**2) / (2 * radius * separation)
    cosine = float(np.clip(cosine, -1 + 1e-9, 1 - 1e-9))
    sine = math.sqrt(1 - cosine**2)
    # Solve B-D's direction, then extend it to full arm length B-C. Guard the
    # angle rather than a dimensional offset so tiny attachment radii still
    # preserve arm length even in rejected, unreachable triangle trials.
    return (bottom_x + arm*(cosine*dx + sine*dz)/separation,
            bottom_z + arm*(cosine*dz - sine*dx)/separation)


def sweep_poses(values, samples=SWEEP_SAMPLES):
    """Equal-angle samples, including the two actuator endpoints."""
    v = values.__getitem__
    arm, bottom, pivot = v("arm_length"), v("bottom_pivot_height"), v("pivot_x")
    fraction = values.get("actuator_attachment_fraction", 1.0)  # historical tip-attachment reports
    start = drum_pose(arm, pivot, bottom, v("actuator_pivot_x"), v("top_pivot_height"), v("travel_actuator_length"), fraction)
    end = drum_pose(arm, pivot, bottom, v("actuator_pivot_x"), v("top_pivot_height"), v("actuator_extended_length"), fraction)
    angles = np.linspace(math.atan2(start[1] - bottom, start[0] - pivot),
                         math.atan2(end[1] - bottom, end[0] - pivot), samples)
    return [(pivot + arm * math.cos(a), bottom + arm * math.sin(a)) for a in angles], angles


class LinkageGeometry(SimpleComponent):
    OUTPUT_UNITS = {name: "m" for name in (
        "stowed_x stowed_y travel_x travel_y extended_x extended_y "
        "stowed_length stowed_height deployed_length deployed_height stroke "
        "pit_depth_margin transport_clearance_margin wheel_first_margin "
        "startup_position_margin dig_position_margin pivot_spacing_margin "
        "pivot_mount_margin pivot_longitudinal_margin bottom_mount_margin "
        "stroke_margin actuator_body_margin triangle_retracted_margin "
        "triangle_extended_margin toggle_margin wheel_clearance_margin "
        "chassis_clearance_margin deployed_height_margin "
        "actuator_pivot_longitudinal_margin sweep_x_min sweep_x_max "
        "actuator_toggle_margin actuator_attachment_distance").split()}
    OUTPUT_UNITS.update({"actuator_leverage_max": None, "actuation_time": "s",
                         "linkage_mass": "kg"})

    def setup(self):
        self.inputs_from_config(
            "actuator_speed", "travel_actuator_length", "actuator_retracted_length", "actuator_extended_length",
            "top_pivot_height", "bottom_pivot_height", "arm_length", "pivot_x", "actuator_pivot_x", "actuator_attachment_fraction",
            "actuator_force_rating", "wheel_diameter", "drum_diameter", "wheelbase",
            "chassis_height", "transport_clearance", "mechanism_clearance",
            "minimum_pivot_spacing", "minimum_stroke", "actuator_housing_allowance",
            "pit_depth", "actuator_efficiency", "actuator_fixed_mass", "actuator_mass_per_force",
            "actuator_mass_per_power", "actuator_mass_per_length")
        self.add_input("arm_linear_mass", units="kg/m")
        for name, units in self.OUTPUT_UNITS.items():
            self.add_output(name, units=units)

    def compute(self, inputs, outputs):
        values = {n: scalar(inputs, n) for n in inputs}
        v = values.__getitem__
        poses, angles = sweep_poses(values)
        xs, ys = np.array(poses).T
        tx, ty = poses[0]
        sx, sy = drum_pose(v("arm_length"), v("pivot_x"), v("bottom_pivot_height"),
                          v("actuator_pivot_x"), v("top_pivot_height"), v("actuator_retracted_length"), v("actuator_attachment_fraction"))
        ex, ey = poses[-1]
        arm = v("arm_length")
        fraction = v("actuator_attachment_fraction")
        attachment_radius = fraction * arm
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
        outputs["stowed_length"] = 2 * max(chassis_end + rw, sx + rd, v("pivot_x"), v("actuator_pivot_x"))
        outputs["stowed_height"] = max(chassis_top, sy + rd, v("top_pivot_height"))
        outputs["deployed_length"] = 2 * max(chassis_end + rw, max(xs) + rd, v("actuator_pivot_x"))
        outputs["deployed_height"] = max(chassis_top, max(ys) + rd, v("top_pivot_height"))
        outputs["deployed_height_margin"] = 2.5 - outputs["deployed_height"]
        travel = v("travel_actuator_length")
        outputs["stroke"] = extended - travel
        outputs["actuation_time"] = 2 * max(extended - travel, 1e-6) / v("actuator_speed")
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
        outputs["actuator_body_margin"] = 2 * travel - extended - v("actuator_housing_allowance")
        outputs["triangle_retracted_margin"] = travel - abs(attachment_radius - gap) - clearance
        outputs["triangle_extended_margin"] = attachment_radius + gap - extended - clearance
        outputs["toggle_margin"] = min(xs) - v("pivot_x") - clearance
        joint_xs, joint_ys = attachment_point(v("pivot_x"), v("bottom_pivot_height"), xs, ys, fraction)
        cross = (joint_xs-v("pivot_x"))*dz - (joint_ys-v("bottom_pivot_height"))*dx
        outputs["actuator_toggle_margin"] = min(cross)/max(gap, 1e-4) - clearance
        actuator_lengths = np.hypot(joint_xs-v("actuator_pivot_x"), joint_ys-v("top_pivot_height"))
        leverage = np.abs((xs-v("pivot_x"))*actuator_lengths/np.maximum(np.abs(cross), 1e-6))
        outputs["actuator_leverage_max"] = max(leverage)
        x_candidates = [tx, ex]
        if min(angles) <= 0 <= max(angles):
            x_candidates.append(v("pivot_x")+arm)
        outputs["sweep_x_min"], outputs["sweep_x_max"] = min(x_candidates), max(x_candidates)
        outputs["wheel_clearance_margin"] = min(wheel_distance) - rw - rd - clearance - sample_allowance
        outputs["chassis_clearance_margin"] = min(box_distance) - rd - clearance - sample_allowance
        rated_power = v("actuator_force_rating") * v("actuator_speed") / v("actuator_efficiency")
        outputs["linkage_mass"] = 2 * (
            v("actuator_fixed_mass") + v("actuator_mass_per_force") * v("actuator_force_rating")
            + v("actuator_mass_per_power") * rated_power
            + v("actuator_mass_per_length") * extended + v("arm_linear_mass") * arm)


class LinkageLoads(SimpleComponent):
    OUTPUT_UNITS = {"required_actuator_force": "N", "actuator_force_margin": "N",
                    "arm_energy": "J", "arm_peak_power": "W"}

    def setup(self):
        self.inputs_from_config("gravity", "actuator_speed",
            "actuator_force_rating",
            "actuator_efficiency", "actuator_safety_factor", "breakout_force", "cut_depth",
            "arm_length", "actuator_idle_power")
        self.add_input("arm_linear_mass", units="kg/m")
        for name, units in {"payload_mass": "kg", "drum_mass": "kg", "travel_y": "m", "extended_y": "m", "actuation_time": "s", "actuator_leverage_max": None}.items():
            self.add_input(name, units=units)
        for name, units in self.OUTPUT_UNITS.items():
            self.add_output(name, units=units)

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        # Per pair: both drums + half payload + half the arm's own mass at its tip.
        empty_weight = (v("drum_mass") / 2 + v("arm_linear_mass") * v("arm_length") / 2) * v("gravity")
        loaded_weight = empty_weight + v("payload_mass") / 2 * v("gravity")
        # Maximum virtual-work leverage across the travel-to-dig arm sweep.
        force = (loaded_weight + v("breakout_force")) * v("actuator_leverage_max")
        lift = max(v("travel_y") - v("extended_y"), 0)
        outputs["required_actuator_force"] = force
        outputs["actuator_force_margin"] = v("actuator_force_rating") / v("actuator_safety_factor") - force
        # Both pairs raise loaded and lower empty in parallel; no regeneration credited.
        outputs["arm_energy"] = 2 * ((loaded_weight + empty_weight) * lift + v("breakout_force") * v("cut_depth")) / v("actuator_efficiency")
        outputs["arm_energy"] += 2 * v("actuator_idle_power") * v("actuation_time")
        outputs["arm_peak_power"] = 2 * force * v("actuator_speed") / v("actuator_efficiency") + 2 * v("actuator_idle_power")
