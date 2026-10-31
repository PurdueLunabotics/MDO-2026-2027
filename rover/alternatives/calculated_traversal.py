"""Default motor/BWR-limited route timing, with reference point-turn physics.

Pure, deterministic profile selection; no fitted data or persistent caches.
See docs/CALCULATED_TRAVERSAL.md for search policy, domains and limitations.
"""
import math

import openmdao.api as om
from scipy.optimize import minimize_scalar

from ..components import scalar, travel_profile
from ..config import ROUTE
from .bekker_wong_reece import (
    BekkerWongReeceMobility, NoDrivingSolution, WheelDomainError,
    rover_on_slope, transfer_flags,
)


TIMING_OUTPUTS = {
    "outbound_travel_time": "s", "return_travel_time": "s",
    "outbound_route_acceleration": "m/s**2", "return_route_acceleration": "m/s**2",
    "outbound_straight_speed_limit": "m/s", "return_straight_speed_limit": "m/s",
    "outbound_bwr_slip": None, "return_bwr_slip": None,
    "outbound_bwr_sinkage": "m", "return_bwr_sinkage": "m",
    "outbound_bwr_transfer_flag_count": None, "return_bwr_transfer_flag_count": None,
}


def motor_rim_speed_limit(motor, force, radius, factor=1.):
    """Ground-speed limit for an aggregate four-wheel force and slip factor.

    Reuse existing driver reserves: 0.3 N aggregate motor force and 0.4 W
    mechanical peak power. Tiny added roundoff reserves are numerical only.
    """
    if not all(math.isfinite(x) for x in (force, radius, factor)) or force < 0 or radius <= 0 or not 0 < factor <= 1:
        raise WheelDomainError("Invalid force, radius or ground-speed factor for traversal timing")
    rpm = motor.speed_limit((force+.3000001)*radius/4)
    power = 4*motor.peak_power()-.4000001
    if not math.isfinite(rpm+power) or rpm <= 0 or power <= 0:
        return 0.
    rim_speed = rpm*math.pi/30*radius
    if force > 0:
        rim_speed = min(rim_speed, power/force)
    return rim_speed*factor


def state_route(soil, values, motor, mass, inertia, route=ROUTE):
    """Select a fast admissible constant-acceleration profile for ONE route.

    Speed/acceleration inputs are upper bounds. One common straight acceleration
    is searched per loading state; turn acceleration retains its configured
    fraction. Turns use the existing rolling/scrub/yaw law, not BWR side forces.
    Other legacy traction/support screens are still evaluated by Mobility.
    """
    v = values.__getitem__
    positive = (mass, v("wheel_diameter"), v("wheel_width"), v("track_width"),
                v("travel_speed"), v("acceleration"), v("turn_speed_fraction"),
                v("turn_accel_fraction"), v("rolling_pressure_scale"))
    if not all(math.isfinite(x) and x > 0 for x in positive) or not math.isfinite(inertia) or inertia < 0:
        raise WheelDomainError("Calculated traversal needs positive finite loads, dimensions and motion bounds")
    if not route or any(not math.isfinite(d+yaw) or d < 0 for d,yaw in route) or sum(d for d,_ in route) <= 0:
        raise WheelDomainError("Calculated traversal needs a finite nonzero route")
    radius, half_track, gravity = v("wheel_diameter")/2, v("track_width")/2, v("gravity")
    steady = rover_on_slope(soil, 2*radius, v("wheel_width"), mass, 0., gravity=gravity)
    steady_limit = motor_rim_speed_limit(motor, 4*steady["torque_nm"]/radius, radius, 1-steady["slip"])
    if steady_limit <= 0:
        raise WheelDomainError("Selected motor cannot sustain positive BWR route motion with the declared reserves")
    delta = v("wheel_deflection_fraction")*2*radius
    if not 0 < delta < radius:
        raise WheelDomainError("Invalid wheel deflection for the retained turning model")
    area = 4*v("wheel_width")*2*math.sqrt(2*radius*delta-delta**2)
    weight = mass*gravity
    rolling = (v("rolling_base")+v("rocker_load_factor")*weight/area/v("rolling_pressure_scale"))*weight
    scrub = v("scrub_coefficient")*weight*v("wheelbase")/(4*half_track)
    cap = v("acceleration")
    # Explicit reuse within this route calculation only; discarded on return.
    evaluated = {}

    def candidate(fraction):
        if fraction in evaluated:
            return evaluated[fraction]
        acceleration = fraction*cap
        result = None
        if acceleration > 0:
            try:
                moving = rover_on_slope(soil, 2*radius, v("wheel_width"), mass, 0.,
                                        gravity=gravity, acceleration=acceleration)
            except NoDrivingSolution:
                moving = None
            if moving is not None:
                speed = min(v("travel_speed"), steady_limit,
                            motor_rim_speed_limit(motor, 4*moving["torque_nm"]/radius,
                                                  radius, 1-moving["slip"]))
                turn_a = acceleration*v("turn_accel_fraction")
                turn_force = rolling+scrub+inertia/half_track**2*turn_a
                turn_speed = min(v("travel_speed")*v("turn_speed_fraction"),
                                 motor_rim_speed_limit(motor, turn_force, radius))
                if speed > 0 and turn_speed > 0:
                    profiles = []
                    for distance, degrees in route:
                        if distance:
                            duration, peak = travel_profile(distance, speed, acceleration)
                            profiles.append(("straight", distance, duration, peak, acceleration))
                        if degrees:
                            distance = abs(math.radians(degrees))*half_track
                            duration, peak = travel_profile(distance, turn_speed, turn_a)
                            profiles.append(("turn", distance, duration, peak, turn_a))
                    result = {"profiles": profiles, "duration": sum(p[2] for p in profiles),
                              "acceleration": acceleration, "speed_limit": speed,
                              "upper_bounds_reached": speed == v("travel_speed") and turn_speed == v("travel_speed")*v("turn_speed_fraction"),
                              "slip": max(steady["slip"], moving["slip"]),
                              "sinkage": max(steady["sinkage_m"], moving["sinkage_m"]),
                              "transfer_flags": sorted(set(transfer_flags(steady)+transfer_flags(moving)))}
        evaluated[fraction] = result
        return result

    # If both requested speed bounds and maximum acceleration are supported,
    # this is already the analytic minimum-time profile in the chosen family.
    # Avoid searching a scalar problem whose optimum is known exactly.
    full = candidate(1.)
    if full is not None and full["upper_bounds_reached"]:
        return full
    grid = [1/16, 1/8, 1/4, 1/2, 1.]
    for fraction in grid:
        candidate(fraction)
    # Search smaller accelerations explicitly if the initial bracket has no
    # supported motion. Never substitute the fixed-time or reference physics.
    lower = grid[0]
    for _ in range(20):
        if any(result is not None for result in evaluated.values()):
            break
        lower /= 2
        grid.insert(0, lower)
        candidate(lower)
    valid = [(result["duration"], fraction) for fraction,result in evaluated.items() if result is not None]
    if not valid:
        raise WheelDomainError("No positive calculated route profile in the declared acceleration search domain")
    _, best_fraction = min(valid)
    index = grid.index(best_fraction)
    bracket = (grid[index-1] if index else 0., grid[index+1] if index+1 < len(grid) else 1.)
    def objective(fraction):
        result = candidate(fraction)
        return result["duration"] if result is not None else float("inf")
    optimum = minimize_scalar(objective, bounds=bracket, method="bounded",
                              options={"xatol": 1e-9, "maxiter": 64})
    if optimum.success:
        candidate(float(optimum.x))
    # Retain every feasible sampled endpoint even when refinement fails.
    return min((r for r in evaluated.values() if r is not None), key=lambda r:r["duration"])


class CalculatedTraversalMobility(BekkerWongReeceMobility):
    TRAVERSAL_MODE = "calculated"
    # Timing diagnostics are mode-specific; fixed/custom replacements retain
    # the shared mobility contract rather than requiring calculated-only ports.
    REQUIRED_OUTPUT_UNITS = BekkerWongReeceMobility.OUTPUT_UNITS
    OUTPUT_UNITS = {**BekkerWongReeceMobility.OUTPUT_UNITS, **TIMING_OUTPUTS}
    DEPENDENCIES = {}

    def setup(self):
        super().setup()
        planner = set("wheel_diameter wheel_width travel_speed acceleration gravity turn_speed_fraction turn_accel_fraction track_width wheelbase rocker_load_factor wheel_deflection_fraction rolling_base rolling_pressure_scale dry_mass payload_mass loaded_yaw_inertia empty_yaw_inertia".split())
        planner |= self.options["motor_model"].input_names("drive")
        for name in self.OUTPUT_UNITS:
            if name in ("route_distance", "route_turn_angle", "no_load_speed", "stall_drive_force", "turn_time"):
                continue
            self.DEPENDENCIES[name] = (self.DEPENDENCIES.get(name, set())-{"traversal_time_one_way"}) | planner
        self.model_evidence = {**self.model_evidence, "timing_revision": 1,
            "notice": "CALCULATED route timing: BWR/motor-limited straight profiles and retained empirical point-turn equations; not hardware calibrated. Legacy traction/stability checks remain active. Deposition and relocation retain their reference timing.",
            "timing_policy": "Fastest sampled/refined common-acceleration route profile per loaded/empty state, bounded by travel_speed and acceleration; not globally time-optimal control",
            "inactive_inputs": ["traversal_time_one_way"],
            "braking": "Symmetric rest-to-rest timing; retained nonregenerative work closure, no braking torque/slip or controller simulation",
            "source_range_policy": "Per-leg BWR transfer flag counts are reported; the existing published GRC-1 proxy is not refitted"}

    def route_plan(self, values):
        inputs = {name: values(name) for name in self._var_rel_names["input"]}
        motor = self.options["motor_model"].create(values, "drive", rated_torque_limit=False)
        plans = []
        for prefix,state,mass in (("outbound", "loaded", values("dry_mass")+values("payload_mass")),
                                  ("return", "empty", values("dry_mass"))):
            route = state_route(self.soil, inputs, motor, mass, values(state+"_yaw_inertia"))
            profiles = route["profiles"] if state == "loaded" else list(reversed(route["profiles"]))
            diagnostics = dict(zip((prefix+"_"+suffix for suffix in
                ("travel_time", "route_acceleration", "straight_speed_limit", "bwr_slip", "bwr_sinkage", "bwr_transfer_flag_count")),
                (route["duration"], route["acceleration"], route["speed_limit"], route["slip"], route["sinkage"], len(route["transfer_flags"]))))
            plans.append(([(mass,state,p) for p in profiles], diagnostics))
        segments = plans[0][0]+plans[1][0]
        return {"segments": segments, "travel_time": sum(p[2][2] for p in segments),
                "translation_time": sum(p[2][2] for p in segments if p[2][0] == "straight"),
                "route_turn_time": sum(p[2][2] for p in segments if p[2][0] == "turn"),
                "peak_speed": max(p[2][3] for p in segments if p[2][0] == "straight"),
                "peak_acceleration": max(p[2][4] for p in segments),
                "diagnostics": {**plans[0][1], **plans[1][1]}}

    def compute(self, inputs, outputs):
        try:
            plan = self.route_plan(lambda n:scalar(inputs,n))
            self.compute_route(inputs, outputs, plan)
            for name,value in plan["diagnostics"].items():
                outputs[name] = value
        except WheelDomainError as exc:
            raise om.AnalysisError(str(exc)) from exc
