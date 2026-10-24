"""Rigid smooth wheel: Bekker pressure, Wong-Reece distribution, J-H shear.

Pure deterministic solvers first; default Mobility adapter last. No calibration,
network calls, hidden caches or writes. See docs/BEKKER_WONG_REECE.md.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import brentq, minimize_scalar

DATA = Path(__file__).resolve().parents[2] / "data/mobility/bekker_wong_reece/grc1_loose.json"
_NODES, _QUAD_WEIGHTS = np.polynomial.legendre.leggauss(64)


class WheelDomainError(ValueError):
    """Invalid inputs or no solution inside the declared physical/numeric domain."""


class NoDrivingSolution(WheelDomainError):
    """A supported contact exists, but no resolved driving solution has s<1."""
    def __init__(self, demand, capacity):
        self.capacity = capacity
        super().__init__(f"No driving solution at slip<1; required {demand:.6g} N, "
                         f"resolved maximum net force {capacity['drawbar_pull_n']:.6g} N")


@dataclass(frozen=True)
class Soil:
    kc: float
    kphi: float
    n: float
    cohesion_pa: float
    friction_angle_rad: float
    shear_modulus_m: float
    stress_c1: float
    stress_c2: float

    def validate(self, width):
        if not all(math.isfinite(x) for x in vars(self).values()):
            raise WheelDomainError("Soil parameters must be finite")
        if self.n <= 0 or self.shear_modulus_m <= 0 or self.cohesion_pa < 0:
            raise WheelDomainError("Need n>0, K>0, cohesion>=0; no silent repair of soil fits")
        if not 0 <= self.friction_angle_rad < math.pi/2:
            raise WheelDomainError("Friction angle outside [0, pi/2)")
        if self.kc/width + self.kphi <= 0:
            raise WheelDomainError("Nonpositive combined pressure-sinkage modulus at this width")
        if not 0 < self.stress_c1 < 1 or self.stress_c2 < 0 or self.stress_c1+self.stress_c2 >= 1:
            raise WheelDomainError("Stress peak must stay strictly inside the contact patch")


def load_preset(path=DATA):
    """Setup/study-time I/O; never call from compute()."""
    path = Path(path)
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest["schema_version"] != 1:
        raise ValueError("Unsupported BWR soil manifest")
    soil = Soil(**manifest["parameters"])
    return soil, {**manifest, "manifest_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def _validate_wheel(radius, width, slip):
    if not all(math.isfinite(x) for x in (radius, width, slip)) or min(radius, width) <= 0 or not 0 <= slip <= 1:
        raise WheelDomainError("Need finite positive radius/width and imposed slip in [0, 1]")


def contact_stress(soil, radius, width, entry_angle, slip, theta):
    """Radial pressure Pa, tangential shear Pa, accumulated shear displacement m.

    Theta measured from the downward plane-normal, forward positive. Exit=0.
    Rear mapping follows Zhou 2014 Eq 6; continuity and both zero-pressure
    endpoints are explicit. No pressure, slip or traction clipping.
    """
    _validate_wheel(radius, width, slip)
    soil.validate(width)
    if not math.isfinite(entry_angle) or not 0 < entry_angle < math.pi/2:
        raise WheelDomainError("Entry angle outside (0, pi/2)")
    theta = np.asarray(theta, dtype=float)
    if not np.all(np.isfinite(theta)) or np.any(theta < 0) or np.any(theta > entry_angle):
        raise WheelDomainError("Stress query outside the wheel contact arc")
    peak = (soil.stress_c1 + soil.stress_c2*slip)*entry_angle
    effective = np.where(theta >= peak, theta, entry_angle-theta/peak*(entry_angle-peak))
    # Roundoff at an exact endpoint is not an invalid physical sinkage.
    penetration = radius*(np.cos(effective)-math.cos(entry_angle))
    if np.any(penetration < -1e-14*radius):
        raise WheelDomainError("Negative effective pressure penetration")
    penetration = np.where(abs(penetration) < 1e-14*radius, 0., penetration)
    pressure = (soil.kc/width+soil.kphi)*penetration**soil.n
    displacement = radius*((entry_angle-theta)-(1-slip)*(math.sin(entry_angle)-np.sin(theta)))
    if np.any(displacement < -1e-14*radius):
        raise WheelDomainError("Negative accumulated driving shear displacement")
    shear = (soil.cohesion_pa+pressure*math.tan(soil.friction_angle_rad))*(-np.expm1(-displacement/soil.shear_modulus_m))
    return pressure, shear, displacement


def wheel_forces(soil, radius, width, entry_angle, slip):
    """Integrate separately on each side of the stress peak, SI units."""
    _validate_wheel(radius, width, slip)
    soil.validate(width)
    if not math.isfinite(entry_angle) or not 0 < entry_angle < math.pi/2:
        raise WheelDomainError("Entry angle outside (0, pi/2)")
    peak = (soil.stress_c1+soil.stress_c2*slip)*entry_angle
    normal = thrust = resistance = torque = 0.
    peak_pressure = (soil.kc/width+soil.kphi)*(radius*(math.cos(peak)-math.cos(entry_angle)))**soil.n
    for lo, hi in ((0., peak), (peak, entry_angle)):
        theta = lo+(hi-lo)*(_NODES+1)/2
        weights = _QUAD_WEIGHTS*(hi-lo)/2
        pressure, shear, _ = contact_stress(soil, radius, width, entry_angle, slip, theta)
        normal += width*radius*np.dot(weights, pressure*np.cos(theta)+shear*np.sin(theta))
        thrust += width*radius*np.dot(weights, shear*np.cos(theta))
        resistance += width*radius*np.dot(weights, pressure*np.sin(theta))
        torque += width*radius**2*np.dot(weights, shear)
    return {"normal_force_n": float(normal), "thrust_n": float(thrust),
            "compaction_resistance_n": float(resistance), "drawbar_pull_n": float(thrust-resistance),
            "torque_nm": float(torque), "entry_angle_rad": entry_angle,
            "sinkage_m": radius*(1-math.cos(entry_angle)), "slip": slip,
            "peak_pressure_pa": peak_pressure,
            "mean_normal_pressure_pa": float(normal)/(width*radius*math.sin(entry_angle))}


def wheel_at_slip(soil, radius, width, normal_load_n, slip, *, max_sinkage_fraction=.95):
    """Solve contact entry angle from normal equilibrium at imposed driving slip."""
    _validate_wheel(radius, width, slip)
    soil.validate(width)
    if not math.isfinite(normal_load_n) or normal_load_n <= 0:
        raise WheelDomainError("Normal wheel load must be finite and positive")
    if not math.isfinite(max_sinkage_fraction) or not 0 < max_sinkage_fraction < 1:
        raise WheelDomainError("Sinkage/radius limit must lie strictly between zero and one")
    upper = math.acos(1-max_sinkage_fraction)
    def residual(angle):
        return wheel_forces(soil, radius, width, angle, slip)["normal_force_n"]-normal_load_n
    if residual(upper) < 0:
        raise WheelDomainError("Wheel cannot support this load before the declared sinkage limit")
    entry = brentq(residual, 1e-8, upper, xtol=1e-12, rtol=1e-12)
    result = wheel_forces(soil, radius, width, entry, slip)
    result["normal_residual_n"] = result["normal_force_n"]-normal_load_n
    return result


def solve_operating_point(soil, radius, width, normal_load_n, required_drawbar_n):
    """First resolved driving-force crossing with s<1, or explicit failure.

    Scan 33 points through the locked-wheel limit and refine sampled local
    maxima. This resolves ordinary nonmonotonic branches, not arbitrary narrow
    features in an untested soil law. Slip=1 is evaluated only as a force limit.
    """
    if not math.isfinite(required_drawbar_n) or required_drawbar_n < 0:
        raise WheelDomainError("Only nonnegative axle demand is supported; braking is not modeled")
    last = wheel_at_slip(soil, radius, width, normal_load_n, 0.)
    if last["drawbar_pull_n"] > required_drawbar_n+1e-8:
        raise WheelDomainError("Demand needs a braking/negative-slip solution, outside this model")
    if abs(last["drawbar_pull_n"]-required_drawbar_n) <= 1e-8:
        return {**last, "force_residual_n": last["drawbar_pull_n"]-required_drawbar_n}
    points = [last]
    for slip in np.linspace(0., 1., 33)[1:]:
        point = wheel_at_slip(soil, radius, width, normal_load_n, float(slip))
        points.append(point)
        if point["drawbar_pull_n"] >= required_drawbar_n:
            solved = brentq(lambda s: wheel_at_slip(soil, radius, width, normal_load_n, s)["drawbar_pull_n"]-required_drawbar_n,
                            last["slip"], float(slip), xtol=1e-11, rtol=1e-11)
            result = wheel_at_slip(soil, radius, width, normal_load_n, solved)
            if solved < 1:
                return {**result, "force_residual_n": result["drawbar_pull_n"]-required_drawbar_n}
        last = point
    capacity = max(points, key=lambda p: p["drawbar_pull_n"])
    for left, middle, right in zip(points, points[1:], points[2:]):
        if middle["drawbar_pull_n"] > max(left["drawbar_pull_n"], right["drawbar_pull_n"]):
            peak = minimize_scalar(lambda s: -wheel_at_slip(soil, radius, width, normal_load_n, s)["drawbar_pull_n"],
                                   bounds=(left["slip"], right["slip"]), method="bounded",
                                   options={"xatol": 1e-12})
            refined = wheel_at_slip(soil, radius, width, normal_load_n, float(peak.x))
            if refined["drawbar_pull_n"] > capacity["drawbar_pull_n"]:
                capacity = refined
            if refined["drawbar_pull_n"] >= required_drawbar_n:
                solved = brentq(lambda s: wheel_at_slip(soil, radius, width, normal_load_n, s)["drawbar_pull_n"]-required_drawbar_n,
                                left["slip"], refined["slip"], xtol=1e-11, rtol=1e-11)
                result = wheel_at_slip(soil, radius, width, normal_load_n, solved)
                return {**result, "force_residual_n": result["drawbar_pull_n"]-required_drawbar_n}
    raise NoDrivingSolution(required_drawbar_n, capacity)


def rover_on_slope(soil, diameter, width, total_mass, slope_rad, *, wheel_count=4, gravity=9.80665, acceleration=0.):
    """Equal mean wheel loads on a uniform incline; slope in radians, Earth preset."""
    if not all(math.isfinite(x) for x in (total_mass, slope_rad, gravity, acceleration)) or total_mass <= 0:
        raise WheelDomainError("Invalid mass, slope, gravity or acceleration")
    if wheel_count < 1 or int(wheel_count) != wheel_count or not 0 <= slope_rad < math.pi/2 or acceleration < 0:
        raise WheelDomainError("Need positive integer wheel count, uphill angle, nonnegative acceleration")
    # Permit local finite-difference perturbations of g; not lunar scaling.
    if abs(gravity-9.80665) > .01:
        raise WheelDomainError("Published GRC-1 preset restricted to Earth gravity; no lunar scaling established")
    normal = total_mass*gravity*math.cos(slope_rad)/wheel_count
    demand = total_mass*(gravity*math.sin(slope_rad)+acceleration)/wheel_count
    return solve_operating_point(soil, diameter/2, width, normal, demand)


def transfer_flags(point):
    """Declared soil-fit windows; flags do not certify validity inside the window."""
    flags = []
    if point["peak_pressure_pa"] > 50000:
        flags.append("peak radial pressure exceeds plotted 50 kPa pressure-sinkage range")
    if point["sinkage_m"] > .010:
        flags.append("sinkage exceeds plotted 10 mm pressure-sinkage range")
    if not 5000 <= point["mean_normal_pressure_pa"] <= 30000:
        flags.append("mean normal pressure outside 5-30 kPa shear-test range")
    return flags


from ..mobility import Mobility
import openmdao.api as om


class BekkerWongReeceMobility(Mobility):
    """Default rigid-wheel level straight segments; point turns unchanged.

    Equal normal loads; retains legacy pressure/traction/stability screens. Those
    screens are not rigid-wheel contact predictions or a slope climbing test.
    """
    def initialize(self):
        super().initialize()
        self.options.declare("soil_manifest", default=DATA, types=(str, Path))

    def setup(self):
        self.soil, manifest = load_preset(self.options["soil_manifest"])
        self.model_evidence = {"exploratory": True, "revision": manifest["revision"],
            "manifest_sha256": manifest["manifest_sha256"], "soil_manifest": manifest,
            "notice": "DEFAULT rigid-wheel BWR: published loose GRC-1 parameters and equal wheel loads. Student-selected conservative design assumption; not prototype calibrated. Point turns retain their existing model. Separate empty/full BWR slope requirements check slip<1.",
            "parameter_fit": "None to rover data; independent published soil tests only",
            "soil_transfer": "14% relative-density GRC-1 proxy; not BP-1 or the Husky 28-31% preparation",
            "source_range_policy": "Published soil proxy selected by student; transfer flags remain reported, without coefficient refitting"}
        super().setup()
        self.DEPENDENCIES = {name: set(values) for name, values in self.DEPENDENCIES.items()}
        changed = ("mobility_energy", "drive_peak_power", "straight_motor_margin", "straight_traction_margin",
                   "straight_power_margin", "loaded_motor_speed_limit", "loaded_power_speed_limit")
        # Straight force now depends on geometry, mass, g and the timed profile.
        # Unchanged pressure/turn paths retain all their existing dependencies.
        for name in changed:
            self.DEPENDENCIES[name] |= {"wheel_diameter", "wheel_width", "dry_mass", "payload_mass", "gravity",
                "track_width", "travel_speed", "acceleration", "turn_speed_fraction", "turn_accel_fraction", "traversal_time_one_way"}

    def straight_response(self, values, *, mass, drag, accel, peak, distance):
        # Legacy drag is intentionally unused: stress integration already includes
        # compaction resistance. Using both would double-count soil work.
        try:
            d, w, g = values("wheel_diameter"), values("wheel_width"), values("gravity")
            steady = rover_on_slope(self.soil, d, w, mass, 0., gravity=g)
            moving = rover_on_slope(self.soil, d, w, mass, 0., gravity=g, acceleration=accel)
            return straight_segment_response(d, steady, moving, accel, peak, distance)
        except WheelDomainError as exc:
            raise om.AnalysisError(str(exc)) from exc

    def straight_responder(self, values, segments):
        """Explicit per-evaluation batch: identical loads share a force solution.

        All data are local and discarded after this compute; no hidden persistent
        cache or approximate reuse across optimizer/derivative perturbations.
        """
        d, w, g = values("wheel_diameter"), values("wheel_width"), values("gravity")
        cases = {(mass, profile[4]) for mass, _, profile in segments if profile[0] == "straight"}
        try:
            steady = {mass: rover_on_slope(self.soil, d, w, mass, 0., gravity=g) for mass, _ in cases}
            moving = {(mass, accel): rover_on_slope(self.soil, d, w, mass, 0., gravity=g, acceleration=accel)
                      for mass, accel in cases}
        except WheelDomainError as exc:
            raise om.AnalysisError(str(exc)) from exc
        def response(*, mass, drag, accel, peak, distance):
            return straight_segment_response(d, steady[mass], moving[mass, accel], accel, peak, distance)
        return response


def straight_segment_response(diameter, steady, moving, accel, peak, distance):
    """Pure work/kinematic closure from explicitly supplied wheel force solves."""
    da = .5*peak**2/accel if accel else 0.
    if da > distance/2+1e-10 or (accel == 0 and peak > 0):
        raise WheelDomainError("Rest-to-rest profile does not fit the straight segment")
    rim_force = 4*moving["torque_nm"]/(diameter/2)
    steady_force = 4*steady["torque_nm"]/(diameter/2)
    factor, factor0 = 1-moving["slip"], 1-steady["slip"]
    return {"force": rim_force, "power": max(rim_force*peak/factor, steady_force*peak/factor0),
            "work": rim_force*da/factor+steady_force*(distance-da)/factor0,
            "peak_rim_speed": peak/factor, "ground_speed_factor": factor}


