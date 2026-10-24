"""Quasi-static rigid-wheel BWR climb checks, separate from static tipping.

Four equal mean normal loads; empty=dry mass, full=dry+payload mass. No motor
or speed requirement is inferred from the student's slip-only acceptance rule.
"""
import math
import sys

import openmdao.api as om

from ..discipline import Discipline
from .bekker_wong_reece import DATA, NoDrivingSolution, WheelDomainError, load_preset, rover_on_slope, transfer_flags


def slope_response(soil, diameter, width, mass, degrees, gravity=9.80665):
    """Finite failed-demand result; invalid contact/inputs still raise errors.

    A failed force solve reports slip=1 ONLY as an explicit sentinel with valid=0.
    The optimization margin is 1-slip when solved; otherwise the negative force
    deficit divided by normal wheel load. No tolerance makes a failed solve pass.
    """
    angle = math.radians(degrees)
    try:
        point = rover_on_slope(soil, diameter, width, mass, angle, gravity=gravity)
        return {"slip": point["slip"], "solution_valid": 1.,
                "traversal_margin": 1-point["slip"], "force_shortfall": 0.,
                "sinkage": point["sinkage_m"], "transfer_flag_count": float(len(transfer_flags(point)))}
    except NoDrivingSolution as exc:
        normal = mass*gravity*math.cos(angle)/4
        demand = mass*gravity*math.sin(angle)/4
        shortfall = max(0., demand-exc.capacity["drawbar_pull_n"])
        return {"slip": 1., "solution_valid": 0.,
                "traversal_margin": -max(shortfall/normal, sys.float_info.epsilon),
                "force_shortfall": shortfall, "sinkage": exc.capacity["sinkage_m"],
                "transfer_flag_count": float(len(transfer_flags(exc.capacity)))}


SLOPE_UNITS = {"slip": None, "solution_valid": None, "traversal_margin": None,
               "force_shortfall": "N", "sinkage": "m", "transfer_flag_count": None}


class SlopeTraversal(Discipline):
    CONFIG_INPUTS = ("wheel_diameter", "wheel_width", "gravity", "empty_slope_degrees", "loaded_slope_degrees")
    COUPLED_INPUTS = {"dry_mass": "kg", "payload_mass": "kg"}
    OUTPUT_UNITS = {f"{state}_slope_{name}": units
                    for state in ("empty", "loaded") for name, units in SLOPE_UNITS.items()}
    DEPENDENCIES = {f"{state}_slope_{name}":
                    {"wheel_diameter", "wheel_width", "gravity", "dry_mass", f"{state}_slope_degrees"}
                    | ({"payload_mass"} if state == "loaded" else set())
                    for state in ("empty", "loaded") for name in SLOPE_UNITS}

    def initialize(self):
        self.options.declare("soil_manifest", default=DATA)

    def setup(self):
        self.soil, evidence = load_preset(self.options["soil_manifest"])
        self.model_evidence = {"revision": evidence["revision"], "manifest_sha256": evidence["manifest_sha256"],
            "acceptance": "Solved driving slip strictly below 1; no added slip reserve",
            "assumptions": "Rigid wheels, four equal mean wheel loads, quasi-static GRC-1 proxy; no slope motor-speed/torque test",
            "failure_contract": "slip=1 with solution_valid=0 is a sentinel, not a predicted operating point; signed force deficit drives failed trials"}
        super().setup()

    def compute(self, inputs, outputs):
        v = lambda name: inputs[name].item()
        try:
            for state, mass in (("empty", v("dry_mass")), ("loaded", v("dry_mass")+v("payload_mass"))):
                response = slope_response(self.soil, v("wheel_diameter"), v("wheel_width"), mass,
                                          v(state+"_slope_degrees"), v("gravity"))
                for name, value in response.items():
                    outputs[f"{state}_slope_{name}"] = value
        except WheelDomainError as exc:
            raise om.AnalysisError(str(exc)) from exc
