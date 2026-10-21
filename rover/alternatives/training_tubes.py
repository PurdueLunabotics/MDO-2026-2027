import math

import openmdao.api as om

from rover.components import scalar
from rover.structure import StructureLoads, tube_section


def tube_response(diameter, wall, length, axial, moment, torque, force,
                  youngs_modulus, safety_factor, *, include_shear=True):
    """Elementary elastic tube screen. SI units; axial is compression >= 0."""
    values = (diameter, wall, length, axial, moment, torque, force,
              youngs_modulus, safety_factor)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Tube inputs must be finite.")
    if diameter <= 0 or not 0 < wall < diameter / 2:
        raise ValueError("Require D > 0 and 0 < wall < D/2.")
    if length <= 0 or youngs_modulus <= 0 or safety_factor <= 0:
        raise ValueError("Length, E, and safety factor must be positive.")
    if axial < 0 or force < 0:
        raise ValueError("Compression and transverse envelopes must be >= 0.")

    area, inertia, polar = tube_section(diameter, wall)
    outer = diameter / 2
    inner = outer - wall
    sigma = axial / area + abs(moment) * outer / inertia
    tau = abs(torque) * outer / polar
    transverse = force * (outer**2 + outer*inner + inner**2) / (3*inertia)
    if include_shear:
        tau += transverse
    return {
        "stress": math.sqrt(sigma**2 + 3*tau**2),
        "deflection": force * length**3 / (3*youngs_modulus*inertia),
        "buckling_margin": (
            math.pi**2 * youngs_modulus * inertia / (2*length)**2
            / safety_factor - axial
        ),
    }

class TrainingTubeLoads(StructureLoads):
    # This class owns its dependency sets, including inactive load cases.
    DEPENDENCIES = {
        name: set(inputs)
        for name, inputs in StructureLoads.DEPENDENCIES.items()
    }

    def setup(self):
        self.model_evidence = {
            "revision": "training-tubes-v1",
            "status": "analytical teaching example; no hardware validation",
            "change": "add annular transverse-shear stress envelope",
            "domain": "isotropic elastic hollow tubes; reference load envelopes",
            "omissions": "joints, fatigue, local buckling, assembled load sharing",
        }
        super().setup()

    def compute(self, inputs, outputs):
        v = lambda name: scalar(inputs, name)
        factor = v("structure_load_factor")
        weight = (v("dry_mass") + v("payload_mass"))*v("gravity")*factor
        pair = (v("drum_mass") + v("payload_mass"))/2*v("gravity")*factor
        actuator = v("required_actuator_force")*factor
        cut = v("cut_force")/2*factor
        wb, track, arm = v("wheelbase"), v("track_width"), v("arm_length")
        tower = max(math.hypot(
            v("actuator_pivot_x") - v("pivot_x"),
            v("top_pivot_height") - v("bottom_pivot_height")), 1e-3)

        # Same four load envelopes and tower-length guard as the reference.
        # family, length, compression, bending moment, torque, transverse force
        cases = [
            ("frame", wb, weight/2, weight*wb/8, pair*track/2, weight/2),
            ("frame", track, weight/2, weight*track/4, pair*arm, weight/2),
            ("frame", tower, actuator, actuator*tower, cut*arm, actuator),
            ("arm", arm, actuator+cut, pair*arm, cut*arm, pair),
        ]
        responses = []
        for family, length, axial, moment, torque, force in cases:
            try:
                response = tube_response(
                    v(family + "_tube_diameter"), v(family + "_wall"),
                    length, axial, moment, torque, force,
                    v("tube_youngs_modulus"), v("structure_safety_factor"))
            except ValueError as exc:
                raise om.AnalysisError(str(exc)) from exc
            responses.append(response)

        stress = sum(item["stress"]**8 for item in responses)**(1/8)
        deflection = sum(item["deflection"]**8 for item in responses)**(1/8)
        outputs["structure_stress"] = stress
        outputs["structure_stress_margin"] = (
            v("tube_yield_strength")/v("structure_safety_factor") - stress)
        outputs["structure_deflection"] = deflection
        outputs["structure_deflection_margin"] = v("allowable_deflection") - deflection
        outputs["structure_buckling_margin"] = min(
            item["buckling_margin"] for item in responses)