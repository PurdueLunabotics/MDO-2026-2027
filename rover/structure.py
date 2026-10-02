"""Tube mass and elementary beam screens; replace loads with beam FEA/CAD later.

Geometry precedes mass; strength follows payload. Section dimensions are design
variables, so no hidden structural sizing / mass fixed-point loop is needed.
"""
import math
from .components import scalar
from .discipline import Discipline, names


def tube_section(diameter, wall):
    """Area [m2], second moment [m4], polar moment [m4]."""
    inner = max(diameter - 2 * wall, 0.0)
    area = math.pi * (diameter**2 - inner**2) / 4
    inertia = math.pi * (diameter**4 - inner**4) / 64
    return area, inertia, 2 * inertia


class StructureGeometry(Discipline):
    CONFIG_INPUTS = tuple(sorted(names("frame_tube_diameter frame_wall arm_tube_diameter arm_wall tube_density wheelbase track_width top_pivot_height wheel_diameter joint_mass differential_mass pivot_x actuator_pivot_x")))
    OUTPUT_UNITS = {"structure_mass": "kg", "arm_linear_mass": "kg/m", "frame_section_margin": "m", "arm_section_margin": "m"}
    DEPENDENCIES = {
        "structure_mass": names("frame_tube_diameter frame_wall tube_density wheelbase track_width top_pivot_height wheel_diameter joint_mass differential_mass pivot_x actuator_pivot_x"),
        "arm_linear_mass": names("arm_tube_diameter arm_wall tube_density"),
        "frame_section_margin": names("frame_tube_diameter frame_wall"),
        "arm_section_margin": names("arm_tube_diameter arm_wall"),
    }

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        # Two longitudinal rockers, one crossmember, two pivot towers.
        tower = math.hypot(v("actuator_pivot_x")-v("pivot_x"),
                           max(v("top_pivot_height")-v("wheel_diameter")/2, 0))
        length = 2*v("wheelbase") + v("track_width") + 2*tower
        area, _, _ = tube_section(v("frame_tube_diameter"), v("frame_wall"))
        outputs["structure_mass"] = area*length*v("tube_density") + v("joint_mass") + v("differential_mass")
        outputs["arm_linear_mass"] = tube_section(v("arm_tube_diameter"), v("arm_wall"))[0]*v("tube_density")
        for part in ("frame", "arm"):
            outputs[part+"_section_margin"] = 0.25*v(part+"_tube_diameter")-v(part+"_wall")


class StructureLoads(Discipline):
    CONFIG_INPUTS = tuple(sorted(names("frame_tube_diameter frame_wall arm_tube_diameter arm_wall tube_youngs_modulus tube_yield_strength structure_safety_factor structure_load_factor allowable_deflection wheelbase track_width top_pivot_height bottom_pivot_height arm_length gravity pivot_x actuator_pivot_x")))
    COUPLED_INPUTS = {"dry_mass": "kg", "payload_mass": "kg", "drum_mass": "kg", "required_actuator_force": "N", "cut_force": "N"}
    OUTPUT_UNITS = {"structure_stress": "Pa", "structure_stress_margin": "Pa", "structure_deflection": "m", "structure_deflection_margin": "m", "structure_buckling_margin": "N"}
    load = names("dry_mass payload_mass drum_mass required_actuator_force cut_force gravity structure_load_factor wheelbase track_width top_pivot_height bottom_pivot_height arm_length pivot_x actuator_pivot_x")
    section = names("frame_tube_diameter frame_wall arm_tube_diameter arm_wall")
    DEPENDENCIES = {
        "structure_stress": load | section,
        "structure_stress_margin": load | section | names("tube_yield_strength structure_safety_factor"),
        "structure_deflection": load | section | {"tube_youngs_modulus"},
        "structure_deflection_margin": load | section | names("tube_youngs_modulus allowable_deflection"),
        "structure_buckling_margin": load | section | names("tube_youngs_modulus structure_safety_factor"),
    }

    def compute(self, inputs, outputs):
        v = lambda n: scalar(inputs, n)
        af, if_, jf = tube_section(v("frame_tube_diameter"), v("frame_wall"))
        aa, ia, ja = tube_section(v("arm_tube_diameter"), v("arm_wall"))
        factor = v("structure_load_factor")
        weight = (v("dry_mass")+v("payload_mass"))*v("gravity")*factor
        pair = (v("drum_mass")+v("payload_mass"))/2*v("gravity")*factor
        actuator = v("required_actuator_force")*factor
        cut = v("cut_force")/2*factor
        wb, track, arm = v("wheelbase"), v("track_width"), v("arm_length")
        tower = max(math.hypot(v("actuator_pivot_x")-v("pivot_x"),
                               v("top_pivot_height")-v("bottom_pivot_height")), 1e-3)
        # Conservative elementary load envelopes, not an assembled frame solution.
        # (A,I,J,D,length,axial,bending moment,torsion,tip transverse force)
        cases = [
            (af, if_, jf, v("frame_tube_diameter"), wb, weight/2, weight*wb/8, pair*track/2, weight/2),
            (af, if_, jf, v("frame_tube_diameter"), track, weight/2, weight*track/4, pair*arm, weight/2),
            (af, if_, jf, v("frame_tube_diameter"), tower, actuator, actuator*tower, cut*arm, actuator),
            (aa, ia, ja, v("arm_tube_diameter"), arm, actuator+cut, pair*arm, cut*arm, pair),
        ]
        stresses, deflections, buckling = [], [], []
        for area, inertia, polar, diameter, length, axial, moment, torque, force in cases:
            sigma = axial/area + moment*diameter/(2*inertia)
            tau = torque*diameter/(2*polar)
            stresses.append(math.sqrt(sigma*sigma+3*tau*tau))
            deflections.append(force*length**3/(3*v("tube_youngs_modulus")*inertia))
            # Effective-length factor 2: conservative cantilever column.
            buckling.append(math.pi**2*v("tube_youngs_modulus")*inertia/(2*length)**2/v("structure_safety_factor")-axial)
        # Smooth conservative upper envelopes avoid load-case switching kinks.
        stress = sum(x**8 for x in stresses)**(1/8)
        deflection = sum(x**8 for x in deflections)**(1/8)
        outputs["structure_stress"] = stress
        outputs["structure_stress_margin"] = v("tube_yield_strength")/v("structure_safety_factor")-stress
        outputs["structure_deflection"] = deflection
        outputs["structure_deflection_margin"] = v("allowable_deflection")-deflection
        outputs["structure_buckling_margin"] = min(buckling)
