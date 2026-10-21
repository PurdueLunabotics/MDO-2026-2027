import math
import unittest

from rover.alternatives.training_tubes import TrainingTubeLoads, tube_response
from rover.models import ModelSet
from rover.problem import build_problem


class TrainingTubeTests(unittest.TestCase):
    def response(self, **changes):
        # SYNTHETIC loads and illustrative material inputs, not measurements.
        values = dict(diameter=.05, wall=.002, length=.6, axial=1000.,
                      moment=80., torque=10., force=200.,
                      youngs_modulus=69e9, safety_factor=2.)
        values.update(changes)
        return tube_response(**values)

    def test_pure_bending_and_zero_load(self):
        # Independent section integration for a circular annulus.
        inertia = math.pi*(.025**4 - .023**4)/4
        bending = self.response(axial=0., torque=0., force=0.)
        self.assertAlmostEqual(bending["stress"], 80*.025/inertia)
        empty = self.response(axial=0., moment=0., torque=0., force=0.)
        self.assertEqual(empty["stress"], 0.)
        self.assertEqual(empty["deflection"], 0.)
        capacity = math.pi**2*69e9*inertia/(1.2**2)/2
        self.assertAlmostEqual(empty["buckling_margin"], capacity)

    def test_solid_and_thin_tube_shear_limits(self):
        # Independent known limits: 4V/(3A) for solid, 2V/A for thin tube.
        for wall, coefficient in ((.025-1e-9, 4/3), (1e-8, 2.)):
            area = math.pi*wall*(.05-wall)
            result = self.response(wall=wall, axial=0., moment=0., torque=0.)
            expected = math.sqrt(3)*coefficient*200/area
            self.assertAlmostEqual(result["stress"]/expected, 1., places=5)

    def test_scaling_and_failed_buckling(self):
        first = self.response()
        longer = self.response(length=1.2)
        self.assertAlmostEqual(longer["deflection"], 8*first["deflection"])
        self.assertAlmostEqual(longer["buckling_margin"] + 1000.,
                               (first["buckling_margin"] + 1000.)/4)
        self.assertLess(self.response(axial=1e9)["buckling_margin"], 0.)

    def test_invalid_domain_is_rejected(self):
        for changes in (dict(diameter=0.), dict(wall=0.), dict(wall=.025),
                        dict(wall=.03), dict(length=0.), dict(axial=-1.),
                        dict(force=-1.), dict(youngs_modulus=0.),
                        dict(safety_factor=0.), dict(torque=math.nan)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.response(**changes)

    def test_no_shear_recovers_original_per_case_stress(self):
        area = math.pi*(.05**2 - .046**2)/4
        inertia = math.pi*(.05**4 - .046**4)/64
        sigma = 1000/area + 80*.025/inertia
        tau = 10*.025/(2*inertia)
        original = self.response(include_shear=False)
        self.assertAlmostEqual(original["stress"], math.sqrt(sigma**2+3*tau**2))
        self.assertGreater(self.response()["stress"], original["stress"])

    def test_replacement_preserves_fixed_design_accounting(self):
        reference = build_problem(structure="reference", motors="reference")
        self.addCleanup(reference.cleanup)
        replacement = build_problem(
            structure="reference", motors="reference",
            models=ModelSet(structure_loads=TrainingTubeLoads))
        self.addCleanup(replacement.cleanup)
        reference.run_model()
        replacement.run_model()
        get = lambda problem, name: problem.get_val(name).item()
        self.assertGreater(get(replacement, "structure_stress"),
                           get(reference, "structure_stress"))
        self.assertLess(get(replacement, "structure_stress_margin"),
                        get(reference, "structure_stress_margin"))
        for name in ("dry_mass", "payload_mass", "cycle_energy",
                     "construction_points", "structure_deflection",
                     "structure_buckling_margin"):
            self.assertAlmostEqual(get(replacement, name), get(reference, name))