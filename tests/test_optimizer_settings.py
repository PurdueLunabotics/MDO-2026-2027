import math
import unittest

from rover.problem import build_problem


class OptimizerSettingsTests(unittest.TestCase):
    def problem(self, **options):
        p = build_problem(**options)
        self.addCleanup(p.cleanup)
        return p

    def test_profile_defaults_are_explicit_and_recorded(self):
        for profile, optimizer, tol, maxiter, radius in (
                ("reference", "SLSQP", 1e-3, 200, None),
                ("plate-frame", "COBYLA", 1e-5, 2000, .02)):
            with self.subTest(profile=profile):
                p = self.problem(structure=profile)
                settings = p.model_metadata["optimizer_settings"]
                self.assertEqual(settings, {"optimizer": optimizer, "tol": tol,
                                 "maxiter": maxiter, "cobyla_radius": radius})
                self.assertEqual(p.driver.options["optimizer"], optimizer)
                self.assertEqual(p.driver.options["tol"], tol)
                self.assertEqual(p.driver.options["maxiter"], maxiter)
                if radius:
                    self.assertEqual(p.driver.opt_settings["rhobeg"], radius)

    def test_explicit_solver_choices_override_profile_defaults(self):
        p = self.problem(structure="plate-frame", optimizer="SLSQP",
                         maxiter=17, solver_tol=7e-6)
        self.assertEqual(p.driver.options["optimizer"], "SLSQP")
        self.assertEqual(p.driver.options["maxiter"], 17)
        self.assertEqual(p.driver.options["tol"], 7e-6)
        self.assertNotIn("rhobeg", p.driver.opt_settings)
        self.assertEqual(p.model_metadata["optimizer_settings"]["tol"], 7e-6)

    def test_x44_uses_tighter_accuracy_with_either_frame(self):
        p = self.problem(motors="x44-standard")
        self.assertEqual(p.driver.options["tol"], 1e-5)
        self.assertEqual(p.driver.options["optimizer"], "SLSQP")

    def test_invalid_numerical_settings_are_rejected(self):
        for tol in (0, -1, math.nan, math.inf):
            with self.subTest(tol=tol), self.assertRaises(ValueError):
                build_problem(solver_tol=tol)
        for maxiter in (0, -1, 1.5, True):
            with self.subTest(maxiter=maxiter), self.assertRaises(ValueError):
                build_problem(maxiter=maxiter)
        with self.assertRaises(ValueError):
            build_problem(optimizer="unsupported")
        with self.assertRaises(ValueError):
            build_problem(optimizer="SLSQP", cobyla_radius=.02)
