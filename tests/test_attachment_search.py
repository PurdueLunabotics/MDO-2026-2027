"""Opt-in pin optimization changes the search space, not physics/defaults."""
import unittest
from reporting.report import snapshot
from rover.config import configuration
from rover.problem import build_problem
from studies.linkage_multistart import selected_failures, normalized_violation


class AttachmentSearchTests(unittest.TestCase):
    def test_opt_in_variable_and_default_problem_are_independent(self):
        fixed = build_problem({"actuator_attachment_fraction": .8})
        free = build_problem({"actuator_attachment_fraction": .8}, optimize_attachment=True)
        self.addCleanup(fixed.cleanup); self.addCleanup(free.cleanup)
        fixed.run_model(); free.run_model()
        a,b = snapshot(fixed),snapshot(free)
        self.assertEqual(a["outputs"],b["outputs"])
        self.assertNotIn("actuator_attachment_fraction",fixed.design_specs)
        self.assertEqual(free.design_specs["actuator_attachment_fraction"][1:3],(.001,1.))
        self.assertTrue(b["constraints"]["actuator_attachment_fraction_bounds"]["passed"])
        self.assertIn("actuator_attachment_fraction",configuration()[1])
        self.assertFalse(fixed.model_metadata["optimize_attachment"])
        self.assertTrue(free.model_metadata["optimize_attachment"])

    def test_selected_ranking_preserves_excluded_failure_and_frozen_bound_failures(self):
        state={"constraints":{"turn_traction_margin":{"passed":False,"margin":-100.},
                              "wheel_diameter_bounds":{"passed":False,"margin":-.01}}}
        meta={"configuration":{"design":{},"frozen_design":{"wheel_diameter":(.359,.18,.359,"m")},"margins":{}}}
        self.assertEqual(set(selected_failures(state)),{"wheel_diameter_bounds"})
        self.assertAlmostEqual(normalized_violation(state,meta),.01/(.359-.18))
        self.assertFalse(state["constraints"]["turn_traction_margin"]["passed"])

    def test_invalid_attachment_settings_are_rejected(self):
        with self.assertRaises(ValueError):
            build_problem(optimize_attachment="yes")
        for value in (0.,1.01):
            with self.assertRaises(ValueError):
                build_problem({"actuator_attachment_fraction":value},optimize_attachment=True)


if __name__=="__main__":
    unittest.main()
