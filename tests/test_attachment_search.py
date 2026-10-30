"""Opt-in pin optimization changes the search space, not physics/defaults."""
import unittest
from unittest.mock import patch
from reporting.report import snapshot
from rover.config import configuration, DESIGN
from rover.problem import build_problem
from studies.linkage_multistart import selected_failures, normalized_violation


class AttachmentSearchTests(unittest.TestCase):
    def test_offsets_are_design_variables_and_can_be_frozen_independently(self):
        changes = {"lin_act_side_arm_offset": .05, "lin_act_top_arm_offset": .05}
        # Original signed-offset search fixture; current hardware bounds stay intact.
        with patch.dict(DESIGN, {"lin_act_side_arm_offset": (0., -.15, .15, "m")}):
            fixed = build_problem(changes, freeze=list(changes))
            free = build_problem(changes)
        self.addCleanup(fixed.cleanup); self.addCleanup(free.cleanup)
        fixed.run_model(); free.run_model()
        a,b = snapshot(fixed),snapshot(free)
        self.assertEqual(a["outputs"],b["outputs"])
        for name in changes:
            self.assertNotIn(name,fixed.design_specs)
            self.assertEqual(free.design_specs[name][1:3],(-.15,.15))
            self.assertTrue(b["constraints"][name+"_bounds"]["passed"])
            self.assertIn(name,configuration()[0])
        self.assertNotIn("actuator_attachment_fraction",configuration()[1])

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
        for value in (-.151,.151):
            with self.assertRaises(ValueError):
                build_problem({"lin_act_side_arm_offset":value},optimize_attachment=True)


if __name__=="__main__":
    unittest.main()
