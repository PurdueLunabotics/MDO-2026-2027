"""Fixed-design input fidelity and report truthfulness, independent of styling."""
import json
import hashlib
import re
from html.parser import HTMLParser
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from reporting.case_inputs import read_case
from reporting.html_report import write_report, top_view_svg, values_from_snapshot
from reporting.archive import archive_cases
from rover.problem import build_problem
from reporting.report import snapshot


class CheckRows(HTMLParser):
    def __init__(self, html):
        super().__init__()
        self.checks = {}
        self.feed(html)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == "tr" and "data-port" in attrs:
            self.checks[attrs["data-port"]] = attrs["data-check"]


class DesignReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p = build_problem(structure="plate-frame", motors="x44-standard")
        try:
            p.run_model()
            cls.state = snapshot(p)
            cls.metadata = p.model_metadata
        finally:
            p.cleanup()

    def record(self):
        return {"baseline": self.state, "final": self.state, "status": "baseline",
                "optimizer_success": None, "openmdao_version": "test",
                "model_metadata": self.metadata}

    def test_failed_baseline_reports_each_saved_check_without_optimizing(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"baseline.json"
            write_report(self.record(), output)
            html = output.with_suffix(".html").read_text(encoding="utf-8")
            self.assertFalse(self.state["feasible"])
            expected = {n: "pass" if c["passed"] else "fail" for n, c in self.state["constraints"].items()}
            self.assertEqual(CheckRows(html).checks, expected)
            self.assertIn("Design baseline evaluation", html)
            self.assertIn("Not assessed", html)
            self.assertIn("Predictions are conditional", html)
            self.assertNotIn('id="mdo"', html)
            self.assertNotIn('id="iteration"', html)
            self.assertNotIn("<th>Final iterate</th>", html)
            self.assertIn('id="analysis-data"', html)
            self.assertIn('Front-left wheel on rock', html)
            self.assertIn('Common roll removed', html)
            self.assertTrue(output.with_name('baseline_obstacle.svg').exists())
            self.assertNotIn('_inputs.json', html)

    def test_failed_optimizer_is_not_labeled_as_a_released_baseline(self):
        record = self.record()
        record.update(status="failed_or_infeasible", optimizer_success=False)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"study.json"
            write_report(record, output)
            html = output.with_suffix(".html").read_text(encoding="utf-8")
            self.assertIn("Optimizer did not converge successfully", html)
            self.assertIn('id="mdo"', html)
            self.assertIn("candidate is separate from a released design baseline", html)
            self.assertIn('id="iteration"', html)

    def test_names_are_escaped_and_snapshot_is_not_modified(self):
        record = self.record()
        record["report_context"] = {"name": '<img src=x onerror="bad()">', "revision": "<draft>"}
        original = json.dumps(record, sort_keys=True)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"case.json"
            write_report(record, output)
            html = output.with_suffix(".html").read_text(encoding="utf-8")
            self.assertNotIn('<img src=x onerror="bad()">', html)
            self.assertIn("&lt;draft&gt;", html)
        self.assertEqual(json.dumps(record, sort_keys=True), original)

    def test_top_view_preserves_four_wheel_and_four_drum_footprints(self):
        values = values_from_snapshot(self.state)
        root = ET.fromstring(top_view_svg(values))
        parts = list(root.iter('{http://www.w3.org/2000/svg}rect'))
        wheels = [p for p in parts if p.get('data-part') == 'wheel']
        drums = [p for p in parts if p.get('data-part') == 'drum']
        self.assertEqual(len(wheels), 4)
        self.assertEqual(len(drums), 4)
        # Orthographic projection must use the same scale on both axes.
        for part, dx, dy in ((wheels[0], values['wheel_diameter'], values['wheel_width']),
                             (drums[0], values['drum_diameter'], values['drum_length'])):
            self.assertAlmostEqual(float(part.get('width'))/float(part.get('height')), dx/dy, places=4)
        self.assertIn('Projection overlap does not prove collision', ''.join(root.itertext()))

    def test_archive_keeps_failed_candidates_failed_and_rejects_unrelated_inputs(self):
        baseline = self.record()
        candidate = self.record()
        candidate.update(status='failed_or_infeasible', optimizer_success=False)
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            a, b, dest = folder/'a.json', folder/'b.json', folder/'archive'
            source, optimized = folder/'design_baseline.json', folder/'design_optimized.json'
            source.write_text(json.dumps({'inputs': {n:i['value'] for g in ('design','assumptions')
                                                    for n,i in self.state[g].items()}}), encoding='utf-8')
            source_before = source.read_bytes()
            a.write_text(json.dumps(baseline), encoding='utf-8')
            b.write_text(json.dumps(candidate), encoding='utf-8')
            archive_cases(a, b, dest, baseline_inputs=source, optimized_inputs=optimized)
            self.assertEqual(source.read_bytes(), source_before)
            self.assertEqual(list(dest.glob('*.json')), [])
            saved = json.loads(optimized.read_text())
            self.assertFalse(saved['notes']['optimizer_success'])
            self.assertFalse(saved['notes']['modeled_feasible'])
            html = (dest/'optimized.html').read_text(encoding='utf-8')
            embedded = re.search(r'<script id="analysis-data" type="application/json">(.*?)</script>', html).group(1)
            self.assertEqual(json.loads(embedded), json.loads(b.read_text()))
            self.assertIn('href="../design_optimized.json"', html)
            self.assertNotIn('href="optimized.json"', html)
            self.assertIn('Optimizer did not converge successfully', html)
            self.assertNotIn('href="optimized_n2.html"', html)
            self.assertTrue((dest/'optimized_top.svg').exists())
            candidate['baseline'] = json.loads(json.dumps(self.state))
            candidate['baseline']['design']['wheelbase']['value'] += .01
            b.write_text(json.dumps(candidate), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'start from'):
                archive_cases(a, b, dest, baseline_inputs=source, optimized_inputs=optimized)
            b.write_text(json.dumps(self.record() | {'status':'failed_or_infeasible', 'optimizer_success':False}), encoding='utf-8')
            data = json.loads(source.read_text())
            data['inputs']['wheelbase'] += .01
            source.write_text(json.dumps(data), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'source has edits'):
                archive_cases(a, b, dest, baseline_inputs=source, optimized_inputs=optimized)

    def test_case_reader_validates_numeric_data_and_hashes_source(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"case.json"
            path.write_text(json.dumps({"name": "Club baseline", "revision": "A",
                                       "inputs": {"wheelbase": .48}}), encoding="utf-8")
            values, metadata = read_case(path)
            self.assertEqual(values, {"wheelbase": .48})
            self.assertEqual(metadata["revision"], "A")
            self.assertEqual(metadata["sha256"], hashlib.sha256(path.read_bytes()).hexdigest())
            for bad in (True, "0.48", None, float("nan"), float("inf")):
                path.write_text(json.dumps({"inputs": {"wheelbase": bad}}), encoding="utf-8")
                with self.subTest(bad=bad), self.assertRaises(ValueError):
                    read_case(path)

    def test_cli_rejects_outputs_that_would_overwrite_source_inputs(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"evaluation.json"
            for path in (output, Path(folder)/"evaluation_top.svg", Path(folder)/"evaluation_obstacle.svg"):
                path.write_text('{"inputs": {"wheelbase": 0.48}}', encoding="utf-8")
                before = path.read_bytes()
                result = subprocess.run([sys.executable, "main.py", "--inputs", str(path),
                    "--output", str(output)], cwd=Path(__file__).resolve().parents[1],
                    capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
                self.assertIn("overwrite the source baseline", result.stderr)
                self.assertEqual(path.read_bytes(), before)

    def test_input_file_defaults_to_fixed_analysis_and_preserves_overrides(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"ssot.json"
            output = Path(folder)/"evaluation.json"
            path.write_text(json.dumps({"name": "Club baseline", "revision": "A",
                "structure": "plate-frame", "motors": "x44-standard",
                "inputs": {"wheelbase": .48, "wheel_width": .12}}), encoding="utf-8")
            raw_before = path.read_bytes()
            result = subprocess.run([sys.executable, "main.py", "--inputs", str(path),
                "--set", "wheel_width=0.13", "--output", str(output)],
                cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            record = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(record["status"], "baseline")
            self.assertIsNone(record["optimizer_success"])
            self.assertEqual(record["baseline"]["design"]["wheelbase"]["value"], .48)
            self.assertEqual(record["final"]["design"]["wheel_width"]["value"], .13)
            self.assertEqual(record["model_metadata"]["motor_profile"], "x44-standard")
            self.assertEqual(record["report_context"]["revision"], "A")
            self.assertEqual(record["report_context"]["cli_override_names"], ["wheel_width"])
            self.assertFalse(record["final"]["feasible"])
            self.assertFalse(output.with_suffix(".sql").exists())
            self.assertEqual(path.read_bytes(), raw_before)
            self.assertFalse(output.with_name("evaluation_inputs.json").exists())
            html = output.with_suffix('.html').read_text(encoding='utf-8')
            embedded = re.search(r'<script id="analysis-data" type="application/json">(.*?)</script>', html).group(1)
            self.assertEqual(json.loads(embedded), record)
            self.assertIn('href="ssot.json"', html)


if __name__ == "__main__":
    unittest.main()
