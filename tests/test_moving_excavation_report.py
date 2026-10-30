"""Report views preserve fresh-layer accounting and saved failure decisions."""
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from reporting.html_report import write_report
from reporting.moving_excavation_drawing import moving_excavation_svg, feed_rpm_svg
from reporting.report import snapshot
from rover.problem import build_problem


SVG = '{http://www.w3.org/2000/svg}'


class MovingExcavationReportTests(unittest.TestCase):
    def drawing_values(self):
        return dict(drum_diameter=.28, inlet_depth=.012,
                    front_dig_x=.65, front_dig_y=.128,
                    rear_dig_x=-.64, rear_dig_y=.116,
                    feed_speed=.005, drum_rpm=20., scoops_per_revolution=4.,
                    feed_per_scoop=.00375,
                    feed_margin=.00825, feed_speed_limit=.016,
                    feed_distance=.8, feed_drive_distance=2.09, feed_time=418.)

    def test_soil_view_distinguishes_second_fresh_layer_and_recovery(self):
        root = ET.fromstring(moving_excavation_svg(self.drawing_values()))
        text = ''.join(root.itertext())
        self.assertIn('Depth 12.0 mm = d', text)
        self.assertIn('Depth 24.0 mm = 2d', text)
        self.assertIn('full drum width', text)
        self.assertIn('does not count', text)
        self.assertIn('rear reaches that start and lowers', text)
        self.assertIn('separate plunge/recovery reach screen', text)
        self.assertEqual(len(list(root.iter(SVG+'circle'))), 4)

    def test_chart_shows_current_evaluated_point_and_failed_feed(self):
        values = self.drawing_values()
        values.update(feed_speed=.03, feed_per_scoop=.0225, feed_margin=-.0105)
        root = ET.fromstring(feed_rpm_svg(values))
        current = next(n for n in root.iter() if n.get('data-current-rpm'))
        self.assertEqual(float(current.get('data-current-rpm')), 20.)
        self.assertEqual(float(current.get('data-current-feed')), .03)
        self.assertEqual(current.get('fill'), '#b74335')
        self.assertIn('Motor, reach and traction checks still apply', ''.join(root.itertext()))

    def test_moving_diagnostics_and_views_are_exported(self):
        problem = build_problem()
        try:
            problem.run_model()
            state = snapshot(problem)
        finally:
            problem.cleanup()
        self.assertIn('feed_per_scoop', state['outputs'])
        self.assertIn('axle_ground_margin', state['constraints'])
        record = dict(baseline=state, final=state, status='baseline', optimizer_success=None)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/'moving.json'
            write_report(record, output)
            html = output.with_suffix('.html').read_text(encoding='utf-8')
            self.assertIn('id="excavation"', html)
            self.assertIn('Maximum extension · plunge / recovery reach', html)
            self.assertIn('Moving excavation has no lowering/cutting overlap credit', html)
            self.assertNotIn('Dig and lower concurrently', html)
            self.assertTrue(output.with_name('moving_excavation.svg').exists())
            self.assertTrue(output.with_name('moving_feed_rpm.svg').exists())

    def test_zero_or_negative_domain_reserve_is_never_tolerated_as_valid(self):
        problem = build_problem()
        try:
            problem.run_model()
            for margin in (0., -1e-12):
                problem.set_val('excavation_domain_margin', margin)
                self.assertFalse(snapshot(problem)['constraints']['excavation_domain_margin']['passed'])
        finally:
            problem.cleanup()


if __name__ == '__main__':
    unittest.main()
