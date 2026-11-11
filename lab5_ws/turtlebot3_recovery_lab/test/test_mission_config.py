import tempfile
import unittest
from pathlib import Path

from turtlebot3_recovery_nodes.mission_config import load_mission


class MissionConfigTests(unittest.TestCase):
    def load(self, text, **kwargs):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mission.yaml'
            path.write_text(text, encoding='utf-8')
            return load_mission(path, **kwargs)

    def test_preserves_goal_order_and_optional_heading(self):
        mission = self.load('''
frame_id: map
waypoints:
  - {name: first, x: 2, y: -1}
  - {name: second, x: 0, y: 3, yaw: 1.57}
recovery_waypoints:
  - {x: -0.5, y: -1.0}
''', require_waypoints=True, require_recovery=True)
        self.assertEqual([point.name for point in mission.waypoints], ['first', 'second'])
        self.assertEqual(mission.waypoints[0].yaw, 0.0)
        self.assertEqual(mission.recovery_waypoints[0].x, -0.5)

    def test_empty_template_cannot_start_a_mission_or_recovery(self):
        for flag in ['require_waypoints', 'require_recovery']:
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                self.load('waypoints: []\nrecovery_waypoints: []', **{flag: True})

    def test_bad_or_ambiguous_coordinate_values_are_rejected(self):
        for value in ['.nan', '.inf', 'true', 'null', '"2.0"']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.load(f'waypoints: [{{x: {value}, y: 0}}]')

    def test_misspelled_keys_and_missing_coordinates_are_rejected(self):
        for text in ['wayponts: []', 'waypoints: [{x: 1, yy: 2}]',
                     'waypoints: [{x: 1}]', 'waypoints: {x: 1, y: 2}',
                     'frame_id: /map', 'frame_id: ""',
                     'waypoints: [{name: p, x: 1, y: 2}, {name: p, x: 3, y: 4}]']:
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.load(text)

    def test_missing_file_and_invalid_yaml_have_clear_errors(self):
        with self.assertRaisesRegex(ValueError, 'Cannot read mission'):
            load_mission('/does-not-exist/lab5-mission.yaml')
        with self.assertRaisesRegex(ValueError, 'Cannot read mission'):
            self.load('waypoints: [')


if __name__ == '__main__':
    unittest.main()
