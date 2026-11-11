import math
import unittest

from turtlebot3_recovery_nodes.mission_config import Waypoint
from turtlebot3_recovery_nodes.recovery_logic import Grid, choose_recovery_waypoint, footprint_is_clear


class RecoveryLogicTests(unittest.TestCase):
    def grid(self, blocked=(), **kwargs):
        cells = [0] * 100
        for col, row, value in blocked:
            cells[row * 10 + col] = value
        return Grid(10, 10, 1.0, kwargs.get('origin_x', 0.0),
                    kwargs.get('origin_y', 0.0), kwargs.get('origin_yaw', 0.0), cells)

    def test_unknown_occupied_and_outside_footprints_are_rejected(self):
        for value in [-1, 50, 100]:
            grid = self.grid([(5, 5, value)])
            self.assertFalse(footprint_is_clear(grid, 5.5, 5.5, 0.2, 50))
        self.assertFalse(footprint_is_clear(self.grid(), 0.1, 3.5, 0.2, 50))
        self.assertFalse(footprint_is_clear(self.grid(), 9.9, 3.5, 0.2, 50))

    def test_footprint_checks_obstacles_beyond_center_cell(self):
        grid = self.grid([(4, 5, 100)])
        self.assertFalse(footprint_is_clear(grid, 5.1, 5.5, 0.2, 50))
        self.assertTrue(footprint_is_clear(grid, 5.3, 5.5, 0.2, 50))

    def test_circle_does_not_reject_a_nonintersecting_diagonal_cell(self):
        grid = self.grid([(4, 4, 100)])
        self.assertTrue(footprint_is_clear(grid, 5.2, 5.2, 0.25, 50))
        self.assertFalse(footprint_is_clear(grid, 5.2, 5.2, 0.3, 50))

    def test_tangent_obstacle_is_rejected_in_all_four_directions(self):
        for col, row in [(0, 1), (2, 1), (1, 0), (1, 2)]:
            cells = [0] * 16
            cells[row * 4 + col] = 100
            grid = Grid(4, 4, 0.5, 0.0, 0.0, 0.0, cells)
            with self.subTest(col=col, row=row):
                self.assertFalse(footprint_is_clear(grid, 0.75, 0.75, 0.25, 50))

    def test_rotated_translated_map_coordinates(self):
        grid = self.grid([(5, 3, 100)], origin_x=20.0, origin_y=-10.0, origin_yaw=math.pi / 2)
        # Grid point (5.5, 3.5) maps to world (16.5, -4.5).
        self.assertFalse(footprint_is_clear(grid, 16.5, -4.5, 0.2, 50))
        self.assertTrue(footprint_is_clear(grid, 16.5, -6.5, 0.2, 50))

    def test_selection_skips_current_position_and_new_obstacle(self):
        points = [Waypoint('here', 2.5, 2.5), Waypoint('blocked', 3.5, 2.5),
                  Waypoint('clear', 4.5, 2.5)]
        point = choose_recovery_waypoint(points, 2.5, 2.5, self.grid(), self.grid([(3, 2, 100)]))
        self.assertEqual(point.name, 'clear')

    def test_static_map_and_costmap_use_distinct_occupancy_semantics(self):
        point = Waypoint('candidate', 5.5, 5.5)
        inflated = self.grid([(5, 5, 80)])
        self.assertEqual(choose_recovery_waypoint([point], 2.5, 2.5, self.grid(), inflated), point)
        self.assertIsNone(choose_recovery_waypoint([point], 2.5, 2.5, inflated, self.grid()))

    def test_no_candidate_and_nonfinite_pose_return_none(self):
        grid = self.grid()
        self.assertIsNone(choose_recovery_waypoint([], 2.5, 2.5, grid, grid))
        self.assertIsNone(choose_recovery_waypoint([Waypoint('p', 3.5, 3.5)], math.nan, 2.5, grid, grid))

    def test_invalid_grid_rejected(self):
        for resolution, cells in [(0.0, [0]), (math.nan, [0]), (0.1, [])]:
            with self.assertRaises(ValueError):
                Grid(1, 1, resolution, 0, 0, 0, cells)


if __name__ == '__main__':
    unittest.main()
