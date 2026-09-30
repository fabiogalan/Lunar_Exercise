"""Behavior checks for homing, clearance, pickup, and mission termination.
Run: python3 -m unittest discover -s tools/sim -p 'test_*.py' -v
"""
import contextlib
import importlib
import io
import math
import unittest

import vex
from run import load_robot


def rotated_cube(cx, angle, front=80.0):
    """Nearest ray intersection with a real 75 mm square, including both faces."""
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    points = [(cx + x * c - z * s, x * s + z * c)
              for x, z in [(-37.5, -37.5), (37.5, -37.5), (37.5, 37.5), (-37.5, 37.5)]]
    offset = front - min(z for x, z in points)

    def depth(x):
        hits = []
        for a, b in zip(points, points[1:] + points[:1]):
            if min(a[0], b[0]) <= x <= max(a[0], b[0]) and a[0] != b[0]:
                hits.append(a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]))
        return min(hits) + offset

    return min(x for x, z in points), max(x for x, z in points), depth


class RobotTests(unittest.TestCase):
    def setUp(self):
        importlib.reload(vex)
        vex.SIM['noise'] = 0
        self.g = load_robot()
        self.output = io.StringIO()
        self.capture = contextlib.redirect_stdout(self.output)
        self.capture.__enter__()
        self.addCleanup(self.capture.__exit__, None, None, None)

    def robot(self):
        return self.g['Robot']()

    def scan(self, objects=(), unknown=(), start=-400, end=-600, missing=()):
        search = self.g['CubeSearch']()
        for x in range(start, end - 1, -1):
            if x in missing:
                continue
            z = 350.0
            for lo, hi, depth in objects:
                if lo <= x <= hi:
                    z = min(z, depth(x) if callable(depth) else depth)
            if any(lo <= x <= hi for lo, hi in unknown):
                z = None
            target = search.add(x, z)
            if target is not None:
                return target
        return None

    def test_full_cube_needs_both_edges_and_second_gap(self):
        cube = [(-537.5, -462.5, 80)]
        self.assertIsNone(self.scan(cube, end=-548))
        self.assertEqual(self.scan(cube), (-500, 80))

    def test_small_gap_on_either_side_is_rejected(self):
        cube = [(-537.5, -462.5, 80)]
        for blocker in [(-454, -440, 80), (-560, -547, 80)]:
            with self.subTest(blocker=blocker):
                self.assertIsNone(self.scan(cube + [blocker]))

    def test_twenty_mm_gaps_are_accepted(self):
        self.assertEqual(self.scan([(-537.5, -462.5, 80),
                                    (-442, -425, 80), (-575, -558, 80)]), (-500, 80))

    def test_unknown_gaps_are_not_clearance(self):
        cube = [(-537.5, -462.5, 80)]
        for unknown in [[(-452, -450)], [(-550, -547)], [(-503, -500)]]:
            with self.subTest(unknown=unknown):
                self.assertIsNone(self.scan(cube, unknown=unknown))

    def test_partial_or_merged_faces_are_rejected(self):
        for lo, hi in [(-520, -480), (-575, -425)]:
            self.assertIsNone(self.scan([(lo, hi, 80)]))

    def test_scan_boundaries_cannot_prove_full_cube(self):
        cube = [(-537.5, -462.5, 80)]
        self.assertIsNone(self.scan(cube, start=-480))
        self.assertIsNone(self.scan(cube, end=-520))

    def test_deeper_surface_must_clear_entire_jaw_path(self):
        cube = [(-537.5, -462.5, 80)]
        self.assertIsNone(self.scan(cube + [(-600, -400, 140)]))
        self.assertEqual(self.scan(cube + [(-600, -400, 175)]), (-500, 80))

    def test_sloping_rotated_face_is_accepted(self):
        target = self.scan([(-543, -457, lambda x: 80 + (x + 543) * 0.15)])
        self.assertIsNotNone(target)
        self.assertAlmostEqual(target[0], -500, delta=1)

    def test_front_cube_partly_covering_rear_cube_is_pickable(self):
        for rear_x in [-540, -460]:
            with self.subTest(rear_x=rear_x):
                target = self.scan([(-537.5, -462.5, 80),
                                    (rear_x - 37.5, rear_x + 37.5, 175)])
                self.assertEqual(target, (-500, 80))

    def test_rear_cube_fragment_is_not_mistaken_for_whole_cube(self):
        # Rear cube centre -540: only its left 40 mm are exposed. Stop the
        # scan before enough left clearance exists for the front cube.
        objects = [(-537.5, -462.5, 80), (-577.5, -502.5, 175)]
        self.assertIsNone(self.scan(objects, end=-548))
        # Block the front cube's other jaw: neither cube should now qualify.
        self.assertIsNone(self.scan(objects + [(-454, -440, 80)]))

    def test_rotated_cube_two_visible_faces_are_one_target(self):
        for angle in [-10, -5, 5, 10]:
            with self.subTest(angle=angle):
                target = self.scan([rotated_cube(-500, angle)])
                self.assertIsNotNone(target)
                self.assertAlmostEqual(target[0], -500, delta=1)

    def test_rotated_front_cube_can_cover_rear_cube(self):
        for angle in [-10, 10]:
            for rear_x in [-540, -460]:
                with self.subTest(angle=angle, rear_x=rear_x):
                    target = self.scan([rotated_cube(-500, angle),
                                        (rear_x - 37.5, rear_x + 37.5, 175)])
                    self.assertIsNotNone(target)
                    self.assertAlmostEqual(target[0], -500, delta=1)

    def test_rotated_grab_depth_accounts_for_sideways_sensor_offset(self):
        for angle in [-10, 10]:
            for offset in [-23, 23]:
                with self.subTest(angle=angle, offset=offset):
                    self.g['DIST_DX_MM'] = offset
                    cube = rotated_cube(-500, angle)
                    target = self.scan([cube])
                    self.assertIsNotNone(target)
                    self.assertAlmostEqual(target[1], cube[2](-500 + offset), delta=1)

    def test_rotation_does_not_override_a_blocked_jaw_path(self):
        objects = [rotated_cube(-500, 10), (-497.5, -422.5, 170)]
        self.g['DIST_DX_MM'] = 23
        self.assertIsNone(self.scan(objects))  # taught grab depth puts tips too close
        self.g['DIST_DX_MM'] = -23
        self.assertIsNotNone(self.scan(objects))

    def test_sample_holes_do_not_hide_cube_edges(self):
        self.assertIsNone(self.scan([(-537.5, -462.5, 80)], missing=range(-505, -495)))

    def test_search_memory_is_bounded(self):
        search = self.g['CubeSearch']()
        for x in range(0, -10000, -1):
            search.add(x, 350)
        self.assertLess(len(search.samples), 150)

    def test_homing_from_unknown_start_preserves_physical_position(self):
        r = self.robot()
        self.assertEqual(r.X.mm(), 0)  # encoder zero at startup is NOT home
        self.assertEqual(vex._mm(5), -200)
        r.home_x()
        self.assertTrue(r.homed)
        self.assertAlmostEqual(r.X.mm(), -10, delta=1)
        self.assertAlmostEqual(vex._mm(5), -10, delta=1.5)
        self.assertFalse(r.hw.home.pressing())
        self.assertEqual(r.hw.mx._mode, 'idle')

    def test_initially_pressed_switch_is_released_and_reapproached(self):
        vex.SIM['x_start'] = -0.1
        r = self.robot()
        r.home_x()
        self.assertTrue(r.homed)
        self.assertLess(vex._mm(5), -8)

    def test_missing_stuck_and_never_triggered_switch_stop(self):
        for mode in ['missing', 'stuck', 'never']:
            with self.subTest(mode=mode):
                importlib.reload(vex)
                vex.SIM['home_mode'] = mode
                self.g = load_robot()
                r = self.robot()
                with self.assertRaises(self.g['MotionError']):
                    r.home_x()
                self.assertFalse(r.homed)
                self.assertEqual(r.hw.mx._mode, 'idle')

    def test_x_requires_homing_and_retracted_arm(self):
        r = self.robot()
        with self.assertRaises(self.g['MotionError']):
            r.move_x(-300, 50)
        r.home_x()
        r.Z.move_to(20, 50)
        with self.assertRaises(self.g['MotionError']):
            r.move_x(-300, 50)

    def test_sensor_offset_is_used_for_alignment(self):
        for offset in [-23, 23]:
            with self.subTest(offset=offset):
                importlib.reload(vex)
                vex.SIM.update(noise=0, dist_dx=offset,
                               cubes=[{'x': -500.0, 'z': 80.0, 'colour': 'green'}])
                self.g = load_robot()
                self.g['DIST_DX_MM'] = offset
                r = self.robot()
                r.home_x()
                target = self.g['find_cube'](r, [], 200000)
                self.assertAlmostEqual(target[0], -500, delta=2)
                self.assertTrue(self.g['pick_cube'](r, target))
                r.stop_all()

    def test_removed_front_cube_exposes_rear_cube_next_cycle(self):
        vex.SIM['cubes'] = [{'x': -500., 'z': 80., 'colour': 'green'},
                            {'x': -500., 'z': 175., 'colour': 'red'}]
        r = self.robot()
        self.g['Mission'](r).run()
        self.assertTrue(all(c.get('placed') for c in vex.SIM['cubes']))
        self.assertLess(vex._t[0], self.g['MISSION_S'] * 1000)
        self.assertAlmostEqual(r.X.mm(), self.g['HOME_CLEAR_X'], delta=1)

    def test_full_zone_does_not_reuse_slot(self):
        self.g['GREEN_SLOTS'] = ((-250., 240.),)
        vex.SIM['cubes'] = [{'x': -500., 'z': 80., 'colour': 'green'},
                            {'x': -700., 'z': 80., 'colour': 'green'}]
        r = self.robot()
        self.g['Mission'](r).run()
        self.assertEqual(sum(bool(c.get('placed')) for c in vex.SIM['cubes']), 1)
        self.assertIsNotNone(vex._held())
        self.assertAlmostEqual(r.X.mm(), self.g['HOME_CLEAR_X'], delta=1)

    def test_unknown_colour_returns_home_holding_cube(self):
        self.g['identify'] = lambda r: 'unknown'
        vex.SIM['cubes'] = [{'x': -500., 'z': 80., 'colour': 'green'}]
        r = self.robot()
        self.g['Mission'](r).run()
        self.assertIsNotNone(vex._held())
        self.assertFalse(vex.SIM['cubes'][0].get('placed'))
        self.assertAlmostEqual(r.X.mm(), self.g['HOME_CLEAR_X'], delta=1)

    def test_drop_stops_x_transport(self):
        r = self.robot()
        r.home_x()
        with self.assertRaisesRegex(self.g['MotionError'], 'cube lost'):
            r.move_x(-400, 50, carrying=True)
        self.assertEqual(r.hw.mx._mode, 'idle')

    def test_failed_pick_positions_are_not_retried_forever(self):
        vex.SIM['cubes'] = [{'x': x, 'z': 80., 'colour': 'green'} for x in [-500., -700., -900.]]
        self.g['pick_cube'] = lambda r, target: False
        r = self.robot()
        self.g['Mission'](r).run()
        self.assertEqual(self.output.getvalue().count('PICK missed'), 3)
        self.assertAlmostEqual(r.X.mm(), self.g['HOME_CLEAR_X'], delta=1)

    def test_uncalibrated_mission_does_not_move(self):
        r = self.robot()
        self.assertFalse(self.g['startup'](r))
        self.assertEqual(vex._mm(5), -200)
        self.assertEqual(vex._t[0], 0)


if __name__ == '__main__':
    unittest.main()
