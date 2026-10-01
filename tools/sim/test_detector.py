"""Small tests for the part that is difficult to verify by inspection."""
import math
import unittest

from run import load_program


def rotated_cube(centre, angle, front=80):
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    points = [(centre + x * c - z * s, x * s + z * c)
              for x, z in ((-37.5, -37.5), (37.5, -37.5),
                           (37.5, 37.5), (-37.5, 37.5))]
    offset = front - min(z for x, z in points)

    def depth(x):
        hits = []
        for a, b in zip(points, points[1:] + points[:1]):
            if min(a[0], b[0]) <= x <= max(a[0], b[0]) and a[0] != b[0]:
                hits.append(a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]))
        return min(hits) + offset
    return min(x for x, z in points), max(x for x, z in points), depth


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.g = load_program()

    def scan(self, objects):
        detector = self.g["CubeDetector"]()
        for x in range(-400, -601, -1):
            z = self.g["WALL_Z"]
            for left, right, depth in objects:
                if left <= x <= right:
                    z = min(z, depth(x) if callable(depth) else depth)
            target = detector.add(x, z)
            if target:
                return target
        return None

    def test_straight_cube(self):
        self.assertEqual(self.scan([(-537.5, -462.5, 80)]), (-500, 80))

    def test_rotated_cube_has_slopes_and_corner(self):
        for angle in (-10, -5, 5, 10):
            target = self.scan([rotated_cube(-500, angle)])
            self.assertIsNotNone(target)
            self.assertAlmostEqual(target[0], -500, delta=1)

    def test_front_cube_may_partly_cover_rear_cube(self):
        objects = [(-537.5, -462.5, 80), (-577.5, -502.5, 175)]
        self.assertEqual(self.scan(objects), (-500, 80))

    def test_too_narrow_fragment_is_rejected(self):
        self.assertIsNone(self.scan([(-520, -480, 80)]))


if __name__ == "__main__":
    unittest.main()
