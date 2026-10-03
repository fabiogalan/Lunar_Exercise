"""Small tests for the part that is difficult to verify by inspection."""
import math
import unittest

from run import load_program

SENSOR_SPREAD = 0.29          # the distance sensor's light spreads: a cube looks wider by 0.29 x its distance
                              # (01.10 logs; tools-only, so it lives here rather than in main.py)


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

    def scan(self, objects, gaps=(), start=-400, end=-1000):
        """Readings every 1 mm like the scan loop. Each object is (left, right, depth); the sensor sees
        it past its edges as a slope (as in the real logs); each gap X in gaps reads 8 mm deeper."""
        detector = self.g["CubeDetector"]()
        for x in range(start, end, -1):
            z = self.g["WALL_Z"]
            for left, right, depth in objects:
                inside = min(max(x, left), right)
                face = depth(inside) if callable(depth) else depth
                ramp = SENSOR_SPREAD * (face + self.g["GRAB_DISTANCE"]) / 2
                off = max(left - x, x - right, 0)
                if off <= ramp:
                    z = min(z, face + 40.0 * off / ramp if off else face)
            if any(abs(x - gap) <= 5 for gap in gaps):
                z += 8
            target = detector.add(x, z)
            if target:
                return target
        return detector.finish()

    def test_straight_cube(self):
        target = self.scan([(-537.5, -462.5, 80)])
        self.assertAlmostEqual(target[0], -500, delta=3)
        self.assertEqual(target[1], 80)

    def test_rotated_cube_has_slopes_and_corner(self):
        for angle in (-10, -5, 5, 10):
            target = self.scan([rotated_cube(-500, angle)])
            self.assertIsNotNone(target, angle)
            self.assertAlmostEqual(target[0], -500, delta=3)

    def test_front_cube_may_partly_cover_rear_cube(self):
        target = self.scan([(-537.5, -462.5, 80), (-577.5, -502.5, 175)])
        self.assertAlmostEqual(target[0], -500, delta=3)

    def test_row_of_three_is_split_at_the_gaps(self):
        row = [(c - 37.5, c + 37.5, 80) for c in (-480, -588, -696)]      # 33 mm gaps
        target = self.scan(row, gaps=(-534, -642))
        self.assertAlmostEqual(target[0], -480, delta=5)

    def test_too_narrow_fragment_is_rejected(self):
        self.assertIsNone(self.scan([(-520, -480, 80)]))


if __name__ == "__main__":
    unittest.main()
