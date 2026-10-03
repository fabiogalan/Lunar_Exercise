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

    def scan(self, objects, gaps=(), start=-400, end=-1000):
        """Readings every 1 mm like the scan loop. Each object is (left, right, depth). The sensor spreads:
        a 75 mm cube at reading d looks cube_width(d) wide (fitted from the real logs), so each object is
        widened (or narrowed) on both sides by half the difference; each gap X in gaps reads 6 mm deeper
        (the 01.10 rows never split at JUMP_MM 7.5)."""
        g = self.g
        detector = g["CubeDetector"]()
        for x in range(start, end, -1):
            z = g["WALL_Z"]
            for left, right, depth in objects:
                inside = min(max(x, left), right)
                face = depth(inside) if callable(depth) else depth
                grow = (g["CUBE_WIDTH_AT_0"] + g["CUBE_WIDTH_PER_MM"] * (face + g["GRAB_DISTANCE"]) - 75) / 2
                if left - grow <= x <= right + grow:
                    z = min(z, face)
            if any(abs(x - gap) <= 5 for gap in gaps):
                z += 6
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

    def test_row_of_three_gives_its_first_cube(self):
        row = [(c - 37.5, c + 37.5, 180) for c in (-480, -575, -670)]      # 20 mm gaps, hidden by the spread
        target = self.scan(row, gaps=(-527, -622))
        self.assertAlmostEqual(target[0], -480, delta=5)

    def test_far_cube_looks_wide_but_is_one_cube(self):     # 03.10: 144 mm wide at reading 222
        target = self.scan([(-537.5, -462.5, 215)])
        self.assertAlmostEqual(target[0], -500, delta=3)

    def test_near_cube_looks_narrow_but_is_one_cube(self):  # 03.10: 66 mm wide at reading 43
        target = self.scan([(-537.5, -462.5, 36)])
        self.assertAlmostEqual(target[0], -500, delta=3)

    def test_too_narrow_fragment_is_rejected(self):
        self.assertIsNone(self.scan([(-510, -490, 80)]))        # real fragments are <= 16 mm (03.10)


if __name__ == "__main__":
    unittest.main()
