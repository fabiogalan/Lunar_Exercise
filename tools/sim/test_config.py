"""Checks on the values at the top of src/main.py (kept off the Brain to save memory)."""
import unittest

from run import load_program


class ConfigTests(unittest.TestCase):
    def test_areas_in_order_along_the_rail(self):
        g = load_program()
        drop = g["DROP_X"]
        # field X: mining | disposal (blue) | storage (green nearer the mining area, rule 7.2) | bumper
        order = [g["MINING_AREA"][0], g["MINING_AREA"][1], g["DISPOSAL_AREA"][0], drop["blue"],
                 g["DISPOSAL_AREA"][1], g["STORAGE_AREA"][0], drop["green"], drop["red"],
                 g["STORAGE_AREA"][1], 0.0]
        self.assertEqual(order, sorted(order), "FIELD AREAS out of order")

    def test_targets_within_x_travel(self):
        g = load_program()
        robot_x = {name: x + g["CLAW_X_OFFSET"] for name, x in g["DROP_X"].items()}
        robot_x.update(search_start=g["SEARCH_START_X"], home_clear=g["HOME_CLEAR_X"])
        for name, x in robot_x.items():
            self.assertTrue(g["X_TRAVEL_MIN"] <= x <= 0.0, "%s at robot X %.0f is outside travel" % (name, x))


if __name__ == "__main__":
    unittest.main()
