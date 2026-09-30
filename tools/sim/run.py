"""Run the source or generated mission against an idealized field.

python3 tools/sim/run.py --seed 3
python3 tools/sim/run.py --built --seed 3

Simulates a calibrated narrow-beam sensor, home switch, encoder zero offset,
and simple jaw fit/obstacles. It does not validate real sensor optics, cube
rotation, friction, motor reversal, or mechanical compliance.
"""
import ast
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vex

ROOT = os.path.join(HERE, "..", "..")


def make_field(seed):
    rnd = random.Random(seed)
    cubes = []
    for x in range(-450, -1051, -100):
        if rnd.random() < 0.8:
            cubes.append({"x": float(x), "z": 80.0, "colour": rnd.choice(["green"] * 3 + ["blue"])})
        if rnd.random() < 0.45:
            cubes.append({"x": float(x), "z": 175.0, "colour": "red"})
    return cubes


def source_constants():
    with open(os.path.join(ROOT, "src", "robot.py")) as f:
        tree = ast.parse(f.read())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                pass
    return out


def load_robot(built=False):
    path = os.path.join(ROOT, "src", "main.py" if built else "robot.py")
    with open(path) as f:
        source = f.read()
    scope = {"__name__": "simulation"}
    exec(compile(source, path, "exec"), scope)
    return scope


def main():
    seed = int(sys.argv[sys.argv.index("--seed") + 1]) if "--seed" in sys.argv else 1
    random.seed(seed)
    vex.SIM["cubes"] = make_field(seed)
    constants = source_constants()
    # Only the simulated machine uses these ideal calibration values. Never
    # modify CALIBRATED in the physical build just to run the desk simulation.
    vex.SIM["k_x"] = constants["X_MM_PER_DEG"]
    vex.SIM["k_z"] = constants["Z_MM_PER_DEG"]
    vex.SIM["dist_dx"] = constants["DIST_DX_MM"]
    vex.SIM["beam_half"] = max(0.0, (constants["CUBE_SEEN_MM"] - constants["CUBE_MM"]) / 2)
    g = load_robot("--built" in sys.argv)
    if "Mission" not in g:
        sys.exit("Generate a mission build first: python3 tools/build.py mission")
    r = g["Robot"]()
    try:
        # Intentionally bypass the physical startup/calibration gate and buttons.
        g["Mission"](r).run()
    finally:
        r.stop_all()
    placed = [c for c in vex.SIM["cubes"] if c.get("placed")]
    correct = 0
    for c in placed:
        slots = constants[c["colour"].upper() + "_SLOTS"]
        if any(abs(c["x"] - x) < 3 and abs(c["z"] - z) < 3 for x, z in slots):
            correct += 1
    sys.stderr.write("SIM seed %d: %d cubes, %d placed, %d in correct slots, %.1f s\n" % (
        seed, len(vex.SIM["cubes"]), len(placed), correct, vex._t[0] / 1000))
    if correct != len(placed):
        sys.exit("incorrect placement")


if __name__ == "__main__":
    main()
