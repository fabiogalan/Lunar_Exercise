"""Run src/main.py against the simulated field and score the result.

    python3 tools/sim/run.py                 # mission, telemetry to the console
    python3 tools/sim/run.py > sim.log       # then: python3 tools/live_plot.py --file sim.log
    python3 tools/sim/run.py --seed 3
    python3 tools/build.py mission && python3 tools/sim/run.py --built   # test the generated file

The field follows the rules in section 7: a front row of mostly green cubes,
reds mostly behind it, 75 mm cubes, gaps of 2 cm.
"""
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vex  # noqa: E402  (the fake one)

# default: the readable source; --built runs the generated src/main.py (tests tools/build.py)
MAIN = os.path.join(HERE, "..", "..", "src", "main.py" if "--built" in sys.argv else "robot.py")


def make_field(seed):
    rnd = random.Random(seed)
    cubes = []
    pitch = 75.0 + 20.0
    x = 50.0
    while x <= 760.0:
        if rnd.random() < 0.8:
            cubes.append({"x": x, "z": 80.0, "colour": rnd.choice(["green"] * 4 + ["blue"])})
        if rnd.random() < 0.5:
            cubes.append({"x": x + rnd.uniform(-10, 10), "z": 80.0 + pitch,
                          "colour": rnd.choice(["red"] * 3 + ["blue"])})
        x += pitch * 2 if rnd.random() < 0.3 else pitch
    return cubes


def field_constants():
    """Zone limits straight from src/robot.py (the built main.py has them inlined)."""
    import ast
    src = os.path.join(HERE, "..", "..", "src", "robot.py")
    out = {}
    for node in ast.parse(open(src).read()).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass
    return out


def score(g):
    """Points per rules 7.3.1 for cubes that ended up in the facility."""
    pts = 0
    lines = []
    for c in vex.SIM["cubes"]:
        if not c.get("placed"):
            continue
        in_green = g["GREEN_X_MIN"] <= c["x"] <= g["GREEN_X_MAX"]
        in_red = g["RED_X_MIN"] <= c["x"] <= g["RED_X_MAX"]
        if c["colour"] == "blue" or not (in_green or in_red):
            continue
        right = (c["colour"] == "green" and in_green) or (c["colour"] == "red" and in_red)
        p = 20 + ((50 if c["colour"] == "green" else 150) if right else 0)
        pts += p
        lines.append("  %-5s at x=%4.0f z=%4.0f  %s  %d" % (c["colour"], c["x"], c["z"],
                                                          "ok " if right else "WRONG", p))
    return pts, lines


def main():
    seed = int(sys.argv[sys.argv.index("--seed") + 1]) if "--seed" in sys.argv else 1
    random.seed(seed)
    vex.SIM["cubes"] = make_field(seed)
    total = len(vex.SIM["cubes"])
    with open(MAIN) as f:
        src = f.read()
    g = {"__name__": "__main__"}
    exec(compile(src, MAIN, "exec"), g)
    pts, lines = score(field_constants())
    placed = sum(1 for c in vex.SIM["cubes"] if c.get("placed"))
    sys.stderr.write("\nSIM seed %d: %d cubes on field, %d placed, %d points\n" % (seed, total, placed, pts))
    for line in lines:
        sys.stderr.write(line + "\n")


if __name__ == "__main__":
    main()
