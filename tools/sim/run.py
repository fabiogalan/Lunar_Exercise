"""Run src/main.py against a simple simulated field: python3 tools/sim/run.py"""
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
import vex


def load_program():
    path = os.path.join(ROOT, "src", "main.py")
    scope = {"__name__": "simulation"}
    with open(path) as source:
        exec(compile(source.read(), path, "exec"), scope)
    vex.SIM.update(port_x=scope["PORT_X"], port_z=scope["PORT_Z"],
                   port_grip=scope["PORT_GRIP"], k_x=scope["X_MM_PER_DEG"],
                   k_z=scope["Z_MM_PER_DEG"], dist_dx=-scope["SENSOR_X_OFFSET"],
                   x_min=scope["X_MIN"], z_max=scope["Z_MAX"],
                   wall_z=scope["WALL_Z"], grab_reading=scope["GRAB_DISTANCE"],
                   hold_reading=max(1.0, scope["HOLD_DISTANCE"] - 1.0),
                   grip_depth=scope["GRIP_DEPTH"])
    return scope


def main():
    random.seed(1)
    vex.SIM["cubes"] = [
        {"x": -500.0, "z": 80.0, "colour": "green"},
        {"x": -700.0, "z": 80.0, "colour": "blue"},
        {"x": -900.0, "z": 80.0, "colour": "red"},
    ]
    program = load_program()
    robot = program["Robot"]()
    try:
        program["run"](robot)
    finally:
        robot.stop()
    placed = sum(bool(c.get("placed")) for c in vex.SIM["cubes"])
    print("simulation: %d/3 cubes placed" % placed)
    if placed != 3:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
