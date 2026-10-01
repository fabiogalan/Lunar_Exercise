# Lunar resource sorter

This VEX IQ2 project contains one robot program: `src/main.py`. The robot homes
the X axis against the bumper, scans the mining area for an accessible cube,
picks it up, identifies its colour, and pushes it into the corresponding
sorting lane.

## Files

- `src/main.py` is the complete program flashed to the Brain.
- `docs/manual-setup.md` explains every value that must be entered by hand.
- `docs/measurements.md` records the known robot geometry and measurements.
- `tools/sim/run.py` runs the mission against a simple desktop simulation.
- `tools/sim/test_detector.py` checks the cube-profile detector.
- `tools/sim/vex.py` supplies the desktop-only VEX API and field model.

There is no separate `robot.py`, generated source, automatic calibration, field
map, or route planner.

## Before starting

1. Measure and enter the values at the top of `src/main.py` as described in
   `docs/manual-setup.md`.
2. Set `MANUAL_VALUES_SET = True` only after checking those values.
3. Physically retract Z fully and close the gripper. The program defines both
   motor positions as zero when it starts.
4. X can start anywhere within its safe travel. The bumper on port 2 defines
   X=0 during every run.
5. Press the Brain Check button or the optional Touch LED to begin.

With `MANUAL_VALUES_SET = False`, the Brain displays an error and does not start
the mission.

## Mission sequence

1. X moves right until the bumper on port 2 is pressed. The program sets that
   position to X=0 and moves left to `HOME_CLEAR_X` so the bumper releases.
2. X moves to `SEARCH_START_X` and scans left toward `SEARCH_END_X` while the
   distance sensor measures the field.
3. The detector accepts a profile only after it sees the cube's two outer edges,
   a width between `CUBE_WIDTH_MIN` and `CUBE_WIDTH_MAX`, and enough open space
   on both sides for the gripper.
4. Continuous depth changes are allowed, so a cube rotated by about 10 degrees
   can be detected. A corner may change the slope without ending the profile.
   A change larger than `DEPTH_JUMP` separates surfaces at different depths.
   This allows a front cube to be selected when it partly hides another cube.
5. The robot centres the gripper, opens it, extends Z quickly to 30 mm before
   the estimated cube position, then approaches slowly until the distance
   reading reaches `GRAB_DISTANCE`.
6. It closes the gripper, retracts Z, and confirms that the cube is present with
   `HOLD_DISTANCE`. The same check runs while X carries the cube.
7. The optical sensor reads the colour. The robot moves to that colour's
   `DROP_X`, extends to the common `DROP_Z`, opens the gripper, retracts Z, and
   closes the gripper again. Repeated cubes use the same coordinates and are
   expected to push earlier cubes farther into the lane.
8. The cycle repeats until no accessible target is found, three pickup positions
   fail, or the 90-second return reserve of the ten-minute mission is reached.

If the colour is unknown, the robot carries the cube to `HOME_CLEAR_X`, stops,
and leaves the cube held.

## Flashing

The VEX project is configured to flash `src/main.py` to slot 1. Use the normal
VEX Build and Download command in VS Code.

## Desktop checks

Run these from the project root:

```sh
python3 -m unittest discover -s tools/sim -p 'test_*.py' -v
python3 tools/sim/run.py
```

The simulator checks mission flow and detector logic. It does not model real
motor loads, sensor timing, gripper tolerances, collisions, or the physical
pushing of cubes already in a sorting lane. Validate the measured values slowly
on the real robot before a full-speed run.
