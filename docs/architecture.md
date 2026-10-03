# How the program is built

The whole robot program is one file, `src/main.py`, flashed to the VEX IQ2
Brain. `from vex import *` loads the real VEX library on the Brain; on a laptop
`tools/sim/vex.py` stands in for it so the same file can run in simulation.

This page maps the file so you can work on one part without reading all of it.

## Coordinate frames (read this first — it is the easiest thing to get wrong)

Everything is in millimetres from the bumper. The bumper is **X = 0**; left is
negative.

- **Robot X** is the trolley encoder position. X = 0 exactly while the bumper is
  pressed (set during homing).
- **Field X** is where a thing actually is on the field.
- A tool mounted on the trolley sits at an offset from the trolley:
  - claw centre is at `robotX - CLAW_X_OFFSET`
  - distance beam is at `robotX - SENSOR_X_OFFSET`
- So to put the **claw centre on field X `x`**, move the trolley to
  `x + CLAW_X_OFFSET`. To know where a **distance reading** was taken, it is at
  field X `robotX - SENSOR_X_OFFSET`.

Key offsets (`main.py` top):

| Constant | Value | Meaning |
| --- | ---: | --- |
| `CLAW_X_OFFSET` | 73 | bumper → claw centre |
| `SENSOR_CLAW_OFFSET` | 17 | claw centre → distance beam (beam is left of the claw) |
| `SENSOR_X_OFFSET` | 90 | bumper → beam ( = 73 + 17 ) |
| `MACHINE_LENGTH` | 228 | bumper (right end of machine) → left wall at full-left |

Z is a separate axis: **Z = 0** is the claw fully retracted; positive Z extends
into the field.

## The field: three areas

From the bumper going left: **STORAGE | DISPOSAL | MINING** (`main.py` area
block).

- Lengths come in `_TEST` and `_COMP` pairs. `COMPETITION` picks which set is
  live. **Set `COMPETITION = True` on competition day.**
- `STORAGE_AREA = (-X_STORAGE, 0)`, then `DISPOSAL_AREA`, then `MINING_AREA`,
  each computed from the one to its right.
- The scan runs from `SEARCH_START_X` (edge of disposal) to
  `SEARCH_END_X = X_MIN`. `X_MIN` is just inside the left wall: the machine is
  `MACHINE_LENGTH` long, so the trolley stops that far before the field end
  (`X_WALL`), plus a 10 mm margin.
- `area(field_x)` turns any field X back into `MINE` / `DISPOSE` / `STORE` for
  logging.

## The building blocks

- **`Axis`** wraps one motor as a linear axis. `mm()` converts encoder degrees to
  mm with `scale`; `start`/`move` command a target with a computed timeout;
  `check()` raises on a motion timeout or a blocked motor. Used for X and Z.
- **`Robot`** holds every device and the high-level moves:
  - `home()` — drive right at `HOME_SPEED` until the bumper presses, define that
    as X = 0, back off to `HOME_CLEAR_X`, confirm the bumper released.
  - `move_x` / `go_x` — drive the trolley to a target. While `carrying=True` it
    watches the distance sensor and raises "cube lost" after 3 bad reads. With
    `BOUNDARY_PAUSE_S > 0` it pauses on each area boundary (a calibration aid).
  - `grip(degrees, speed)` — drive the gripper and raise if it stalls short.
  - `read_distance()` — a reading, or `None` if out of the 0–1000 mm window.
  - `report()` — prints `area, arm X, sensor X, Z, distance` whenever the reading
    changes; this is the distance map used for tuning.
- **`CubeDetector`** turns the scan into a target. It keeps a sliding window of
  `(field-X, grab-Z)` samples (enough for one cube plus its side gaps) and, in
  `find_target`, walks a continuous run of samples that are nearer than the wall,
  ends the run on a depth jump `> DEPTH_JUMP`, checks the edge-to-edge width is in
  `[CUBE_WIDTH_MIN, CUBE_WIDTH_MAX]`, and checks `SIDE_CLEARANCE` on both sides so
  the gripper fits. On success it returns `(centre field-X, grab Z)`.

## The mission loop

`run(robot)`:
1. `home()`, then print the area map.
2. Until the 90 s return reserve of the 10-minute mission is used up:
   - `find_cube` — drive to `SEARCH_START_X`, then scan toward `SEARCH_END_X`
     feeding readings to the detector (readings are corrected to field X with
     velocity/`SENSOR_DELAY_S`/`SENSOR_X_OFFSET`). Skips targets near a `failed`
     position.
   - `pick` — move the claw onto the cube, open, extend Z fast to just short of
     the cube, creep until the reading reaches `GRAB_DISTANCE`, close, retract,
     and confirm the cube is held within `HOLD_DISTANCE`.
   - `colour` — median hue from the optical sensor → `red`/`green`/`blue` via the
     `HUES` ranges, or `unknown`.
   - `place` — carry to that colour's `DROP_X`, extend to `DROP_Z`, open, retract,
     close. (All cubes of a colour currently go to the **same** spot; see
     `docs/design-notes.md`.)
3. On `unknown` colour the robot carries the cube to `HOME_CLEAR_X` and stops.
4. `main()` wraps everything: it checks all devices are installed and
   `MANUAL_VALUES_SET`, waits for the start button, runs, and on any error shows
   the message and stops the motors.

## Running it

From the project root:

```sh
python3 -m unittest discover -s tools/sim -p 'test_*.py' -v   # detector tests
python3 tools/sim/run.py                                      # full mission in sim
```

To test single movements on the real robot, the `__main__` block has a commented
example that builds a `Robot` and drives the axes directly; comment out
`main()` and use that.
