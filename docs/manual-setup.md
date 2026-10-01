# Manual setup

All editable settings are grouped at the top of `src/main.py`. Measure them on
the assembled robot, enter them directly, and set `MANUAL_VALUES_SET = True`
only after checking every motion coordinate. There is no automatic calibration.

## Required starting position

Before turning on or running the program:

- retract the Z axis fully;
- close the gripper;
- place X anywhere within its safe travel, clear of obstacles.

The program sets the current Z and gripper motor positions to zero when `Robot`
is created. Therefore the configured closed position should normally be
`GRIP_CLOSED_DEG = 0`. It then establishes X=0 by driving right until the bumper
on port 2 is pressed. The Touch LED is only an optional start button; it is not
used for homing.

## Wiring

| Constant | Port | Device |
| --- | ---: | --- |
| `PORT_Z` | 1 | Z extension motor |
| `PORT_BUMPER` | 2 | Bumper that defines X=0 |
| `PORT_TOUCH` | 4 | Optional Touch LED/start button |
| `PORT_X` | 6 | X trolley motor |
| `PORT_OPTICAL` | 7 | Optical colour sensor |
| `PORT_DISTANCE` | 8 | Forward-facing distance sensor |
| `PORT_GRIP` | 9 | Gripper motor |

Change these constants if the physical wiring differs.

## Axis and field values

| Constant | Meaning and measurement |
| --- | --- |
| `X_MM_PER_DEG` | X travel in millimetres divided by motor rotation in degrees. Positive motor motion must move X right. |
| `Z_MM_PER_DEG` | Z travel in millimetres divided by motor rotation in degrees. Positive motor motion must extend into the field. |
| `X_MIN` | Safe leftmost X coordinate relative to the bumper at X=0. |
| `Z_MAX` | Safe maximum Z extension from the fully retracted Z=0 position. |
| `HOME_CLEAR_X` | Small negative X coordinate that reliably releases the bumper after homing. |
| `SEARCH_START_X` | Right edge of the mining-area scan. |
| `SEARCH_END_X` | Left edge of the mining-area scan. It must be more negative than `SEARCH_START_X` and no farther left than `X_MIN`. |
| `WALL_Z` | Estimated Z coordinate of the background seen through an empty part of the mining area, in the same gripper-position frame as a detected cube. It separates cube faces from empty background. |
| `DROP_X` | One X coordinate for each colour lane. Every cube of that colour uses the same coordinate. |
| `DROP_Z` | Common extension where the gripper opens. A new cube is expected to push cubes already in its lane forward. |

To measure either motor scale, command a small known motor rotation, measure the
physical axis travel, and divide millimetres by degrees. Verify the sign before
allowing automatic movement.

## Distance sensor and gripper values

| Constant | Meaning and measurement |
| --- | --- |
| `GRAB_DISTANCE` | Distance-sensor reading when the open gripper is at the correct depth to close around a cube. It also converts a scan reading into the estimated Z grab coordinate. |
| `HOLD_DISTANCE` | Largest reading that reliably indicates a held cube after Z retracts. It is also used to detect a lost cube while moving X. |
| `SENSOR_X_OFFSET` | Gripper-centre X minus the X coordinate measured by the distance beam. Positive means the gripper centre is to the right of the beam. |
| `SENSOR_DELAY_S` | Delay between the physical measurement and returned reading, in seconds. Keep zero for initial slow tests unless repeated scans show a speed-dependent X shift. |
| `GRIP_OPEN_DEG` | Gripper motor position when fully open. |
| `GRIP_CLOSED_DEG` | Gripper position at program startup; normally zero. |
| `GRIP_DEPTH` | Front-to-back length of the gripper that must pass beside the cube. It is used to check side clearance and is not a commanded Z position. |

Measure `GRAB_DISTANCE` by manually positioning an isolated cube in a good
grabbing pose and reading the distance sensor. Measure `HOLD_DISTANCE` with a
cube securely held and Z fully retracted. Leave a small allowance for sensor
noise without making an empty gripper count as occupied.

## Detection values

- `CUBE_WIDTH_MIN` and `CUBE_WIDTH_MAX` are acceptable apparent widths in X.
  The current 72–90 mm range includes a straight 75 mm cube and its wider
  projection at roughly 10 degrees.
- `SIDE_CLEARANCE` is the visible open distance required outside each edge. It
  must include the desired gap plus measurement tolerance.
- `DEPTH_JUMP` is the adjacent-sample depth change that separates two surfaces.
  Smaller continuous changes, including the slopes and corner of a rotated
  cube, remain one profile.
- `LOOP_MS` controls how often motion and sensors are checked. Changing it also
  changes the distance between scan samples.

The detector accepts a cube only after both outer edges and both side corridors
have been observed. A nearer cube may partly cover a farther cube and still be
accepted if its visible profile has a valid width and sufficient side clearance.

## Colour and speed values

`HUES` contains inclusive optical-sensor hue ranges for red, green, and blue.
Red may wrap through 0 degrees, so its lower value can exceed its upper value.
Record readings with cubes held in the real gripper and the optical light on.

The six speed constants are motor percentages. Begin with low values and test
homing, scanning, travel, carrying, Z travel, and the final Z approach
separately. Recheck `SENSOR_DELAY_S` if `SCAN_SPEED` changes.

## Recommended validation order

1. Confirm wiring and motor directions with the mechanism clear.
2. Confirm that Z=0 and the closed-gripper position are correct at startup.
3. Confirm that positive X moves right, presses the bumper, sets X=0, and backs
   away to `HOME_CLEAR_X`.
4. Confirm all X and Z limits at low speed.
5. Test pickup of one isolated, straight cube.
6. Test colour readings and each `DROP_X` at a conservative `DROP_Z`.
7. Test a cube rotated about 10 degrees, neighbouring cubes, and a front cube
   that partly hides a rear cube.
8. Test repeated deliveries to one lane and check that earlier cubes move out
   of the way without jamming.
