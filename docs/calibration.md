# Calibration and first run

Edit **`src/robot.py`**, then generate `src/main.py`. Configuration values marked
MEASURE or CALIBRATE remain placeholders. Mission builds refuse to move until
`CALIBRATED = True`; set it only after the measurements below are complete.

## Frame and starting pose

X=0 is the **right-hand home bumper**, port 2. Right is positive; travel into the
mining area is negative. Z is positive outward, with Z=0 fully retracted.
The Touch LED on port 3 is optional and is only a user button, never home.

Before launching a program, manually retract Z and close the gripper. Those two
encoders are zeroed at launch. X's startup encoder value is temporary until a
homing routine observes the switch. `test` and `cal-x` display relative X until
homing; `cal-teach`, `cal-cube`, and `cal-rate` explicitly home before proceeding.

## Builds and controls

```sh
python3 tools/build.py test       # manual jogging and live sensor values
python3 tools/build.py check      # small individual motor movements
python3 tools/build.py cal-x
python3 tools/build.py cal-z
python3 tools/build.py cal-home
python3 tools/build.py cal-grip
python3 tools/build.py cal-cube
python3 tools/build.py cal-colour
python3 tools/build.py cal-teach
python3 tools/build.py cal-dist
python3 tools/build.py cal-rate
python3 tools/build.py mission
```

Build and Download with VEX, or run `python3 tools/go.py cal-home` to build,
download, start, and open the live view. Close the VS Code VEX terminal first;
only one program can own the serial port. `go.py` starts the downloaded program.

A single-routine calibration build still shows a menu: press Check to run it.
Follow any further Check prompts. In the combined calibration build, Left/Right
select the routine. During jogging:

- Hold Left/Right to move the selected axis in its negative/positive direction.
- Tap the Touch LED or press Left+Right together to switch axis.
- Tap Check to confirm; hold it for at least one second to leave the routine.

Manual jogging bypasses software travel limits so uncalibrated axes can be
measured. Rightward X jogging does stop when the home bumper is pressed. Use
short movements and provide physical clearance; do not jog X with Z extended
among cubes. Do not use program startup to zero an extended arm.

`tools/live_plot.py` shows X/Z, beam hits and distance history, and saves serial
output to `logs/`. Calibration routines print results for copying into CONFIG;
they do not persist them automatically. Copy a result and rebuild before running
a routine that depends on it.

## Calibration order

| Step | Build | Procedure / result |
| --- | --- | --- |
| 1 | `test` / `check` | Verify positive X degrees move right and positive Z extends. Set motor reversal flags as needed. Test the port-2 bumper by hand in `test`; the displayed `home` value must follow it. |
| 2 | `cal-x` | Arm retracted. Mark the carriage, jog left a known distance, measure it with a ruler, and fill in the printed `X_MM_PER_DEG` formula. Check returns to the first mark. |
| 3 | `cal-z` | Place a flat board across the arm path. The arm creeps towards it, then retracts in steps and fits distance versus encoder angle. Copy `Z_MM_PER_DEG`. |
| 4 | `cal-home` | Clear the rail and press Check. Robot seeks right, zeroes at the bumper and backs off to `HOME_CLEAR_X`. Repeat from several start positions, including with the switch already pressed. Verify physical repeatability. |
| 5 | `cal-grip` | Jog fully open and then closed. Copy `GRIP_OPEN_DEG` / `GRIP_CLOSED_DEG`. |
| 6 | `cal-teach` | Homes first. Teach mining scan bounds, usable left travel, the background wall and every sorting pose. X values must now be negative. Measure the wall in grab-Z coordinates (`Z + distance - GRAB_DIST_MM` once calibrated). |
| 7 | `cal-rate` | Homes, moves to `SEARCH_START_X`, and travels 300 mm farther left and back. Keep that entire path clear with Z retracted. Its printed speed/loop suggestions estimate sensor update limits; validate actual edge repeatability with the next step. |
| 8 | `cal-cube` | Homes first. Use one isolated cube with room to scan 110 mm either side. Jog into its correct grab pose. The routine records the grab distance, retracts, then scans both ways to estimate `GRAB_DIST_MM`, `DIST_DX_MM`, `DIST_LAG_S`, `CUBE_SEEN_MM`. Run at the final scan speed. |
| 9 | `cal-colour` | Hold a cube pulled into the claw, with the optical sensor above its top face. Sample all three colours and tune `HUE_RANGES` / brightness threshold. |
| 10 | `cal-dist` / `test` | Measure the held-cube reading and empty-claw reading; tune `HOLD_DIST_MAX_MM`. Confirm the sensor works at the grab distance and distinguishes a held cube after retracting. |

The home timeout is 120 s by default; it must accommodate the full rail at the
measured homing speed. A lack of encoder progress for 1.5 s stops homing early.
Home is not established if the switch is missing, remains pressed, never trips,
or fails to release after the backoff.

## Field settings to fill in

- `HOME_CLEAR_X`: enough negative travel to release the switch reliably.
- `X_TRAVEL_MIN`: measured safe left travel limit relative to home.
- `SEARCH_START_X`: the move left before scanning (currently **-350 mm placeholder**).
- `SEARCH_END_X`: final scan position before the left rail limit.
- `MINING_Z_WALL`, `Z_TRAVEL_MAX`: measured depth and extension bounds.
- `GREEN_SLOTS`, `RED_SLOTS`, `BLUE_SLOTS`: actual gripper poses, deepest first.
  Place green closest to mining and red farther right; choose a separate legal
  location for blue. Never put a shallow slot before a deeper slot behind it.

With all calibrated values installed, set `CALIBRATED = True`, build `mission`,
and first run with one isolated cube. Then test two adjacent cubes and a cube
behind another before running a full field. Measure the worst-case pick/return
cycle and adjust `EST_CYCLE_S` (default 90 s).

## Search tuning

The robot must see the first gap, a complete face, and the second gap before it
stops. Leave enough scan travel on both sides of the mining area, including the
sensor's sideways offset. It will reject cubes cut off by a scan boundary.

`SIDE_GAP_MM = 15` plus `EDGE_MARGIN_MM = 2` requires at least 17 mm of measured
clear gap on each side. Setting the minimum to 20 requires 22 mm with that
margin. This intentionally skips marginal layouts until measurement accuracy
supports a smaller margin. The task permits 15–25 mm physical gaps; a wide sensor
beam can make those appear smaller. Measure this on the actual robot. Do not
assume invalid/out-of-range returns establish clearance: the current algorithm
requires valid deeper returns, normally from a background wall or a rear cube.

`CUBE_SEEN_MM` is the apparent width of a straight cube, not necessarily its
physical 75 mm width. `WIDTH_TOL_MM` and `ROTATION_WIDTH_MM` allow measurement
variation and rotation: at 10 degrees the physical projection is about 86.9 mm.
`DEPTH_JUMP_MM` separates abrupt changes between neighboring samples, while
allowing continuous slopes across two visible faces of the same rotated cube.
Tune it against real scans: a sensor cannot distinguish a sharp slope from a
depth step if sampling is too coarse. The width and both-side clearance checks
still apply after segmentation.

A front cube that partly hides a rear cube is eligible. The rear cube's deeper
reading may establish clearance beside the front cube, but the exposed rear
fragment alone does not establish a complete pickup target. A gap return must
lie beyond both the jaw tips at the predicted grab pose and the deepest observed
point of the target, plus `CLEARANCE_DEPTH_MM`. Jaw depth is measured from the
predicted grab pose; it is not added again behind a visible rear corner of a
rotated cube. That grab pose uses the measured depth where the offset sensor
will point after the gripper has been centred.

The rate routine measures intervals between changed readings; noise or a static
scene can distort its estimate. Confirm the chosen speed on real edge scans.
`SCAN_MAX_SPACING_MM` rejects windows containing a spatial sampling hole; sensor
latency and repeated/stale readings still require real calibration. Repeat cube
pose calibration after changing speed or sensor mounting.

## Results log

| Date | Setting | Measured value | Notes |
| --- | --- | --- | --- |
| | X/Z scale | | |
| | Home repeatability / backoff | | |
| | Search start/end / left limit | | |
| | Grip open/closed | | |
| | Grab/held distance | | |
| | Sensor offset / latency / apparent cube width | | |
| | Colour ranges | | |
| | Sorting slots | | |
| | Worst-case cycle time | | |
