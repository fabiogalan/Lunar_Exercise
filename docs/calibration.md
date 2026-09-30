# Calibration

Everything the robot knows about the world is in section 1 CONFIG of `src/robot.py`.
Values marked `MEASURE` come from the pad (tape measure). Values marked `CAL`
come from the calibration menu. Each routine prints a line you paste into CONFIG.

## World frame (2D, mm)

- **X**: along the rail. `X = 0` is the start pose. `+X` runs from the mining area towards the facility.
- **Z**: into the field. `Z = 0` is the arm fully retracted.
- A cube's `(x, z)` is **where the gripper must be to grab it**.

Start pose, before every run and every calibration: trolley at X = 0, arm fully
retracted, gripper closed. All encoders are zeroed there when the program starts.

## Building: the brain can't hold the whole program

The IQ2 brain compiles the program itself and runs out of memory on the full
source (`MemoryError: memory allocation failed`). So:

- **Edit `src/robot.py`.** It holds the CONFIG and all the code.
- **Generate `src/main.py`** for the step you want, then Build and Download as usual:

```
python3 tools/build.py test          # jog everything, live sensor values
python3 tools/build.py cal-z         # one calibration routine per build:
python3 tools/build.py cal-x         #   cal-z cal-x cal-grip cal-cube
python3 tools/build.py cal-grip      #   cal-colour cal-teach cal-dist cal-rate
python3 tools/build.py mission       # the competition program
```

`build.py` reports the memory the program needs to compile. The old template
(59 KB) is known to run on the brain.

## One command per step (recommended)

Robot in the start pose, VS Code terminal closed, then:

```
python3 tools/go.py cal-z
```

This builds, downloads to the brain, starts the program and opens the live view.
It works for any build name (test, check, cal-x, ... mission).

## Running a calibration build

1. Put the robot in the start pose.
2. Build, download and start. The build's routine starts immediately; in test builds the screen shows the live values.
3. At start-up the program prints `E,heap free ... used ...`. Send that line to Ilya: it tells us how much room the mission build has.
4. Live view on the laptop: close the VS Code terminal, then run `python3 tools/live_plot.py`. It shows the map, the arm, where the distance beam hits, and the distance over time. The lines to paste into CONFIG are printed in the same window and saved to `logs/`.
5. Use the brain buttons:
   - in the menu, Check runs the routine
   - while jogging, hold Left or Right to move the selected axis
   - to switch axis (X, Z, grip), tap the Touch LED or press Left and Right together
   - tap Check to confirm, or hold Check for 1 s to cancel
6. Paste the printed values into CONFIG in **`src/robot.py`**, then build the next step.

## Order (later steps use the earlier results)

| # | Routine | Method | Produces |
|---|---------|--------|----------|
| 1 | Z scale | Put a flat board across the arm's path in front of the robot. Press Check. **Fully automatic:** the arm creeps out until the board is 6 cm away (or the arm reaches its end), then steps back in 8 steps and fits a line to the distance readings. No ruler needed. | `Z_MM_PER_DEG` |
| 2 | X scale | Arm retracted. Tape-mark the carriage on the rail, jog X as far as possible towards the facility, press Check, mark again and measure between the marks with a ruler. Paste the measurement into the printed formula. | `X_MM_PER_DEG` |
| 3 | Grip | Jog to fully open (arm lifted) and press Check, then jog to closed (arm down) and press Check. | `GRIP_OPEN_DEG`, `GRIP_CLOSED_DEG` |
| 4 | Cube pose | Put a cube in the mining area. Jog X, Z and the gripper until the gripper would grab it, then press Check. The robot records the distance reading, retracts, and sweeps past the cube both ways. | `GRAB_DIST_MM`, `DIST_DX_MM`, `DIST_LAG_S`, `CUBE_SEEN_MM` |
| 5 | Colour | Grab and pull in a cube, so the colour sensor looks down on its top from 17 mm. Press Check to sample. Do red, green and blue, several cubes each. Set `HUE_RANGES` between the printed min and max values; also note `bright`. Left exits. | `HUE_RANGES`, `COLOUR_MIN_BRIGHTNESS` |
| 6 | Teach pts | Jog to any point and press Check to print its X, Z. Use it for the facility zones, the mining area edges and the back wall. | the `MEASURE` field values |
| 7 | Distance | Live readout of the distance sensor. Compare it with a ruler and find its minimum range. | sanity check |
| 8 | Rates (`cal-rate`) | After X scale. Arm retracted, X clear for 30 cm, a few cubes in front. Press Check. X drives 30 cm out and back at `SPEED_X_SCAN` while polling every 2 ms; each sensor's update period is the median time between value changes. Edge error = v · (T_dist + T_loop + T_enc) / 2, kept within `RATE_EDGE_TOL_MM` (1 mm of the ±2 mm grab margin) and at least one reading per map bin. | `SPEED_X_SCAN`, `LOOP_MS`, optical update period |

After filling everything in, set `CALIBRATED = True`. Until then the robot shows a
yellow LED and prints a warning at start-up.

Tips:
- For X, use the longest travel you can. A 1 mm ruler error over 1 m is 0.1 %.
- Repeat X and Z scale twice each. If the two results differ by more than 1 %, something slips: check the gear mesh and the rack.
- Re-check the facility and mining-area positions (Teach pts) on the demo field. The TAs say it is larger and the trolley differs.

## Results log

| Date | Value | Result | Who | Notes |
|------|-------|--------|-----|-------|
|      | Z_MM_PER_DEG | | | |
|      | X_MM_PER_DEG | | | |
|      | GRIP_OPEN_DEG / CLOSED | | | |
|      | GRAB_DIST_MM | | | |
|      | DIST_DX_MM | | | |
|      | DIST_LAG_S | | | |
|      | CUBE_SEEN_MM | | | |
|      | HUE red / green / blue | | | |

## Desk testing without the robot

```
python3 tools/sim/run.py --seed 3 > sim.log     # full 10 min mission on a simulated field, prints a score
python3 tools/build.py mission && python3 tools/sim/run.py --built   # same, with the generated main.py
python3 tools/live_plot.py --file sim.log       # plot what the robot mapped
```
