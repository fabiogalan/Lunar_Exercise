# Robot and field measurements

Values the code uses are in CONFIG in `src/robot.py` (sections 1.2 and 1.2b). `tools/build.py`
turns that into the `src/main.py` the brain runs. This page keeps the full table, what each value means,
and the checks derived from it.

## Ports (30.09)

| Port | Device |
|------|--------|
| 1 | Z motor (arm extension) |
| 2 | Bumper |
| 3 | Touch LED (optional; without it the brain buttons are used) |
| 5 | X motor (trolley along the rail) |
| 7 | Optical (colour) sensor |
| 8 | Distance sensor |
| 9 | Gripper motor |

## Robot geometry (measured 30.09)

| # | Measurement | mm | CONFIG |
|---|-------------|----|--------|
| 1 | Cube edge length and height | 75 | `CUBE_MM` |
| 2 | Inner width between the side grabbers | 89 | `GRIP_INNER_MM` |
| 3 | Outer width of the gripper | 100 | `GRIP_OUTER_MM` |
| 3b | Inner base to grabber tip | 80 | `GRIP_DEPTH_MM` |
| 8 | Distance sensor sideways offset from gripper centre | 23 | `DIST_SENSOR_DX_MM` (sign comes from "Cube pose") |
| 9 | Distance sensor height above floor | 45 | `DIST_SENSOR_HEIGHT_MM` |
| 10 | Colour sensor to held cube | 17 | `COLOUR_SENSOR_GAP_MM` (table said 175, read as 17.5; spoken value 1.7 cm) |
| 4 | Pivot to tip of the moving arm, closed | **todo** | |
| 5 | Pivot to pusher face, closed | **todo** | |
| 6 | Arm tip height above floor, closed / open | **todo** | |
| 7 | Distance sensor face to pusher face (expected `GRAB_DIST_MM`) | **todo** | |
| 11 | Arm front to front-row cubes, retracted | **todo** | |

**Colour sensor mounting:** it sits above the held cube and looks down onto the cube's
top face, 17 mm away once the cube is pulled in. Colour is therefore read only
while a cube is held (state IDENTIFY, right after GRAB). The "Colour" calibration
routine must be run with a cube held the same way.

## Checks derived from the measurements

| Check | Numbers | Margin | Verdict |
|-------|---------|--------|---------|
| Cube rotated up to 10° fits between the grabbers | footprint 75·(cos10° + sin10°) = 86.9 vs 89 inner | **±1.0 mm** per side | **critical** |
| Cube straight fits | 75 vs 89 | ±7 mm | ok |
| Gripper passes between neighbours, minimum gap 15 mm | space 75 + 2·15 = 105 vs 100 outer | **±2.5 mm** per side | **critical** |
| Same at nominal gap 20 mm | 115 vs 100 | ±7.5 mm | ok |
| Jaws clear the back row | rows 95 mm apart (min 90) vs depth 80 | 10–15 mm | ok |
| Beam hits the target cube, not the neighbour | offset 23 vs half-cube 37.5 | 14.5 mm | ok |
| Beam hits the cube, not over it | height 45 vs cube 75 | 30 mm | ok |
| Colour sensor distance | 17 vs ~20 spec | | ok |

**Consequence:** X positioning plus the cube-centre estimate from the scan must be
good to about **±2 mm**. L2 requirement R01.DET.02 (±5 mm) is not enough. Options:
1. Software: before grabbing, do a slow scan pass across the target cube to re-measure its centre.
2. Software: pick cubes whose neighbours are already gone first (wider gaps).
3. Mechanics: flare the grabber tips outward so a slightly off cube gets guided in.
