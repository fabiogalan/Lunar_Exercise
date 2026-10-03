# Cube width calibration

The distance sensor spreads, so a cube looks wider the further away it is. The detector expects one cube to be

    cube_width(d) = CUBE_WIDTH_AT_0 + CUBE_WIDTH_PER_MM * d        (d = sensor reading in mm)

and decides per segment (`CUBE_WIDTH_TOL` = 40):

| segment width            | decision                                           |
|--------------------------|----------------------------------------------------|
| < cube_width(d) - TOL    | no cube (edge fragment, divider wall)              |
| within +-TOL             | one cube, centre = middle of the segment           |
| > cube_width(d) + TOL    | a row; take the first (right) cube: start - cube_width(d)/2 |

03.10 fit from 5 single cubes in old logs: `45.7 + 0.401 d`, residuals within +-4 mm. The runs below measure it
properly across the whole depth range, twice per depth, so the tolerance can be read from the residuals.

## Where to put the cubes

Reference points: **X** = cube centre, measured left from the disposal/mining divider.
**Back gap** = space between the cube's back face and the back wall of the mining area.
On 03.10 the empty floor read 348 mm, so a cube with back gap `g` reads about `d = 348 - 75 - g = 273 - g`.
Exact placement does not matter (the fit uses the logged reading); covering the range does.

| run | right cube: X from divider, back gap (reading) | left cube: X from divider, back gap (reading) |
|-----|------------------------------------------------|-----------------------------------------------|
| 1   | 110 mm, 228 mm (~45)                            | 320 mm, 8 mm (~265)                           |
| 2   | 110 mm, 173 mm (~100)                           | 320 mm, 63 mm (~210)                          |
| 3   | 110 mm, 118 mm (~155)                           | 320 mm, 173 mm (~100)                         |
| 4   | 110 mm, 8 mm (~265)                             | 320 mm, 228 mm (~45)                          |
| 5   | 110 mm, 63 mm (~210)                            | 320 mm, 118 mm (~155)                         |
| 6   | row of 2: X 110 and 205 mm (20 mm gap), both back gap 118 mm (~155) | nothing                |

Square to the rail, single cubes only in runs 1-5 (the fit treats every near segment as one cube). Run 6
checks the row decision and where the first cube's centre lands (it should be X 110 mm). Keep cubes more than
~30 mm from the divider and from the left end of the scan, so floor is visible on both sides of each cube.

## Procedure

1. In `src/main.py` set `CALIBRATION_SCAN = True` and download. It scans the whole mining area once, prints a
   `FOUND,X,Z` line for every cube it would take, picks nothing, and goes home.
2. Quit VS Code, run `python3 tools/scan_log.py`, start the program. Rename the saved log to `logs/cal-runN.log`.
3. After all runs: `python3 tools/fit_width.py logs/cal-run[1-5].log` and paste the two printed lines into
   `src/main.py`. Keep `CUBE_WIDTH_TOL` between 3x the max residual and 47.
   `--degree 2` shows whether a parabola fits clearly better (keep linear unless it does).
4. `python3 tools/fit_width.py logs/cal-run6.log` must say `row`; compare its FOUND X with the measured one.
5. Set `CALIBRATION_SCAN = False` again.

If the sensor is moved, the floor reading changes (01.10: ~395, 03.10: 348): re-measure `WALL_Z` and repeat.
