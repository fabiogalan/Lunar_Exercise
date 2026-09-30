# Lunar resource sorter

The robot homes against the **right-hand bumper on port 2**, moves left to the
mining area, finds the first accessible cube, picks it up, and returns it to the
sorting area. It repeats from the start of the search area after each placement.

## Mission sequence

1. Start with Z fully retracted and the gripper closed. X may be anywhere on the
   usable rail. Press the brain Check button or the optional Touch LED to start.
2. Move slowly right until the bumper is pressed. Stop, confirm the contact,
   set X=0, and back off to release it. An initially pressed switch is first
   released and approached again. A missing/stuck switch or stalled motor stops
   the run; an encoder reset alone never establishes home.
3. Move left to `SEARCH_START_X`, then scan slowly towards `SEARCH_END_X`.
4. Accept a target only after seeing a complete cube-width face and enough clear
   space on **both** sides. The default is 15 mm per side plus a 2 mm measurement
   margin. Readings in those gaps must extend beyond the jaws' approach path.
   A front cube may partly hide another cube and still qualify: the deeper
   return counts as clearance if the jaws can pass in front of it. The visible
   fragment of the rear cube is not treated as a complete cube.
5. Stop scanning and move back right to the measured cube centre, corrected for
   the sensor's sideways offset and latency. Open the claw, extend Z, close at
   the taught distance, retract, and confirm that the cube is held.
6. Read the colour while holding the cube. Return right to the next unused slot
   for that colour, extend, release, retract, and search again.
7. Return near home when no accessible cube remains, the search time expires,
   or three picks have failed. Unknown colour or a full sorting zone stops near
   home **with the cube held**. Motion/sensor faults stop at the current position.

Home is established once per run. Returning to the sorting area uses encoder
positions; it does not push the bumper again on every delivery.

## Coordinates and settings

- X=0 is the right-hand switch contact; positions to the left are negative.
- Z=0 is retracted; positive Z extends into the field.
- Cube/slot coordinates describe the **gripper pose**, not the sensor position.
- `SEARCH_START_X = -350.0` is the configurable distance left before searching.
  **350 mm is a placeholder**, not a field measurement.
- `SEARCH_END_X`, `X_TRAVEL_MIN`, wall distance, all sorting slots, and calibration
  values also require measurement. The first/last cube need space for observing
  their entire face and both gaps within the scan, including the sensor offset.

Edit configuration at the top of [src/robot.py](src/robot.py). The mission refuses
to start while `CALIBRATED = False`. Calibration/test builds remain available.
See [calibration](docs/calibration.md) and [measurements](docs/measurements.md).

## Code layout

| File | Purpose |
| --- | --- |
| `src/robot.py` | Editable source: configuration, hardware/motion, local search, mission, bench calibration |
| `src/main.py` | Generated program downloaded to the brain; currently a mission build |
| `tools/build.py` | Keeps only the selected mode to reduce the brain's compile-memory demand |
| `tools/go.py` | Build, download, start, and open the live view |
| `tools/live_plot.py` | Robot/beam position, identified cube locations, distance history, status |
| `tools/sim/` | Desktop simulation and behavioral checks |

The mission uses a short moving scan window, at most about 133 samples with the
current settings. It has no persistent field map, global cube planner, or dynamic
state dispatch. Only a few failed X positions and sorting-slot counters persist.
A failed position is skipped for the rest of the run, including cubes behind it.

The detector splits abrupt changes between adjacent distance samples (default
`DEPTH_JUMP_MM = 20`). Gradual changes across a rotated cube stay together,
including its two visible faces. At 10 degrees, a 75 mm cube projects to about
86.9 mm along X; the width check allows this increase. The target is centred
between its outer edges, and its approach depth is taken at the sensor's
position after centring the gripper. A rear cube must still clear that jaw path.

## Build and run

```sh
python3 tools/build.py cal-home  # isolate homing for bench checks
python3 tools/build.py mission  # generate the competition program
```

Then use VEX Build and Download. Alternatively `python3 tools/go.py mission`
builds, downloads **and starts** it; the mission still waits for the start button.
Restore the retracted/closed starting pose before launching any program.

```sh
python3 -m unittest discover -s tools/sim -p 'test_*.py' -v
python3 tools/sim/run.py --seed 3 > sim.log
python3 tools/sim/run.py --built --seed 3
python3 tools/live_plot.py --file sim.log
```

The simulator bypasses the physical calibration/start-button gate without
changing the downloaded program. It models an ideal narrow beam, encoder
zeroing, switch faults, and basic jaw fit. Hardware testing is still required for
sensor beam width/latency, rotated cubes, friction, and repeatable positioning.
The 600 s budget includes homing; the default search reserves 90 s for completing
a pick and returning. Tune that reserve from measured worst-case cycle times;
it is a scheduling allowance, not a hard interruption of every actuator call.
