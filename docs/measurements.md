# Robot and field measurements

This page records known geometry and maps each measurement to the constants
currently used by `src/main.py`. Enter confirmed values at the top of that file.

## Known geometry

| Physical measurement | Current value | Used by the program |
| --- | ---: | --- |
| Cube edge and height | 75 mm | Basis for `CUBE_WIDTH_MIN` and `CUBE_WIDTH_MAX` |
| Inner width between side grabbers | 89 mm | Physical centring tolerance; simulator only |
| Outer gripper width | 100 mm | Basis for `SIDE_CLEARANCE`; simulator only |
| Gripper side arms (two plastic bars) | 6 mm thick, 80 mm long | Each arm must fit into the gap next to the cube; consistent with (100 - 89) / 2 = 5.5 mm and `GRIP_DEPTH` |
| Inner base to grabber tip | 80 mm | `GRIP_DEPTH` |
| Bumper to claw centre (bumper pressed) | 73 mm | `CLAW_X_OFFSET` |
| Bumper to distance-sensor beam (bumper pressed) | 93 mm | `SENSOR_X_OFFSET` |
| Distance-sensor height above floor | 45 mm | Confirms that the beam intersects a cube face |
| Optical sensor height above held cube | about 17 mm | Check reliable colour detection |

`GRIP_DEPTH` describes how far the gripper body enters alongside a cube. It is
used only to check whether both side corridors remain clear deeply enough. It
does not control pickup depth.

## Values still requiring physical measurement

| Measurement | Constant or setting |
| --- | --- |
| X millimetres per motor degree | `X_MM_PER_DEG` |
| Z millimetres per motor degree | `Z_MM_PER_DEG` |
| Safe left travel limit | `X_MIN` |
| Safe maximum extension | `Z_MAX` |
| Bumper-release position | `HOME_CLEAR_X` |
| Area lengths on the competition field | `X_STORAGE_COMP`, `X_DISPOSAL_COMP`, `X_MINING_COMP` |
| Empty mining-area background depth | `WALL_Z` |
| Distance reading at grabbing pose | `GRAB_DISTANCE` |
| Maximum reading for a securely held cube | `HOLD_DISTANCE` |
| Open and closed gripper angles | `GRIP_OPEN_DEG`, `GRIP_CLOSED_DEG` |
| Common release/push depth | `DROP_Z` |
| Held-cube hue ranges | `HUES` |

## Geometry checks

| Check | Result |
| --- | --- |
| Straight 75 mm cube inside an 89 mm opening | 7 mm nominal centring margin per side |
| 75 mm cube at 10 degrees | About 86.9 mm projected width, leaving about 1 mm per side in an 89 mm opening |
| 100 mm outer gripper with 15 mm visible gap on each side | 105 mm corridor, leaving about 2.5 mm per side |
| Rule 7.2 minimum gap 15 mm (2 +- 0.5 cm) to walls and cubes | Side arm (6 mm) + 7 mm inner clearance = 13 mm needed; centring error toward the neighbour may be at most ~2 mm at a 15 mm gap, ~7 mm at 20 mm |
| 100 mm outer gripper with 20 mm visible gap on each side | 115 mm corridor, leaving about 7.5 mm per side |
| Nominal 95 mm row pitch compared with 80 mm gripper depth | About 15 mm depth margin for straight cubes |

The detector currently accepts apparent widths from 72 to 90 mm and requires
17 mm of visible clearance on both sides. These settings cover the calculated
width of a cube rotated about 10 degrees, but the mechanical margin is small.
The single distance beam cannot prove full three-dimensional clearance, so
verify rotated and partly overlapping arrangements physically.

The distance sensor points into the field. During a scan, the program subtracts
`GRAB_DISTANCE` from its reading to express the observed face as a gripper Z
coordinate. It converts robot X to the beam's field X with `SENSOR_X_OFFSET` and, if configured,
`SENSOR_DELAY_S`. See `manual-setup.md` for measurement and validation steps.
