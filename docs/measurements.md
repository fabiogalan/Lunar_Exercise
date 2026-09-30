# Robot and field measurements

The editable configuration is at the top of `src/robot.py`. These measurements
came from 30 September; field coordinates and calibration results still need to
be filled in. Regenerate `src/main.py` after updating the source.

## Wiring

| Port | Device | Role |
| --- | --- | --- |
| 1 | Z motor | Arm extension |
| 2 | Bumper | Right-hand X home switch (`PORT_HOME`) |
| 3 | Touch LED, optional | Start/menu button; not a home sensor |
| 5 | X motor | Trolley, positive degrees move right |
| 7 | Optical sensor | Colour of the held cube |
| 8 | Distance sensor | Cube face/gaps, approach distance, held-cube confirmation |
| 9 | Gripper motor | Lift/lower the grabbing arm |

## Measured geometry

| Measurement | mm | Use |
| --- | --- | --- |
| Cube edge and height | 75 | `CUBE_MM` |
| Inner width between side grabbers | 89 | Limits centring error and cube rotation |
| Outer width of gripper | 100 | Requires free space around the cube |
| Inner base to grabber tip | 80 | `GRIP_DEPTH_MM` |
| Distance sensor lateral offset magnitude | 23 | Calibrate the sign/value of `DIST_DX_MM` |
| Distance sensor height above floor | 45 | Beam should intersect the cube face |
| Optical sensor above held cube | about 17 | Read colour after pulling the cube into the claw |
| Sensor face to cube face at grab pose | TODO | `GRAB_DIST_MM` |
| Safe left travel / search bounds | TODO | Negative X coordinates from the right switch |
| Wall depth / sorting poses | TODO | Measure in the new home-relative frame |

The colour sensor looks **down** onto the held cube. The distance sensor points
into the field. `DIST_DX_MM` is the correction added to the observed trolley X to
obtain the aligned gripper X; teach it with `cal-cube` rather than assigning a
sign to the 23 mm measurement by eye.

## Clearances

| Geometry check | Result |
| --- | --- |
| Straight 75 mm cube in an 89 mm opening | 7 mm centring margin per side |
| Cube at 10 degrees: projected width about 86.9 mm | Only about 1 mm centring margin per side |
| 100 mm outer gripper, straight cube with 15 mm gaps | 105 mm corridor; 2.5 mm margin per side |
| Same with 20 mm gaps | 115 mm corridor; 7.5 mm margin per side |
| Nominal 95 mm row pitch versus 80 mm jaw depth | About 15 mm depth margin for straight cubes |

These are geometric checks, not evidence of achieved positioning accuracy.
Rotated cubes remain particularly demanding. The software checks observed face
width and side-gap depth but cannot establish the full 3D shape from one sensor
height. Test alignment, rotated cubes and actual jaw clearance on the robot.

The search defaults to 15 mm visible side gaps plus 2 mm measurement margin.
It may conservatively skip a physical 15–20 mm gap, especially with beam
broadening. Unknown readings are not treated as empty space. See
[calibration.md](calibration.md) for the required measurements and tuning.
