# Finetuning and TODO

Checkable items to tune and verify on the real robot. Grouped by topic. See
`docs/architecture.md` for how the code fits together and `docs/design-notes.md`
for the not-yet-built pushing and detection designs.

## Must not forget

- [ ] **`COMPETITION = True` on competition day.** It switches every field
  length between the `_TEST` and `_COMP` sets. `False` runs the small test field.

## Measurements still to take

- [ ] **Z depth from the 39.5 cm rail limit.** 395 mm is measured from the rails,
  but the code works in the Z-encoder frame (Z = 0 = claw fully retracted).
  Measure where the claw face sits at Z = 0 and convert, then set `Z_MAX` and
  `WALL_Z` (currently placeholders 400 / 350).
- [ ] Add a **`Z_MIN` for pushing** — the shallow Z at which the claw pushes
  cubes along a lane without knocking them off. Not present yet.
- [ ] `Z_MM_PER_DEG` still the 0.10 placeholder (X scale is measured: 0.3245).
- [ ] `GRAB_DISTANCE` / `HOLD_DISTANCE` still placeholders (5.0). Measure with an
  isolated cube in a good grab pose, and with a cube held and Z retracted.
- [ ] Gripper angles: `GRIP_OPEN_DEG` is now +80, `GRIP_CLOSED_DEG` = 0. Confirm
  open/closed directions and that +80 clears a 75 mm cube.

## Lane layout to verify against the rules

- [ ] **Blue sits in the disposal area**, so blue ends up *closer* to mining than
  green. "Green nearest the mining area" (rule 7.2) is only true among the two
  storage lanes (red, green). Confirm blue belongs in disposal — if it is really
  a storage lane, `DROP_X` for blue is wrong.
- [ ] Keep each lane at least **35 mm clear of the rails** (minimum cube offset).

## Code cleanups

- [ ] `SENSOR_X_OFFSET = 93.0` is a dead line — it is overwritten two lines later
  by `CLAW_X_OFFSET + SENSOR_CLAW_OFFSET` (= 90). Remove the 93 to avoid
  confusion.

## Physical / mechanical notes (from the team)

- Track the arm's own position and hold Z against gravity (act downward so the
  extended arm does not sag).
- Place the left grabber as close as possible to the cube's left edge when
  grabbing (`linker Arm möglichst nah an linkem Ende von Cube`).
- Check the max X when fully assembled (the machine length at full-left).
- Set `WALL_Z` so a placed/pushed cube cannot fall off the back.
