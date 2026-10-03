# Design notes: detection and pushing (work in progress)

The shared working document for the two parts that are **not implemented yet**.
Everything here is design to refine together before changing `src/main.py`.

---

## 1. Cube detection pipeline

Replaces the current `CubeDetector` logic with an explicit scan → segment →
validate → collect pipeline. We scan across the mining area, cut the stream into
candidate cubes on a depth jump, and accept a candidate only when its geometry
matches a single 75 mm cube with a clear side for the arm.

All depths are the distance-sensor reading in the same frame as the code's
grab-Z; smaller = nearer, larger = farther. "Field X" is a position on the
field (see `docs/architecture.md`).

### Constants (measure / set)

| Symbol | Meaning | Start value |
| --- | --- | ---: |
| `S` | sampling step along X; take a new sample only after X has moved `S` (by position, not loop time) | 2.5 mm |
| `Z_MAX` | depth of the empty back wall / background (the "max distance", to measure) | measure |
| `Z_DETECT` | a reading is a cube surface only if its depth `< Z_DETECT` | `Z_MAX − 10 mm` |
| `SEG_JUMP` | depth jump between neighbours that ends a cube (75 mm cube + margin) | 80 mm |
| `CUBE` | cube edge | 75 mm |
| `ARM_WIDTH` | arm thickness that must fit beside the cube | ≈ 7 mm |
| `CLEAR_MARGIN` | extra side gap | 3 mm |
| `CLEAR_DEPTH` | how much deeper the side must read to count as open | 10 mm |

`Z_DETECT = Z_MAX − 10 mm` is the "z_max − 1 cm" used **everywhere** below.

### Scan and segment

1. Start only inside the mining area, from `mining_near_edge + 10 mm` onward
   (ignore the first 1 cm).
2. For each new sample (depth `z` at field X `x`): ignore it while `z ≥ Z_DETECT`
   (wall/background). The **first** sample with `z < Z_DETECT` is the cube's
   first point — record it.
3. Keep adding samples to the current cube while **both**:
   - `|z_i − z_{i−1}| < SEG_JUMP`, and
   - `z_i < Z_DETECT`.
   The first sample that fails either test is `x_start_new`; it does **not**
   belong to the current cube, and every sample before it does.
4. Validate the stored run (below). If it is a collectable cube → stop scanning
   and go collect it. Otherwise restart from step 2 using `x_start_new` as the
   next first candidate when `z(x_start_new) < Z_DETECT`, else keep scanning
   until the next sample with `z < Z_DETECT`.
5. Stop when a collectable cube is found (collect it) or the scan reaches the end
   of the mining track → return home.

### Validate a stored run

1. **Dominant face / angle.** For the run's consecutive samples take the sign of
   each depth step `z_i − z_{i−1}`. Keep only the steps sharing the **majority**
   sign (this isolates one face across the cube's corner). Let `d` = mean of
   those signed steps, `m = |d| / S` (a slope), and
   `angle = atan(m)` in degrees (acute).
2. **Expected width.** `width = 75 · (sin(angle) + cos(angle))`.
   - angle 0° → 75 mm, 10° → 86.9 mm, 45° → 106 mm.
3. **Span check.** `span = |x_first − x_last|`. Accept the run as one cube when
   ```
   width − 2·S − 2 mm  ≤  span  ≤  width + 2 mm
   ```
   A finer `S` tightens the lower bound → higher precision.
4. **Left clearance → collectable.** From the left-outermost point (depth
   `z_left`), look within `ARM_WIDTH + CLEAR_MARGIN` (≈ 10 mm) further left:
   every reading there must be deeper than `z_left + CLEAR_DEPTH` (≥ 1 cm
   farther). If so the arm fits on the left → the cube is collectable; go for it.

### Approach and grab

Align the **left arm** just to the left of the cube's left-outermost detected
point (keep the left arm as close as possible to the cube's left end). Move the
trolley so the left arm sits there, then nudge `+2 mm` in X — that is the grab
pose. The next step is collection: extend Z, close the gripper, retract, confirm
the hold.

The trolley X comes from the left-arm-to-claw offset (gripper geometry):
`arm_left_inner = claw_centre − INNER_WIDTH/2`. Target so `arm_left_inner` lands
just left of `x_left`, then `+2 mm`. The offset constant is TBD from the build.

Grab Z: extend so the claw reaches the cube — use the nearest (smallest-`z`)
sample depth of the run. `pick()` then creeps in until the reading reaches
`GRAB_DISTANCE`, so this only needs to be roughly right.

### Resolved decisions

- **Run ends at the wall** through step 3's `z_i < Z_DETECT` condition — no
  separate rule needed, since `Z_MAX` is the farthest the sensor reads.
- **Right side not checked** — the scan has already passed the right side (its
  cube is collected, or the cube here is not fully collectable), so only the
  **left** clearance matters.
- **Slope `m` = mean depth step / `S`** (depth and X both in mm), so
  `angle = atan(m)` is a real angle.

---

## 2. Pushing / packing strategy

Not built. Today `place()` drops **every** cube of a colour at the same
`DROP_X` / `DROP_Z` and relies on physically shoving earlier ones deeper. The
only hint is the comment "maximum 4 cubes in the row, then push with x offset".

### What we want

- One lane per colour, never mixing colours in a lane.
- Place the **first** cube of a lane at its target position (set down, not
  pushed).
- The claw can **push at most two cubes at a time**.
- **First pair** → all the way to the back wall, spread to the two edges of the
  wall.
- **Next pair** → fill in toward the centre; complete the back line.
- **Further cubes** → next line, then toward the centre, if there is space.
- Keep enough depth (`WALL_Z`, and a `Z_MIN` for pushing) that nothing falls off
  the back, and ≥ 35 mm clear of the rails.

### What the code needs

Per-lane **state**, not just the `used` counter:
- how many cubes placed in the lane,
- where the **last green / red** cube sits (so the next placement, and the next
  search, know where to go),
- which back-line / centre sub-slots are filled.

Then `place(name)` computes the next drop X and Z from that state instead of a
constant. Sketch of the state we need to track per lane:

```
lane[name] = {count, line, last_x, filled_slots}
```

### Open questions to resolve together

- Exact sub-slot X positions per lane (edges first, then centre) and how many
  fit in the 8 cm row length requirement.
- Push vs. place: when do we set down (first cube) vs. push two? How do we know
  two are in front of the claw to push?
- How "remember where the last green/red went" feeds back into `find_cube`
  (do we bias the search, or just the placement?).
