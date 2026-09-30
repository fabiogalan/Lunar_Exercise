# =============================================================================
# LUNAR RESOURCE SORTING - Team 2
#
# THIS is the source you edit. The brain gets src/main.py, which is generated:
#     python3 tools/build.py mission      (or: calibrate)
# then Build and Download in VS Code. The brain only runs one file and runs
# out of memory compiling all of this, so build.py strips comments/docstrings
# and keeps only the code the chosen mode can reach.
#
# Sections, each only uses the ones above it:
#
#   1 CONFIG       every number the robot uses (MEASURE / CAL markers)
#   2 HAL          the only code that talks to vex devices
#   3 AXES         degree <-> mm mapping, motion with timeouts
#   4 WORLD MAP    2D distance profile + cube list (pure python, no vex)
#   5 TELEMETRY    brain screen plot + serial lines for tools/live_plot.py
#   6 SKILLS       sweep, approach, grab, identify, place
#   7 MISSION      state machine, time budget, recovery
#   8 CALIBRATION  start-up self check + bench calibration menu
#   9 ENTRY        MODE switch
#
# 2D world frame, millimetres (height is ignored everywhere):
#   X  along the rail. X = 0 is the start pose (robot is always placed there).
#      +X points from the mining area towards the production facility.
#   Z  perpendicular to the rail, into the field. Z = 0 is the arm fully
#      retracted.
#   A cube's (x, z) is "where the gripper must be to grab it", so the planner
#   never deals with sensor geometry. Only CAL values below know about sensors.
#
# Start pose (setup checklist): trolley at X = 0, arm fully retracted,
# gripper closed. All encoders are zeroed there when the program starts.
# =============================================================================
from vex import *

MODE = "MISSION"   # "MISSION" | "CALIBRATE" | "TEST"
BUILD_MODES = ("MISSION", "CALIBRATE", "TEST")    # tools/build.py narrows this
# Shortcut without re-downloading, during the first 2 s after start
# ("hold ..." on the screen):
#   brain Left button  -> CALIBRATE      brain Right button -> TEST
#   (or Touch LED, if one is plugged in: release < 4 s CALIBRATE, hold > 4 s TEST)
MODE_SELECT_MS = 2000


# =============================================================================
# 1 CONFIG
# =============================================================================

# ---- 1.1 Ports ---------------------------------------------------------------
PORT_Z = Ports.PORT1
PORT_BUMPER = Ports.PORT2
PORT_TOUCH = Ports.PORT3        # optional: without a Touch LED the brain buttons are used
PORT_X = Ports.PORT5
PORT_OPTICAL = Ports.PORT7
PORT_DISTANCE = Ports.PORT8
PORT_GRIP = Ports.PORT9

X_MOTOR_REVERSED = False    # True if positive motor degrees move AWAY from the facility
Z_MOTOR_REVERSED = False    # True if positive motor degrees RETRACT the arm

# ---- 1.2 Field geometry [MEASURE on the pad, mm, world frame] ----------------
CUBE_MM = 75.0              # measured 30.09: edge length (and height) of a cube
CUBE_GAP_MM = 20.0          # rules 7.2: cube-cube gap 2 +- 0.5 cm

X_TRAVEL_MAX = 1200.0       # MEASURE: largest X the trolley may drive to
Z_TRAVEL_MAX = 400.0        # MEASURE: full arm extension (L2 R01.GRIP.2: 40 cm)
Z_SAFE = 5.0                # X only moves while Z <= this (arm clear of cubes)

MINING_X_MIN = 0.0          # MEASURE: mining area, X range of cube centres
MINING_X_MAX = 800.0        # MEASURE
MINING_Z_WALL = 350.0       # MEASURE: grab-Z of the far wall; anything this deep = empty

GREEN_X_MIN = 900.0         # MEASURE: facility half closest to the mining area
GREEN_X_MAX = 1000.0        # MEASURE
RED_X_MIN = 1020.0          # MEASURE: facility half further away
RED_X_MAX = 1120.0          # MEASURE
FACILITY_Z_MIN = 60.0       # MEASURE: shallowest grab-Z where a cube is fully inside
FACILITY_Z_MAX = 250.0      # MEASURE: deepest grab-Z (open back: stay clear of the edge)
FACILITY_PITCH_MM = 85.0    # slot spacing inside the facility (cube + 10 mm)
BLUE_DUMP = (1160.0, 60.0)  # MEASURE/DECIDE: where blue cubes go (x, z)
UNKNOWN_COLOUR_AS = "green" # colour sensor unsure -> treat as this

# ---- 1.2b Robot geometry [measured 30.09, see docs/measurements.md] ---------
GRIP_INNER_MM = 89.0        # between the side grabbers (cube rotated 10 deg needs 86.9)
GRIP_OUTER_MM = 100.0       # outside of the side grabbers (space between neighbours: 105-115)
GRIP_DEPTH_MM = 80.0        # inner base to grabber tip (cube is 75 deep)
DIST_SENSOR_DX_MM = 23.0    # distance sensor beam, sideways from gripper centre (sign: CAL)
DIST_SENSOR_HEIGHT_MM = 45.0  # beam height above the floor (cube is 75 high)
# Colour sensor: mounted above the held cube, looking DOWN onto its top face,
# 17 mm away when the cube is pulled in. So colour is read only while holding.
COLOUR_SENSOR_GAP_MM = 17.0

# ---- 1.3 Calibration results [CAL: filled in by MODE = "CALIBRATE"] ----------
CALIBRATED = False          # set True once every CAL value below is measured
X_MM_PER_DEG = 0.10         # CAL "X scale"   (placeholder)
Z_MM_PER_DEG = 0.10         # CAL "Z scale"   (placeholder)
GRAB_DIST_MM = 10.0         # CAL "Cube pose": distance reading in the grab pose
DIST_DX_MM = 0.0            # CAL "Cube pose": add to sweep X to get the cube's gripper X (expect +-23)
DIST_LAG_S = 0.0            # CAL "Cube pose": distance sensor latency
CUBE_SEEN_MM = 85.0         # CAL "Cube pose": cube width as the sweep sees it (placeholder)
HOLD_DIST_MAX_MM = 20.0     # distance reading while a cube is held (ConOps: < 2 cm)
GRIP_OPEN_DEG = -500        # CAL "Grip" (tested value from template)
GRIP_CLOSED_DEG = 0         # CAL "Grip" (power-on position)
HUE_RANGES = {              # CAL "Colour": hue degrees (lo, hi), wraps through 0
    "red": (330.0, 25.0),
    "green": (70.0, 170.0),
    "blue": (180.0, 270.0),
}
COLOUR_SAMPLES = 7
COLOUR_MIN_BRIGHTNESS = 5.0 # accept a hue sample if proximity OR brightness says a cube is there

# ---- 1.4 Motion --------------------------------------------------------------
SPEED_X_TRAVEL = 80         # % empty X moves
SPEED_X_SCAN = 40           # % X while scanning (resolution = speed * LOOP_MS)
SPEED_X_CARRY = 50          # % X with a cube (a dropped red costs 75 + Comfy Ride)
SPEED_Z = 80                # % Z empty
SPEED_Z_CREEP = 20          # % final approach onto a cube
SPEED_Z_CARRY = 50          # % Z with a cube
SPEED_GRIP_OPEN = 20        # % (tested value from template)
SPEED_GRIP_CLOSE = 50       # %
SPEED_JOG = 25              # % calibration jogging
GRIP_TOL_DEG = 30           # open arm must be this close to GRIP_OPEN_DEG

MOTOR_MAX_DPS = 720.0       # IQ smart motor free speed at 100 % (120 rpm)
POS_TOL_MM = 3.0
MOVE_TIMEOUT_FACTOR = 2.0   # a move may take this x its ideal duration ...
MOVE_TIMEOUT_EXTRA_MS = 1500  # ... plus this, before it counts as blocked
APPROACH_STANDOFF_MM = 30.0 # fast Z stops this far before the cube, then creep
CREEP_OVERSHOOT_MM = 20.0   # creep this far past the expected cube before giving up
DROP_CONFIRM_SAMPLES = 3    # consecutive "no cube" readings while carrying = lost

# ---- 1.5 Mission -------------------------------------------------------------
MISSION_S = 600
EST_CYCLE_S = 30            # L1 R01.GRIP: 30 s per cube; don't start one with less left
MAX_ATTEMPTS = 2            # per cube position, then it is skipped
MAX_EMPTY_SCANS = 2         # full scans without a target before parking
MAX_FAULTS = 3              # consecutive faults before a cool-down pause
FAULT_PAUSE_MS = 3000
BATTERY_WARN_PCT = 50

# ---- 1.6 Map / telemetry -----------------------------------------------------
MAP_BIN_MM = 5.0
MAP_MAX_GAP_BINS = 1        # a 1-bin dropout inside a cube does not split it
MAP_DEPTH_JUMP_MM = 25.0    # a depth step bigger than this splits two cubes
DIST_MAX_VALID_MM = 1000.0  # readings above this = nothing seen
LOOP_MS = 15
ROBOT_PRINT_MS = 200        # live R-line rate during the mission
LIVE_CAL_MS = 100           # live R-line rate during calibration / test
SCREEN_W = 160
SCREEN_H = 108

# ---- 1.7 Calibration routine settings ----------------------------------------
CAL_Z_STEPS = 8
CAL_Z_NEAR_MM = 60.0        # auto Z scale stops extending this close to the board
CAL_Z_MAX_MS = 30000        # ... or after this long
CAL_CUBE_SWEEP_MM = 100.0   # sweep +- this around the taught cube
CAL_SETTLE_MS = 500
RATE_SWEEP_MM = 300.0       # "Rates" drives X out this far and back
RATE_POLL_MS = 2            # ... polling this often to catch every sensor update
RATE_EDGE_TOL_MM = 1.0      # share of the +-2 mm grab margin allowed for sampling error
RATE_MAX_SAMPLES = 400


# =============================================================================
# 2 HAL - the only place that touches vex devices
# =============================================================================
brain = Brain()


class MotionError(Exception):
    pass


class CubeLost(MotionError):
    pass


class Hardware:
    def __init__(self):
        self.touch = Touchled(PORT_TOUCH)
        self.optical = Optical(PORT_OPTICAL)
        self.distance = Distance(PORT_DISTANCE)
        self.bumper = Bumper(PORT_BUMPER)
        # Motor(port, False) could be read as gear ratio 0: only pass True
        self.mx = Motor(PORT_X, True) if X_MOTOR_REVERSED else Motor(PORT_X)
        self.mz = Motor(PORT_Z, True) if Z_MOTOR_REVERSED else Motor(PORT_Z)
        self.mg = Motor(PORT_GRIP)
        self.has_touch = self.touch.installed()

    def missing(self):
        """Required devices that are not plugged in (the Touch LED is optional)."""
        devices = (("optical", self.optical),
                   ("distance", self.distance), ("bumper", self.bumper),
                   ("motor X", self.mx), ("motor Z", self.mz), ("grip", self.mg))
        return [name for name, dev in devices if not dev.installed()]

    def stop_all(self):
        for m in (self.mx, self.mz, self.mg):
            m.stop()


def now_ms():
    return brain.timer.time(MSEC)


def median(values):
    s = sorted(values)
    return s[len(s) // 2]


LED_COLOURS = {"red": Color.RED, "green": Color.GREEN, "blue": Color.BLUE,
               "yellow": Color.YELLOW, "purple": Color.PURPLE, "white": Color.WHITE}


def set_led(hw, name):
    if not hw.has_touch:
        return
    if name == "off":
        hw.touch.off()
    else:
        hw.touch.set_color(LED_COLOURS[name])


def distance_mm(hw):
    """One reading in mm, or None if nothing is in range."""
    d = hw.distance.object_distance(MM)
    if d <= 0 or d > DIST_MAX_VALID_MM:
        return None
    return d


def distance_median(hw, n):
    vals = []
    for _ in range(n):
        d = distance_mm(hw)
        if d is not None:
            vals.append(d)
        wait(LOOP_MS, MSEC)
    if len(vals) < n // 2 + 1:
        return None
    return median(vals)


def hue_median(hues):
    """Median on the hue circle: reds sit on both sides of 0."""
    if max(hues) - min(hues) > 180:
        hues = [h + 360 if h < 180 else h for h in hues]
    return median(hues) % 360


def classify_hue(h):
    if h is None:
        return "unknown"
    for name in ("red", "green", "blue"):
        lo, hi = HUE_RANGES[name]
        if (lo <= h <= hi) if lo <= hi else (h >= lo or h <= hi):
            return name
    return "unknown"


def read_hue(hw, n):
    """Median hue of n samples taken while the sensor sees an object, else None."""
    hues = []
    for _ in range(n):
        if hw.optical.is_near_object() or hw.optical.brightness() >= COLOUR_MIN_BRIGHTNESS:
            hues.append(hw.optical.hue())
        wait(LOOP_MS, MSEC)
    if len(hues) < n // 2 + 1:
        return None
    return hue_median(hues)


def buttons(hw):
    """(left, right, check, touch) pressed states."""
    return (brain.buttonLeft.pressing(), brain.buttonRight.pressing(),
            brain.buttonCheck.pressing(), hw.has_touch and hw.touch.pressing())


def start_pressed(hw):
    """The start button: Touch LED if plugged in, else the brain Check button."""
    return hw.touch.pressing() if hw.has_touch else brain.buttonCheck.pressing()


def wait_press(hw):
    """Block until a button is pressed and released. Returns 'L', 'R', 'C' or 'T'."""
    while True:
        l, r, c, t = buttons(hw)
        if l or r or c or t:
            while True in buttons(hw):
                wait(20, MSEC)
            return "L" if l else "R" if r else "C" if c else "T"
        wait(20, MSEC)


def say(*lines):
    """Show short lines on the brain screen and echo them to the console."""
    brain.screen.clear_screen()
    for i, line in enumerate(lines):
        brain.screen.set_cursor(i + 1, 1)
        brain.screen.print(line)
    print("# " + " | ".join(lines))


# =============================================================================
# 3 AXES - degrees exist only inside this section
# =============================================================================
class Axis:
    """Linear axis: mm = deg * mm_per_deg. Zero = encoder position at program start."""

    def __init__(self, name, motor, mm_per_deg, lo, hi):
        self.name = name
        self.m = motor
        self.k = mm_per_deg
        self.lo = lo
        self.hi = hi
        self.timeout = 0
        self.target = 0.0
        self.t_start = 0
        motor.set_stopping(HOLD)
        motor.set_position(0, DEGREES)

    def expected_ms(self, dist_mm, speed):
        """Generous duration for a move: ideal time * factor + extra."""
        mm_per_s = abs(self.k) * MOTOR_MAX_DPS * speed / 100.0
        return MOVE_TIMEOUT_FACTOR * 1000.0 * abs(dist_mm) / mm_per_s + MOVE_TIMEOUT_EXTRA_MS

    def deg(self):
        return self.m.position(DEGREES)

    def mm(self):
        return self.deg() * self.k

    def start_move(self, mm, speed):
        """Non-blocking move; poll arrived() and check()."""
        self.target = min(max(mm, self.lo), self.hi)
        self.t_start = now_ms()
        self.timeout = self.expected_ms(self.target - self.mm(), speed)
        self.m.set_timeout(self.timeout + 500, MSEC)    # motor's own default gives up at 10 s
        self.m.spin_to_position(self.target / self.k, DEGREES, speed, PERCENT, False)

    def arrived(self):
        return abs(self.mm() - self.target) <= POS_TOL_MM

    def check(self):
        """Raise MotionError if the move timed out or the motor gave up early."""
        elapsed = now_ms() - self.t_start
        if elapsed > self.timeout:
            self.halt()
            raise MotionError("%s timeout at %.0f mm (target %.0f)" % (self.name, self.mm(), self.target))
        if elapsed > 150 and self.m.is_done() and not self.arrived():
            self.halt()
            raise MotionError("%s blocked at %.0f mm (target %.0f)" % (self.name, self.mm(), self.target))

    def move_to(self, mm, speed):
        self.start_move(mm, speed)
        while not self.arrived():
            self.check()
            wait(LOOP_MS, MSEC)

    def move_deg(self, deg, speed):
        """Raw move for calibration (no mm scale, no soft limits)."""
        self.m.set_timeout(self.expected_ms((deg - self.deg()) * self.k, speed), MSEC)
        self.m.spin_to_position(deg, DEGREES, speed, PERCENT, True)

    def jog(self, speed):
        self.m.spin(FORWARD, speed, PERCENT)

    def halt(self):
        self.m.stop()

    def show(self):
        return "%s %.1fmm %.0fdeg" % (self.name, self.mm(), self.deg())


class Gripper:
    """Arm that drops behind a cube (closed) and lifts clear of it (open)."""

    def __init__(self, motor):
        self.name = "G"
        self.m = motor
        motor.set_stopping(HOLD)
        motor.set_position(0, DEGREES)

    def _go(self, deg, speed):
        # Done when the target is reached or the arm stalls (e.g. against a cube).
        timeout = (MOVE_TIMEOUT_FACTOR * 1000.0 * abs(deg - self.deg()) / (MOTOR_MAX_DPS * speed / 100.0)
                   + MOVE_TIMEOUT_EXTRA_MS)
        self.m.set_timeout(timeout, MSEC)
        self.m.spin_to_position(deg, DEGREES, speed, PERCENT, False)
        t0 = now_ms()
        while now_ms() - t0 < timeout:
            wait(LOOP_MS, MSEC)
            if self.m.is_done():
                return
            if now_ms() - t0 > 200 and abs(self.m.velocity(PERCENT)) < 2:
                return

    def open(self):
        self._go(GRIP_OPEN_DEG, SPEED_GRIP_OPEN)
        if abs(self.deg() - GRIP_OPEN_DEG) > GRIP_TOL_DEG:
            raise MotionError("grip did not open (%.0f deg)" % self.deg())

    def close(self):
        self._go(GRIP_CLOSED_DEG, SPEED_GRIP_CLOSE)

    def jog(self, speed):
        self.m.spin(FORWARD, speed, PERCENT)

    def halt(self):
        self.m.stop()

    def deg(self):
        return self.m.position(DEGREES)

    def show(self):
        return "G %.0fdeg" % self.deg()


class Robot:
    def __init__(self):
        self.hw = Hardware()
        self.X = Axis("X", self.hw.mx, X_MM_PER_DEG, 0.0, X_TRAVEL_MAX)
        self.Z = Axis("Z", self.hw.mz, Z_MM_PER_DEG, 0.0, Z_TRAVEL_MAX)
        self.grip = Gripper(self.hw.mg)

    def stop_all(self):
        self.hw.stop_all()


# =============================================================================
# 4 WORLD MAP - pure python, testable on a laptop
# =============================================================================
FAR = 1.0e9     # bin was scanned and nothing was there


class Cube:
    def __init__(self, x, z, w):
        self.x = x          # gripper X to grab it
        self.z = z          # gripper Z to grab it
        self.w = w          # width as seen by the sweep
        self.colour = None


class WorldMap:
    """Distance profile along X ("the graph") + cubes segmented from it.

    z[i] is the grab-Z of the nearest surface seen in bin i:
      None = never scanned, FAR = scanned and empty.
    A new pass overwrites the bins it crosses, so a removed cube disappears
    and whatever stood behind it shows up on the next pass.
    """

    def __init__(self, x_min, x_max, bin_mm, z_empty):
        self.x_min = x_min
        self.bin = bin_mm
        self.z_empty = z_empty
        self.n = int((x_max - x_min) / bin_mm) + 1
        self.z = [None] * self.n
        self.stamp = [0] * self.n
        self.pass_no = 0
        self.cubes = []
        self.failed = []    # [x, attempts]
        self.changed = []   # bin indices changed since the last telemetry flush

    def bin_of(self, x):
        i = int((x - self.x_min) / self.bin)
        return i if 0 <= i < self.n else -1

    def x_of(self, i):
        return self.x_min + (i + 0.5) * self.bin

    def new_pass(self):
        self.pass_no += 1

    def add(self, x, z):
        """One sample: z = grab-Z of the nearest thing at x, or None if nothing."""
        i = self.bin_of(x)
        if i < 0:
            return
        if z is None or z >= self.z_empty:
            z = FAR
        if self.stamp[i] != self.pass_no:
            self.stamp[i] = self.pass_no
        elif self.z[i] is not None and z >= self.z[i]:
            return
        if self.z[i] != z:
            self.z[i] = z
            self.changed.append(i)

    def forget(self, x0, x1):
        for i in range(max(0, self.bin_of(x0)), self.n):
            if self.x_of(i) > x1:
                break
            self.z[i] = None
            self.changed.append(i)

    def segment(self):
        """Rebuild self.cubes from the profile."""
        runs = []
        start = None
        last = 0
        for i in range(self.n):
            if self.z[i] is not None and self.z[i] < FAR:
                if start is not None and abs(self.z[i] - self.z[last]) > MAP_DEPTH_JUMP_MM:
                    runs.append((start, last))      # front cube next to one further back
                    start = None
                if start is None:
                    start = i
                last = i
            elif start is not None and i - last > MAP_MAX_GAP_BINS:
                runs.append((start, last))
                start = None
        if start is not None:
            runs.append((start, last))

        cubes = []
        for i0, i1 in runs:
            width = (i1 - i0 + 1) * self.bin
            if width < CUBE_SEEN_MM * 0.5:
                continue    # noise
            count = max(1, int(width / (CUBE_SEEN_MM + CUBE_GAP_MM * 0.5) + 0.5))
            step = (i1 - i0 + 1) / count
            for k in range(count):
                a = i0 + int(k * step)
                b = i0 + int((k + 1) * step) - 1
                zs = [self.z[j] for j in range(a, b + 1) if self.z[j] is not None and self.z[j] < FAR]
                if zs:
                    cubes.append(Cube((self.x_of(a) + self.x_of(b)) / 2, max(0.0, min(zs)), width / count))
        self.cubes = cubes

    def attempts(self, x):
        for entry in self.failed:
            if abs(entry[0] - x) < CUBE_MM / 2:
                return entry[1]
        return 0

    def mark_failed(self, x):
        for entry in self.failed:
            if abs(entry[0] - x) < CUBE_MM / 2:
                entry[1] += 1
                return
        self.failed.append([x, 1])

    def nearest(self, x):
        best = None
        for c in self.cubes:
            if abs(c.x - x) < CUBE_MM / 2 and (best is None or abs(c.x - x) < abs(best.x - x)):
                best = c
        return best

    def next_target(self, x_ref, z_reach):
        """Cheapest reachable cube: short X trip to the facility, shallow first."""
        best = None
        best_cost = 0
        for c in self.cubes:
            if c.z > z_reach or self.attempts(c.x) >= MAX_ATTEMPTS:
                continue
            cost = abs(c.x - x_ref) + c.z
            if best is None or cost < best_cost:
                best = c
                best_cost = cost
        return best

    def take(self, cube):
        """Cube has left this spot: drop it and forget its bins until rescanned."""
        if cube in self.cubes:
            self.cubes.remove(cube)
        half = cube.w / 2 + 2 * self.bin
        self.forget(cube.x - half, cube.x + half)


def make_slots(x_min, x_max):
    """Drop positions inside one facility zone, deepest row first."""
    pitch = FACILITY_PITCH_MM
    xs = []
    x = x_min + CUBE_MM / 2
    while x + CUBE_MM / 2 <= x_max:
        xs.append(x)
        x += pitch
    if not xs:
        xs = [(x_min + x_max) / 2]
    zs = []
    z = FACILITY_Z_MAX
    while z >= FACILITY_Z_MIN:
        zs.append(z)
        z -= pitch
    if not zs:
        zs = [FACILITY_Z_MIN]
    return [(x, z) for z in zs for x in xs]


# =============================================================================
# 5 TELEMETRY - brain screen + serial lines
#   D,x,z        profile bin (z = -1: empty, -2: unknown)
#   C,x,z,colour cube identified
#   R,x,z        robot position
#   S,state,ms   state finished
#   E,text       event
# =============================================================================
_t_live = [0]


def live(r, d, period=ROBOT_PRINT_MS):
    """Rate-limited live line: R,x,z,d[,hit_x,hit_z]  (d = -1: nothing seen).
    hit = where the distance beam hits, in world coordinates, with current CAL values."""
    t = now_ms()
    if t - _t_live[0] < period:
        return
    _t_live[0] = t
    x = r.X.mm()
    z = r.Z.mm()
    if d is None:
        print("R,%.0f,%.0f,-1" % (x, z))
    else:
        print("R,%.0f,%.0f,%.0f,%.0f,%.0f" % (x, z, d, x + DIST_DX_MM, z + d - GRAB_DIST_MM))


class Telemetry:
    def __init__(self, world):
        self.w = world
        self.header = ""

    def event(self, text):
        print("E," + text)

    def state(self, name, ms):
        print("S,%s,%.0f" % (name, ms))

    def cube(self, c):
        print("C,%.0f,%.0f,%s" % (c.x, c.z, c.colour))

    def flush_map(self, x_robot):
        w = self.w
        for i in w.changed:
            z = w.z[i]
            print("D,%.0f,%.0f" % (w.x_of(i), -2 if z is None else (-1 if z >= FAR else z)))
        w.changed = []
        self.draw(x_robot)

    def draw(self, x_robot):
        """Profile on the brain screen: X left->right, Z top->bottom."""
        w = self.w
        top = 22
        scale_x = float(SCREEN_W) / w.n
        scale_z = float(SCREEN_H - top) / MINING_Z_WALL
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print(self.header)
        brain.screen.set_pen_color(Color.WHITE)
        for i in range(w.n):
            z = w.z[i]
            if z is not None and z < FAR:
                px = int(i * scale_x)
                brain.screen.draw_line(px, top, px, top + int(z * scale_z))
        for c in w.cubes:
            brain.screen.draw_rectangle(int((c.x - w.x_min) / w.bin * scale_x) - 2,
                                        top + int(c.z * scale_z), 5, 5, Color.YELLOW)
        px = int((x_robot - w.x_min) / w.bin * scale_x)
        if 0 <= px < SCREEN_W:
            brain.screen.draw_rectangle(px - 1, top - 4, 3, 4, Color.RED)


# =============================================================================
# 6 SKILLS - each is one step of the ConOps, raises MotionError on trouble
# =============================================================================
def retract(r):
    r.Z.move_to(0.0, SPEED_Z)


def sweep_to(r, world, tel, x_target, speed, carrying=False):
    """The only way X moves. Empty: every sample updates the map.
    Carrying: the sensor sees the held cube, so it is used to detect a drop."""
    if r.Z.mm() > Z_SAFE:
        retract(r)
    if not carrying:
        world.new_pass()
    r.X.start_move(x_target, speed)
    last_x = r.X.mm()
    last_t = now_ms()
    v = 0.0
    lost = 0
    while not r.X.arrived():
        r.X.check()
        x = r.X.mm()
        t = now_ms()
        if t > last_t:
            v = 0.7 * v + 0.3 * (x - last_x) * 1000.0 / (t - last_t)
        last_x = x
        last_t = t
        d = distance_mm(r.hw)
        if carrying:
            lost = lost + 1 if d is None or d > HOLD_DIST_MAX_MM else 0
            if lost >= DROP_CONFIRM_SAMPLES:
                r.X.halt()
                raise CubeLost("cube lost at x=%.0f" % x)
        else:
            z = None if d is None else r.Z.mm() + d - GRAB_DIST_MM
            world.add(x - v * DIST_LAG_S + DIST_DX_MM, z)
        if v > 0 and r.hw.bumper.pressing():
            r.X.halt()
            raise MotionError("bumper hit at x=%.0f" % x)
        live(r, d)
        wait(LOOP_MS, MSEC)
    if not carrying:
        world.segment()
        tel.flush_map(r.X.mm())


def approach(r, cube):
    """Arm open, Z out to the cube, stop when the sensor shows the grab pose.
    Returns False if the cube is not where the map says."""
    r.grip.open()
    r.Z.move_to(max(0.0, cube.z - APPROACH_STANDOFF_MM), SPEED_Z)
    r.Z.start_move(min(cube.z + CREEP_OVERSHOOT_MM, Z_TRAVEL_MAX), SPEED_Z_CREEP)
    while True:
        d = distance_mm(r.hw)
        live(r, d)
        if d is not None and d <= GRAB_DIST_MM:
            r.Z.halt()
            return True
        if r.Z.arrived():
            return False
        r.Z.check()
        wait(LOOP_MS, MSEC)


def grab(r):
    """Drop the arm behind the cube, pull it in, confirm it came along."""
    r.grip.close()
    r.Z.move_to(0.0, SPEED_Z_CARRY)
    d = distance_median(r.hw, 5)
    return d is not None and d <= HOLD_DIST_MAX_MM


def identify(r):
    colour = classify_hue(read_hue(r.hw, COLOUR_SAMPLES))
    if colour == "unknown":
        colour = classify_hue(read_hue(r.hw, COLOUR_SAMPLES))
    return colour


def place(r, world, tel, slot):
    x, z = slot
    sweep_to(r, world, tel, x, SPEED_X_CARRY, carrying=True)
    r.Z.move_to(z, SPEED_Z_CARRY)
    r.grip.open()
    retract(r)


# =============================================================================
# 7 MISSION - state machine. Every state returns the name of the next one.
# =============================================================================
class Mission:
    def __init__(self, robot):
        self.r = robot
        self.w = WorldMap(MINING_X_MIN, MINING_X_MAX, MAP_BIN_MM, MINING_Z_WALL - CUBE_MM / 2)
        self.tel = Telemetry(self.w)
        self.t0 = now_ms()
        self.target = None
        self.colour = None
        self.slots = {"green": make_slots(GREEN_X_MIN, GREEN_X_MAX),
                      "red": make_slots(RED_X_MIN, RED_X_MAX),
                      "blue": [BLUE_DUMP]}
        self.slot_used = {"green": 0, "red": 0, "blue": 0}
        self.empty_scans = 0
        self.faults = 0
        self.state_ms = {}
        self.stats = {"green": 0, "red": 0, "blue": 0, "lost": 0, "failed": 0}

    def time_left(self):
        return MISSION_S - (now_ms() - self.t0) / 1000.0

    def run(self):
        set_led(self.r.hw, "blue")
        state = "SCAN"
        while state != "DONE":
            t = now_ms()
            prev = state
            self.tel.header = "%s %.0fs" % (state, self.time_left())
            try:
                state = getattr(self, "s_" + state)()
            except MotionError as e:
                state = self.recover(prev, e)
            dt = now_ms() - t
            self.tel.state(prev, dt)
            self.state_ms[prev] = self.state_ms.get(prev, 0) + dt
        self.report()

    # ---- states ------------------------------------------------------------
    def s_SCAN(self):
        """Full pass over the mining area towards whichever end is further away."""
        x = self.r.X.mm()
        end = MINING_X_MIN if abs(x - MINING_X_MIN) > abs(x - MINING_X_MAX) else MINING_X_MAX
        sweep_to(self.r, self.w, self.tel, end, SPEED_X_SCAN)
        self.faults = 0
        self.empty_scans += 1
        return "SELECT"

    def s_SELECT(self):
        if self.time_left() < EST_CYCLE_S:
            self.tel.event("out of time")
            return "PARK"
        self.target = self.w.next_target(GREEN_X_MIN, Z_TRAVEL_MAX)
        if self.target is None:
            return "SCAN" if self.empty_scans < MAX_EMPTY_SCANS else "PARK"
        self.empty_scans = 0
        return "APPROACH"

    def s_APPROACH(self):
        # the move to the cube is itself a scan; re-read the target afterwards
        sweep_to(self.r, self.w, self.tel, self.target.x, SPEED_X_TRAVEL)
        fresh = self.w.nearest(self.target.x)
        if fresh is None:
            self.tel.event("target gone at x=%.0f" % self.target.x)
            return "SELECT"
        self.target = fresh
        if not approach(self.r, self.target):
            self.fail("no cube at approach")
            return "SELECT"
        return "GRAB"

    def s_GRAB(self):
        if not grab(self.r):
            self.fail("grab not confirmed")
            self.w.take(self.target)    # it may have moved: rescan that spot
            return "SELECT"
        return "IDENTIFY"

    def s_IDENTIFY(self):
        colour = identify(self.r)
        if colour == "unknown":
            self.tel.event("colour unknown at x=%.0f" % self.target.x)
            colour = UNKNOWN_COLOUR_AS
        self.colour = colour
        self.target.colour = colour
        self.tel.cube(self.target)
        self.w.take(self.target)
        return "PLACE"

    def s_PLACE(self):
        slots = self.slots[self.colour]
        i = self.slot_used[self.colour]
        if i >= len(slots):
            self.tel.event("%s zone full, reusing last slot" % self.colour)
            i = len(slots) - 1
        place(self.r, self.w, self.tel, slots[i])
        self.slot_used[self.colour] += 1
        self.stats[self.colour] += 1
        self.faults = 0
        return "SELECT"

    def s_PARK(self):
        retract(self.r)
        self.r.grip.close()
        return "DONE"

    # ---- recovery ----------------------------------------------------------
    def fail(self, why):
        self.tel.event("%s at x=%.0f" % (why, self.target.x))
        self.stats["failed"] += 1
        self.w.mark_failed(self.target.x)
        self.r.grip.open()
        retract(self.r)

    def recover(self, state, err):
        self.faults += 1
        self.tel.event("fault in %s: %s" % (state, err))
        self.r.stop_all()
        if state == "PARK":
            return "DONE"
        if self.faults >= MAX_FAULTS:
            # parking ends the run and scores nothing more; a pause and retry only costs time
            set_led(self.r.hw, "red")
            wait(FAULT_PAUSE_MS, MSEC)
            set_led(self.r.hw, "blue")
            self.faults = 0
        if isinstance(err, CubeLost):
            self.stats["lost"] += 1
            return "SELECT"
        try:
            retract(self.r)
        except MotionError:
            return "DONE"
        if state in ("APPROACH", "GRAB") and self.target is not None:
            self.w.mark_failed(self.target.x)
        if state == "PLACE":
            return "PLACE"      # still holding the cube: try again
        return "SELECT"

    def report(self):
        set_led(self.r.hw, "white")
        print("E,done: green=%d red=%d blue=%d lost=%d failed=%d time=%.0fs" % (
            self.stats["green"], self.stats["red"], self.stats["blue"],
            self.stats["lost"], self.stats["failed"], MISSION_S - self.time_left()))
        for name in self.state_ms:
            print("E,time in %s: %.1fs" % (name, self.state_ms[name] / 1000.0))


# =============================================================================
# 8 CALIBRATION
#   startup()          every run, before the start button: self check
#   calibration_menu() MODE = "CALIBRATE": bench routines that print
#                      paste-ready CONFIG lines. Order: Z, X, Grip, Cube, Colour
# =============================================================================
def startup(r):
    say("LUNAR T2", "self check...")
    missing = r.hw.missing()
    if missing:
        say("MISSING:", ", ".join(missing))
        set_led(r.hw, "red")
        return False
    r.hw.optical.set_light(100)
    warnings = []
    if not r.hw.has_touch:
        warnings.append("no Touch LED: Check=start")
    battery = brain.battery.capacity()
    if battery < BATTERY_WARN_PCT:
        warnings.append("battery %.0f%%" % battery)
    if not CALIBRATED:
        warnings.append("placeholder CAL values")
    # actuator check before the clock starts (FMEA #1/#3): arm must open and close
    try:
        r.grip.open()
    except MotionError as e:
        say("GRIP FAILED", str(e))
        set_led(r.hw, "red")
        return False
    r.grip.close()
    if abs(r.grip.deg() - GRIP_CLOSED_DEG) > 50:
        warnings.append("grip did not close")
    for w in warnings:
        print("E,WARNING " + w)
    set_led(r.hw, "yellow" if warnings else "green")
    say("READY", "press start", *warnings)
    return True


def jog(r, title, axes, dashboard=False):
    """Hold Left/Right: move selected axis. Tap LED: next axis.
    Tap Check: confirm (True). Hold Check > 1 s: leave (False).
    Without a Touch LED: press Left+Right together to switch axis."""
    k = 0
    moving = 0
    touch_was = False
    t_show = 0
    while True:
        left, right, check, touch = buttons(r.hw)
        if left and right:
            touch, left, right = True, False, False
        ax = axes[k]
        live(r, distance_mm(r.hw), LIVE_CAL_MS)
        want = -1 if left else (1 if right else 0)
        if want != moving:
            if want == 0:
                ax.halt()
            else:
                ax.jog(want * SPEED_JOG)
            moving = want
        if touch and not touch_was:
            ax.halt()
            moving = 0
            k = (k + 1) % len(axes)
        touch_was = touch
        if check:
            ax.halt()
            t0 = now_ms()
            while brain.buttonCheck.pressing():
                wait(20, MSEC)
            return now_ms() - t0 < 1000
        if now_ms() - t_show > 300:
            t_show = now_ms()
            lines = [title, "> " + axes[k].show(), "L/R move, LED or L+R", "=axis. Check ok/hold"]
            if dashboard:
                lines.append("d=%s hue=%.0f bump=%d" % (distance_mm(r.hw), r.hw.optical.hue(),
                                                       int(r.hw.bumper.pressing())))
            say(*lines)
        wait(20, MSEC)


def cal_move_deg(r, axis, deg, speed):
    """Raw blocking move that keeps the live feed running."""
    axis.m.set_timeout(axis.expected_ms((deg - axis.deg()) * axis.k, speed), MSEC)
    axis.m.spin_to_position(deg, DEGREES, speed, PERCENT, False)
    wait(50, MSEC)
    while not axis.m.is_done():
        live(r, distance_mm(r.hw), LIVE_CAL_MS)
        wait(LOOP_MS, MSEC)


def fit_line(pts):
    """Least squares y = a + b*x. Returns (a, b, rms residual)."""
    n = float(len(pts))
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    b = sxy / sxx
    a = my - b * mx
    rms = (sum((p[1] - a - b * p[0]) ** 2 for p in pts) / n) ** 0.5
    return a, b, rms


def extend_until_near(r):
    """Creep Z outward until the board is CAL_Z_NEAR_MM away or the arm stalls
    at the end of its travel. Returns the encoder degrees reached."""
    r.Z.jog(SPEED_Z_CREEP)
    t0 = now_ms()
    t_still = None
    while now_ms() - t0 < CAL_Z_MAX_MS:
        d = distance_mm(r.hw)
        live(r, d, LIVE_CAL_MS)
        if d is not None and d < CAL_Z_NEAR_MM:
            break
        if abs(r.hw.mz.velocity(PERCENT)) < 2 and now_ms() - t0 > 500:
            t_still = t_still or now_ms()
            if now_ms() - t_still > 300:
                break               # end of travel
        else:
            t_still = None
        wait(LOOP_MS, MSEC)
    r.Z.halt()
    return r.Z.deg()


def cal_z_scale(r):
    """Z via the distance sensor: it looks along Z, so no ruler is needed.
    Fully automatic: extends until close to the board (or the end), then steps back."""
    say("Z SCALE", "Flat board across the", "arm path, in front.", "Check = start (auto)")
    wait_press(r.hw)
    deg_max = extend_until_near(r)
    if deg_max < 100:
        say("Z SCALE FAILED", "arm did not extend:", "check Z_MOTOR_REVERSED")
        print("# Z did not extend (%.0f deg): Z_MOTOR_REVERSED wrong or arm blocked" % deg_max)
        return
    pts = []
    for i in range(CAL_Z_STEPS + 1):
        deg = deg_max * (CAL_Z_STEPS - i) / CAL_Z_STEPS
        cal_move_deg(r, r.Z, deg, SPEED_Z_CREEP)
        wait(300, MSEC)
        d = distance_median(r.hw, 15)
        live(r, d, 0)
        print("CAL Z deg=%.0f d=%s" % (r.Z.deg(), d))
        if d is not None:
            pts.append((r.Z.deg(), d))
    if len(pts) < 3:
        say("Z SCALE FAILED", "board not seen")
        return
    a, b, rms = fit_line(pts)
    k = -b
    print("Z_MM_PER_DEG = %.5f    # cal_z_scale, rms %.2f mm, %d pts" % (k, rms, len(pts)))
    print("# reached Z = %.0f mm (board close or end of travel)" % (deg_max * k))
    if k < 0:
        print("# negative: flip Z_MOTOR_REVERSED")
    say("Z_MM_PER_DEG", "%.5f" % k, "rms %.2f mm" % rms, "copy from console")
    wait_press(r.hw)


def cal_x_scale(r):
    """X via one ruler measurement over the longest travel."""
    say("X SCALE", "Arm retracted!", "Tape-mark the carriage.",
        "Jog X far to facility", "then Check")
    if not jog(r, "X towards facility", [r.X]):
        return
    deg = r.X.deg()
    print("CAL X moved %.0f deg. Mark again and measure between the marks:" % deg)
    print("X_MM_PER_DEG = <measured_mm> / %.1f    # cal_x_scale" % deg)
    if deg < 0:
        print("# negative: flip X_MOTOR_REVERSED")
    say("X moved %.0f deg" % deg, "measure the marks", "Check = drive back")
    wait_press(r.hw)
    cal_move_deg(r, r.X, 0, SPEED_X_SCAN)


def cal_grip(r):
    say("GRIP", "Jog to OPEN (lifted)", "then Check")
    if not jog(r, "grip open", [r.grip]):
        return
    print("GRIP_OPEN_DEG = %.0f    # cal_grip" % r.grip.deg())
    say("GRIP", "Jog to CLOSED (down)", "then Check")
    if not jog(r, "grip closed", [r.grip]):
        return
    print("GRIP_CLOSED_DEG = %.0f    # cal_grip" % r.grip.deg())


def sweep_edges(r, x_from, x_to, limit):
    """Sweep X and return (centre, width, speed mm/s) of the longest run of
    readings closer than `limit` (= the cube), or None. Streams, stores nothing."""
    r.X.move_to(x_from, SPEED_X_TRAVEL)
    wait(CAL_SETTLE_MS, MSEC)       # let a lagging sensor catch up before recording
    r.X.start_move(x_to, SPEED_X_SCAN)
    t0 = now_ms()
    run = None                      # [first x, last x] of the current run
    best = None
    while not r.X.arrived():
        r.X.check()
        x = r.X.mm()
        d = distance_mm(r.hw)
        live(r, d, LIVE_CAL_MS)
        if d is not None and d < limit:
            run = [x, x] if run is None else [run[0], x]
            if best is None or abs(run[1] - run[0]) > abs(best[1] - best[0]):
                best = list(run)
        else:
            run = None
        wait(LOOP_MS, MSEC)
    if best is None:
        return None
    v = abs(x_to - x_from) * 1000.0 / max(1, now_ms() - t0)
    return (best[0] + best[1]) / 2, abs(best[1] - best[0]), v


def cal_cube(r):
    """Teach the grab pose on a real cube, then sweep it both ways.
    Needs X and Z scale done first."""
    say("CUBE POSE", "Cube in front of arm.", "Jog X/Z/grip into the", "grab pose, Check")
    if not jog(r, "grab pose", [r.X, r.Z, r.grip]):
        return
    x_align = r.X.mm()
    wait(CAL_SETTLE_MS, MSEC)
    d_grab = distance_median(r.hw, 15)
    if d_grab is None:
        say("CUBE FAILED", "no distance reading", "at the grab pose")
        return
    print("GRAB_DIST_MM = %.1f    # cal_cube, at Z=%.0f" % (d_grab, r.Z.mm()))
    limit = r.Z.mm() + d_grab + CUBE_MM / 2     # cube face seen from Z = 0, plus margin
    r.grip.open()
    r.Z.move_to(0.0, SPEED_Z_CREEP)
    lo = max(0.0, x_align - CAL_CUBE_SWEEP_MM)
    hi = min(X_TRAVEL_MAX, x_align + CAL_CUBE_SWEEP_MM)
    fwd = sweep_edges(r, lo, hi, limit)
    back = sweep_edges(r, hi, lo, limit)
    if fwd is None or back is None:
        say("CUBE FAILED", "cube not seen in sweep")
        return
    centre = (fwd[0] + back[0]) / 2
    v = (fwd[2] + back[2]) / 2
    lag = (fwd[0] - back[0]) / 2 / v if v > 0 else 0.0
    print("DIST_DX_MM = %.1f    # cal_cube" % (x_align - centre))
    print("DIST_LAG_S = %.3f    # cal_cube, sweep %.0f mm/s" % (lag, v))
    print("CUBE_SEEN_MM = %.1f    # cal_cube" % ((fwd[1] + back[1]) / 2))
    say("CUBE POSE done", "copy from console")
    wait_press(r.hw)


def cal_colour(r):
    while True:
        say("COLOUR", "Cube held (pulled in),", "sensor 17mm above top.", "Check=sample L=exit")
        if wait_press(r.hw) in ("T", "L"):
            return
        hues = []
        bright = []
        near = 0
        for _ in range(30):
            b = r.hw.optical.brightness()
            bright.append(b)
            if r.hw.optical.is_near_object():
                near += 1
            if r.hw.optical.is_near_object() or b >= COLOUR_MIN_BRIGHTNESS:
                hues.append(r.hw.optical.hue())
            wait(LOOP_MS, MSEC)
        if not hues:
            say("no object near", "sensor")
            wait_press(r.hw)
            continue
        h = hue_median(hues)
        print("CAL COLOUR hue min=%.0f med=%.0f max=%.0f bright=%.0f near=%d/30 -> %s" % (
            min(hues), h, max(hues), median(bright), near, classify_hue(h)))


def cal_teach(r):
    """Jog anywhere, Check prints the pose: facility corners, walls, limits."""
    n = 0
    while jog(r, "TEACH pt %d" % (n + 1), [r.X, r.Z, r.grip]):
        n += 1
        print("TEACH %d: X=%.1f Z=%.1f grip=%.0f" % (n, r.X.mm(), r.Z.mm(), r.grip.deg()))


def cal_distance(r):
    """Live distance readout, compare against a ruler. Any button exits."""
    while True not in buttons(r.hw):
        vals = []
        for _ in range(10):
            d = distance_mm(r.hw)
            if d is not None:
                vals.append(d)
            wait(LOOP_MS, MSEC)
        if vals:
            say("DISTANCE", "med %.1f mm" % median(vals), "min %.1f max %.1f" % (min(vals), max(vals)))
            live(r, median(vals), 0)
        else:
            say("DISTANCE", "nothing seen")
            live(r, None, 0)
        wait(300, MSEC)
    wait_press(r.hw)


def median_interval(ts):
    """Median time between consecutive change times, 0 if too few."""
    d = sorted([ts[k + 1] - ts[k] for k in range(len(ts) - 1)])
    return d[len(d) // 2] if d else 0


def cal_rates(r):
    """How often each sensor really updates, and from that the fastest X scan
    speed whose sampling error stays within RATE_EDGE_TOL_MM. Run after X scale.
    Arm retracted, X clear for RATE_SWEEP_MM; cubes in front help (changing readings).
    An edge is seen up to one distance update + one loop late and tagged with an
    encoder value up to one encoder update old: error = v * (Td + Tloop + Tx) / 2
    after DIST_LAG_S removes the mean."""
    hw = r.hw
    say("RATES", "arm in, X clear", "for %.0f mm" % RATE_SWEEP_MM, "Check = start")
    wait_press(hw)
    n = 200
    cost = []
    for name, f in (("distance", lambda: hw.distance.object_distance(MM)), ("hue", hw.optical.hue),
                    ("encoder", lambda: hw.mx.position(DEGREES)),
                    ("scan loop", lambda: (r.X.mm(), distance_mm(hw), hw.bumper.pressing()))):
        t0 = now_ms()
        for _ in range(n):
            f()
        cost.append((now_ms() - t0) / n)
        print("# RATE call %s: %.2f ms" % (name, cost[-1]))

    last = [None, None, None]
    times = [[], [], []]
    x0 = r.X.mm()
    t1 = t2 = 0
    for x_to in (x0 + RATE_SWEEP_MM, x0):
        r.X.start_move(x_to, SPEED_X_SCAN)
        while not r.X.arrived():
            r.X.check()
            t = now_ms()
            x = r.X.mm()
            if x_to > x0:       # steady speed between 1/4 and 3/4 of the way out
                if not t1 and x >= x0 + RATE_SWEEP_MM / 4:
                    t1 = t
                if not t2 and x >= x0 + RATE_SWEEP_MM * 3 / 4:
                    t2 = t
            vals = (hw.distance.object_distance(MM), (hw.optical.hue(), hw.optical.brightness()), hw.mx.position(DEGREES))
            for i in range(3):
                if vals[i] != last[i]:
                    last[i] = vals[i]
                    if len(times[i]) < RATE_MAX_SAMPLES:
                        times[i].append(t)
            wait(RATE_POLL_MS, MSEC)
    td, to, tx = [median_interval(ts) for ts in times]
    for name, ts, p in (("distance", times[0], td), ("optical", times[1], to), ("encoder", times[2], tx)):
        print("# RATE %s: %d changes, updates every %.0f ms" % (name, len(ts), p))
    if td <= 0 or t2 <= t1:
        say("RATES", "no data:", "nothing changed", "or X did not move")
        wait_press(hw)
        return

    v = RATE_SWEEP_MM / 2 * 1000.0 / (t2 - t1)
    loop = max(RATE_POLL_MS, int(td / 2))              # poll twice per distance update
    err_per_v = (td + tx + loop + cost[3]) / 2000.0    # mm of edge error per mm/s
    v_max = min(RATE_EDGE_TOL_MM / err_per_v, 1000.0 * MAP_BIN_MM / td)
    pct = min(100, int(SPEED_X_SCAN * v_max / v))
    print("# now: %d%% = %.0f mm/s, edge error +-%.1f mm" % (SPEED_X_SCAN, v, v * err_per_v))
    print("SPEED_X_SCAN = %d    # cal_rates: %.0f mm/s, edge error +-%.1f mm" % (pct, v_max, RATE_EDGE_TOL_MM))
    print("LOOP_MS = %d    # cal_rates: distance updates every %.0f ms" % (loop, td))
    print("# colour: optical updates every %.0f ms, %d samples need >= %.0f ms to be independent"
          % (to, COLOUR_SAMPLES, COLOUR_SAMPLES * to))
    say("RATES dist %.0f ms" % td, "opt %.0f enc %.0f ms" % (to, tx),
        "scan %d%% %.0f mm/s" % (pct, v_max), "LOOP_MS %d" % loop)
    wait_press(hw)


CAL_ROUTINES = (("Z scale", cal_z_scale), ("X scale", cal_x_scale), ("Grip", cal_grip),
                ("Cube pose", cal_cube), ("Colour", cal_colour), ("Teach pts", cal_teach),
                ("Distance", cal_distance), ("Rates", cal_rates))


def calibration_menu(r):
    r.hw.optical.set_light(100)
    i = 0
    while True:
        say("CALIBRATE", "< %s >" % CAL_ROUTINES[i][0], "L/R choose", "Check run")
        b = wait_press(r.hw)
        if b == "L":
            i = (i - 1) % len(CAL_ROUTINES)
        elif b == "R":
            i = (i + 1) % len(CAL_ROUTINES)
        elif b == "C":
            try:
                CAL_ROUTINES[i][1](r)
            except MotionError as e:
                r.stop_all()
                say("MOTION ERROR", str(e))
                wait_press(r.hw)


def motor_check(r):
    """Move every motor a little on its own and report whether its encoder
    followed. Clear space around the robot first."""
    say("MOTOR CHECK", "each motor moves a", "bit and back", "Check = start")
    wait_press(r.hw)
    # X and Z start at their zero end, so they must go positive (towards the
    # facility / out); the gripper starts closed, so it goes towards open.
    to_open = 1 if GRIP_OPEN_DEG > GRIP_CLOSED_DEG else -1
    for name, m, sign in (("X", r.hw.mx, 1), ("Z", r.hw.mz, 1), ("grip", r.hw.mg, to_open)):
        before = m.position(DEGREES)
        m.spin(FORWARD, 30 * sign, PERCENT)
        wait(400, MSEC)
        m.stop()
        after = m.position(DEGREES)
        wait(200, MSEC)
        m.spin(FORWARD, -30 * sign, PERCENT)
        wait(400, MSEC)
        m.stop()
        moved = abs(after - before)
        print("CHECK %s: moved %.0f deg (installed=%s) -> %s" % (
            name, moved, m.installed(),
            "OK" if moved > 20 else "NOT MOVING (if it pushed into its end stop: set its _REVERSED)"))
    say("MOTOR CHECK done", "see console", "X %.0f Z %.0f" % (r.hw.mx.position(DEGREES), r.hw.mz.position(DEGREES)))
    wait_press(r.hw)


def test_mode(r):
    """Jog every actuator while watching all sensors."""
    r.hw.optical.set_light(100)
    while True:
        jog(r, "TEST", [r.X, r.Z, r.grip], dashboard=True)


# =============================================================================
# 9 ENTRY
# =============================================================================
def select_mode(r):
    """MODE from CONFIG, unless a button is pressed during start-up."""
    default = MODE if MODE in BUILD_MODES else BUILD_MODES[0]
    if len(BUILD_MODES) == 1:
        return default
    set_led(r.hw, "purple")
    say("hold now:", "Left  = CALIBRATE", "Right = TEST", "(else " + default + ")")
    t0 = now_ms()
    while now_ms() - t0 < MODE_SELECT_MS:
        if brain.buttonLeft.pressing() or brain.buttonRight.pressing():
            mode = "CALIBRATE" if brain.buttonLeft.pressing() else "TEST"
            while brain.buttonLeft.pressing() or brain.buttonRight.pressing():
                wait(20, MSEC)
            return mode if mode in BUILD_MODES else default
        if r.hw.has_touch and r.hw.touch.pressing():
            while r.hw.touch.pressing():
                set_led(r.hw, "yellow" if now_ms() - t0 >= 2 * MODE_SELECT_MS else "white")
                wait(20, MSEC)
            mode = "TEST" if now_ms() - t0 >= 2 * MODE_SELECT_MS else "CALIBRATE"
            return mode if mode in BUILD_MODES else default
        wait(20, MSEC)
    return default


def main():
    r = Robot()     # start pose = world origin: all encoders zeroed here
    try:
        try:
            import gc
            gc.collect()
            print("E,heap free %d used %d" % (gc.mem_free(), gc.mem_alloc()))
        except Exception:   # noqa: BLE001 - only a diagnostic
            pass
        mode = select_mode(r)
        print("E,mode " + mode)
        if mode == "CALIBRATE":
            calibration_menu(r)
        elif mode == "TEST":
            test_mode(r)
        elif mode == "CHECK":
            motor_check(r)
        else:
            if not startup(r):
                return
            while not start_pressed(r.hw):
                wait(20, MSEC)
            Mission(r).run()
    finally:
        r.stop_all()


if __name__ == "__main__":
    main()
