"""Lunar resource sorter. Edit this file; build.py generates main.py.

X = 0 at the RIGHT home switch; left is negative. Z = 0 is retracted.
Start with the arm retracted and the gripper closed. X is homed automatically.
Sections: configuration, hardware/motion, local search, mission, calibration.
"""
from vex import *

MODE = "MISSION"
BUILD_MODES = ("MISSION", "CALIBRATE", "TEST")
MODE_SELECT_MS = 2000

# 1. CONFIGURATION -----------------------------------------------------------
# The mechanical bumper is the home switch, not the optional Touch LED.
PORT_Z = Ports.PORT1
PORT_HOME = Ports.PORT2
PORT_TOUCH = Ports.PORT3
PORT_X = Ports.PORT5
PORT_OPTICAL = Ports.PORT7
PORT_DISTANCE = Ports.PORT8
PORT_GRIP = Ports.PORT9
X_MOTOR_REVERSED = False       # positive degrees must move RIGHT
Z_MOTOR_REVERSED = False       # positive degrees must extend the arm

# MEASURE these positions from the right-hand home switch, in millimetres.
X_TRAVEL_MIN = -1200.0
Z_TRAVEL_MAX = 400.0
HOME_CLEAR_X = -10.0           # release the home switch after zeroing
SEARCH_START_X = -350.0        # move 350 mm left before searching (placeholder)
SEARCH_END_X = -1150.0         # finish scanning before the left end of the rail
MINING_Z_WALL = 350.0          # grab-Z of the background wall
# Drop poses: deepest first, so the arm avoids already placed cubes.
# All are placeholders. Teach real positions; each slot is used only once.
GREEN_SLOTS = ((-250.0, 240.0), (-250.0, 150.0), (-250.0, 60.0))
RED_SLOTS = ((-140.0, 240.0), (-140.0, 150.0), (-140.0, 60.0))
BLUE_SLOTS = ((-50.0, 240.0), (-50.0, 150.0), (-50.0, 60.0))

# CALIBRATE before running a physical mission (see docs/calibration.md).
CALIBRATED = False
X_MM_PER_DEG = 0.10
Z_MM_PER_DEG = 0.10
GRAB_DIST_MM = 10.0
DIST_DX_MM = 0.0              # taught gripper centre minus observed scan centre
DIST_LAG_S = 0.0
CUBE_SEEN_MM = 75.0           # apparent width of a straight cube at scan speed
HOLD_DIST_MAX_MM = 20.0
GRIP_OPEN_DEG = -500
GRIP_CLOSED_DEG = 0
HUE_RANGES = {"red": (330.0, 25.0), "green": (70.0, 170.0), "blue": (180.0, 270.0)}
COLOUR_SAMPLES = 7
COLOUR_MIN_BRIGHTNESS = 5.0

# Local search. Unknown distance readings never count as clear space.
CUBE_MM = 75.0
GRIP_DEPTH_MM = 80.0
SIDE_GAP_MM = 15.0            # minimum visible clearance on EACH side
EDGE_MARGIN_MM = 2.0          # additional allowance for edge/sampling error
WIDTH_TOL_MM = 3.0
ROTATION_WIDTH_MM = 12.0      # 75 mm cube at 10 degrees projects to about 87 mm
DEPTH_JUMP_MM = 20.0          # split abrupt steps BETWEEN samples, not a sloping face
CLEARANCE_DEPTH_MM = 5.0      # space beyond the jaw tips
SCAN_SAMPLE_MM = 1.0
SCAN_MAX_SPACING_MM = 2.0    # a larger hole in samples invalidates the window
DIST_MAX_VALID_MM = 1000.0

# Motion: positive degrees are right/out; both scales above must be positive.
SPEED_HOME = 20
HOME_TIMEOUT_MS = 120000
HOME_RELEASE_MAX_MM = 25.0
HOME_DEBOUNCE_MS = 60
SPEED_X_TRAVEL = 80
SPEED_X_SCAN = 25
SPEED_X_CARRY = 50
SPEED_Z = 80
SPEED_Z_CREEP = 20
SPEED_Z_CARRY = 50
SPEED_GRIP_OPEN = 20
SPEED_GRIP_CLOSE = 50
SPEED_JOG = 25
GRIP_TOL_DEG = 30
MOTOR_MAX_DPS = 720.0
POS_TOL_MM = 1.0
MOVE_TIMEOUT_FACTOR = 2.0
MOVE_TIMEOUT_EXTRA_MS = 1500
Z_SAFE = 2.0
APPROACH_STANDOFF_MM = 30.0
CREEP_OVERSHOOT_MM = 20.0
DROP_CONFIRM_SAMPLES = 3
LOOP_MS = 15
MISSION_S = 600
EST_CYCLE_S = 90             # reserve time for picking/returning before next search
MAX_PICK_FAILURES = 3
ROBOT_PRINT_MS = 200
LIVE_CAL_MS = 100

# Bench calibration settings.
CAL_Z_STEPS = 8
CAL_Z_NEAR_MM = 60.0
CAL_Z_MAX_MS = 30000
CAL_CUBE_SWEEP_MM = 110.0
CAL_SETTLE_MS = 500
RATE_SWEEP_MM = 300.0
RATE_POLL_MS = 2
RATE_EDGE_TOL_MM = 1.0
RATE_MAX_SAMPLES = 400


# 2. HARDWARE AND MOTION -----------------------------------------------------
brain = Brain()


class MotionError(Exception):
    pass


class Hardware:
    def __init__(self):
        self.home = Bumper(PORT_HOME)
        self.touch = Touchled(PORT_TOUCH)
        self.has_touch = self.touch.installed()
        self.distance = Distance(PORT_DISTANCE)
        self.optical = Optical(PORT_OPTICAL)
        self.mx = Motor(PORT_X, True) if X_MOTOR_REVERSED else Motor(PORT_X)
        self.mz = Motor(PORT_Z, True) if Z_MOTOR_REVERSED else Motor(PORT_Z)
        self.mg = Motor(PORT_GRIP)

    def missing(self):
        devices = (("home switch", self.home), ("distance", self.distance),
                   ("optical", self.optical), ("X", self.mx), ("Z", self.mz), ("grip", self.mg))
        return [name for name, dev in devices if not dev.installed()]

    def stop_all(self):
        for m in (self.mx, self.mz, self.mg):
            m.stop()


def now_ms():
    return brain.timer.time(MSEC)


def median(values):
    return sorted(values)[len(values) // 2]


def distance_mm(hw):
    d = hw.distance.object_distance(MM)
    return d if 0 < d <= DIST_MAX_VALID_MM else None


def distance_median(hw, n):
    values = []
    for _ in range(n):
        d = distance_mm(hw)
        if d is not None:
            values.append(d)
        wait(LOOP_MS, MSEC)
    return median(values) if len(values) > n // 2 else None


def hue_median(hues):
    if max(hues) - min(hues) > 180:
        hues = [h + 360 if h < 180 else h for h in hues]
    return median(hues) % 360


def classify_hue(h):
    if h is not None:
        for colour in ("red", "green", "blue"):
            lo, hi = HUE_RANGES[colour]
            if (lo <= h <= hi) if lo <= hi else (h >= lo or h <= hi):
                return colour
    return "unknown"


def buttons(hw):
    return (brain.buttonLeft.pressing(), brain.buttonRight.pressing(),
            brain.buttonCheck.pressing(), hw.has_touch and hw.touch.pressing())


def wait_press(hw):
    while True:
        l, r, c, t = buttons(hw)
        if l or r or c or t:
            while True in buttons(hw):
                wait(20, MSEC)
            return "L" if l else "R" if r else "C" if c else "T"
        wait(20, MSEC)


def say(*lines):
    brain.screen.clear_screen()
    for i, line in enumerate(lines):
        brain.screen.set_cursor(i + 1, 1)
        brain.screen.print(line)
    print("E," + " | ".join(lines))


_t_live = [0]


def live(r, d, period=ROBOT_PRINT_MS):
    if now_ms() - _t_live[0] < period:
        return
    _t_live[0] = now_ms()
    x, z = r.X.mm(), r.Z.mm()
    if d is None:
        print("R,%.1f,%.1f,-1" % (x, z))
    else:
        print("R,%.1f,%.1f,%.1f,%.1f,%.1f" % (x, z, d, x + DIST_DX_MM, z + d - GRAB_DIST_MM))


class Axis:
    def __init__(self, name, motor, scale, lo, hi):
        self.name, self.m, self.k = name, motor, scale
        self.lo, self.hi = lo, hi
        self.target = 0.0
        self.t_start = 0
        self.timeout = 0
        motor.set_stopping(HOLD)
        motor.set_position(0, DEGREES)

    def deg(self):
        return self.m.position(DEGREES)

    def mm(self):
        return self.deg() * self.k

    def expected_ms(self, distance, speed):
        mm_s = self.k * MOTOR_MAX_DPS * speed / 100.0
        return MOVE_TIMEOUT_FACTOR * abs(distance) * 1000.0 / mm_s + MOVE_TIMEOUT_EXTRA_MS

    def start_move(self, target, speed):
        if not self.lo <= target <= self.hi:
            raise MotionError("%s target outside travel: %.1f" % (self.name, target))
        self.target = target
        self.t_start = now_ms()
        self.timeout = self.expected_ms(target - self.mm(), speed)
        self.m.set_timeout(self.timeout + 500, MSEC)
        self.m.spin_to_position(target / self.k, DEGREES, speed, PERCENT, False)

    def arrived(self):
        return abs(self.mm() - self.target) <= POS_TOL_MM

    def check(self):
        if now_ms() - self.t_start > self.timeout:
            raise MotionError(self.name + " motion timed out")
        if now_ms() - self.t_start > 150 and self.m.is_done() and not self.arrived():
            raise MotionError(self.name + " motion blocked")

    def move_to(self, target, speed):
        self.start_move(target, speed)
        try:
            while not self.arrived():
                self.check()
                wait(LOOP_MS, MSEC)
        finally:
            self.halt()

    def jog(self, speed):
        self.m.spin(FORWARD, speed, PERCENT)

    def halt(self):
        self.m.stop()

    def show(self):
        return "%s %.1fmm %.0fdeg" % (self.name, self.mm(), self.deg())


class Gripper:
    def __init__(self, motor):
        self.m = motor
        motor.set_stopping(HOLD)
        motor.set_position(0, DEGREES)

    def move(self, degrees, speed):
        self.m.set_timeout(5000, MSEC)
        self.m.spin_to_position(degrees, DEGREES, speed, PERCENT, True)
        if abs(self.deg() - degrees) > GRIP_TOL_DEG:
            raise MotionError("gripper did not reach %.0f degrees" % degrees)

    def open(self):
        self.move(GRIP_OPEN_DEG, SPEED_GRIP_OPEN)

    def close(self):
        self.move(GRIP_CLOSED_DEG, SPEED_GRIP_CLOSE)

    def deg(self):
        return self.m.position(DEGREES)

    def jog(self, speed):
        self.m.spin(FORWARD, speed, PERCENT)

    def halt(self):
        self.m.stop()

    def show(self):
        return "G %.0fdeg" % self.deg()


class Robot:
    def __init__(self):
        self.hw = Hardware()
        self.X = Axis("X", self.hw.mx, X_MM_PER_DEG, X_TRAVEL_MIN, 0.0)
        self.Z = Axis("Z", self.hw.mz, Z_MM_PER_DEG, 0.0, Z_TRAVEL_MAX)
        self.grip = Gripper(self.hw.mg)
        self.homed = False

    def stop_all(self):
        self.hw.stop_all()

    def home_x(self):
        """Seek the RIGHT switch, zero X, then move left clear of it."""
        if self.Z.mm() > Z_SAFE:
            raise MotionError("retract Z before homing")
        if not self.hw.home.installed():
            raise MotionError("home switch missing")
        self.homed = False
        say("HOME", "moving right to switch")
        t0, x0 = now_ms(), self.X.mm()
        try:
            # An initially pressed switch must release, then be approached again.
            while self.hw.home.pressing():
                if now_ms() - t0 > 5000 or abs(self.X.mm() - x0) > HOME_RELEASE_MAX_MM:
                    raise MotionError("home switch stuck pressed")
                self.X.jog(-SPEED_HOME)
                wait(LOOP_MS, MSEC)
            self.X.halt()
            t0, x0 = now_ms(), self.X.mm()
            last_motion, last_x = t0, x0
            while True:
                if now_ms() - t0 > HOME_TIMEOUT_MS or self.X.mm() - x0 > -X_TRAVEL_MIN + HOME_RELEASE_MAX_MM:
                    raise MotionError("home switch not reached")
                if self.hw.home.pressing():
                    self.X.halt()       # stop pushing before debouncing contact
                    wait(HOME_DEBOUNCE_MS, MSEC)
                    if self.hw.home.pressing():
                        break
                if abs(self.X.mm() - last_x) >= 0.5:
                    last_motion, last_x = now_ms(), self.X.mm()
                elif now_ms() - last_motion > 1500:
                    raise MotionError("X stalled before home switch")
                self.X.jog(SPEED_HOME)
                wait(LOOP_MS, MSEC)
            self.X.m.set_position(0, DEGREES)
            self.X.move_to(HOME_CLEAR_X, SPEED_HOME)
            if self.hw.home.pressing():
                raise MotionError("home switch did not release")
            self.homed = True
            say("HOME set", "X=0 at right switch")
        finally:
            self.X.halt()

    def move_x(self, target, speed, carrying=False):
        if not self.homed or self.Z.mm() > Z_SAFE:
            raise MotionError("X needs home and retracted Z")
        self.X.start_move(target, speed)
        lost = 0
        try:
            while not self.X.arrived():
                self.X.check()
                if target > self.X.mm() and self.hw.home.pressing():
                    raise MotionError("unexpected home switch during travel")
                d = distance_mm(self.hw)
                if carrying:
                    lost = lost + 1 if d is None or d > HOLD_DIST_MAX_MM else 0
                    if lost >= DROP_CONFIRM_SAMPLES:
                        raise MotionError("cube lost during transport")
                live(self, d)
                wait(LOOP_MS, MSEC)
        finally:
            self.X.halt()


# 3. LOCAL SEARCH -----------------------------------------------------------
class CubeSearch:
    """One small moving window, ordered right to left. No persistent field map.

    Samples use gripper-alignment X and grab-Z. Only complete cube-width runs with
    measured free space on BOTH sides can become targets. A deeper return is
    clear only if it lies beyond the entire jaw path, including the jaw tips.
    """
    def __init__(self):
        self.samples = []

    def add(self, x, z):
        if self.samples:
            step = self.samples[-1][0] - x
            if step < SCAN_SAMPLE_MM:
                return None
            if step > SCAN_MAX_SPACING_MM:
                self.samples = []
        self.samples.append((x, z))
        span = CUBE_SEEN_MM + ROTATION_WIDTH_MM + WIDTH_TOL_MM + 2 * (SIDE_GAP_MM + EDGE_MARGIN_MM) + 4 * SCAN_MAX_SPACING_MM
        while self.samples[0][0] - x > span:
            self.samples.pop(0)
        return self.target()

    def clear_side(self, index, step, edge, depth):
        """Require valid deep readings all the way past the requested gap."""
        while 0 <= index < len(self.samples):
            x, z = self.samples[index]
            if z is None or z < depth:
                return False
            if abs(x - edge) >= SIDE_GAP_MM + EDGE_MARGIN_MM:
                return True
            index += step
        return False

    def target(self):
        s = self.samples
        i = 0
        while i < len(s):
            z = s[i][1]
            if z is None or z < 0 or z >= MINING_Z_WALL - CUBE_MM / 2:
                i += 1
                continue
            first = i
            near = far = z
            i += 1
            while i < len(s) and s[i][1] is not None:
                z = s[i][1]
                if abs(z - s[i - 1][1]) > DEPTH_JUMP_MM:
                    break
                near, far = min(near, z), max(far, z)
                i += 1
            # A window/scan boundary cannot establish a cube edge.
            if first == 0 or i == len(s):
                continue
            right = (s[first - 1][0] + s[first][0]) / 2
            left = (s[i - 1][0] + s[i][0]) / 2
            width = right - left
            if not CUBE_SEEN_MM - WIDTH_TOL_MM <= width <= CUBE_SEEN_MM + ROTATION_WIDTH_MM + WIDTH_TOL_MM:
                continue
            if far - near > CUBE_MM + ROTATION_WIDTH_MM + WIDTH_TOL_MM:
                continue  # even a rotated cube cannot account for this depth span
            centre = (left + right) / 2
            # At pickup the beam is offset from the centred gripper. Use the
            # face depth THERE, rather than the nearest corner of a rotated cube.
            beam_x = centre + DIST_DX_MM
            if not left <= beam_x <= right:
                continue
            best = first
            for j in range(first + 1, i):
                if abs(s[j][0] - beam_x) < abs(s[best][0] - beam_x):
                    best = j
            grab_z = s[best][1]
            # The deepest visible point may be a side face of this same cube.
            # Do not add a second full jaw depth behind that point.
            depth = max(grab_z + GRIP_DEPTH_MM, far) + CLEARANCE_DEPTH_MM
            if self.clear_side(first - 1, -1, right, depth) and self.clear_side(i, 1, left, depth):
                return (centre, grab_z)
        return None


def find_cube(r, failed, deadline):
    """Travel left to SEARCH_START_X, then scan left until a target is complete."""
    r.move_x(SEARCH_START_X, SPEED_X_TRAVEL)
    wait(CAL_SETTLE_MS, MSEC)
    search = CubeSearch()
    r.X.start_move(SEARCH_END_X, SPEED_X_SCAN)
    last_x, last_t = r.X.mm(), now_ms()
    velocity = 0.0
    try:
        while not r.X.arrived() and now_ms() < deadline:
            r.X.check()
            x, t = r.X.mm(), now_ms()
            if t > last_t:
                velocity = 0.7 * velocity + 0.3 * (x - last_x) * 1000.0 / (t - last_t)
            last_x, last_t = x, t
            d = distance_mm(r.hw)
            z = None if d is None else r.Z.mm() + d - GRAB_DIST_MM
            target = search.add(x - velocity * DIST_LAG_S + DIST_DX_MM, z)
            live(r, d)
            if target is not None and not any(abs(target[0] - old) < CUBE_MM / 2 for old in failed):
                if X_TRAVEL_MIN <= target[0] <= 0 and target[1] <= Z_TRAVEL_MAX:
                    return target
            wait(LOOP_MS, MSEC)
    finally:
        r.X.halt()
    return None


# 4. PICK, SORT, REPEAT ------------------------------------------------------
def pick_cube(r, target):
    x, z = target
    r.move_x(x, SPEED_X_SCAN)    # return to the measured centre, including sensor offset
    r.grip.open()
    r.Z.move_to(max(0.0, z - APPROACH_STANDOFF_MM), SPEED_Z)
    r.Z.start_move(min(z + CREEP_OVERSHOOT_MM, Z_TRAVEL_MAX), SPEED_Z_CREEP)
    reached = False
    try:
        while True:
            d = distance_mm(r.hw)
            live(r, d)
            if d is not None and d <= GRAB_DIST_MM:
                reached = True
                break
            if r.Z.arrived():
                break
            r.Z.check()
            wait(LOOP_MS, MSEC)
    finally:
        r.Z.halt()
    if reached:
        r.grip.close()
    r.Z.move_to(0.0, SPEED_Z_CARRY)
    d = distance_median(r.hw, 5)
    if reached and d is None:
        raise MotionError("cannot confirm whether cube is held")
    return reached and d is not None and d <= HOLD_DIST_MAX_MM


def identify(r):
    for _ in range(2):
        hues = []
        for _ in range(COLOUR_SAMPLES):
            if r.hw.optical.is_near_object() or r.hw.optical.brightness() >= COLOUR_MIN_BRIGHTNESS:
                hues.append(r.hw.optical.hue())
            wait(LOOP_MS, MSEC)
        colour = classify_hue(hue_median(hues)) if len(hues) > COLOUR_SAMPLES // 2 else "unknown"
        if colour != "unknown":
            return colour
    return "unknown"


def place_cube(r, slot):
    x, z = slot
    r.move_x(x, SPEED_X_CARRY, carrying=True)
    r.Z.move_to(z, SPEED_Z_CARRY)
    r.grip.open()
    r.Z.move_to(0.0, SPEED_Z)
    # Do not count a release if the cube is still in the claw.
    d = distance_median(r.hw, 5)
    if d is not None and d <= HOLD_DIST_MAX_MM:
        raise MotionError("cube still held after release")
    r.grip.close()


class Mission:
    def __init__(self, robot):
        self.r = robot

    def run(self):
        r = self.r
        slots = {"green": GREEN_SLOTS, "red": RED_SLOTS, "blue": BLUE_SLOTS}
        used = {"green": 0, "red": 0, "blue": 0}
        failed = []
        deadline = now_ms() + MISSION_S * 1000
        r.home_x()
        reason = "time reserved for return"
        while now_ms() < deadline - EST_CYCLE_S * 1000:
            say("SEARCH", "moving left")
            target = find_cube(r, failed, deadline - EST_CYCLE_S * 1000)
            if target is None:
                reason = "no accessible cube or search time finished"
                break
            say("PICK", "X %.1f Z %.1f" % target)
            if not pick_cube(r, target):
                failed.append(target[0])
                say("PICK missed", "skipping this position")
                if len(failed) >= MAX_PICK_FAILURES:
                    reason = "pick failure limit"
                    break
                continue
            colour = identify(r)
            print("C,%.1f,%.1f,%s" % (target[0], target[1], colour))
            if colour == "unknown" or used[colour] >= len(slots[colour]):
                r.move_x(HOME_CLEAR_X, SPEED_X_CARRY, carrying=True)
                say("STOP: cube held", "unknown colour or zone full")
                return
            say("RETURN / SORT", colour)
            place_cube(r, slots[colour][used[colour]])
            used[colour] += 1
            print("E,placed %s (%d)" % (colour, used[colour]))
        r.move_x(HOME_CLEAR_X, SPEED_X_TRAVEL)
        say("DONE", reason, "G%d R%d B%d" % (used["green"], used["red"], used["blue"]))


def startup(r):
    missing = r.hw.missing()
    if missing:
        say("MISSING", ", ".join(missing))
        return False
    if not CALIBRATED:
        say("CALIBRATE FIRST", "see docs/calibration.md")
        return False
    if not X_TRAVEL_MIN < SEARCH_END_X < SEARCH_START_X < HOME_CLEAR_X < 0:
        raise MotionError("check X home/search limits")
    if X_MM_PER_DEG <= 0 or Z_MM_PER_DEG <= 0 or SIDE_GAP_MM < 15:
        raise MotionError("check scales and side clearance")
    for slots in (GREEN_SLOTS, RED_SLOTS, BLUE_SLOTS):
        for x, z in slots:
            if not SEARCH_START_X < x < HOME_CLEAR_X or not 0 < z <= Z_TRAVEL_MAX:
                raise MotionError("check sorting slot positions")
    r.hw.optical.set_light(100)
    say("READY", "arm in, gripper closed", "Check or LED = start")
    return True


# 5. BENCH CALIBRATION (excluded from the mission build) ---------------------


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
        if ax is r.X and want > 0 and r.hw.home.pressing():
            want = 0
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
                lines.append("d=%s hue=%.0f home=%d" % (distance_mm(r.hw), r.hw.optical.hue(),
                                                       int(r.hw.home.pressing())))
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
    r.move_x(x_from, SPEED_X_TRAVEL)
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
    r.X.halt()
    if best is None:
        return None
    v = abs(x_to - x_from) * 1000.0 / max(1, now_ms() - t0)
    return (best[0] + best[1]) / 2, abs(best[1] - best[0]), v


def cal_cube(r):
    """Teach the grab pose on a real cube, then sweep it both ways.
    Needs X and Z scale done first."""
    cal_home(r)
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
    lo = max(X_TRAVEL_MIN, x_align - CAL_CUBE_SWEEP_MM)
    hi = min(HOME_CLEAR_X, x_align + CAL_CUBE_SWEEP_MM)
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
    cal_home(r)
    r.move_x(SEARCH_START_X, SPEED_X_TRAVEL)
    hw = r.hw
    say("RATES", "arm in, X clear", "for %.0f mm" % RATE_SWEEP_MM, "Check = start")
    wait_press(hw)
    n = 200
    cost = []
    for name, f in (("distance", lambda: hw.distance.object_distance(MM)), ("hue", hw.optical.hue),
                    ("encoder", lambda: hw.mx.position(DEGREES)),
                    ("scan loop", lambda: (r.X.mm(), distance_mm(hw), hw.home.pressing()))):
        t0 = now_ms()
        for _ in range(n):
            f()
        cost.append((now_ms() - t0) / n)
        print("# RATE call %s: %.2f ms" % (name, cost[-1]))

    last = [None, None, None]
    times = [[], [], []]
    x0 = r.X.mm()
    t1 = t2 = 0
    for x_to in (x0 - RATE_SWEEP_MM, x0):
        r.X.start_move(x_to, SPEED_X_SCAN)
        while not r.X.arrived():
            r.X.check()
            t = now_ms()
            x = r.X.mm()
            if x_to < x0:       # steady speed between 1/4 and 3/4 of the way out
                if not t1 and x <= x0 - RATE_SWEEP_MM / 4:
                    t1 = t
                if not t2 and x <= x0 - RATE_SWEEP_MM * 3 / 4:
                    t2 = t
            vals = (hw.distance.object_distance(MM), (hw.optical.hue(), hw.optical.brightness()), hw.mx.position(DEGREES))
            for i in range(3):
                if vals[i] != last[i]:
                    last[i] = vals[i]
                    if len(times[i]) < RATE_MAX_SAMPLES:
                        times[i].append(t)
            wait(RATE_POLL_MS, MSEC)
        r.X.halt()
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
    v_max = min(RATE_EDGE_TOL_MM / err_per_v, 1000.0 * SCAN_MAX_SPACING_MM / td)
    pct = max(1, min(100, int(SPEED_X_SCAN * v_max / v)))
    print("# now: %d%% = %.0f mm/s, edge error +-%.1f mm" % (SPEED_X_SCAN, v, v * err_per_v))
    print("SPEED_X_SCAN = %d    # cal_rates: %.0f mm/s, edge error +-%.1f mm" % (pct, v_max, RATE_EDGE_TOL_MM))
    print("LOOP_MS = %d    # cal_rates: distance updates every %.0f ms" % (loop, td))
    print("# colour: optical updates every %.0f ms, %d samples need >= %.0f ms to be independent"
          % (to, COLOUR_SAMPLES, COLOUR_SAMPLES * to))
    say("RATES dist %.0f ms" % td, "opt %.0f enc %.0f ms" % (to, tx),
        "scan %d%% %.0f mm/s" % (pct, v_max), "LOOP_MS %d" % loop)
    wait_press(hw)


def cal_home(r):
    say("HOME TEST", "arm in, path clear", "Check = home right")
    wait_press(r.hw)
    r.home_x()


def cal_x_scale(r):
    say("X SCALE", "arm in, mark carriage", "jog LEFT, then Check")
    before = r.X.deg()
    if not jog(r, "X scale (left)", [r.X]):
        return
    change = r.X.deg() - before
    if abs(change) < 100:
        say("X SCALE", "move further and retry")
        return
    print("X_MM_PER_DEG = <measured_mm> / %.1f" % abs(change))
    say("Measure between marks", "Check = return")
    wait_press(r.hw)
    cal_move_deg(r, r.X, before, SPEED_X_SCAN)


def cal_teach(r):
    cal_home(r)
    while jog(r, "TEACH home-relative", [r.X, r.Z, r.grip]):
        print("TEACH X=%.1f Z=%.1f grip=%.0f" % (r.X.mm(), r.Z.mm(), r.grip.deg()))


CAL_ROUTINES = (("Home", cal_home), ("Z scale", cal_z_scale), ("X scale", cal_x_scale),
                ("Grip", cal_grip), ("Cube pose", cal_cube), ("Colour", cal_colour),
                ("Teach pts", cal_teach), ("Distance", cal_distance), ("Rates", cal_rates))


def calibration_menu(r):
    r.hw.optical.set_light(100)
    i = 0
    while True:
        say("CALIBRATE", "< %s >" % CAL_ROUTINES[i][0], "L/R choose, Check run")
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
                say("STOP", str(e))
                wait_press(r.hw)


def test_mode(r):
    r.hw.optical.set_light(100)
    while True:
        jog(r, "TEST (relative X)", [r.X, r.Z, r.grip], dashboard=True)


def motor_check(r):
    say("MOTOR CHECK", "clear space both ways", "Check = start")
    wait_press(r.hw)
    # X starts with a LEFT move, away from the home end.
    for name, m, sign in (("X", r.hw.mx, -1), ("Z", r.hw.mz, 1), ("grip", r.hw.mg, -1)):
        before = m.position(DEGREES)
        m.spin(FORWARD, 30 * sign, PERCENT)
        wait(400, MSEC)
        m.stop()
        after = m.position(DEGREES)
        m.set_timeout(2000, MSEC)
        m.spin_to_position(before, DEGREES, 30, PERCENT, True)
        print("CHECK %s: moved %.0f deg" % (name, abs(after - before)))
    wait_press(r.hw)


# 6. ENTRY ------------------------------------------------------------------
def select_mode(r):
    default = MODE if MODE in BUILD_MODES else BUILD_MODES[0]
    say("START", "Left=cal, Right=test")
    t0 = now_ms()
    while now_ms() - t0 < MODE_SELECT_MS:
        if brain.buttonLeft.pressing():
            return "CALIBRATE" if "CALIBRATE" in BUILD_MODES else default
        if brain.buttonRight.pressing():
            return "TEST" if "TEST" in BUILD_MODES else default
        wait(20, MSEC)
    return default


def main():
    r = Robot()  # Z/grip start in their known pose; X is not referenced yet.
    try:
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
            while wait_press(r.hw) not in ("C", "T"):
                pass
            Mission(r).run()
    except MotionError as e:
        say("STOP", str(e))
    finally:
        r.stop_all()


if __name__ == "__main__":
    main()
