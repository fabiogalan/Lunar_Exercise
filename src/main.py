"""Lunar cube sorter - the complete program flashed to the VEX IQ2 Brain.

Before starting: retract Z fully and close the gripper. Check all MANUAL VALUES.
X=0 is established by driving right until the bumper on port 6 is pressed.

COORDINATES (read this first)
  Field X = millimetres from the bumper, NEGATIVE to the left. The bumper is X = 0
  (right end of travel); the mining area is far left (most negative X).
  From the bumper going left:  STORAGE | DISPOSAL | MINING.

  A tool on the trolley sits LEFT of the trolley reference:
    claw centre   = trolleyX - CLAW_X_OFFSET     -> to put the claw on field x, move to x + CLAW_X_OFFSET
    distance beam = trolleyX - SENSOR_X_OFFSET    -> a reading at trolleyX is at field trolleyX - SENSOR_X_OFFSET
"""
from vex import *


# MANUAL VALUES --------------------------------------------------------------
# Set True only after replacing the measured placeholders below.
MANUAL_VALUES_SET = True

# Ports currently used on the robot.
PORT_Z = Ports.PORT1
PORT_BUMPER = Ports.PORT6
PORT_TOUCH = Ports.PORT3       # optional start button
PORT_X = Ports.PORT5
PORT_OPTICAL = Ports.PORT7
PORT_DISTANCE = Ports.PORT8
PORT_GRIP = Ports.PORT9

X_MM_PER_DEG = 0.3245       # 01.10 measured; positive motor motion moves the trolley right (toward the bumper)
Z_MM_PER_DEG = 0.3245       # 01.10: Z moved 83 mm in 255 deg

CLAW_X_OFFSET = 73.0        # bumper -> claw centre (where the cube centre goes)
MACHINE_LENGTH = 228.0      # at max left, the bumper (right end of the machine) is 228 mm from the left wall

# Area lengths in mm. Final competition uses the _COMP set (bigger field).
X_STORAGE_TEST = 305.0
X_STORAGE_COMP = 610.0
X_DISPOSAL_TEST = 305.0
X_DISPOSAL_COMP = 305.0
X_MINING_TEST = 610.0
X_MINING_COMP = 1830.0

COMPETITION = False         # True on competition day: uses the _COMP lengths
X_STORAGE = X_STORAGE_COMP if COMPETITION else X_STORAGE_TEST
X_DISPOSAL = X_DISPOSAL_COMP if COMPETITION else X_DISPOSAL_TEST
X_MINING = X_MINING_COMP if COMPETITION else X_MINING_TEST

# Field areas as (far/left edge, near/right edge) in field X, from the bumper outward.
STORAGE_AREA = (-X_STORAGE, 0.0)
DISPOSAL_AREA = (STORAGE_AREA[0] - X_DISPOSAL, STORAGE_AREA[0])
MINING_AREA = (DISPOSAL_AREA[0] - X_MINING, DISPOSAL_AREA[0])

Z_MAX = 400.0
HOME_CLEAR_X = -10.0        # back 10 mm left off the bumper after homing
SEARCH_Z = -10.0           # !to measure! Z held here (slightly retracted) while scanning so the arm clears
                           # cubes that sit closer to the lanes; 0 is the collect/push reference
X_TRAVEL_MIN = -3000.0     # generous left limit until the far wall is found by stall

WALL_Z = 325.0             # grab-Z depth of the empty floor (01.10: empty floor reads ~345 - GRAB_DISTANCE)  !!TODO measure!!

GRAB_MARGIN = 7.0          # sensor-to-cube reading when grabbing (noise + creep overshoot; 5 pushed the cube, 01.10)
GRAB_DISTANCE = GRAB_MARGIN                  # reading at which we close the gripper
HOLD_DISTANCE = 2 * GRAB_MARGIN              # largest reading that still means "a cube is in the claw"
SENSOR_CLAW_OFFSET = 10.0   # claw centre -> distance beam (beam left of the claw centre; was 17 until 01.10)
SENSOR_X_OFFSET = CLAW_X_OFFSET + SENSOR_CLAW_OFFSET    # bumper -> beam = 83
GRIP_OPEN_DEG = 90          # 90 grabbed well (01.10)
GRIP_CLOSED_DEG = 0

# Scan: the beam starts SCAN_START_MARGIN before the mining edge (so a cube on the
# edge shows its right side); the far end is the wall found at runtime.
SCAN_START_MARGIN = 50.0
SEARCH_START_X = (MINING_AREA[1] + SCAN_START_MARGIN) + SENSOR_X_OFFSET   # trolley X that puts the beam there

CUBE_SIZE = 75.0
CUBE_MARGIN = 10.0          # spare space for grabbing and pushing

# Sorting lanes (claw-centre field X). Red and green are two piles in the storage (production) area;
# blue is a single constant spot in the disposal area that each new blue cube just pushes outward.
#   red   starts at the near (right) edge and its lines march toward the centre (-X);
#   green starts at the far (left) edge and its lines march toward the centre (+X).
# Each colour fills a line from the BACK (deep Z) first; after CUBES_PER_LINE the next line steps X.
# The pose for cube n of a colour is computed from how many of that colour are already stored.
PRODUCTION_AREA = STORAGE_AREA               # where the red and green piles go
EDGE_MARGIN = 5.0                            # gap from the production edge to the first pile
CLAW_WIDTH = 20.0           # !to measure! one claw's width
WIDTH_BETWEEN_CLAWS = 90.0  # full opening between the two claws (the cube centre sits here); confirm on the build
                            # red and green pile centres must stay > WIDTH_BETWEEN_CLAWS + CUBE_MARGIN apart
CUBES_PER_LINE = 4          # cubes stacked in depth per line before stepping X
LANE_STEP_X = CUBE_SIZE + CUBE_MARGIN        # gap between lines
LANE_STEP_Z = CUBE_SIZE                      # cubes in a line sit back-to-back
Z_BACK = Z_MAX - CUBE_SIZE - CUBE_MARGIN     # deepest placement (each line is filled from the back)

# The first pile is one CLAW_X_OFFSET inside the edge so the claw can reach it and push the cube to the edge.
DROP_X = {"red": PRODUCTION_AREA[1] - EDGE_MARGIN - CLAW_X_OFFSET,
          "green": PRODUCTION_AREA[0] + EDGE_MARGIN + CLAW_X_OFFSET,
          "blue": (DISPOSAL_AREA[0] + DISPOSAL_AREA[1]) / 2}
LANE_DIR = {"red": -1, "green": 1, "blue": 0}   # X direction the lines march (toward the open centre)
DROP_Z = CUBE_SIZE + CUBE_MARGIN             # shallowest release depth (one cube + margin from the chassis)
HUES = {"red": (330.0, 25.0), "green": (70.0, 170.0), "blue": (180.0, 270.0)}

# Detection. The scan is cut into segments of similar depth, one reading per
# SAMPLE_STEP mm, keeping only the last SMOOTH_COUNT depths (memory). A segment is
# a cube when it is CUBE_WIDTH_MIN..MAX long, nearer than the floor, and the space
# just past its far (left) edge is open for SIDE_CLEARANCE mm.
SAMPLE_STEP = 2.5
SMOOTH_COUNT = 5
JUMP_MM = 7.5               # new segment when a reading is this far from the segment average
                           # (7.5 kept a real row of 3 together, 5 split it: 01.10 logs; face noise +-2)
CUBE_WIDTH_MIN = CUBE_SIZE - CUBE_MARGIN     # 65: a cube's flat face (sloped edges fall into short side segments)
CUBE_WIDTH_MAX = 90.0 + CUBE_MARGIN          # 100
SIDE_CLEARANCE = CUBE_MARGIN                 # clear space required past the far edge so the side arm fits

POSITION_TOLERANCE = 3.0    # mm a finished motor may be off target before it counts as blocked
LOOP_MS = 15                # loop time for all motion and sensor checks

# Speeds in percent.
HOME_SPEED = 20
SCAN_SPEED = 10             # ~2.5 mm between readings (01.10: the scan loop takes ~90 ms)
TRAVEL_SPEED = 80
CARRY_SPEED = 50
Z_SPEED = 40
Z_CREEP_SPEED = 20


# HARDWARE AND BASIC MOVEMENT ------------------------------------------------
brain = Brain()


class RobotError(Exception):
    pass


def now():
    return brain.timer.time(MSEC)


def show(*lines):
    brain.screen.clear_screen()
    for row, line in enumerate(lines):
        brain.screen.set_cursor(row + 1, 1)
        brain.screen.print(line)
    print(" | ".join(lines))


def wait_for_start(touch):
    show("READY", "Z in, gripper closed", "Check or LED to start")
    while not brain.buttonCheck.pressing() and not (touch.installed() and touch.pressing()):
        wait(20, MSEC)
    while brain.buttonCheck.pressing() or (touch.installed() and touch.pressing()):
        wait(20, MSEC)


def area(field_x):
    if field_x <= MINING_AREA[1]:
        return "MINE"
    if field_x <= DISPOSAL_AREA[1]:
        return "DISPOSE"
    if field_x <= 0:
        return "STORE"
    return "OUT"


class Axis:
    def __init__(self, name, motor, mm_per_degree, minimum, maximum):
        self.name = name
        self.motor = motor
        self.scale = mm_per_degree
        self.minimum = minimum
        self.maximum = maximum
        self.target = 0.0
        self.started = 0
        self.timeout = 0
        motor.set_stopping(HOLD)
        motor.set_position(0, DEGREES)

    def mm(self):
        return self.motor.position(DEGREES) * self.scale

    def start(self, target, speed):
        if target < self.minimum or target > self.maximum:
            raise RobotError(self.name + " target outside travel")
        self.target = target
        self.started = now()
        # twice the ideal time at 720 deg/s x speed %, plus 1.5 s
        self.timeout = abs(target - self.mm()) * 2000.0 / (abs(self.scale) * 7.2 * speed) + 1500
        self.motor.set_timeout(self.timeout + 500, MSEC)
        self.motor.spin_to_position(target / self.scale, DEGREES, speed, PERCENT, False)

    def arrived(self):
        # At low speed the motor may report done a few degrees short (01.10: X 1.9 mm short at speed 10).
        error = abs(self.mm() - self.target)
        return error <= 1.0 or (error <= POSITION_TOLERANCE and self.motor.is_done())

    def check(self):
        if now() - self.started > self.timeout:
            raise RobotError(self.name + " motion timeout")
        if now() - self.started > 150 and self.motor.is_done() and not self.arrived():
            raise RobotError(self.name + " motion blocked")

    def move(self, target, speed, tick=None):
        # The one motion loop: tick() runs every loop; a truthy result stops the move and is returned.
        self.start(target, speed)
        try:
            while not self.arrived():
                self.check()
                result = tick and tick()
                if result:
                    return result
                wait(LOOP_MS, MSEC)
        finally:
            self.motor.stop()


class Robot:
    def __init__(self):
        self.bumper = Bumper(PORT_BUMPER)
        self.touch = Touchled(PORT_TOUCH)
        self.distance = Distance(PORT_DISTANCE)
        self.optical = Optical(PORT_OPTICAL)
        self.x_motor = Motor(PORT_X)
        self.z_motor = Motor(PORT_Z, True)
        self.grip_motor = Motor(PORT_GRIP)
        self.x = Axis("X", self.x_motor, X_MM_PER_DEG, X_TRAVEL_MIN, 0.0)
        self.z = Axis("Z", self.z_motor, Z_MM_PER_DEG, SEARCH_Z, Z_MAX)
        self.grip_motor.set_stopping(HOLD)
        self.grip_motor.set_position(0, DEGREES)
        self.homed = False
        self.x_wall = X_TRAVEL_MIN
        self.last = -1

    def stop(self):
        self.x_motor.stop()
        self.z_motor.stop()
        self.grip_motor.stop()

    def read_distance(self):
        value = self.distance.object_distance(MM)
        return value if 0 < value <= 1000 else None

    def report(self, distance):
        # area,claw X,beam X,Z,distance (field X), printed whenever the reading changes
        if distance != self.last:
            self.last = distance
            x = self.x.mm()
            print("%s,%.1f,%.1f,%.0f,%s" % (area(x - SENSOR_X_OFFSET), x - CLAW_X_OFFSET,
                                            x - SENSOR_X_OFFSET, self.z.mm(),
                                            "-" if distance is None else "%.0f" % distance))

    def home(self):
        """Drive right to the bumper, define X=0, then back away."""
        if self.z.mm() > 2:
            raise RobotError("retract Z before homing")
        show("HOMING X", "moving right")
        self.homed = False
        start = now()
        self.x_motor.spin(FORWARD, HOME_SPEED, PERCENT)
        try:
            while not self.bumper.pressing():
                if now() - start > 120000:
                    raise RobotError("bumper not reached")
                wait(LOOP_MS, MSEC)
        finally:
            self.x_motor.stop()
        wait(60, MSEC)
        self.x_motor.set_position(0, DEGREES)
        self.x.move(HOME_CLEAR_X, HOME_SPEED)
        if self.bumper.pressing():
            raise RobotError("bumper did not release")
        self.homed = True

    def hold_firm(self):
        # keep a firm grip (and resist Z sag) while a cube is carried  # !to measure! torque %
        self.grip_motor.set_max_torque(100, PERCENT)

    def move_x(self, target, speed, carrying=False):
        if not self.homed or self.z.mm() > 2:
            raise RobotError("home X and retract Z first")
        lost = [0]
        regripped = [False]

        def tick():
            distance = self.read_distance()
            self.report(distance)
            if carrying:
                lost[0] = lost[0] + 1 if distance is None or distance > HOLD_DISTANCE else 0
                if lost[0] >= 3:
                    return "lost"
        while True:
            if self.x.move(target, speed, tick) != "lost":
                return True
            if regripped[0]:
                return False                       # still gone after one re-grip: caller decides
            regripped[0] = True
            self.grip(GRIP_CLOSED_DEG, 50)         # re-close once and keep going
            lost[0] = 0

    def grip(self, degrees, speed):
        self.grip_motor.set_timeout(5000, MSEC)
        self.grip_motor.spin_to_position(degrees, DEGREES, speed, PERCENT, True)
        if abs(self.grip_motor.position(DEGREES) - degrees) > 30:
            raise RobotError("gripper blocked")


# CUBE SEARCH ---------------------------------------------------------------
# The detector is compiled on its own when the program starts (exec), not together with the rest of
# the file: the Brain runs out of memory compiling everything at once. Edit it like normal code, but
# avoid triple quotes and backslashes inside.
exec('''
class CubeDetector:
    # One reading per SAMPLE_STEP mm. A segment grows while readings stay within JUMP_MM of its average
    # (z_mean of the last SMOOTH_COUNT). When a segment ends into DEEPER space it may be a cube; we then
    # confirm the far (left, more negative) side is open for SIDE_CLEARANCE mm before returning it.
    def __init__(self):
        self.last_x = None
        self.recent = []
        self.start = self.end = self.nearest = None
        self.pending = None          # (z_mean, start, end, nearest) awaiting left-clearance confirmation

    def z_mean(self):
        return sum(self.recent) / len(self.recent)

    def new_segment(self, x, z):
        self.recent, self.start, self.end, self.nearest = [z], x, x, z

    def is_cube(self, mean):
        return (mean < WALL_Z - 5 and self.start is not None
                and CUBE_WIDTH_MIN <= self.start - self.end <= CUBE_WIDTH_MAX)

    def add(self, x, z):
        if self.last_x is not None and self.last_x - x < SAMPLE_STEP:
            return None
        self.last_x = x
        z = 9999 if z is None else z

        if self.pending is not None:
            mean, pstart, pend, pnear = self.pending
            if z <= mean + 1:                      # as near as the cube -> no gap, discard
                self.pending = None
                self.new_segment(x, z)
                return None
            if x <= pend - SIDE_CLEARANCE:          # open space past the far edge: a real cube
                self.pending = None
                return (pstart + pend) / 2, pnear
            return None

        if self.recent:
            mean = self.z_mean()
            if abs(z - mean) <= JUMP_MM:
                self.recent = (self.recent + [z])[-SMOOTH_COUNT:]
                self.end = x
                self.nearest = min(self.nearest, z)
                return None
            if self.is_cube(mean) and z > mean:     # ended into a deeper gap -> confirm the far side
                self.pending = (mean, self.start, self.end, self.nearest)
                self.recent = []
                return None
        self.new_segment(x, z)
        return None

    def finish(self):
        # The scan ended at the wall. Accept the open segment if it is nearer than the floor, even if it is
        # too short (no width or clearance check): the beam cannot reach the last bit before the wall.
        # !TODO measure! the gap from the beam at the leftmost reachable X to the wall must be < one cube.
        if self.recent and self.z_mean() < WALL_Z - 5 and self.start is not None:
            return (self.start + self.end) / 2, self.nearest
        if self.pending is not None:
            mean, pstart, pend, pnear = self.pending
            return (pstart + pend) / 2, pnear
        return None
''')


def find_cube(robot, failed):
    robot.grip(GRIP_OPEN_DEG, 50)       # closed side arms sit in front of the sensor (reads ~90 everywhere, 01.10)
    robot.move_x(SEARCH_START_X, TRAVEL_SPEED)
    robot.z.move(SEARCH_Z, Z_SPEED)     # slight retract so the arm clears cubes near the lanes
    wait(300, MSEC)
    detector = CubeDetector()

    def skip(target):
        return bool(target) and any(abs(target[0] - position) < CUBE_SIZE / 2 for position in failed)

    # Scan left while reading the sensor. No separate wall calibration: when the trolley stops advancing
    # it has reached the far wall, so stop and judge the last segment (detector.finish, z_mean only), then
    # the caller goes back.
    robot.x_motor.spin(REVERSE, SCAN_SPEED, PERCENT)
    start = now()
    checkpoint = robot.x.mm()
    loops = 0
    try:
        while now() - start < 120000:
            wait(LOOP_MS, MSEC)
            x = robot.x.mm()
            distance = robot.read_distance()
            robot.report(distance)
            z = None if distance is None else robot.z.mm() + distance - GRAB_DISTANCE   # grab Z in the Z=0 frame
            target = detector.add(x - SENSOR_X_OFFSET, z)                               # field X of the beam
            if target and not skip(target):
                return target
            loops += 1
            if loops >= 20:                     # every ~0.3 s: moved < 2 mm -> stalled on the far wall
                if abs(x - checkpoint) < 2.0:
                    robot.x_wall = x
                    robot.x.minimum = x
                    break
                checkpoint, loops = x, 0
    finally:
        robot.x_motor.stop()
    target = detector.finish()
    return None if skip(target) else target


# PICK AND SORT --------------------------------------------------------------
def held_distance(robot):
    values = []
    for _ in range(5):
        values.append(robot.read_distance() or 9999)     # no reading counts as far away
        wait(LOOP_MS, MSEC)
    return sorted(values)[2]                              # median of 5


def pick(robot, target):
    x, z = target                       # field X of the cube centre, grab Z
    robot.move_x(x + CLAW_X_OFFSET, SCAN_SPEED)
    robot.grip(GRIP_OPEN_DEG, 20)

    def touching():
        distance = robot.read_distance()
        return distance is not None and distance <= GRAB_DISTANCE
    # The fast part also watches the sensor: Z_MM_PER_DEG may be off, so the estimate z may be too far.
    reached = robot.z.move(max(0, z - 30), Z_SPEED, touching) or \
        robot.z.move(min(z + 20, Z_MAX), Z_CREEP_SPEED, touching)
    print("GRAB,deg,reading", robot.z_motor.position(DEGREES), robot.read_distance())
    if reached:
        robot.grip(GRIP_CLOSED_DEG, 50)
        robot.hold_firm()
    robot.z.move(0, CARRY_SPEED)
    distance = held_distance(robot)
    return reached and distance is not None and distance <= HOLD_DISTANCE


def colour(robot):
    hues = []
    for _ in range(7):
        if robot.optical.is_near_object() or robot.optical.brightness() >= 5:
            hues.append(robot.optical.hue())
        wait(LOOP_MS, MSEC)
    if len(hues) < 4:
        return "unknown"
    if max(hues) - min(hues) > 180:
        hues = [h + 360 if h < 180 else h for h in hues]
    hue = sorted(hues)[len(hues) // 2] % 360
    for name in ("red", "green", "blue"):
        low, high = HUES[name]
        if (low <= hue <= high) if low <= high else (hue >= low or hue <= high):
            return name
    return "unknown"


def place(robot, x, z):
    if not robot.move_x(x + CLAW_X_OFFSET, CARRY_SPEED, True):
        return False                                # cube lost on the way (already re-gripped once)
    robot.z.move(z, CARRY_SPEED)
    robot.grip(GRIP_OPEN_DEG, 20)
    robot.z.move(0, Z_SPEED)
    if (robot.read_distance() or 9999) <= HOLD_DISTANCE:     # still something in the claw
        raise RobotError("cube not released")
    robot.grip(GRIP_CLOSED_DEG, 50)
    return True


def run(robot):
    used = {"green": 0, "red": 0, "blue": 0}
    drop = {name: (DROP_X[name], Z_BACK) for name in used}   # current (claw X, Z) for the next cube of each colour
    failed = []
    end_time = now() + 600000
    robot.home()
    print("area,claw X,beam X,Z,distance", STORAGE_AREA, DISPOSAL_AREA, MINING_AREA)
    while now() < end_time:              # work till the very end of the match
        show("SEARCHING")
        target = find_cube(robot, failed)
        if target is None:
            break
        show("PICK", "X %.0f Z %.0f" % target)
        if not pick(robot, target):
            failed.append(target[0])                # record and skip this spot; keep going
            continue
        name = colour(robot)
        if name == "unknown":
            robot.move_x(HOME_CLEAR_X, CARRY_SPEED, True)
            show("STOP: cube held", name)
            return
        x, z = drop[name]
        if not place(robot, x, z):
            show("lost on the way", name)           # cube dropped in transit; back to searching
            continue
        used[name] += 1
        # Advance this colour's drop pose for the next cube: back->front within a line of CUBES_PER_LINE,
        # then step X to the next line. If red and green would collide, dump this colour at the blue spot.
        if drop[name] != drop["blue"] and LANE_DIR[name] != 0:
            if used[name] % CUBES_PER_LINE == 0:    # line finished -> next line, back of the lane
                drop[name] = (x + LANE_DIR[name] * LANE_STEP_X, Z_BACK)
                if abs(drop["red"][0] - drop["green"][0]) < WIDTH_BETWEEN_CLAWS + CUBE_MARGIN:
                    drop[name] = drop["blue"]        # no correct spot left: dump this colour with blue
            else:
                drop[name] = (x, z - LANE_STEP_Z)   # same line, next cube toward the front
        # !TODO! push_pair(robot, name) after every two cubes once the core run works (see bottom)
    robot.move_x(HOME_CLEAR_X, TRAVEL_SPEED)
    show("DONE", "G%d R%d B%d" % (used["green"], used["red"], used["blue"]))


def main():
    robot = None
    try:
        robot = Robot()
        for name in ("bumper", "distance", "optical", "x_motor", "z_motor", "grip_motor"):
            if not getattr(robot, name).installed():
                raise RobotError("missing: " + name)
        if not MANUAL_VALUES_SET:
            raise RobotError("enter manual values, then set MANUAL_VALUES_SET=True")
        robot.optical.set_light(100)
        wait_for_start(robot.touch)
        run(robot)
    except Exception as error:
        if robot:
            robot.stop()
        message = str(error)
        show("ERROR", message[:22], message[22:44], message[44:66])
        while not brain.buttonCheck.pressing():
            wait(20, MSEC)
    finally:
        if robot:
            robot.stop()


if __name__ == "__main__":
    main()  # comment out for testing single functions / movements


# EDGE-PUSHING PACKING (add-on, not wired in yet) ----------------------------
# Integrate once the core run works: call push_pair(robot, name) from run() after every two cubes of a
# colour. Keep it separate so a bug here cannot break collecting/sorting.
#
# Idea (per colour lane, filling each line from the back wall outward):
#   - The first cube of a line is set down at the line's near slot (placed, not pushed).
#   - Every following pair is pushed with the long side arm toward that colour's wall edge:
#       green -> toward the left/outer edge, red -> toward the right/outer edge.
#   - Fill the back line first and spread to the edges, then start the next line toward the centre.
#   - Stop when no room is left, decided by hardcoded cube-width slots (far cubes are hard to sense).
#   - Track per colour: cubes placed, current line, last X; never mix colours in a lane; keep >= CUBE_MARGIN
#     between cubes and >= 35 mm clear of the rails; never push past WALL_Z (cubes must not fall off).
#
# def push_pair(robot, name):
#     ...
