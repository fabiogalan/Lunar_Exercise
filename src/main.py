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
Z_MM_PER_DEG = 0.3245       # 01.10: Z moved 83 mm in 255 deg #TODO

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
CALIBRATION_SCAN = False   # True: scan the whole mining area once, print FOUND lines, pick nothing
X_STORAGE = X_STORAGE_COMP if COMPETITION else X_STORAGE_TEST
X_DISPOSAL = X_DISPOSAL_COMP if COMPETITION else X_DISPOSAL_TEST
X_MINING = X_MINING_COMP if COMPETITION else X_MINING_TEST

# Field areas as (far/left edge, near/right edge) in field X, from the bumper outward.
STORAGE_AREA = (-X_STORAGE, 0.0)
DISPOSAL_AREA = (STORAGE_AREA[0] - X_DISPOSAL, STORAGE_AREA[0])
MINING_AREA = (DISPOSAL_AREA[0] - X_MINING, DISPOSAL_AREA[0])

Z_MAX = 305                 # 380 - CUBE_SIZE for furthest claw position 
HOME_CLEAR_X = -10.0        # back 10 mm left off the bumper after homing
REHOME_X = -60.0            # after every delivered cube: drive here fast, then re-home on the bumper (X drifts, 03.10)
                            # more than 30 mm left, so X drift cannot run the fast move into the bumper
SEARCH_Z = -12.0           # !to measure! Z held here (slightly retracted) while scanning so the arm clears
                           # cubes that sit closer to the lanes; 0 is the collect/push reference
X_TRAVEL_MIN = -3000.0     # generous left limit until the far wall is found by stall

WALL_Z = 380             # grab-Z depth of the empty floor (01.10: empty floor reads ~345 - GRAB_DISTANCE)  !!TODO measure!!

GRAB_MARGIN = 7.0          # sensor-to-cube reading when grabbing (noise + creep overshoot; 5 pushed the cube, 01.10)
GRAB_DISTANCE = GRAB_MARGIN                  # reading at which we close the gripper
HOLD_DISTANCE = 2 * GRAB_MARGIN              # largest reading that still means "a cube is in the claw"
SENSOR_CLAW_OFFSET = 10.0   # claw centre -> distance beam (beam left of the claw centre; was 17 until 01.10)
SENSOR_X_OFFSET = CLAW_X_OFFSET + SENSOR_CLAW_OFFSET    # bumper -> beam = 83
GRIP_OPEN_DEG = 90          # 90 grabbed well (01.10)
GRIP_CLOSED_DEG = 0
GRAB_SEAT = 5.0        #mm to push cube further in after touched to make it straighter
PICK_TRIES = 2              # failed picks at one spot before it is skipped for the rest of the run
PUSH_MM = 5.0               # Z moved in while the reading stopped shrinking = the claw is pushing the cube: grab
                            # (03.10: reading stuck at 20 while Z went on, the cube slid back 40 mm and was not grabbed)

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
Z_BACK = Z_MAX - CUBE_MARGIN     # deepest placement (each line is filled from the back)

# The first pile is one CLAW_X_OFFSET inside the edge so the claw can reach it and push the cube to the edge.
DROP_X = {"red": PRODUCTION_AREA[1] - CLAW_X_OFFSET,  # get rid of margin here EDGE_MARGIN
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
SMOOTH_COUNT = 5            # not used any more: the segment mean is over ALL its readings (sum / count, 03.10)
JUMP_MM = 10.0              # new segment when a reading is this far from the segment average
                           # (7.5 kept a real row of 3 together, 5 split it: 01.10 logs; face noise +-2)
                           # 03.10: now the offset of z_jump_mm = JUMP_MM + JUMP_PER_MM * z (face noise ~8-12 at any z)
JUMP_PER_MM = 0.02          # allowed jump = JUMP_PER_MM * z (03.10 logs: face noise 2.3 at z 36 ... 8.2 at z 213) FITTED CONSTAND FROM EXPERIMENT ON 3.10
                           # 03.10 staircase (20mm_x_diff_test): 0.10 alone chopped near cubes; offset 10 + 0.02 with the
                           # fixed mean found all 16 cubes in 7 logs (the last-5 mean followed the ramp between cubes 4 and 5)
CUBE_WIDTH_MIN = CUBE_SIZE - CUBE_MARGIN     # 65: a cube's flat face (sloped edges fall into short side segments)
CUBE_WIDTH_MAX = 90.0 + CUBE_MARGIN          # 100
SIDE_CLEARANCE = CUBE_MARGIN                 # clear space required past the far edge so the side arm fits
CUBE_WIDTH_MIN = 40.0       # replaces 65 above. 03.10, all 7 logs: fragments <= 22 mm, single cubes 56..144
CUBE_WIDTH_MAX = 175.0      # replaces 100 above. Wider = a row (2+ cubes merged, 214 mm and more)
ROW_FIRST_OFFSET = 60.0     # a row's first (right) cube centre = segment start - this (half a far single cube)

POSITION_TOLERANCE = 3.0    # mm a finished motor may be off target before it counts as blocked
LOOP_MS = 15                # loop time for all motion and sensor checks

# Speeds in percent. #TODO optimise 
HOME_SPEED = 80
SCAN_SPEED = 20             # ~2.5 mm between readings (01.10: the scan loop takes ~90 ms)
                            # 03.10: 10 gave ~0.9 mm between readings; replays at 2x-4x still found every cube
TRAVEL_SPEED = 80
CARRY_SPEED = 80
Z_SPEED = 60
Z_CREEP_SPEED = 40


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

    def report(self, distance, mean=None):
        # area,claw X,beam X,Z,distance,z mean (field X), printed whenever the reading changes
        # z mean = the detector's segment mean (grab Z), "-" outside the scan or while confirming a cube
        if distance != self.last:
            self.last = distance
            x = self.x.mm()
            print("%s,%.1f,%.1f,%.0f,%s,%s" % (area(x - SENSOR_X_OFFSET), x - CLAW_X_OFFSET,
                                               x - SENSOR_X_OFFSET, self.z.mm(),
                                               "-" if distance is None else "%.0f" % distance,
                                               "-" if mean is None else "%.1f" % mean))

    def home(self):
        """Drive right to the bumper, define X=0, then back away."""
        if self.z.mm() > 5:
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
        # Opening eases into 90 deg and ran into the 5 s timeout (the pause after homing, 03.10): 1.5 s is plenty.
        # Closing keeps the full 5 s squeeze; that is what holds the cube.
        self.grip_motor.set_timeout(5000 if degrees == GRIP_CLOSED_DEG else 1500, MSEC)
        self.grip_motor.spin_to_position(degrees, DEGREES, speed, PERCENT, True)
        if abs(self.grip_motor.position(DEGREES) - degrees) > 30:     # e.g. a neighbouring cube: keep going (03.10)
            print("GRIP-SHORT,%.0f,target %d" % (self.grip_motor.position(DEGREES), degrees))


# CUBE SEARCH ---------------------------------------------------------------
# tools/ship.py strips comments and compiles each function and class on its own for the Brain (memory),
# so comments here cost nothing on the Brain.
#   is_cube: 0 no cube, 1 one cube, 2 a row (cubes side by side; the sensor spread hides their 20 mm gaps).
#   A row returns its first (right) cube at once, whatever comes next: rule 7.2 keeps 20 mm free to its
#   neighbour for the side arm. Pending: "as near as the cube" was mean + 1 (inside the +-2 face noise),
#   now mean + z_jump_mm(mean), the same tolerance that keeps a segment together.

class CubeDetector:
    # One reading per SAMPLE_STEP mm. A segment grows while readings stay within JUMP_MM of its average
    # (z_mean of the last SMOOTH_COUNT; since 03.10 of all its readings). When a segment ends into DEEPER space it may be a cube; we then
    # confirm the far (left, more negative) side is open for SIDE_CLEARANCE mm before returning it.
    def __init__(self):
        self.last_x = None
        self.recent = []
        self.start = self.end = self.nearest = None
        self.pending = None          # (z_mean, start, end, nearest) awaiting left-clearance confirmation

    def z_jump_mm(self, mean):
        return JUMP_MM + JUMP_PER_MM * min(mean, WALL_Z)

    def z_mean(self):
        # mean of ALL readings of the segment: a ramp between two cubes cannot drag it along (03.10 staircase)
        return self.total / self.count

    def new_segment(self, x, z):
        self.recent, self.start, self.end, self.nearest = [z], x, x, z    # recent: non-empty = segment open
        self.total, self.count = z, 1

    def is_cube(self, mean):
        if mean >= WALL_Z - 5 or self.start is None:
            return 0
        width = self.start - self.end
        if width < CUBE_WIDTH_MIN:
            return 0
        return 1 if width <= CUBE_WIDTH_MAX else 2

    def log(self, mean, cubes):
        print("SEG,%.0f,%.0f,%.0f,%.1f,%s" % (self.start, self.end, self.start - self.end, mean,
                                              ("no cube", "cube", "row")[cubes]))

    def first_of_row(self, mean):
        return self.start - ROW_FIRST_OFFSET, self.nearest

    def add(self, x, z):
        # if self.last_x is not None and self.last_x - x < SAMPLE_STEP:
        #     return None
        self.last_x = x
        z = 9999 if z is None else z

        if self.pending is not None:
            mean, pstart, pend, pnear = self.pending
            if z <= mean + self.z_jump_mm(mean):   # as near as the cube -> no gap, discard
                print("DISCARD,%.0f,%.0f,%.1f,%.0f" % (pstart, pend, mean, z))
                self.pending = None
                self.new_segment(x, z)
                return None
            if x <= pend - SIDE_CLEARANCE:          # open space past the far edge: a real cube
                self.pending = None
                return (pstart + pend) / 2, pnear
            return None

        if self.recent:
            mean = self.z_mean()
            jump = self.z_jump_mm(mean)
            if abs(z - mean) <= jump:
                self.total += z
                self.count += 1
                self.end = x
                self.nearest = min(self.nearest, z)
                return None
            cubes = self.is_cube(mean)
            self.log(mean, cubes)
            if cubes == 2:
                found = self.first_of_row(mean)
                self.new_segment(x, z)
                return found
            if cubes and z > mean:                  # ended into a deeper gap -> confirm the far side
                self.pending = (mean, self.start, self.end, self.nearest)
                self.recent = []
                return None
        self.new_segment(x, z)
        return None

    def finish(self):
        # The scan ended at the wall. Accept the open segment if it is nearer than the floor, even if it is
        # too short (no width or clearance check): the beam cannot reach the last bit before the wall.
        # !TODO measure! the gap from the beam at the leftmost reachable X to the wall must be < one cube.
        if self.recent:
            self.log(self.z_mean(), self.is_cube(self.z_mean()))
        if self.recent and self.is_cube(self.z_mean()) == 2:
            return self.first_of_row(self.z_mean())
        if self.recent and self.z_mean() < WALL_Z - 5 and self.start is not None:
            return (self.start + self.end) / 2, self.nearest
        if self.pending is not None:
            mean, pstart, pend, pnear = self.pending
            return (pstart + pend) / 2, pnear
        return None



def find_cube(robot, failed):
    robot.grip(GRIP_OPEN_DEG, 50)       # closed side arms sit in front of the sensor (reads ~90 everywhere, 01.10)
    robot.z.move(SEARCH_Z, Z_SPEED)     # slight retract so the arm clears cubes near the lanes
    robot.move_x(SEARCH_START_X, TRAVEL_SPEED)
    detector = CubeDetector()

    def skip(target):
        # a spot is skipped only after PICK_TRIES failed picks there (03.10: one failed grab hid a cube all run)
        if not target:
            return False
        tries = sum(1 for position in failed if abs(target[0] - position) < CUBE_SIZE / 2)
        if tries >= PICK_TRIES:
            print("SKIP,%.0f,%.0f,failed %d times" % (target[0], target[1], tries))
            return True
        return False

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
            z = None if distance is None else robot.z.mm() + distance - GRAB_DISTANCE   # grab Z in the Z=0 frame
            target = detector.add(x - SENSOR_X_OFFSET, z)                               # field X of the beam
            robot.report(distance, detector.z_mean() if detector.recent else None)
            if target and CALIBRATION_SCAN:
                print("FOUND,%.0f,%.0f" % target)
            elif target and not skip(target):
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
    if target and CALIBRATION_SCAN:
        print("FOUND,%.0f,%.0f" % target)
        return None
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
    robot.move_x(x + CLAW_X_OFFSET, CARRY_SPEED)
    robot.grip(GRIP_OPEN_DEG, 20)
    seen = [9999.0, 0.0]                # nearest reading so far and the Z where it was seen

    def touching():
        distance = robot.read_distance()
        if distance is None:
            return False
        if distance < seen[0] - 1:
            seen[0], seen[1] = distance, robot.z.mm()
        # only inside the claw: at <= 28 it fired 24 mm short of a far cube and closed on nothing (03.10)
        pushing = distance <= HOLD_DISTANCE and robot.z.mm() - seen[1] > PUSH_MM
        return distance <= GRAB_DISTANCE or pushing
    # The fast part also watches the sensor: Z_MM_PER_DEG may be off, so the estimate z may be too far.
    reached = (robot.z.move(max(0, z - 30), Z_SPEED, touching) or
               robot.z.move(min(z + 20, Z_MAX), Z_CREEP_SPEED, touching) or
               (robot.read_distance() or 9999) <= HOLD_DISTANCE)   # 03.10: stopped at 11, inside the claw
    print("GRAB,deg,reading", robot.z_motor.position(DEGREES), robot.read_distance())
    if reached:
        robot.z.move(min(robot.z.mm() + GRAB_SEAT, Z_MAX), Z_CREEP_SPEED) # seat cube deper, then close
        robot.grip(GRIP_CLOSED_DEG, 50)
        robot.hold_firm()
    robot.z.move(0, CARRY_SPEED)
    distance = held_distance(robot)
    held = reached and distance is not None and distance <= HOLD_DISTANCE
    if not held:
        print("GRAB-FAIL,reached %s,reading %s" % (reached, distance))
    return held


def colour(robot):
    hues = []
    for _ in range(7):
        if robot.optical.is_near_object() or robot.optical.brightness() >= 5:
            hues.append(robot.optical.hue())
        wait(LOOP_MS, MSEC)
    if len(hues) < 4:
        print("COLOUR,unknown,-,%d,%s" % (len(hues), hues))
        return "unknown"
    raw = list(hues)
    if max(hues) - min(hues) > 180:
        hues = [h + 360 if h < 180 else h for h in hues]
    hue = sorted(hues)[len(hues) // 2] % 360
    found = "unknown"
    for name in ("red", "green", "blue"):
        low, high = HUES[name]
        if (low <= hue <= high) if low <= high else (hue >= low or hue <= high):
            found = name
            break
    # COLOUR,result,median hue,readings used,all hues: 03.10 a green cube came out red
    print("COLOUR,%s,%.0f,%d,%s" % (found, hue, len(raw), [round(h) for h in raw]))
    return found


def place(robot, x, z):
    if not robot.move_x(x + CLAW_X_OFFSET, CARRY_SPEED, True):
        return False                                # cube lost on the way (already re-gripped once)
    last = [robot.z.mm(), now()]

    def stalled():
        # The cube runs into the lane already placed: push as long as Z still moves, release once it has
        # stood still for 0.3 s instead of stopping the run with "Z motion blocked" (03.10).
        if abs(robot.z.mm() - last[0]) > 1:
            last[0], last[1] = robot.z.mm(), now()
        return now() - last[1] > 300
    try:
        robot.z.move(z, CARRY_SPEED, stalled)
    except RobotError:          # the motor gave up against the lane ("blocked"/"timeout"): release here anyway
        pass
    robot.grip(GRIP_OPEN_DEG, 20)
    robot.z.move(0, Z_SPEED)
    if (robot.read_distance() or 9999) <= HOLD_DISTANCE:     # still something in the claw
        raise RobotError("cube not released")
    # gripper stays open: it only closes around a cube (collect, carry, push), so it cannot close by
    # accident next to the walls or the cubes already in storage (03.10)
    return True


def run(robot):
    used = {"green": 0, "red": 0, "blue": 0}
    drop = {name: (DROP_X[name], Z_BACK) for name in used}   # current (claw X, Z) for the next cube of each colour
    failed = []
    end_time = now() + 600000
    robot.grip(GRIP_OPEN_DEG, 50)       # starts closed; open before any move and keep it open unless holding a cube
    robot.home()
    print("area,claw X,beam X,Z,distance,z mean", STORAGE_AREA, DISPOSAL_AREA, MINING_AREA)
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
        robot.move_x(REHOME_X, TRAVEL_SPEED)        # re-zero X on the bumper after every cube (X drifts, 03.10)
        robot.home()
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
