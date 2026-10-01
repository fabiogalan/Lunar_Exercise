"""Lunar cube sorter — the complete program flashed to the VEX IQ2 Brain.

Before starting: retract Z fully and close the gripper. Check all MANUAL VALUES.
X=0 is established by driving right until the bumper on port 6 is pressed.
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

# Measure these values by hand and replace the placeholders.

#test mode is _TEST ground, actual competitoon has bigger playgrounf so its _COMP, final code must use only _COMP parameters 

#0 is defined at the bump sensor, for position of Z distance sensor one must deduct sensor offset! 
X_MM_PER_DEG = 0.3245  
Z_MM_PER_DEG = 0.325        # 01.10: Z moved 83 mm in 255 deg (PICK Z 83, GRAB 255 deg, reading 20); same as X

# COORDINATES. Everything is measured in mm from the bumper (X=0), negative = left.
# Robot X (encoder) is the trolley: X=0 while the bumper is pressed.
# Field X is where something is on the field. A tool at robot X sits at
#   claw centre   = X - CLAW_X_OFFSET     -> to put the claw on field x: move to x + CLAW_X_OFFSET
#   distance beam = X - SENSOR_X_OFFSET   -> a reading taken at X is at field X - SENSOR_X_OFFSET
CLAW_X_OFFSET = 73.0        # bumper -> claw centre (axis of symmetry, where the cube centre goes)
MACHINE_LENGTH = 228.0      # at max left, the bumper (right end of the machine) is 228 mm from the left wall


X_MINING_TEST = 610.0  #length of mining area in mm
X_MINING_COMP = 1830 

X_DISPOSAL_TEST = 305.0   #length of disposal area in mm, ignoring the wall for a while 
X_DISPOSAL_COMP = 305.0  

X_STORAGE_TEST = 305.0  
X_STORAGE_COMP = 610.0  

# Field areas (field X). From the bumper: storage | disposal | mining.
COMPETITION = False         # True on competition day: uses the _COMP lengths
X_STORAGE = X_STORAGE_COMP if COMPETITION else X_STORAGE_TEST
X_DISPOSAL = X_DISPOSAL_COMP if COMPETITION else X_DISPOSAL_TEST
X_MINING = X_MINING_COMP if COMPETITION else X_MINING_TEST
STORAGE_AREA = (-X_STORAGE, 0.0)                                 # (left edge, right edge)
DISPOSAL_AREA = (STORAGE_AREA[0] - X_DISPOSAL, STORAGE_AREA[0])
MINING_AREA = (DISPOSAL_AREA[0] - X_MINING, DISPOSAL_AREA[0])


Z_MAX = 400.0
HOME_CLEAR_X = -10.0
#TODO make competition variables 
SEARCH_START_X_TEST = 0 - X_DISPOSAL_TEST - X_STORAGE_TEST # -610mm
SEARCH_END_X_TEST = - ( X_MINING_TEST + X_DISPOSAL_TEST + X_STORAGE_TEST) #-1820mm 
SEARCH_START_X_COMP = 0 - X_DISPOSAL_COMP - X_STORAGE_COMP
SEARCH_END_X_COMP = - ( X_MINING_COMP + X_DISPOSAL_COMP + X_STORAGE_COMP)

WALL_Z = 325.0              # Z depth of the wall at the end of the production area (01.10: empty floor reads ~345 - GRAB_DISTANCE)

SENSOR_Z_OFFSET = 0.0       # distance sensor is level with the base of the claw (was 25 mm behind it until 01.10)
GRAB_MARGIN = 20.0          # grab when the reading is within this of SENSOR_Z_OFFSET (noise, creep overshoot; 5 pushed the cube, 01.10)
GRAB_DISTANCE = SENSOR_Z_OFFSET + GRAB_MARGIN        # distance from the gripper to the cube when it is grabbed
HOLD_DISTANCE = SENSOR_Z_OFFSET + 2 * GRAB_MARGIN        # distance from the gripper to the cube when it is held securely
SENSOR_X_OFFSET = 93.0
SENSOR_CLAW_OFFSET = 10.0   # claw centre -> distance beam, beam left of the claw centre (was 17 until 01.10)
SENSOR_X_OFFSET = CLAW_X_OFFSET + SENSOR_CLAW_OFFSET   # bumper -> beam: 73 + 10 = 83, replaces the 93 above
SENSOR_DELAY_S = 0.0
GRIP_OPEN_DEG = 90          # 90 grabbed well (01.10)
GRIP_CLOSED_DEG = 0

# The SEARCH values above are field X for the beam; the robot X that puts the beam there:
SCAN_START_MARGIN = 50.0    # beam starts this far before the mining edge, so a cube on the edge shows its right side
SEARCH_START_X = (SEARCH_START_X_COMP if COMPETITION else SEARCH_START_X_TEST) + SCAN_START_MARGIN + SENSOR_X_OFFSET
# The left wall stops the machine, so X ends MACHINE_LENGTH before the end of the field:
X_WALL = (SEARCH_END_X_COMP if COMPETITION else SEARCH_END_X_TEST) + MACHINE_LENGTH   # -1220 + 228 = -992
X_MIN = X_WALL + 10.0       # robot X; 10 mm before the wall so the motor does not stall against it
SEARCH_END_X = X_MIN

# One X lane per colour. Every cube is pushed to the same Z depth; it pushes
# cubes already in that lane farther into the production area.
# Red and green at 1/4 and 3/4 of the storage area (green nearer the mining area,
# rule 7.2), blue in the middle of the disposal area.
DROP_X = {"red": STORAGE_AREA[0] * 0.25, "green": STORAGE_AREA[0] * 0.75,
          "blue": (DISPOSAL_AREA[0] + DISPOSAL_AREA[1]) / 2} #ACHTUNG: coordintates of the claw, sensor has offset from the center of the claw
DROP_Z = 100.0
HUES = {"red": (330.0, 25.0), "green": (70.0, 170.0), "blue": (180.0, 270.0)}

CUBE_SIZE = 75.0

# Detection values. A 75 mm cube rotated 10 degrees appears about 87 mm wide.
# The scan is cut into segments: a reading more than JUMP_MM from the average of the last
# SMOOTH_COUNT readings starts a new segment. A segment is a cube when it is CUBE_WIDTH_MIN..MAX long,
# nearer than the empty floor, and both neighbouring segments are deeper.
SAMPLE_STEP = 5.0           # mm between the readings the detector uses (the scan loop gives one per ~2.5 mm)
SMOOTH_COUNT = 5            # readings averaged; the oldest is dropped when a 6th comes
JUMP_MM = 5.0               # new segment when a reading is this far from that average (7.5 did not split a real
                            # row of 3 cubes, 5 did: 01.10 logs; face noise is +-2)
CUBE_WIDTH_MIN = 65.0       # segment length of a cube (01.10 logs: real cubes 70..95; the edges' slopes are
CUBE_WIDTH_MAX = 100.0      # cut off as separate short segments, so the sensor spread no longer matters)

SENSOR_SPREAD = 0.29          # the distance sensor's light spreads out: a cube looks wider by 0.29 x its distance (tools only)
                              # (01.10: 75 mm cube read 123 mm wide at 142 mm, 111 mm wide at 127 mm)
SIDE_CLEARANCE = 17.0         # 15 mm gap + 2 mm measurement margin (not used by the segment detector)
DEPTH_JUMP = 40.0             # abrupt jump; slopes/corners remain one cube (not used by the segment detector, see JUMP_MM)
GRIP_DEPTH = 90.0       # Z depth of the gripper when closed on a cube
POSITION_TOLERANCE = 3.0     # mm a finished motor may be off its target before it counts as blocked
LOOP_MS = 15                # loop time for all motion and sensor checks

# Speeds in percent.
HOME_SPEED = 20
SCAN_SPEED = 10             # ~2.5 mm between readings (25 gave 5 mm: the scan loop takes ~90 ms, 01.10)
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
        self.timeout = abs(target - self.mm()) * 2000.0 / (self.scale * 7.2 * speed) + 1500
        self.motor.set_timeout(self.timeout + 500, MSEC)
        self.motor.spin_to_position(target / self.scale, DEGREES, speed, PERCENT, False)

    def arrived(self):
        # At low speed the motor may report done a few degrees short (01.10: X 1.9 mm short at speed 10).
        error = abs(self.mm() - self.target)
        return error <= 1.0 or error <= POSITION_TOLERANCE and self.motor.is_done()

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
        self.x = Axis("X", self.x_motor, X_MM_PER_DEG, X_MIN, 0.0)
        self.z = Axis("Z", self.z_motor, Z_MM_PER_DEG, 0.0, Z_MAX)
        self.grip_motor.set_stopping(HOLD)
        self.grip_motor.set_position(0, DEGREES)
        self.homed = False
        self.last = -1

    def stop(self):
        self.x_motor.stop()
        self.z_motor.stop()
        self.grip_motor.stop()

    def read_distance(self):
        value = self.distance.object_distance(MM)
        return value if 0 < value <= 1000 else None

    def report(self, distance):
        # area,arm X,sensor X,Z,distance (field X), printed whenever the reading changes
        if distance != self.last:
            self.last = distance
            x = self.x.mm()
            print("%s,%.1f,%.1f,%.0f,%s" % ("MINE" if x - SENSOR_X_OFFSET <= MINING_AREA[1] else "DISPOSE" if x - SENSOR_X_OFFSET <= DISPOSAL_AREA[1] else "STORE", x - CLAW_X_OFFSET, x - SENSOR_X_OFFSET,
                                          self.z.mm(), "-" if distance is None else "%.0f" % distance))

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

    def move_x(self, target, speed, carrying=False):
        if not self.homed or self.z.mm() > 2:
            raise RobotError("home X and retract Z first")
        lost = [0]

        def tick():
            distance = self.read_distance()
            self.report(distance)
            if carrying:
                lost[0] = lost[0] + 1 if distance is None or distance > HOLD_DISTANCE else 0
                if lost[0] >= 3:
                    raise RobotError("cube lost")
        self.x.move(target, speed, tick)

    def grip(self, degrees, speed):
        self.grip_motor.set_timeout(5000, MSEC)
        self.grip_motor.spin_to_position(degrees, DEGREES, speed, PERCENT, True)
        if abs(self.grip_motor.position(DEGREES) - degrees) > 30:
            raise RobotError("gripper blocked")


# CUBE SEARCH ---------------------------------------------------------------
# The detector is compiled on its own when the program starts (exec), not together with the rest of
# the file: the Brain runs out of memory compiling everything at once (79 KB failed; this way 71 KB).
# Edit it like normal code, but avoid ''' and backslashes inside.
exec('''
class CubeDetector:
    # Cuts the scan into segments of similar depth, one reading per SAMPLE_STEP mm, keeping only the
    # last SMOOTH_COUNT depths of the current segment (memory). A cube face is a flat step; the sensor
    # turns its edges into slopes, which become short separate segments. Inside a row of cubes each
    # gap (rule 7.2: >= 2 cm) reads deeper and splits the row into one segment per cube.
    def __init__(self):
        self.last_x = None
        self.recent = []                # depths of the current segment, at most SMOOTH_COUNT
        self.start = self.end = self.nearest = None
        self.before = 9999              # average depth of the previous segment

    def add(self, x, z):
        if self.last_x is not None and self.last_x - x < SAMPLE_STEP:
            return None
        self.last_x = x
        z = 9999 if z is None else z     # no reading: nothing near
        if self.recent:
            average = sum(self.recent) / len(self.recent)
            if abs(z - average) <= JUMP_MM:
                self.recent = (self.recent + [z])[-SMOOTH_COUNT:]
                self.end, self.nearest = x, min(self.nearest, z)
                if len(self.first) < SMOOTH_COUNT:
                    self.first.append(z)
                return None
            found = self.cube(z > average)
            self.before = average
        else:
            found = None
        self.recent, self.first, self.start, self.end, self.nearest = [z], [z], x, x, z
        return found

    def cube(self, next_deeper=True):
        # The segment just ended (or the scan ended): a cube if long enough, nearer than the floor and
        # with deeper neighbours on both sides (a nearer neighbour would block the side arms).
        if not self.recent:
            return None
        average = sum(self.recent) / len(self.recent)
        if (next_deeper and self.before > average and average < WALL_Z - 5
                and CUBE_WIDTH_MIN <= self.start - self.end <= CUBE_WIDTH_MAX):
            # A rotated cube shows its long face (this segment, sloped) and a short steep side face at its
            # nearer end, cut off as a separate segment. The long face's depth changes by 75 sin(angle), twice
            # the shift that hides: move the centre towards the nearer end by half that depth difference.
            slope = average - sum(self.first) / len(self.first)
            return (self.start + self.end + slope) / 2, self.nearest
        return None
''')


def find_cube(robot, failed):
    robot.grip(GRIP_OPEN_DEG, 50)       # closed side arms sit in front of the sensor (reads ~90 everywhere, 01.10)
    robot.move_x(SEARCH_START_X, TRAVEL_SPEED)
    wait(300, MSEC)
    detector = CubeDetector()

    def tick():
        x = robot.x.mm()        # SENSOR_DELAY_S is not applied (velocity estimate removed to save memory)
        distance = robot.read_distance()
        robot.report(distance)
        z = None if distance is None else distance - GRAB_DISTANCE
        target = detector.add(x - SENSOR_X_OFFSET, z)   # field X
        if target and not any(abs(target[0] - position) < CUBE_SIZE / 2 for position in failed):
            return target
    target = robot.x.move(SEARCH_END_X, SCAN_SPEED, tick)
    if target is None:
        # Scan end: the last segment has not ended; judge it as if floor followed (rule 7.2 keeps the
        # next cube >= 2 cm away; 01.10: a cube 12 mm before the scan end).
        target = detector.cube()
        if target and any(abs(target[0] - position) < CUBE_SIZE / 2 for position in failed):
            target = None
    return target


# PICK AND SORT --------------------------------------------------------------
def held_distance(robot):
    values = []
    for _ in range(5):
        values.append(robot.read_distance() or 9999)     # no reading counts as far away
        wait(LOOP_MS, MSEC)
    return sorted(values)[2]                              # median of 5


def pick(robot, target):
    x, z = target                       # field X of the cube centre
    robot.move_x(x + CLAW_X_OFFSET, SCAN_SPEED)
    robot.grip(GRIP_OPEN_DEG, 20)

    def touching():
        distance = robot.read_distance()
        return distance is not None and distance <= GRAB_DISTANCE
    # The fast part also watches the sensor: Z_MM_PER_DEG may be off, so the estimate z may be too far.
    reached = robot.z.move(max(0, z - 30), Z_SPEED, touching) or \
        robot.z.move(min(z + 20, Z_MAX), Z_CREEP_SPEED, touching)
    print("GRAB,deg,reading", robot.z_motor.position(DEGREES), robot.read_distance())   # Z_MM_PER_DEG = (PICK Z + GRAB_DISTANCE - reading) / deg
    if reached:
        robot.grip(GRIP_CLOSED_DEG, 50)
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


def place(robot, name):
    robot.move_x(DROP_X[name] + CLAW_X_OFFSET, CARRY_SPEED, True)
    robot.z.move(DROP_Z, CARRY_SPEED)
    robot.grip(GRIP_OPEN_DEG, 20)
    robot.z.move(0, Z_SPEED)
    if (robot.read_distance() or 9999) <= HOLD_DISTANCE:     # still something in the claw
        raise RobotError("cube not released")
    robot.grip(GRIP_CLOSED_DEG, 50)


def run(robot):
    used = {"green": 0, "red": 0, "blue": 0} #maximum 4 cubes in the row, then push with x offset 
    failed = []
    end_time = now() + 600000
    robot.home()
    print("area,arm X,sensor X,Z,distance", STORAGE_AREA, DISPOSAL_AREA, MINING_AREA)   # + store, dispose, mine (field X)
    while now() < end_time - 90000:

        #check limits ? 
        show("SEARCHING")
        target = find_cube(robot, failed)
        if target is None:
            break
        show("PICK", "X %.0f Z %.0f" % target)
        if not pick(robot, target):
            failed.append(target[0])
            if len(failed) >= 3:
                break
            continue
        name = colour(robot)
        if name == "unknown":
            robot.move_x(HOME_CLEAR_X, CARRY_SPEED, True)
            show("STOP: cube held", name)
            return
        place(robot, name)
        used[name] += 1
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
        #main loop 
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

    # (test block kept as comments: a string costs compile memory, comments do not)

    #for example, test moving
    # robot = Robot()
    # missing = []
    #         for name, device in (("bumper", robot.bumper), ("distance", robot.distance),
    #                              ("optical", robot.optical), ("X", robot.x_motor),
    #                              ("Z", robot.z_motor), ("grip", robot.grip_motor)):
    #             if not device.installed():
    #                 missing.append(name)
    # if missing: print("missing: " + ", ".join(missing))

    #test moving the robot
    # robot.x.move(-100, 50)
    # robot.z.move(100, 50)
    # robot.grip(GRIP_OPEN_DEG, 50)
    # robot.grip(GRIP_CLOSED_DEG, 50)
    # robot.x.move(-200, 50)
    # robot.z.move(0, 50)
    # robot.stop()


    main()  #comment out for testing single functions / movements
