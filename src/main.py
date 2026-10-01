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
Z_MM_PER_DEG = 0.10

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

WALL_Z = 350.0              # Z depth of the wall at the end of the production area

GRAB_DISTANCE = 5.0        # distance from the gripper to the cube when it is grabbed
HOLD_DISTANCE = 5.0        # distance from the gripper to the cube when it is held securely
SENSOR_X_OFFSET = 93.0
SENSOR_CLAW_OFFSET = 17.0   # claw centre -> distance beam (beam is left of the claw centre)
SENSOR_X_OFFSET = CLAW_X_OFFSET + SENSOR_CLAW_OFFSET   # bumper -> beam: 73 + 17 = 90, replaces the 93 above
SENSOR_DELAY_S = 0.0
GRIP_OPEN_DEG = 80
GRIP_CLOSED_DEG = 0

# The SEARCH values above are field X for the beam; the robot X that puts the beam there:
SEARCH_START_X = (SEARCH_START_X_COMP if COMPETITION else SEARCH_START_X_TEST) + SENSOR_X_OFFSET
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
BOUNDARY_PAUSE_S = 5        # test: stop with the sensor on each area boundary for 5 s (0 = off)
HUES = {"red": (330.0, 25.0), "green": (70.0, 170.0), "blue": (180.0, 270.0)}

CUBE_SIZE = 75.0

# Detection values. A 75 mm cube rotated 10 degrees appears about 87 mm wide.
CUBE_WIDTH_MIN = 75.0
CUBE_WIDTH_MAX = 90.0

SIDE_CLEARANCE = 17.0         # 15 mm gap + 2 mm measurement margin
DEPTH_JUMP = 20.0             # abrupt jump; slopes/corners remain one cube
GRIP_DEPTH = 80.0       # Z depth of the gripper when closed on a cube
LOOP_MS = 15                # loop time for all motion and sensor checks

# Speeds in percent.
HOME_SPEED = 20
SCAN_SPEED = 25
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
        ideal_ms = abs(target - self.mm()) * 1000.0 / (self.scale * 720.0 * speed / 100.0)
        self.timeout = ideal_ms * 2.0 + 1500
        self.motor.set_timeout(self.timeout + 500, MSEC)
        self.motor.spin_to_position(target / self.scale, DEGREES, speed, PERCENT, False)

    def arrived(self):
        return abs(self.mm() - self.target) <= 1.0

    def check(self):
        if now() - self.started > self.timeout:
            raise RobotError(self.name + " motion timeout")
        if now() - self.started > 150 and self.motor.is_done() and not self.arrived():
            raise RobotError(self.name + " motion blocked")

    def move(self, target, speed):
        self.start(target, speed)
        try:
            while not self.arrived():
                self.check()
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
        self.z_motor = Motor(PORT_Z)
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
            print("%s,%.1f,%.1f,%.0f,%s" % (area(x - SENSOR_X_OFFSET), x - CLAW_X_OFFSET, x - SENSOR_X_OFFSET,
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
        # With BOUNDARY_PAUSE_S, first stop with the sensor on every boundary on the way.
        start = self.x.mm()
        for edge in sorted((STORAGE_AREA[0], DISPOSAL_AREA[0]), reverse=target < start):
            x = edge + SENSOR_X_OFFSET
            if BOUNDARY_PAUSE_S and abs(x - start) > 2 and (start - x) * (x - target) > -4:
                self.go_x(x, speed, carrying)
                print("BOUNDARY,field %.0f" % edge)
                self.last = -1
                self.report(self.read_distance())
                wait(BOUNDARY_PAUSE_S, SECONDS)
        self.go_x(target, speed, carrying)

    def go_x(self, target, speed, carrying):
        if not self.homed or self.z.mm() > 2:
            raise RobotError("home X and retract Z first")
        self.x.start(target, speed)
        lost = 0
        try:
            while not self.x.arrived():
                self.x.check()
                distance = self.read_distance()
                self.report(distance)
                if carrying:
                    lost = lost + 1 if distance is None or distance > HOLD_DISTANCE else 0
                    if lost >= 3:
                        raise RobotError("cube lost")
                wait(LOOP_MS, MSEC)
        finally:
            self.x_motor.stop()

    def grip(self, degrees, speed):
        self.grip_motor.set_timeout(5000, MSEC)
        self.grip_motor.spin_to_position(degrees, DEGREES, speed, PERCENT, True)
        if abs(self.grip_motor.position(DEGREES) - degrees) > 30:
            raise RobotError("gripper blocked")


# CUBE SEARCH ---------------------------------------------------------------
class CubeDetector:
    """Keeps only enough distance samples for one cube and its two side gaps."""
    def __init__(self):
        self.samples = []              # (corrected X, grab Z)

    def add(self, x, z):
        if self.samples:
            spacing = self.samples[-1][0] - x
            if spacing < 1.0:
                return None
            if spacing > 10.0:                  # a gap in the data, not just a slow loop (robot: ~2 mm per loop)
                self.samples = []
        self.samples.append((x, z))
        while self.samples[0][0] - x > 132:
            self.samples.pop(0)
        return self.find_target()

    def side_is_clear(self, index, direction, edge, required_depth):
        while 0 <= index < len(self.samples):
            x, z = self.samples[index]
            if z is None or z < required_depth:
                return False
            if abs(x - edge) >= SIDE_CLEARANCE:
                return True
            index += direction
        return False

    def find_target(self):
        s = self.samples
        i = 0
        while i < len(s):
            z = s[i][1]
            if z is None or z >= WALL_Z - CUBE_SIZE / 2:#TODO make a glb variable 
                i += 1
                continue
            first = i
            nearest = deepest = z
            i += 1
            # A rotated cube produces continuous slopes and a slope change at
            # its corner. Only an abrupt distance jump ends the cube profile.
            while i < len(s) and s[i][1] is not None:
                z = s[i][1]
                if abs(z - s[i - 1][1]) > DEPTH_JUMP:
                    break
                nearest, deepest = min(nearest, z), max(deepest, z)
                i += 1
            if first == 0 or i == len(s):
                continue                    # both outer edges are not known yet
            right = (s[first - 1][0] + s[first][0]) / 2
            left = (s[i - 1][0] + s[i][0]) / 2
            width = right - left
            if width < CUBE_WIDTH_MIN or width > CUBE_WIDTH_MAX:
                continue
            centre = (left + right) / 2
            beam_x = centre - SENSOR_CLAW_OFFSET    # beam while the claw is on the cube
            sample = min(range(first, i), key=lambda j: abs(s[j][0] - beam_x))
            grab_z = s[sample][1]
            clear_depth = max(grab_z + GRIP_DEPTH, deepest) + 5
            if self.side_is_clear(first - 1, -1, right, clear_depth) and \
                    self.side_is_clear(i, 1, left, clear_depth):
                return centre, grab_z
        return None


def area(x):
    # field X -> area code
    for name, (left, right) in (("MINE", MINING_AREA), ("DISPOSE", DISPOSAL_AREA), ("STORE", STORAGE_AREA)):
        if left <= x <= right:
            return name
    return "OUT"


def find_cube(robot, failed):
    robot.move_x(SEARCH_START_X, TRAVEL_SPEED)
    wait(300, MSEC)
    detector = CubeDetector()
    robot.x.start(SEARCH_END_X, SCAN_SPEED)
    old_x, old_time, velocity = robot.x.mm(), now(), 0.0
    try:
        while not robot.x.arrived():
            robot.x.check()
            x, time = robot.x.mm(), now()
            if time > old_time:
                velocity = 0.7 * velocity + 0.3 * (x - old_x) * 1000.0 / (time - old_time)
            old_x, old_time = x, time
            distance = robot.read_distance()
            robot.report(distance)
            z = None if distance is None else distance - GRAB_DISTANCE
            target = detector.add(x - velocity * SENSOR_DELAY_S - SENSOR_X_OFFSET, z)   # field X
            if target and not any(abs(target[0] - position) < CUBE_SIZE / 2 for position in failed):
                return target
            wait(LOOP_MS, MSEC)
    finally:
        robot.x_motor.stop()
    return None


# PICK AND SORT --------------------------------------------------------------
def held_distance(robot):
    values = []
    for _ in range(5):
        value = robot.read_distance()
        if value is not None:
            values.append(value)
        wait(LOOP_MS, MSEC)
    return sorted(values)[len(values) // 2] if len(values) >= 3 else None


def pick(robot, target):
    x, z = target                       # field X of the cube centre
    robot.move_x(x + CLAW_X_OFFSET, SCAN_SPEED)
    robot.grip(GRIP_OPEN_DEG, 20)
    robot.z.move(max(0, z - 30), Z_SPEED)
    robot.z.start(min(z + 20, Z_MAX), Z_CREEP_SPEED)
    reached = False
    try:
        while not robot.z.arrived():
            distance = robot.read_distance()
            if distance is not None and distance <= GRAB_DISTANCE:
                reached = True
                break
            robot.z.check()
            wait(LOOP_MS, MSEC)
    finally:
        robot.z_motor.stop()
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
    robot.grip(GRIP_CLOSED_DEG, 50)


def run(robot):
    used = {"green": 0, "red": 0, "blue": 0} #maximum 4 cubes in the row, then push with x offset 
    failed = []
    end_time = now() + 600000
    robot.home()
    print("area,arm X,sensor X,Z,distance")
    print("AREAS (field X) store", STORAGE_AREA, "dispose", DISPOSAL_AREA, "mine", MINING_AREA, "drop", DROP_X)
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
        missing = []
        for name, device in (("bumper", robot.bumper), ("distance", robot.distance),
                             ("optical", robot.optical), ("X", robot.x_motor),
                             ("Z", robot.z_motor), ("grip", robot.grip_motor)):
            if not device.installed():
                missing.append(name)
        if missing:
            raise RobotError("missing: " + ", ".join(missing))
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

    """

    #for example, test moving 
    robot = Robot()
    missing = []
            for name, device in (("bumper", robot.bumper), ("distance", robot.distance),
                                 ("optical", robot.optical), ("X", robot.x_motor),
                                 ("Z", robot.z_motor), ("grip", robot.grip_motor)):
                if not device.installed():
                    missing.append(name)
    if missing: print("missing: " + ", ".join(missing))

    #test moving the robot
    robot.x.move(-100, 50)
    robot.z.move(100, 50)
    robot.grip(GRIP_OPEN_DEG, 50)
    robot.grip(GRIP_CLOSED_DEG, 50)
    robot.x.move(-200, 50)
    robot.z.move(0, 50)
    robot.stop()

    """
    main()  #comment out for testing single functions / movements
