from vex import *
MANUAL_VALUES_SET = True
PORT_Z = Ports.PORT1
PORT_BUMPER = Ports.PORT6
PORT_TOUCH = Ports.PORT3
PORT_X = Ports.PORT5
PORT_OPTICAL = Ports.PORT7
PORT_DISTANCE = Ports.PORT8
PORT_GRIP = Ports.PORT9
X_MM_PER_DEG = 0.3245
Z_MM_PER_DEG = 0.3245
CLAW_X_OFFSET = 73.0
MACHINE_LENGTH = 228.0
X_STORAGE_TEST = 305.0
X_STORAGE_COMP = 610.0
X_DISPOSAL_TEST = 305.0
X_DISPOSAL_COMP = 305.0
X_MINING_TEST = 610.0
X_MINING_COMP = 1830.0
COMPETITION = False
CALIBRATION_SCAN = False
X_STORAGE = X_STORAGE_COMP if COMPETITION else X_STORAGE_TEST
X_DISPOSAL = X_DISPOSAL_COMP if COMPETITION else X_DISPOSAL_TEST
X_MINING = X_MINING_COMP if COMPETITION else X_MINING_TEST
STORAGE_AREA = (-X_STORAGE, 0.0)
DISPOSAL_AREA = (STORAGE_AREA[0] - X_DISPOSAL, STORAGE_AREA[0])
MINING_AREA = (DISPOSAL_AREA[0] - X_MINING, DISPOSAL_AREA[0])
Z_MAX = 305
HOME_CLEAR_X = -10.0
REHOME_X = -60.0
SEARCH_Z = -12.0
X_TRAVEL_MIN = -3000.0
WALL_Z = 380
GRAB_MARGIN = 7.0
GRAB_DISTANCE = GRAB_MARGIN
HOLD_DISTANCE = 2 * GRAB_MARGIN
SENSOR_CLAW_OFFSET = 10.0
SENSOR_X_OFFSET = CLAW_X_OFFSET + SENSOR_CLAW_OFFSET
GRIP_OPEN_DEG = 90
GRIP_CLOSED_DEG = 0
GRAB_SEAT = 5.0
PUSH_MM = 5.0
SCAN_START_MARGIN = 50.0
SEARCH_START_X = MINING_AREA[1] + SCAN_START_MARGIN + SENSOR_X_OFFSET
CUBE_SIZE = 75.0
CUBE_MARGIN = 10.0
PRODUCTION_AREA = STORAGE_AREA
EDGE_MARGIN = 5.0
CLAW_WIDTH = 20.0
WIDTH_BETWEEN_CLAWS = 90.0
CUBES_PER_LINE = 4
LANE_STEP_X = CUBE_SIZE + CUBE_MARGIN
LANE_STEP_Z = CUBE_SIZE
Z_BACK = Z_MAX - CUBE_MARGIN
DROP_X = {'red': PRODUCTION_AREA[1] - EDGE_MARGIN - CLAW_X_OFFSET, 'green': PRODUCTION_AREA[0] + EDGE_MARGIN + CLAW_X_OFFSET, 'blue': (DISPOSAL_AREA[0] + DISPOSAL_AREA[1]) / 2}
LANE_DIR = {'red': -1, 'green': 1, 'blue': 0}
DROP_Z = CUBE_SIZE + CUBE_MARGIN
HUES = {'red': (330.0, 25.0), 'green': (70.0, 170.0), 'blue': (180.0, 270.0)}
SAMPLE_STEP = 2.5
SMOOTH_COUNT = 5
JUMP_MM = 10.0
JUMP_PER_MM = 0.02
CUBE_WIDTH_MIN = CUBE_SIZE - CUBE_MARGIN
CUBE_WIDTH_MAX = 90.0 + CUBE_MARGIN
SIDE_CLEARANCE = CUBE_MARGIN
CUBE_WIDTH_MIN = 40.0
CUBE_WIDTH_MAX = 175.0
ROW_FIRST_OFFSET = 60.0
POSITION_TOLERANCE = 3.0
LOOP_MS = 15
HOME_SPEED = 80
SCAN_SPEED = 20
TRAVEL_SPEED = 80
CARRY_SPEED = 80
Z_SPEED = 60
Z_CREEP_SPEED = 40
brain = Brain()
exec('class RobotError(Exception):\n pass\n')
exec('def now():\n return brain.timer.time(MSEC)\n')
exec("def show(*lines):\n brain.screen.clear_screen()\n for row, line in enumerate(lines):\n  brain.screen.set_cursor(row + 1, 1)\n  brain.screen.print(line)\n print(' | '.join(lines))\n")
exec("def wait_for_start(touch):\n show('READY', 'Z in, gripper closed', 'Check or LED to start')\n while not brain.buttonCheck.pressing() and (not (touch.installed() and touch.pressing())):\n  wait(20, MSEC)\n while brain.buttonCheck.pressing() or (touch.installed() and touch.pressing()):\n  wait(20, MSEC)\n")
exec("def area(field_x):\n if field_x <= MINING_AREA[1]:\n  return 'MINE'\n if field_x <= DISPOSAL_AREA[1]:\n  return 'DISPOSE'\n if field_x <= 0:\n  return 'STORE'\n return 'OUT'\n")
exec("class Axis:\n def __init__(self, name, motor, mm_per_degree, minimum, maximum):\n  self.name = name\n  self.motor = motor\n  self.scale = mm_per_degree\n  self.minimum = minimum\n  self.maximum = maximum\n  self.target = 0.0\n  self.started = 0\n  self.timeout = 0\n  motor.set_stopping(HOLD)\n  motor.set_position(0, DEGREES)\n def mm(self):\n  return self.motor.position(DEGREES) * self.scale\n def start(self, target, speed):\n  if target < self.minimum or target > self.maximum:\n   raise RobotError(self.name + ' target outside travel')\n  self.target = target\n  self.started = now()\n  self.timeout = abs(target - self.mm()) * 2000.0 / (abs(self.scale) * 7.2 * speed) + 1500\n  self.motor.set_timeout(self.timeout + 500, MSEC)\n  self.motor.spin_to_position(target / self.scale, DEGREES, speed, PERCENT, False)\n def arrived(self):\n  error = abs(self.mm() - self.target)\n  return error <= 1.0 or (error <= POSITION_TOLERANCE and self.motor.is_done())\n def check(self):\n  if now() - self.started > self.timeout:\n   raise RobotError(self.name + ' motion timeout')\n  if now() - self.started > 150 and self.motor.is_done() and (not self.arrived()):\n   raise RobotError(self.name + ' motion blocked')\n def move(self, target, speed, tick=None):\n  self.start(target, speed)\n  try:\n   while not self.arrived():\n    self.check()\n    result = tick and tick()\n    if result:\n     return result\n    wait(LOOP_MS, MSEC)\n  finally:\n   self.motor.stop()\n")
exec("class Robot:\n def __init__(self):\n  self.bumper = Bumper(PORT_BUMPER)\n  self.touch = Touchled(PORT_TOUCH)\n  self.distance = Distance(PORT_DISTANCE)\n  self.optical = Optical(PORT_OPTICAL)\n  self.x_motor = Motor(PORT_X)\n  self.z_motor = Motor(PORT_Z, True)\n  self.grip_motor = Motor(PORT_GRIP)\n  self.x = Axis('X', self.x_motor, X_MM_PER_DEG, X_TRAVEL_MIN, 0.0)\n  self.z = Axis('Z', self.z_motor, Z_MM_PER_DEG, SEARCH_Z, Z_MAX)\n  self.grip_motor.set_stopping(HOLD)\n  self.grip_motor.set_position(0, DEGREES)\n  self.homed = False\n  self.x_wall = X_TRAVEL_MIN\n  self.last = -1\n def stop(self):\n  self.x_motor.stop()\n  self.z_motor.stop()\n  self.grip_motor.stop()\n def read_distance(self):\n  value = self.distance.object_distance(MM)\n  return value if 0 < value <= 1000 else None\n def report(self, distance, mean=None):\n  if distance != self.last:\n   self.last = distance\n   x = self.x.mm()\n   print('%s,%.1f,%.1f,%.0f,%s,%s' % (area(x - SENSOR_X_OFFSET), x - CLAW_X_OFFSET, x - SENSOR_X_OFFSET, self.z.mm(), '-' if distance is None else '%.0f' % distance, '-' if mean is None else '%.1f' % mean))\n def home(self):\n  if self.z.mm() > 5:\n   raise RobotError('retract Z before homing')\n  show('HOMING X', 'moving right')\n  self.homed = False\n  start = now()\n  self.x_motor.spin(FORWARD, HOME_SPEED, PERCENT)\n  try:\n   while not self.bumper.pressing():\n    if now() - start > 120000:\n     raise RobotError('bumper not reached')\n    wait(LOOP_MS, MSEC)\n  finally:\n   self.x_motor.stop()\n  wait(60, MSEC)\n  self.x_motor.set_position(0, DEGREES)\n  self.x.move(HOME_CLEAR_X, HOME_SPEED)\n  if self.bumper.pressing():\n   raise RobotError('bumper did not release')\n  self.homed = True\n def hold_firm(self):\n  self.grip_motor.set_max_torque(100, PERCENT)\n def move_x(self, target, speed, carrying=False):\n  if not self.homed or self.z.mm() > 2:\n   raise RobotError('home X and retract Z first')\n  lost = [0]\n  regripped = [False]\n  def tick():\n   distance = self.read_distance()\n   self.report(distance)\n   if carrying:\n    lost[0] = lost[0] + 1 if distance is None or distance > HOLD_DISTANCE else 0\n    if lost[0] >= 3:\n     return 'lost'\n  while True:\n   if self.x.move(target, speed, tick) != 'lost':\n    return True\n   if regripped[0]:\n    return False\n   regripped[0] = True\n   self.grip(GRIP_CLOSED_DEG, 50)\n   lost[0] = 0\n def grip(self, degrees, speed):\n  self.grip_motor.set_timeout(5000 if degrees == GRIP_CLOSED_DEG else 1500, MSEC)\n  self.grip_motor.spin_to_position(degrees, DEGREES, speed, PERCENT, True)\n  if abs(self.grip_motor.position(DEGREES) - degrees) > 30:\n   raise RobotError('gripper blocked')\n")
exec("class CubeDetector:\n def __init__(self):\n  self.last_x = None\n  self.recent = []\n  self.start = self.end = self.nearest = None\n  self.pending = None\n def z_jump_mm(self, mean):\n  return JUMP_MM + JUMP_PER_MM * min(mean, WALL_Z)\n def z_mean(self):\n  return self.total / self.count\n def new_segment(self, x, z):\n  self.recent, self.start, self.end, self.nearest = ([z], x, x, z)\n  self.total, self.count = (z, 1)\n def is_cube(self, mean):\n  if mean >= WALL_Z - 5 or self.start is None:\n   return 0\n  width = self.start - self.end\n  if width < CUBE_WIDTH_MIN:\n   return 0\n  return 1 if width <= CUBE_WIDTH_MAX else 2\n def log(self, mean, cubes):\n  print('SEG,%.0f,%.0f,%.0f,%.1f,%s' % (self.start, self.end, self.start - self.end, mean, ('no cube', 'cube', 'row')[cubes]))\n def first_of_row(self, mean):\n  return (self.start - ROW_FIRST_OFFSET, self.nearest)\n def add(self, x, z):\n  self.last_x = x\n  z = 9999 if z is None else z\n  if self.pending is not None:\n   mean, pstart, pend, pnear = self.pending\n   if z <= mean + self.z_jump_mm(mean):\n    print('DISCARD,%.0f,%.0f,%.1f,%.0f' % (pstart, pend, mean, z))\n    self.pending = None\n    self.new_segment(x, z)\n    return None\n   if x <= pend - SIDE_CLEARANCE:\n    self.pending = None\n    return ((pstart + pend) / 2, pnear)\n   return None\n  if self.recent:\n   mean = self.z_mean()\n   jump = self.z_jump_mm(mean)\n   if abs(z - mean) <= jump:\n    self.total += z\n    self.count += 1\n    self.end = x\n    self.nearest = min(self.nearest, z)\n    return None\n   cubes = self.is_cube(mean)\n   self.log(mean, cubes)\n   if cubes == 2:\n    found = self.first_of_row(mean)\n    self.new_segment(x, z)\n    return found\n   if cubes and z > mean:\n    self.pending = (mean, self.start, self.end, self.nearest)\n    self.recent = []\n    return None\n  self.new_segment(x, z)\n  return None\n def finish(self):\n  if self.recent:\n   self.log(self.z_mean(), self.is_cube(self.z_mean()))\n  if self.recent and self.is_cube(self.z_mean()) == 2:\n   return self.first_of_row(self.z_mean())\n  if self.recent and self.z_mean() < WALL_Z - 5 and (self.start is not None):\n   return ((self.start + self.end) / 2, self.nearest)\n  if self.pending is not None:\n   mean, pstart, pend, pnear = self.pending\n   return ((pstart + pend) / 2, pnear)\n  return None\n")
exec("def find_cube(robot, failed):\n robot.grip(GRIP_OPEN_DEG, 50)\n robot.z.move(SEARCH_Z, Z_SPEED)\n robot.move_x(SEARCH_START_X, TRAVEL_SPEED)\n detector = CubeDetector()\n def skip(target):\n  return bool(target) and any((abs(target[0] - position) < CUBE_SIZE / 2 for position in failed))\n robot.x_motor.spin(REVERSE, SCAN_SPEED, PERCENT)\n start = now()\n checkpoint = robot.x.mm()\n loops = 0\n try:\n  while now() - start < 120000:\n   wait(LOOP_MS, MSEC)\n   x = robot.x.mm()\n   distance = robot.read_distance()\n   z = None if distance is None else robot.z.mm() + distance - GRAB_DISTANCE\n   target = detector.add(x - SENSOR_X_OFFSET, z)\n   robot.report(distance, detector.z_mean() if detector.recent else None)\n   if target and CALIBRATION_SCAN:\n    print('FOUND,%.0f,%.0f' % target)\n   elif target and (not skip(target)):\n    return target\n   loops += 1\n   if loops >= 20:\n    if abs(x - checkpoint) < 2.0:\n     robot.x_wall = x\n     robot.x.minimum = x\n     break\n    checkpoint, loops = (x, 0)\n finally:\n  robot.x_motor.stop()\n target = detector.finish()\n if target and CALIBRATION_SCAN:\n  print('FOUND,%.0f,%.0f' % target)\n  return None\n return None if skip(target) else target\n")
exec('def held_distance(robot):\n values = []\n for _ in range(5):\n  values.append(robot.read_distance() or 9999)\n  wait(LOOP_MS, MSEC)\n return sorted(values)[2]\n')
exec("def pick(robot, target):\n x, z = target\n robot.move_x(x + CLAW_X_OFFSET, CARRY_SPEED)\n robot.grip(GRIP_OPEN_DEG, 20)\n seen = [9999.0, 0.0]\n def touching():\n  distance = robot.read_distance()\n  if distance is None:\n   return False\n  if distance < seen[0] - 1:\n   seen[0], seen[1] = (distance, robot.z.mm())\n  pushing = distance <= 2 * HOLD_DISTANCE and robot.z.mm() - seen[1] > PUSH_MM\n  return distance <= GRAB_DISTANCE or pushing\n reached = robot.z.move(max(0, z - 30), Z_SPEED, touching) or robot.z.move(min(z + 20, Z_MAX), Z_CREEP_SPEED, touching) or (robot.read_distance() or 9999) <= HOLD_DISTANCE\n print('GRAB,deg,reading', robot.z_motor.position(DEGREES), robot.read_distance())\n if reached:\n  robot.z.move(min(robot.z.mm() + GRAB_SEAT, Z_MAX), Z_CREEP_SPEED)\n  robot.grip(GRIP_CLOSED_DEG, 50)\n  robot.hold_firm()\n robot.z.move(0, CARRY_SPEED)\n distance = held_distance(robot)\n return reached and distance is not None and (distance <= HOLD_DISTANCE)\n")
exec("def colour(robot):\n hues = []\n for _ in range(7):\n  if robot.optical.is_near_object() or robot.optical.brightness() >= 5:\n   hues.append(robot.optical.hue())\n  wait(LOOP_MS, MSEC)\n if len(hues) < 4:\n  return 'unknown'\n if max(hues) - min(hues) > 180:\n  hues = [h + 360 if h < 180 else h for h in hues]\n hue = sorted(hues)[len(hues) // 2] % 360\n for name in ('red', 'green', 'blue'):\n  low, high = HUES[name]\n  if low <= hue <= high if low <= high else hue >= low or hue <= high:\n   return name\n return 'unknown'\n")
exec("def place(robot, x, z):\n if not robot.move_x(x + CLAW_X_OFFSET, CARRY_SPEED, True):\n  return False\n robot.z.move(z, CARRY_SPEED)\n robot.grip(GRIP_OPEN_DEG, 20)\n robot.z.move(0, Z_SPEED)\n if (robot.read_distance() or 9999) <= HOLD_DISTANCE:\n  raise RobotError('cube not released')\n robot.grip(GRIP_CLOSED_DEG, 50)\n return True\n")
exec("def run(robot):\n used = {'green': 0, 'red': 0, 'blue': 0}\n drop = {name: (DROP_X[name], Z_BACK) for name in used}\n failed = []\n end_time = now() + 600000\n robot.home()\n print('area,claw X,beam X,Z,distance,z mean', STORAGE_AREA, DISPOSAL_AREA, MINING_AREA)\n while now() < end_time:\n  show('SEARCHING')\n  target = find_cube(robot, failed)\n  if target is None:\n   break\n  show('PICK', 'X %.0f Z %.0f' % target)\n  if not pick(robot, target):\n   failed.append(target[0])\n   continue\n  name = colour(robot)\n  if name == 'unknown':\n   robot.move_x(HOME_CLEAR_X, CARRY_SPEED, True)\n   show('STOP: cube held', name)\n   return\n  x, z = drop[name]\n  if not place(robot, x, z):\n   show('lost on the way', name)\n   continue\n  used[name] += 1\n  robot.move_x(REHOME_X, TRAVEL_SPEED)\n  robot.home()\n  if drop[name] != drop['blue'] and LANE_DIR[name] != 0:\n   if used[name] % CUBES_PER_LINE == 0:\n    drop[name] = (x + LANE_DIR[name] * LANE_STEP_X, Z_BACK)\n    if abs(drop['red'][0] - drop['green'][0]) < WIDTH_BETWEEN_CLAWS + CUBE_MARGIN:\n     drop[name] = drop['blue']\n   else:\n    drop[name] = (x, z - LANE_STEP_Z)\n robot.move_x(HOME_CLEAR_X, TRAVEL_SPEED)\n show('DONE', 'G%d R%d B%d' % (used['green'], used['red'], used['blue']))\n")
exec("def main():\n robot = None\n try:\n  robot = Robot()\n  for name in ('bumper', 'distance', 'optical', 'x_motor', 'z_motor', 'grip_motor'):\n   if not getattr(robot, name).installed():\n    raise RobotError('missing: ' + name)\n  if not MANUAL_VALUES_SET:\n   raise RobotError('enter manual values, then set MANUAL_VALUES_SET=True')\n  robot.optical.set_light(100)\n  wait_for_start(robot.touch)\n  run(robot)\n except Exception as error:\n  if robot:\n   robot.stop()\n  message = str(error)\n  show('ERROR', message[:22], message[22:44], message[44:66])\n  while not brain.buttonCheck.pressing():\n   wait(20, MSEC)\n finally:\n  if robot:\n   robot.stop()\n")
if __name__ == '__main__':
 main()
