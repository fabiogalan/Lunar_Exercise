# VEX IQ Robot Library - TEST VERSION
from vex import *


brain = Brain()

# PORT CONFIG
SENSOR_TOUCH_PORT = Ports.PORT1
SENSOR_OPTICAL_PORT = Ports.PORT2
SENSOR_DISTANCE_PORT = Ports.PORT3
BUMPER_BUMPER_PORT = Ports.PORT4
MOTOR_MOVE_X_PORT = Ports.PORT7
MOTOR_MOVE_Z_PORT = Ports.PORT8
MOTOR_GRABBER_PORT = Ports.PORT5
MOTOR_4_PORT = Ports.PORT10

# DEVICES
touch_sensor = None
optical_sensor = None
distance_sensor = None
bumper_sensor = None
motor_X = None
motor_Z = None
motor_grabber = None
motor_4 = None

# SETUP
def setup():
    global touch_sensor, optical_sensor, distance_sensor, bumper_sensor, motor_X, motor_Z, motor_grabber, motor_4
    print("Init...")
    
    try:
        touch_sensor = Touchled(SENSOR_TOUCH_PORT)
        print("TouchLED OK")
    except:
        print("TouchLED Error")
    
    try:
        optical_sensor = Optical(SENSOR_OPTICAL_PORT)
        print("Color OK")
    except:
        optical_sensor = None
        print("Color Error")
    
    try:
        distance_sensor = Distance(SENSOR_DISTANCE_PORT)
        print("Distance OK")
    except:
        distance_sensor = None
        print("Distance Error")
    
    try:
        bumper_sensor = Bumper(BUMPER_BUMPER_PORT)
        print("Bumper OK")
    except:
        print("Bumper Error")
    
    try:
        motor_X = Motor(MOTOR_MOVE_X_PORT)
        # Test if motor is actually connected by trying to read position
        motor_X.position(DEGREES)
        print("MotorX OK")
    except:
        motor_X = None
        print("MotorX Error")
    
    try:
        motor_Z = Motor(MOTOR_MOVE_Z_PORT)
        # Test if motor is actually connected by trying to read position
        motor_Z.position(DEGREES)
        print("MotorZ OK")
    except:
        motor_Z = None
        print("MotorZ Error")
    
    try:
        motor_grabber = Motor(MOTOR_GRABBER_PORT)
        # Test if motor is actually connected by trying to read position
        motor_grabber.position(DEGREES)
        print("MotorGrabber OK")
    except:
        motor_grabber = None
        print("MotorGrabber Error")
EAF2-1E29

    try:
        motor_4 = Motor(MOTOR_4_PORT)
        # Test if motor is actually connected by trying to read position
        motor_4.position(DEGREES)
        print("Motor4 OK")
    except:
        motor_4 = None
        print("Motor4 Error")

# FUNCTIONS
def set_led(color):
    if touch_sensor:
        try:
            if color == "red":
                touch_sensor.set_color(Color.RED)
            elif color == "green":
                touch_sensor.set_color(Color.GREEN)
            elif color == "blue":
                touch_sensor.set_color(Color.BLUE)
            elif color == "yellow":
                touch_sensor.set_color(Color.YELLOW)
            elif color == "purple":
                touch_sensor.set_color(Color.PURPLE)
            elif color == "white":
                touch_sensor.set_color(Color.WHITE)
            elif color == "off":
                touch_sensor.off()
        except:
            pass

def led_pressed():
    if touch_sensor:
        try:
            return touch_sensor.pressing()
        except:
            return False
    return False

def set_color_sensor_led(on=True):
    """
    Controls the color sensor's built-in LED.
    
    Args:
        on: True to turn on the LED, False to turn it off
    """
    if optical_sensor:
        print("Setting color sensor LED to " + ("ON" if on else "OFF"))
        try:
            if on:
                optical_sensor.set_light(100)  # Turn on LED at full power
                print("Color sensor LED turned ON")
            else:
                optical_sensor.set_light(0)    # Turn off LED
                print("Color sensor LED turned OFF")
        except Exception as e:
            print("Color sensor LED error: " + str(e))
    else:
        print("Optical sensor not available")

def get_color():
    if optical_sensor:
        try:
            c = optical_sensor.color()
            if c == Color.RED:
                return "red"
            elif c == Color.GREEN:
                return "green"
            elif c == Color.BLUE:
                return "blue"
            elif c == Color.YELLOW:
                return "yellow"
            else:
                return "none"
        except Exception as e:
            return "error: " + str(e)
    return "sensor not available"

def get_distance():
    if distance_sensor:
        try:
            dist_mm = distance_sensor.object_distance(MM)  # Use object_distance instead of distance
            return dist_mm / 10.0  # Convert to cm
        except Exception as e:
            return -1  # Error indicator
    return -2  # Sensor not available

def bumper_pressed():
    if bumper_sensor:
        try:
            return bumper_sensor.pressing()
        except:
            return False
    return False

# MOTOR CONTROL FUNCTIONS

def spin_motor(motor, speed_percent):
    """
    Spins a motor at a certain speed and direction.
    
    Args:
        motor: The motor object to control
        speed_percent: Speed in percentage (-100 to 100). 
                      Positive values = forward, negative = reverse
    """
    if motor:
        try:
            if speed_percent > 0:
                motor.spin(FORWARD, abs(speed_percent), PERCENT)
            elif speed_percent < 0:
                motor.spin(REVERSE, abs(speed_percent), PERCENT)
            else:
                motor.stop()
        except:
            pass

def spin_motor_to_position(motor, position_degrees, speed_percent=50):
    """
    Spins a motor to a specific position.
    
    Args:
        motor: The motor object to control
        position_degrees: Target position in degrees
        speed_percent: Speed percentage (1-100, default 50)
    """
    if motor:
        try:
            motor.spin_to_position(position_degrees, DEGREES, abs(speed_percent), PERCENT)
        except:
            pass

def stop_motor(motor):
    """
    Stops a specific motor.
    
    Args:
        motor: The motor object to stop
    """
    if motor:
        try:
            motor.stop()
        except:
            pass

def set_motor_torque(motor, torque_percent):
    """
    Forces a motor to apply a certain torque.
    
    Args:
        motor: The motor object to control
        torque_percent: Torque percentage (-100 to 100).
                       Positive = forward torque, negative = reverse torque
    """
    if motor:
        try:
            if torque_percent > 0:
                motor.set_torque(abs(torque_percent), PERCENT)
                motor.spin(FORWARD)
            elif torque_percent < 0:
                motor.set_torque(abs(torque_percent), PERCENT)
                motor.spin(REVERSE)
            else:
                motor.stop()
        except:
            pass

# SEPECIFIC MISSION FUNCTIONS

def open_claw():
    brain.screen.clear_screen()
    brain.screen.print("Opening claw...")
    print("Opening claw...")
    spin_motor_to_position(motor_grabber, 25, 50)  # Assuming 25 degrees opens the claw
    wait(1000, MSEC)  # Wait for the motor to reach position
    brain.screen.clear_screen()
    brain.screen.print("Claw opened.")
    print("Claw opened.")

def close_claw():
    """Closes the claw if it's open."""
    brain.screen.clear_screen()
    brain.screen.print("Closing claw...")
    print("Closing claw...")
    spin_motor_to_position(motor_grabber, 0, 50)  # Assuming 0 degrees closes the claw
    wait(1000, MSEC)  # Wait for the motor to reach position
    brain.screen.clear_screen()
    brain.screen.print("Claw closed.")
    print("Claw closed.")

def move_Z_forward():
    if motor_Z:
        print("Moving Z forward...")
        spin_motor_to_position(motor_Z, 90, 50)  # Move Z to 90 degrees
        wait(1000, MSEC)
        print("Z moved forward.")
# TEST HELPER FUNCTIONS

def wait_for_touch():
    """Wait for touch sensor to be pressed and released."""
    print("Press touch sensor to continue...")
    
    # Wait for press
    while not led_pressed():
        wait(50, MSEC)
    
    # Wait for release
    while led_pressed():
        wait(50, MSEC)
    
    wait(200, MSEC)  # Small delay to avoid double presses

def test_touch_sensor():
    """Test touch sensor with color changes."""
    print("\n=== TOUCH SENSOR TEST ===")
    print("Testing TouchLED sensor...")
    
    # Start with red
    set_led("red")
    print("LED set to RED - Press to turn GREEN")
    wait_for_touch()
    
    # Turn green
    set_led("green")
    print("LED set to GREEN - Press to turn YELLOW")
    wait_for_touch()
    
    # Turn yellow
    set_led("yellow")
    print("LED set to YELLOW - Press to continue")
    wait_for_touch()
    
    set_led("off")
    print("Touch sensor test completed!")

def test_color_sensor():
    """Test color sensor."""
    print("\n=== COLOR SENSOR TEST ===")
    if not optical_sensor:
        print("Color sensor not available!")
        return
        
    print("Testing Color sensor...")
    
    # First, let's see what methods are available
    print("Available color sensor methods:")
    methods = [method for method in dir(optical_sensor) if not method.startswith('_') and 'light' in method.lower()]
    print("Light-related methods: " + str(methods))
    
    # Turn on color sensor LED
    set_color_sensor_led(True)
    print("Color sensor LED ON")
    print("Place colored objects in front of sensor")
    print("Press touch sensor to continue...")
    
    # Monitor colors until touch sensor is pressed
    while not led_pressed():
        color = get_color()
        print("Detected color: " + color + "    ", end="\r")
        wait(500, MSEC)
    print()  # New line after loop
    # Turn off color sensor LED
    set_color_sensor_led(False)
    print("Color sensor LED OFF")
    
    # Wait for release if pressed
    while led_pressed():
        wait(50, MSEC)
    
    print("Color sensor test completed!")

def test_distance_sensor():
    """Test distance sensor with live readings."""
    print("\n=== DISTANCE SENSOR TEST ===")
    if not distance_sensor:
        print("Distance sensor not available!")
        return
        
    print("Testing Distance sensor...")
    print("Move objects in front of sensor")
    print("Press touch sensor to continue...")
    
    # Monitor distance until touch sensor is pressed
    while not led_pressed():
        distance = get_distance()
        if distance == -1:
            print("Distance sensor error    ", end="\r")
        elif distance == -2:
            print("Distance sensor not available", end="\r")
        else:
            print("Distance: " + str(round(distance, 1)) + " cm    ", end="\r")
        wait(200, MSEC)
    print()  # New line after loop
    # Wait for release if pressed
    while led_pressed():
        wait(50, MSEC)
    
    print("Distance sensor test completed!")

def test_bumper_sensor():
    """Test bumper sensor."""
    print("\n=== BUMPER SENSOR TEST ===")
    print("Testing Bumper sensor...")
    print("Press the bumper switch")
    print("Press touch sensor to continue...")
    
    # Monitor bumper until touch sensor is pressed
    while not led_pressed():
        bumper_state = bumper_pressed()
        if bumper_state:
            print("Bumper state: PRESSED    ", end="\r")
        else:
            print("Bumper state: NOT PRESSED", end="\r")
        wait(200, MSEC)
    print()  # New line after loop
    
    # Wait for release if pressed
    while led_pressed():
        wait(50, MSEC)
    
    print("Bumper sensor test completed!")

def test_motor(motor, motor_name):
    """Test a single motor with different speeds and positions."""
    if not motor:
        print("Motor " + motor_name + " not available!")
        return
    
    print("\n=== " + motor_name.upper() + " TEST ===")
    print("Testing " + motor_name + "...")
    
    # Reset motor position
    motor.set_position(0, DEGREES)
    
    # Test 1: 90° slow (25% speed)
    print(motor_name + ": Spinning 90 deg at 25% speed")
    spin_motor_to_position(motor, 90, 25)
    wait(2000, MSEC)  # Wait for completion
    
    # Test 2: 180° medium speed (50% speed)
    print(motor_name + ": Spinning 180 deg at 50% speed")
    spin_motor_to_position(motor, 180 + 90, 50)
    wait(2000, MSEC)  # Wait for completion
    
    # Test 3: 360° full speed (100% speed)
    print(motor_name + ": Spinning 360 deg at 100% speed")
    spin_motor_to_position(motor, 360 + 180 + 90, 100)
    wait(2000, MSEC)  # Wait for completion
    
    # Return to starting position
    print(motor_name + ": Returning to 0 deg")
    spin_motor_to_position(motor, 0, 50)
    wait(2000, MSEC)  # Wait for completion
    
    stop_motor(motor)
    print(motor_name + " test completed!")

def test_all_motors():
    """Test all motors sequentially."""
    print("\n=== MOTOR TESTS ===")
    print("Testing all motors...")
    print("Press touch sensor to advance between motors")
    
    motors = [
        (motor_X, "Motor 1"),
        (motor_Z, "Motor 2"), 
        (motor_grabber, "Motor 3"),
        (motor_4, "Motor 4")
    ]
    
    for motor, name in motors:
        if motor:
            print("\nReady to test " + name)
            print("Press touch sensor to start..." if name == "Motor 1" else "Testing " + name)
            if name == "Motor 1":
                wait_for_touch()
            test_motor(motor, name)
        else:
            print("\n" + name + " not available - skipping")
        
        if motor != motors[-1][0]:  # Not the last motor
            print("Press touch sensor for next motor...")
            wait_for_touch()
    
    print("All motor tests completed!")

# MAIN TEST FUNCTION
def main_initial_tests():
    """Main test function that goes through all sensors and motors."""
    print("========================================")
    print("VEX IQ ROBOT COMPREHENSIVE TEST")
    print("========================================")
    
    # Initialize everything
    setup()
    wait(1000, MSEC)
    
    print("\nStarting comprehensive test sequence...")
    print("Use touch sensor to advance between test steps")
    set_color_sensor_led(True)
    
    # Test sequence
    test_touch_sensor()
    test_color_sensor()
    test_distance_sensor() 
    test_bumper_sensor()
    test_all_motors()
    
    print("\n========================================")
    print("ALL TESTS COMPLETED!")
    print("========================================")
    
    # Final celebration
    set_led("green")
    wait(1000, MSEC)
    set_led("off")

def main():
    if motor_grabber is None:
        brain.screen.print("Motor Grabber not available. Cannot test claw.")
    wait(10000, MSEC)
    open_claw()  # Ensure claw is open at start
    close_claw()  # Close claw to test functionality

# Run the test when this file is executed
if __name__ == "__main__":
    main()
