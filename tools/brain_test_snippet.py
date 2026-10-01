# Paste into the __main__ block of src/main.py (instead of main()) to test single
# movements on the Brain. Kept here because main.py has no memory to spare.

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
