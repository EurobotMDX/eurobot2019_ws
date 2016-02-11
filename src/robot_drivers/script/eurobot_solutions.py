def move_yellow(robot):
    rospy.loginfo("pushing left")
    robot.push_left()

    robot.move_linear(0.09947)
    robot.move_angular(1.57)

    robot.move_linear(0.01)
    robot.move_angular(1.57)

    robot.move_linear(-1.14336) #-1.14336
    robot.move_angular(1.57)

    robot.move_linear(-0.043118)
    distance = robot.move_to_surface("back")

    rospy.loginfo("pushing right")
    robot.push_right(); rospy.sleep(1)

    robot.move_linear(distance)
    robot.move_linear(0.069348)
    robot.move_angular(1.57)

    robot.move_linear(-0.63947)
    robot.move_angular(1.57)

    rospy.loginfo("opening gripper")
    robot.open_gripper()

    robot.move_linear(0.24805)
    distance = robot.move_to_surface("front")

    rospy.loginfo("closing gripper")
    robot.close_gripper()

    robot.move_linear(-distance)
    robot.move_linear(-0.24805)
    robot.move_angular(-1.57)

    robot.move_linear(0.63947, should_avoid_obstacles=True)
    robot.move_angular(-1.57)

    robot.move_linear(0.63947, should_avoid_obstacles=True, move_timeout=10)
    distance = robot.move_to_surface("front")

    rospy.loginfo("open gripper")
    robot.open_gripper()

def move_purple(robot):
    rospy.loginfo("pushing right")
    robot.push_right()

    robot.move_linear(0.09947)
    robot.move_angular(-1.57)

    robot.move_linear(0.01)
    robot.move_angular(-1.57)

    robot.move_linear(-1.14336)
    robot.move_angular(-1.57)

    robot.move_linear(-0.043118) #-0.025118
    distance = robot.move_to_surface("back")

    rospy.loginfo("pushing left")
    robot.push_left(); rospy.sleep(1)

    robot.move_linear(distance)
    robot.move_linear(0.069348) # 0.051348
    robot.move_angular(-1.57)

    robot.move_linear(-0.63947)
    robot.move_angular(-1.57)

    rospy.loginfo("opening gripper")
    robot.open_gripper()

    robot.move_linear(0.24805)
    distance = robot.move_to_surface("front")

    rospy.loginfo("closing gripper")
    robot.close_gripper()

    robot.move_linear(-distance)
    robot.move_linear(-0.24805)
    robot.move_angular(1.57)

    robot.move_linear(0.63947, should_avoid_obstacles=True)
    robot.move_angular(1.57)

    robot.move_linear(0.63947, should_avoid_obstacles=True, move_timeout=10)
    distance = robot.move_to_surface("front")


    rospy.loginfo("open gripper")
    robot.open_gripper()