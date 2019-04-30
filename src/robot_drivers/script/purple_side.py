rospy.loginfo("adding score for experiment")
robot.update_robot_score(40.0)

robot.move_to(0.16943, 0.34233, -1.23446, target_speed=0.25)
robot.move_to(0.05432, 1.48142, -90.18769, target_speed=0.3, should_avoid_obstacles=True)

robot.push_left()
d = robot.move_to_surface(target_speed=0.15, access_key="back"); rospy.sleep(0.5)

robot.push_right(); rospy.sleep(0.5)
robot.move_linear(d)

rospy.loginfo("adding score for particle accelerator")
robot.update_robot_score(10.0)

robot.move_to(0.02273, 2.05537, 88.13082, target_speed=0.25, should_avoid_obstacles=True)
robot.open_gripper(); rospy.sleep(0.5)

rospy.loginfo("adding score for flap open")
robot.update_robot_score(10.0)

d = robot.move_to_surface(target_speed=0.15)
robot.close_gripper(); rospy.sleep(1.0)
robot.move_linear(-d, target_speed=0.15)

rospy.loginfo("adding score for releasing goldium")
robot.update_robot_score(20.0)

robot.move_to(-0.8771, 1.1, -83.07387, target_speed=0.25, should_avoid_obstacles=True)
d = robot.move_to_surface(target_speed=0.15)
robot.open_gripper(); rospy.sleep(1.0)

rospy.loginfo("adding score for goldium in weighing area")
robot.update_robot_score(24.0)