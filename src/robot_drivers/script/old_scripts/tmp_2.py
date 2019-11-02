#!/usr/bin/env python

from __future__ import division

import time
import copy
import rospy
from pid_controller import PID
from robot_interface import RobotInterfaceBase


class AdvancedRobotInterface(RobotInterfaceBase):
    def __init__(self):
        super(AdvancedRobotInterface, self).__init__()

    def initialize(self):
        super(AdvancedRobotInterface, self).initialize()
    
    def get_sign(self, value):
        if value == 0:
            return 1.0
        else:
            return abs(value) / value
    
    def ensure_clear_path(self, access_key, collision_range=0.3, timeout=5.0):

        status = False

        start_time = time.time()
        while not rospy.is_shutdown():
            distance_to_obstacle = self.proximity_sensors[access_key]

            # rospy.loginfo("distance_to_obstacle: {} vs col_rang {}".format(distance_to_obstacle, collision_range))

            if distance_to_obstacle > collision_range:
                status = True
                break

            if timeout > 0:
                if (time.time() - start_time) >= timeout:
                    status = False
                    break
            
            self.set_motion(0, 0)
        
        return status
    
    def move_linear(self, displacement, target_speed=0.2, precision=3, should_avoid_obstacles=True, collision_distance=0.5, move_timeout=50, sensor_timeout=10):
        rospy.sleep(0.5); self.set_motion(0.0, 0.0)

        target_speed = abs(target_speed)
        status = False

        current_position = copy.deepcopy(self.robot_position)
        end_position = copy.deepcopy(self.robot_position)
        end_position["y"] = end_position["y"] + displacement

        x_pid_controller = PID(-1.0, 1.0, 0.4, 0, 0.1)
        y_pid_controller = PID(-1.0, 1.0, 10.0, 0, 1.5)

        yaw_error = 0.0
        x_pid_error = 0.0
        y_pid_error = 0.0

        x_scale = x_pid_controller.calculate(end_position["x"], self.robot_position["x"])
        y_scale = y_pid_controller.calculate(end_position["y"], self.robot_position["y"])

        start_time = time.time()
        while not rospy.is_shutdown():
            
            if move_timeout > 0:
                if (time.time() - start_time) >= move_timeout:
                    break
            
            x_pid_error = end_position["x"] - self.robot_position["x"]
            y_pid_error = end_position["y"] - self.robot_position["y"]

            x_scale = x_pid_controller.calculate(end_position["x"], self.robot_position["x"])
            y_scale = y_pid_controller.calculate(end_position["y"], self.robot_position["y"])

            if (round(abs(y_pid_error), precision) == 0):
                break

            if should_avoid_obstacles:
                access_key = ""
                if self.get_sign(y_scale) >= 0:
                    access_key = "front"
                else:
                    access_key = "back"

                distance_remaining = y_pid_controller.pid_impl.pre_error * displacement
                if not self.ensure_clear_path(access_key=access_key, collision_range=min(collision_distance, abs(distance_remaining)), timeout=sensor_timeout):
                    status = False

                    rospy.loginfo("Timed out while waiting for obstacle")
                    break

            linear_velocity  = y_scale * target_speed
            angular_velocity = ((x_scale * target_speed) / self.base_width) * self.get_sign(y_scale)

            self.set_motion(linear_velocity, -angular_velocity)
            rospy.sleep(0.1)

        yaw_error = end_position["yaw"] - self.robot_position["yaw"]

        rospy.loginfo("yaw_error: {}".format(yaw_error))
        rospy.loginfo("x_pid_error: {}".format(x_pid_error))
        rospy.loginfo("y_pid_error: {}".format(y_pid_error))
        rospy.loginfo("")

        self.set_motion(0.0, 0.0); rospy.sleep(0.5)
        return status
    
    def move_angular(self, angular_displacement, target_speed=2.0, precision=2, should_avoid_obstacles=True, collision_distance=0.22, move_timeout=10, sensor_timeout=10):

        rospy.sleep(0.5); self.set_motion(0.0, 0.0)

        target_speed = abs(target_speed)
        status = False

        current_position = copy.deepcopy(self.robot_position)
        end_position = copy.deepcopy(current_position)
        end_position["yaw"] = end_position["yaw"] + angular_displacement

        yaw_pid_controller = PID(-1.0, 1.0, 1.0, 0, 0)
        yaw_pid_error = 0.0

        start_time = time.time()
        while not rospy.is_shutdown():
            
            if move_timeout > 0:
                if (time.time() - start_time) >= move_timeout:
                    break
            
            yaw_pid_error = end_position["yaw"] - self.robot_position["yaw"]
            if (round(abs(yaw_pid_error), precision) == 0):
                break

            yaw_scale = yaw_pid_controller.calculate(end_position["yaw"], self.robot_position["yaw"])

            linear_velocity  = 0.0
            angular_velocity = yaw_scale * target_speed

            self.set_motion(0, angular_velocity)

            
            rospy.sleep(0.1)

        yaw_error = end_position["yaw"] - self.robot_position["yaw"]

        rospy.loginfo("yaw_error: {}".format(yaw_error))
        rospy.loginfo("")

        self.set_motion(0.0, 0.0); rospy.sleep(0.5)
        return status
    
if __name__ == "__main__":
    rospy.init_node("py_robot_interface", anonymous=False)
    robot = AdvancedRobotInterface()

    rospy.loginfo("initializing the robot")
    robot.initialize()

    rospy.loginfo("waiting for pull to start")
    robot.wait_for_pull_to_start()

    # rospy.loginfo("moving forward")
    # robot.move_linear(1.0)

    # rospy.loginfo("moving backward")
    # robot.move_linear(-1.0)

    rospy.loginfo("rotating right")
    robot.move_angular(angular_displacement=1.5)

    rospy.loginfo("rotating left")
    robot.move_angular(angular_displacement=-1.5)


    rospy.loginfo("ctrl-c to terminate")
    rospy.spin()

    rospy.loginfo("terminating....")
    robot.terminate()