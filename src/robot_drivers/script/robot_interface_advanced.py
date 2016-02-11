#!/usr/bin/env python

from __future__ import division

import math
import time
import copy
import json
import rospy
from pid_controller import PID
from robot_interface import RobotInterfaceBase
import support_functions as spf

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
    
    def get_distance(self, position_1, position_2):
        return math.sqrt(
            (position_2["x"] - position_1["x"]) ** 2 + (position_2["y"] - position_1["y"]) ** 2
        )
    
    def get_heading(self, position_1, position_2):
        dx = position_2["x"] - position_1["x"]
        dy = position_2["y"] - position_1["y"]

        return math.atan2(dx, dy)
    
    def move_linear(self, displacement, target_speed=0.2, precision=2, should_avoid_obstacles=False, collision_distance=0.55, move_timeout=10, sensor_timeout=1000):
        rospy.sleep(0.5); self.set_motion(0.0, 0.0)

        target_speed = abs(target_speed)
        target_angular_speed = copy.deepcopy(target_speed) / self.base_width
        status = False

        start_position = copy.deepcopy(self.robot_position)
        end_position = copy.deepcopy(self.robot_position)

        robot_heading = spf.convert_180_to_360_radians(end_position["yaw"])
        end_position["x"] += math.sin(robot_heading) * displacement
        end_position["y"] += math.cos(robot_heading) * displacement
        
        total_distance_to_travel = self.get_distance(start_position, end_position)
        distance_pid_controller = PID(-1.0, 1.0, 10.0, 0, 1.5)
        yaw_pid_controller = PID(-1.0, 1.0, 10.0, 0, 0)

        def _f(x):
            return  round(x, 3)

        start_time = time.time()
        while not rospy.is_shutdown():
            
            if move_timeout > 0:
                if (time.time() - start_time) >= move_timeout:
                    self.stop_motors()
                    rospy.loginfo("[INFO] move linear timed out")
                    break

            current_distance = self.get_distance(self.robot_position, start_position)
            theta = self.get_heading(self.robot_position, end_position)

            scale = distance_pid_controller.calculate(total_distance_to_travel, current_distance)
            yaw_scale = yaw_pid_controller.calculate(end_position["yaw"], self.robot_position["yaw"])

            if (round(abs(distance_pid_controller.pid_impl.pre_error), precision) == 0):
                self.stop_motors()
                break

            if should_avoid_obstacles:
                access_key = ""
                if self.get_sign(displacement) >= 0:
                    access_key = "front"
                else:
                    access_key = "back"

                distance_remaining = distance_pid_controller.pid_impl.pre_error * displacement
                if not self.ensure_clear_path(access_key=access_key, collision_range=min(collision_distance, abs(distance_remaining)), timeout=sensor_timeout):
                    status = False

                    rospy.loginfo("Timed out while waiting for obstacle")
                    break

            # rospy.loginfo("[INFO] current_distance = {}".format(current_distance))
            # rospy.loginfo("[INFO] current_distance = {}".format(current_distance))
            # rospy.loginfo("[INFO] scale = {}\n".format(scale))
            
            

            linear_velocity  = scale * target_speed * self.get_sign(displacement)
            # angular_velocity = yaw_scale * (linear_velocity / self.base_width)
            angular_velocity = 0.0 #(x_scale * target_angular_speed) * self.get_sign(y_scale)

            # rospy.loginfo("[INFO] r = {}".format(r))
            # rospy.loginfo("[INFO] robot_heading = {}".format(math.degrees(robot_heading)))
            # rospy.loginfo("[INFO] yaw_scale = {}".format(yaw_scale))
            # rospy.loginfo("[INFO] current heading = {}".format(math.degrees(self.robot_position["yaw"])))
            # rospy.loginfo("[INFO] angular_velocity = {}\n".format(angular_velocity))

            self.set_motion(linear_velocity, angular_velocity)
            rospy.sleep(0.1)

        self.stop_motors(); rospy.sleep(0.5)
        return status
    
    def move_angular(self, angle_degrees, target_speed=2.0, precision=1, should_avoid_obstacles=False, collision_distance=0.20, move_timeout=15, sensor_timeout=10):

        rospy.sleep(0.5); self.set_motion(0.0, 0.0)
        status = False

        angle_radians = math.radians(-angle_degrees)
        target_speed = abs(target_speed)
        
        start_position = copy.deepcopy(self.robot_position)
        end_position = copy.deepcopy(start_position)

        end_position["yaw"] = spf.add_angles_in_180_mode_radians(end_position["yaw"], angle_radians)
        total_angle_to_travel = copy.deepcopy(angle_radians)

        def _f(angle_rad):
            return round(math.degrees(spf.convert_180_to_360_radians(angle_rad)))

        # rospy.loginfo("[INFO] start angle: {} deg".format(_f(start_position["yaw"])))
        # rospy.loginfo("[INFO] destination angle: {} deg".format(_f(end_position["yaw"])))
        # rospy.loginfo("[INFO] angle to travel: {} deg".format(_f(total_angle_to_travel)))

        yaw_pid_controller = PID(-1.0, 1.0, 1.0, 0, 0)
        yaw_pid_error = 0.0

        def _fr(x):
            return  round(x, 3)

        start_time = time.time()
        while not rospy.is_shutdown():
            
            if move_timeout > 0:
                if (time.time() - start_time) >= move_timeout:
                    self.stop_motors()
                    rospy.loginfo("[INFO] move angular timed out")
                    break

            current_angle_travelled = spf.add_angles_in_180_mode_radians(start_position["yaw"], -self.robot_position["yaw"])
            # rospy.loginfo("[INFO] current angle travelled: {}".format(_f(current_angle_travelled)))
            scale = yaw_pid_controller.calculate(total_angle_to_travel, current_angle_travelled)

            if (round(abs(yaw_pid_controller.pid_impl.pre_error), precision) == 0):
                self.stop_motors()
                break

            linear_velocity  = 0.0
            angular_velocity = scale * target_speed

            self.set_motion(0, angular_velocity)
            rospy.sleep(0.1)

        self.stop_motors(); rospy.sleep(0.5)
        return status
    
    def get_average_distance(self, access_key="front", howmany_samples=5):
        distance = 0.0

        for i in range(howmany_samples):
            distance_to_obstacle = self.proximity_sensors[access_key]
            distance += distance_to_obstacle

            rospy.sleep(0.1)

        return (distance / float(howmany_samples))

    def move_to_surface(self, access_key="front", offset=0.005):

        if access_key == "front":
            distance = self.get_average_distance(access_key) - offset
            rospy.loginfo("front distance: {}".format(distance))
            self.move_linear(distance)
        elif access_key == "back":
            distance = self.get_average_distance(access_key)
            rospy.loginfo("back distance: {}".format(distance))
            self.move_linear(-distance)
        
        return distance
    
    def move_to(self, x, y, final_yaw_heading_degrees, target_speed=0.2):
        rospy.sleep(0.5); self.set_motion(0.0, 0.0)

        target_position = {
            "x" : x,
            "y" : y,
            "yaw" : math.radians(final_yaw_heading_degrees),
        }
        start_position = copy.deepcopy( self.robot_position )

        dy = target_position["y"] - start_position["y"]
        dx = target_position["x"] - start_position["x"]

        theta = math.atan2(dx, dy)
        angle = math.degrees(spf.add_angles_in_180_mode_radians(theta, -self.robot_position["yaw"]))

        self.move_angular(angle)
        self.move_linear( self.get_distance( start_position, target_position ), target_speed=target_speed )

        angle = math.degrees( spf.add_angles_in_180_mode_radians(target_position["yaw"], -self.robot_position["yaw"]))
        self.move_angular(angle)

    def move_waypoints(self, way_points):
        for way_point in way_points:
            self.move_to(way_point["x"], way_point["y"], way_point["yaw"])

    
if __name__ == "__main__":
    rospy.init_node("py_robot_interface", anonymous=False)
    robot = AdvancedRobotInterface()

    rospy.loginfo("initializing the robot")
    robot.initialize()

    # rospy.loginfo("reseting robot odometry")
    # robot.reset_odometry()
    # rospy.sleep(5)
    # rospy.loginfo("done")

    # rospy.loginfo("waiting for pull to start")
    # robot.wait_for_pull_to_start()

    # robot.move_to(0.37, 2.777, 72.1)
    # rospy.sleep(5)

    # robot.move_to(1.5773, 2.844, 89.8)
    # rospy.sleep(5)

    # robot.move_to(0.37, 2.777, 72.1)
    # rospy.sleep(5)

    # robot.move_to(0, 0, 0)
    # rospy.sleep(5)

    def _f(x, y, yaw=0):
        rospy.loginfo("[INFO] moving to x:{}, y:{}, yaw:{}".format(x, y, yaw))
        robot.move_to(x, y, yaw)
        rospy.loginfo("done")
        rospy.sleep(2)

    _f(0, 1.3909, 90.0)
    robot.move_linear(-0.63948)
    _f(0.0, 0.12975, 0.0)

    # robot.move_linear(1.0)
    
    rospy.loginfo("ctrl-c to terminate")
    rospy.spin()

    rospy.loginfo("terminating....")
    robot.terminate()



