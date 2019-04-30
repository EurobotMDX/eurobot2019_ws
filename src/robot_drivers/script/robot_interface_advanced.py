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
    
    def move_linear(self, displacement, target_speed=0.4, precision=3, should_avoid_obstacles=False, obstacle_backup_distance=0.12, clearing_distance=0.32, move_timeout=30, sensor_timeout=1000):
        
        status = False
        self.set_motion(0.0, 0.0)

        target_speed = abs(target_speed)
        target_angular_speed = copy.deepcopy(target_speed) / self.base_width

        start_position = copy.deepcopy(self.robot_position)
        end_position = copy.deepcopy(self.robot_position)

        target_heading = spf.convert_180_to_360_radians(end_position["yaw"])
        end_position["x"] += math.sin(target_heading) * displacement
        end_position["y"] += math.cos(target_heading) * displacement
        
        total_distance_to_travel = self.get_distance(start_position, end_position)
        
        yaw_pid_controller = PID(-1.0, 1.0, 1.2, 0, 0)
        distance_pid_controller = PID(-1.0, 1.0, 4.0, 0, 0.0)
        collision_avoidance_pid_controller = PID(0.0, 1.0, 10.0, 0, 0.0)

        access_key = "front"
        if self.get_sign(displacement) < 0:
            access_key = "back"

        def _f(x):
            return  round(x, 3)
        
        def get_heading_deviation():
            end_heading = spf.convert_180_to_360_radians(end_position["yaw"])
            current_heading = spf.convert_180_to_360_radians(self.robot_position["yaw"])

            deviation = spf.add_angles_in_180_mode_radians(target_heading, -current_heading)
            return deviation

        start_time = time.time()
        while not rospy.is_shutdown():
            
            if (move_timeout > 0) and ((time.time() - start_time) >= move_timeout):
                self.stop_motors(); rospy.loginfo("[INFO] move linear timed out")
                break

            scale = distance_pid_controller.calculate(total_distance_to_travel, self.get_distance(self.robot_position, start_position))
            yaw_scale = yaw_pid_controller.calculate(0, get_heading_deviation())

            distance_remaining = distance_pid_controller.pid_impl.pre_error * displacement
            distance_to_obstacle = self.proximity_sensors[access_key]

            speed_limiter_scale = collision_avoidance_pid_controller.calculate(min(obstacle_backup_distance, distance_remaining), distance_to_obstacle)
            rospy.loginfo("[INFO] speed_limiter_scale: {}".format(speed_limiter_scale))

            if round(abs(distance_pid_controller.pid_impl.pre_error), precision) == 0:
                self.stop_motors(); break

            if should_avoid_obstacles:
                if not self.ensure_clear_path(access_key=access_key, collision_range=min(clearing_distance, abs(distance_remaining)), timeout=sensor_timeout):
                    status = False

                    rospy.loginfo("Timed out while waiting for obstacle")
                    break

            linear_velocity  = scale * target_speed * self.get_sign(displacement)
            angular_velocity = yaw_scale * target_angular_speed

            self.set_motion(linear_velocity, angular_velocity)
        
        rospy.loginfo("heading deviation = {} deg".format(math.degrees(get_heading_deviation())))
        
        self.stop_motors(); rospy.sleep(0.1)
        return status
    
    def move_angular(self, angle_degrees, target_speed=1.5, precision=2, should_avoid_obstacles=False, clearing_distance=0.20, move_timeout=15, sensor_timeout=10):

        self.set_motion(0.0, 0.0)
        status = False

        angle_radians = math.radians(-angle_degrees)
        target_speed = abs(target_speed)
        
        start_position = copy.deepcopy(self.robot_position)
        end_position = copy.deepcopy(start_position)

        end_position["yaw"] = spf.add_angles_in_180_mode_radians(end_position["yaw"], angle_radians)
        total_angle_to_travel = copy.deepcopy(angle_radians)

        def _f(angle_rad):
            return round(math.degrees(spf.convert_180_to_360_radians(angle_rad)))

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
            scale = yaw_pid_controller.calculate(total_angle_to_travel, current_angle_travelled)

            if (round(abs(yaw_pid_controller.pid_impl.pre_error), precision) == 0):
                self.stop_motors()
                break

            linear_velocity  = 0.0
            angular_velocity = scale * target_speed

            self.set_motion(0, angular_velocity)
        self.stop_motors(); rospy.sleep(0.1)
        return status
    
    def get_average_distance(self, access_key="front", howmany_samples=5):
        distance = 0.0

        for i in range(howmany_samples):
            distance_to_obstacle = self.proximity_sensors[access_key]
            distance += distance_to_obstacle

            rospy.sleep(0.1)

        return (distance / float(howmany_samples))

    def move_to_surface(self, access_key="front", offset=0.005, target_speed=0.4, move_timeout=30):

        if access_key == "front":
            distance = self.get_average_distance(access_key) - offset
            rospy.loginfo("front distance: {}".format(distance))
            self.move_linear(distance, target_speed=target_speed, move_timeout=move_timeout)
        elif access_key == "back":
            distance = self.get_average_distance(access_key)
            rospy.loginfo("back distance: {}".format(distance))
            self.move_linear(-distance, target_speed=target_speed, move_timeout=move_timeout)
        
        return distance
    
    def move_to(self, x, y, final_yaw_heading_degrees, target_speed=0.4, should_avoid_obstacles=False):
        self.set_motion(0.0, 0.0)

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
        self.move_linear( self.get_distance( start_position, target_position ), target_speed=target_speed, should_avoid_obstacles=should_avoid_obstacles )

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

    rospy.loginfo("resetting the robot's odometry")
    robot.reset_odometry();

    def _f(x, y, yaw=0):
        rospy.loginfo("[INFO] moving to x:{}, y:{}, yaw:{}".format(x, y, yaw))
        robot.move_to(x, y, yaw)
        rospy.loginfo("done")
        rospy.sleep(2)

    robot.move_linear(1.0, should_avoid_obstacles=True)
    robot.move_linear(0.2, target_speed=0.15)
    
    robot.move_angular(90)
    robot.move_linear(0.2)

    robot.move_linear(-0.2)
    robot.move_angular(-90)

    robot.move_linear(-0.2, target_speed=0.15)
    robot.move_linear(-1.0, should_avoid_obstacles=True)

    rospy.loginfo("ctrl-c to terminate")
    rospy.spin()

    rospy.loginfo("terminating....")
    robot.terminate()



