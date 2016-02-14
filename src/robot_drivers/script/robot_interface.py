#!/usr/bin/env python

from __future__ import division

import time
import rospy
import urllib2
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import copy

from tf.transformations import euler_from_quaternion, quaternion_from_euler

EXPERIMENT_ID = 104

class RobotInterfaceBase(object):
    def __init__(self):
        self.proximity_sensors = {
            "front" : 0.0,
            "back" : 0.0,
            "left" : 0.0,
            "right" : 0.0
        }

        self.robot_position = {
            "x" : 0.0,
            "y" : 0.0,
            "yaw" : 0.0
        }

        self.robot_score = 0.0

        self.base_width = 0.2214 #0.1714
        self.smooting_val = 10

        self.pull_to_start_state = None
        # self.experiment_activate_url = "http://192.168.100.{}/activate"
        # self.experiment_deactivate_url = "http://192.168.100.{}/deactivate"
    
    def initialize(self):
        rospy.Subscriber("odom", Odometry, self.update_robot_position)

        rospy.Subscriber("/front_ultrasonic_data", Range, self.proximity_sensor_front_callback)
        rospy.Subscriber("/back_ultrasonic_data", Range, self.proximity_sensor_back_callback)
        rospy.Subscriber("/left_ultrasonic_data", Range, self.proximity_sensor_left_callback)
        rospy.Subscriber("/right_ultrasonic_data", Range, self.proximity_sensor_right_callback)
        
        rospy.Subscriber('pull_to_start', Bool, self.pull_to_start_callback)

        self.serial_data_publisher = rospy.Publisher('serial_data_handler_msg', String, queue_size=10)
        self.motion_data_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.reset_odometry_publisher = rospy.Publisher('/reset_drive_train', Bool, queue_size=10)
        self.robot_score_publisher = rospy.Publisher('/robot_score', Float32, queue_size=10)

        self.robot_score = 0.0

    def terminate(self):
        rospy.loginfo("terminating robot_interface")
        self.set_motion(0.0, 0.0)
    
    def reset_odometry(self):
        msg = Bool()
        msg.data = True
        self.reset_odometry_publisher.publish(msg)
        rospy.sleep(5)
    
    def pull_to_start_callback(self, msg):
        new_state = msg.data

        if (new_state != self.pull_to_start_state):
            self.pull_to_start_state = new_state

            if new_state == True:
                rospy.loginfo("activating the experiment - nothing happened")
                # self.activate_experiment()
            else:
                rospy.loginfo("deactivating the experiment -nothing happened")
                # self.deactivate_experiment()
    
    def wait_for_pull_to_start(self, timeout=-1, state=True):

        start_time = time.time()
        while not rospy.is_shutdown():

            if self.pull_to_start_state is not None:
                if self.pull_to_start_state == state:
                    break
            
            if timeout > 0:
                if (time.time() - start_time) > timeout:
                    rospy.loginfo("Timed out while waiting for pull_to_start")
                    break

            rospy.loginfo("Waiting for pull to start")
            rospy.sleep(0.3)
    
    def update_robot_score(self, score):
        self.robot_score += score

        msg = Float32()
        msg.data = self.robot_score

        self.robot_score_publisher.publish(msg)

    def proximity_sensor_front_callback(self, range_msg):
        self.proximity_sensors["front"] = self.smoothing(self.proximity_sensors["front"], range_msg.range, self.smooting_val)

        # rospy.loginfo("{}, {}".format(round(range_msg.range, 3), round(self.proximity_sensors["front"], 3)))
    
    def proximity_sensor_back_callback(self, range_msg):
        self.proximity_sensors["back"] = self.smoothing(self.proximity_sensors["back"], range_msg.range, self.smooting_val)

    def proximity_sensor_left_callback(self, range_msg):
        self.proximity_sensors["left"] = self.smoothing(self.proximity_sensors["left"], range_msg.range, self.smooting_val)

    def proximity_sensor_right_callback(self, range_msg):
        self.proximity_sensors["right"] = self.smoothing(self.proximity_sensors["right"], range_msg.range, self.smooting_val)
    
    def update_robot_position(self, odometry_msg):
        robot_pose = odometry_msg.pose.pose
        
        _, _, self.robot_position["yaw"] = copy.deepcopy(euler_from_quaternion([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w]))
        self.robot_position["x"] = copy.deepcopy(robot_pose.position.x)
        self.robot_position["y"] = copy.deepcopy(robot_pose.position.y)

        # rospy.loginfo(self.robot_position)


    def close_gripper(self):
        return self.publish_string("{s,0,0.8}{s,1,1.98}", self.serial_data_publisher)


    def open_gripper(self):
        return self.publish_string("{s,0,1.98}{s,1,0.8}", self.serial_data_publisher)

    # Grippers for world skills:

    def close_gripper_left(self):
        return self.publish_string("{s,0,0.33}", self.serial_data_publisher)

    def open_gripper_left(self):
        return self.publish_string("{s,0,1.98}", self.serial_data_publisher)

    def close_gripper_right(self):
        return self.publish_string("{s,1,2.25}", self.serial_data_publisher)

    def open_gripper_right(self):
        return self.publish_string("{s,1,0.0.8}", self.serial_data_publisher)

    # def push_left(self):
    #     return self.publish_string("{s,2,0.0}", self.serial_data_publisher)
    #
    # def push_right(self):
    #     return self.publish_string("{s,2,1.3}", self.serial_data_publisher)
    #
    # def activate_experiment(self, address=EXPERIMENT_ID):
    #     try:
    #         urllib2.urlopen(self.experiment_activate_url.format(address))
    #     except IOError, urllib2.HTTPError:
    #         pass
    #
    # def deactivate_experiment(self, address=EXPERIMENT_ID):
    #     try:
    #         urllib2.urlopen(self.experiment_deactivate_url.format(address))
    #     except IOError, urllib2.HTTPError:
    #         pass
    
    def set_motion(self, linear_velocity=0, angular_velocity=0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.motion_data_publisher.publish(msg)
    
    def stop_motors(self):
        return self.set_motion(0,0)
    
    def publish_string(self, string, publisher):
        msg = String()
        msg.data = string
        publisher.publish(msg)
    
    def smoothing(self, current_value, previous_value, scale):
        return (current_value + (previous_value * scale)) / (scale + 1.0)


if __name__ == "__main__":
    import time

    rospy.init_node("py_robot_interface", anonymous=False)
    robot = RobotInterfaceBase()

    robot.initialize()
    # robot.wait_for_pull_to_start()

    # rospy.loginfo("Testing Pusher")
    # robot.push_left()
    # robot.push_right(); time.sleep(2)
    # robot.push_left()

    rospy.loginfo("Testing Grippers")
    robot.open_gripper()
    robot.close_gripper()
    time.sleep(2)
    robot.open_gripper()
    robot.close_gripper()

    time.sleep(2)
    robot.semi_open_gripper()
    robot.semi_close_gripper()


    rospy.loginfo("Moving Forward")
    robot.set_motion(0.1, 0.0); time.sleep(2)
    robot.set_motion(0.0, 0.0); time.sleep(2)

    rospy.loginfo("Moving Backward")
    robot.set_motion(-0.1, 0.0); time.sleep(2)
    robot.set_motion(0.0, 0.0); time.sleep(2)

    rospy.loginfo("Turning Left")
    robot.set_motion(0.0, -1.5); time.sleep(2)
    robot.set_motion(0.0, 0.0); time.sleep(2)

    rospy.loginfo("Turning Right")
    robot.set_motion(0.0, 1.5); time.sleep(2)
    robot.set_motion(0.0, 0.0); time.sleep(2)

    robot.terminate()
