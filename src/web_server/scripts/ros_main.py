#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import json
from server import *

def __log(status="INFO", message=""):
    rospy.loginfo("[{status}] {message}".format(status=status, message=message))

__log("initializing node {}".format(NODE_NAME))


rospy.init_node("{}".format(NODE_NAME), anonymous=False)
rospy.on_shutdown(shutdown_server)

robot_position = {
            "x"   : 0.0,
            "y"   : 0.0,
            "yaw" : 0.0
        }

def update_robot_position(odometry_msg):
    robot_pose = odometry_msg.pose.pose

    _, _, robot_position["yaw"] = euler_from_quaternion([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
    robot_position["x"] = robot_pose.position.x
    robot_position["y"] = robot_pose.position.y

#["{s,0,0}{s,1,3.14}", "{s,0,1}{s,1,.8}", "{s,0,0}{s,1,3.14}"]
serial_data_pub = rospy.Publisher('serial_data_handler_msg', String, queue_size=10)
reset_drive_train_pub = rospy.Publisher('/reset_drive_train', Bool, queue_size=10)

rospy.Subscriber("odom", Odometry, update_robot_position)

@app.route("/reset_odometry")
def reset_odometry():
    msg = Bool()
    msg.data = True
    reset_drive_train_pub.publish(msg)
    return "Odometry Reset"

@app.route("/get_robot_position")
def get_robot_position():
    return json.dumps(robot_position)

@app.route("/open_gripper")
def open_gripper():
    msg = String()
    msg.data = "{s,0,0.4}{s,1,1.4}" #"{s,0,0}{s,1,3.14}"
    serial_data_pub.publish(msg)
    return "Gripper Opened"

@app.route("/close_gripper")
def close_gripper():
    msg = String()
    msg.data = "{s,0,0.8}{s,1,1.0}" #"{s,0,1}{s,1,.8}"
    serial_data_pub.publish(msg)
    return "Gripper Closed"

@app.route("/push_left")
def push_left():
    msg = String()
    msg.data = "{s,2,2.2}"
    serial_data_pub.publish(msg)
    return "Pushing Left"

@app.route("/push_right")
def push_right():
    msg = String()
    msg.data = "{s,2,0.6}"
    serial_data_pub.publish(msg)
    return "Pushing Right"


__log("Starting web server")
thread.start_new_thread(app.run, (HOST, PORT))

time.sleep(5)
__log("Server should be running")

rospy.spin()

