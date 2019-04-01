#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from server import *

def __log(status="INFO", message=""):
    rospy.loginfo("[{status}] {message}".format(status=status, message=message))

__log("initializing node {}".format(NODE_NAME))

rospy.init_node("{}".format(NODE_NAME), anonymous=False)
rospy.on_shutdown(shutdown_server)

#["{s,0,0}{s,1,3.14}", "{s,0,1}{s,1,.8}", "{s,0,0}{s,1,3.14}"]
serial_data_pub = rospy.Publisher('serial_data_handler_msg', String, queue_size=10)


@app.route("/open_gripper")
def open_gripper():
    msg = String()
    msg.data = "{s,0,0}{s,1,3.14}"
    serial_data_pub.publish(msg)
    return "Gripper Opened"

@app.route("/close_gripper")
def close_gripper():
    msg = String()
    msg.data = "{s,0,1}{s,1,.8}"
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

