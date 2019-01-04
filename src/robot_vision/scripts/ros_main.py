#!/usr/bin/env python
from __future__ import division, print_function

import rospy
from sensor_msgs.msg import Image
from testing_camera import CameraTest

NODE_NAME = "robot_vision"

def __log(status="INFO", message=""):
    rospy.loginfo("[{status}] {message}".format(status=status, message=message))


def main():
    # initialize this node
    rospy.init_node("{}".format(NODE_NAME), anonymous=False)

    # print status
    __log("{} node has started".format(NODE_NAME))

    camera_test_object = CameraTest()

    def shutdown():
        __log("terminating {} node".format(NODE_NAME))
        
        camera_test_object.terminate()

    # when ros terminates
    rospy.on_shutdown(shutdown)

    # create an image publisher
    my_image_publisher = rospy.Publisher("test_image", Image, queue_size=2)

    if camera_test_object.initialize(0):
        camera_test_object.run(my_image_publisher)
        rospy.spin()
    else:
        __log("could not access your camera, please check the given id")
        shutdown()

if __name__ == '__main__':
    main()