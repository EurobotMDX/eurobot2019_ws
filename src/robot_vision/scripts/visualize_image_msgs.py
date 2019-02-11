#!/usr/bin/env python
from __future__ import division, print_function

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vector_illustration_processing import pi_video

ros_image_msg_names = [
    "test_image",
    "test_image"
]

class RosImageBuffer(object):
    def __init__(self, msg_name="test_image"):
        self.msg_name = msg_name
        self.cv_bridge = CvBridge()

        self.frame = None

        self.subscriber = rospy.Subscriber(self.msg_name, Image, self.callback)
    
    def callback(self, msg):
        cv_image = None
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo("[INFO] error reading cv image")
        
        if pi_video.is_cv_image_valid(cv_image):
            self.frame = cv_image
    
    def get_frame(self):
        return self.frame

class RosImageViewer(object):
    def __init__(self, title="robot vision", fps=15):
        self.ros_image_msg_names = []
        self.ros_image_subscribers = []
        
        self.frame = None
        self.viewer = pi_video.Viewer(fps=fps, title=title)
        self.viewer.start() 
    
    def initialize(self, ros_image_msg_names):
        self.ros_image_msg_names = ros_image_msg_names

        for msg_name in self.ros_image_msg_names:
            self.ros_image_subscribers.append(
                RosImageBuffer(msg_name=msg_name)
            )
    
    def terminate(self):
        rospy.loginfo("[INFO] terminating image viewer")
        self.viewer.stop()
    
    def run(self):
        while not rospy.is_shutdown():

            frame = None

            for ros_image_sub in self.ros_image_subscribers:
                if not pi_video.is_cv_image_valid(frame):
                    frame = ros_image_sub.get_frame()
                else:
                    frame = pi_video.combine_images(frame, ros_image_sub.get_frame())
            
            if pi_video.is_cv_image_valid(frame):
                self.frame = frame
            
            if pi_video.is_cv_image_valid(self.frame):
                if self.viewer.show_frame(self.frame) in [ord('q'), 27]:
                    self.terminate()
                    break
            
            if rospy.is_shutdown():
                self.terminate()
                break

def __log(status="INFO", message=""):
    rospy.loginfo("[{status}] {message}".format(status=status, message=message))

def main():
    NODE_NAME = "robot_vision_viewer"

    # initialize this node
    rospy.init_node("{}".format(NODE_NAME), anonymous=False)

    # print status
    __log("{} node has started".format(NODE_NAME))

    image_viewer = RosImageViewer()

    def shutdown():
        __log("terminating {} node".format(NODE_NAME))
        
        image_viewer.terminate()

    # when ros terminates
    rospy.on_shutdown(shutdown)

    image_viewer.initialize(ros_image_msg_names)
    image_viewer.run()

if __name__ == "__main__":
    main()