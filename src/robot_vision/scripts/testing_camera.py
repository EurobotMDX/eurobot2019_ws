#!/usr/bin/env python
from __future__ import division, print_function

import rospy
from cv_bridge import CvBridge, CvBridgeError
from vector_illustration_processing import pi_video

class CameraTest(object):
    def __init__(self):
        self.camera_id = 0
        self.camera = None

        self.cv_bridge = CvBridge()
    
    def initialize(self, camera_id):
        self.camera_id = camera_id
        self.camera = pi_video.Camera(self.camera_id)

        # start the camera
        return self.camera.start()
    
    def terminate(self):
        if self.camera is not None:
            self.camera.stop()
    
    def run(self, ros_publisher, fps=5):
        
        rate = rospy.Rate(fps)
        while not rospy.is_shutdown():

            # read frame from the camera
            frame = self.camera.get_next_frame()

            # validate the frame
            if not pi_video.is_cv_image_valid(frame):
                continue

            # convert cv image to ros image
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
            
            # publish ros image
            ros_publisher.publish(ros_image)

            # maintains frame rate
            rate.sleep()