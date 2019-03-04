#!/usr/bin/env python

'''
Script/Node that publishes tf of the sensor

'''

__author__ = "Marlon Gwira"
__copyright__ = "Copyright 2019, Eurobot Project"
__credits__ = ["Chib .etc"]
__license__ = "GPL"
__version__ = "0.0.1"
__maintaner__= "Chibuike"
__email__ = ""
__status__= "Development"

import roslib
#roslib.load_manifest('tf_broadcaster')

import rospy
import tf




if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2.0, 2.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "world",
                         "base_link")
        rate.sleep()





