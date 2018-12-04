#!/usr/bin/env python
import roslib; roslib.load_manifest('navigation')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import math

def callback(msg):
    #rospy.loginfo("Received a /cmd_vel message!")
    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands
	x = msg.linear.x
	z = msg.angular.z
    
	#double y = msg.getAngular().getZ();
    #if (Math.abs(x) < 0.02) x = 0;

	#if (abs(y) < 0.02):
	#	 y = 0
	
	#s = -1 if x < 0 else 1
	
	#speed = math.sqrt(x * x + y * y) / math.sqrt(2)

	lx = x * 635 
	lz = z * 70

	#twist = - 200 * y / speed
	speed1 = (lx + lz) / 10
	speed2 = (lx - lz) / 10

	rospy.loginfo("Speed1: %s Speed2: %s"% (speed1, speed2))
 
    #if (Math.abs(y) < 0.02) y = 0;
    #double s = x < 0 ? -1 : 1;
    #double speed = Math.sqrt(x * x + y * y) / Math.sqrt(2);
    #double twist = -200 * y / speed;
    
	#v_l = ...
    #v_r = ...

    # Then set your wheel speeds (using wheel_left and wheel_right as examples)
    #wheel_left.set_speed(v_l)
    #wheel_right.set_speed(v_r)

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/mobile_base/commands/velocity", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


