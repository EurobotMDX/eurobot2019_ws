#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

def init_serial():
    port_name=""
    try:
        with serial.Serial(port_name) as ser:
	   try:
	       ser.isOpen()
	       print('serial port is open')
	   except:
	       print('Error opening')
	       exit()
	   if(ser.isOpen()):
	       try:
	           while(1):
		       print(ser.readline())
	       except Exception:
	           print'error'
	   else:
	       print 'cannot open serial port'
	       x = ser.read(20)
	       print('data: '+ser.readline())
    except:
        print("check port by running: python -m serial.tools.list_ports")
	   

def getData():
    return ser.realine()


def serial_read():
    pub = rospy.Publisher('Pozyx_position', String, queue_size=10)
    rospy.init_node('Serial_pozyx_position', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(init_serial())
        rate.sleep()

if __name__ == '__main__':
    try:
        serial_read()
    except rospy.ROSInterruptException:
        pass













#class myNode:
#
#    def __init__(self, *args):
#        self.tf = TransformListener()
#        rospy.Subscriber(...)
#        ...

#    def some_method(self):
#        if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
 #           t = self.tf.getLatestCommonTime("/base_link", "/map")
#            position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
#            print position, quaternion
