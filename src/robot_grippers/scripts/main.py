#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class RobotGripper(object):
    def __init__(self):
        self.ARM_GRIPPER_ID = 0
        self.LEFT_GRIPPER_ID = 0
        self.RIGHT_GRIPPER_ID = 0

        self.data_sender = rospy.Publisher('serial_data_handler_msg', String, queue_size=10)

    def initialize(self):
        self.reset()
    
    def terminate(self):
        pass
    
    def close_gripper(self):
        pass
    
    def open_gripper(self):
        pass
    
    # def extend_gripper(self):
    #     pass
    #
    # def retract_gripper(self):
    #     pass
    #
    # def push_left(self):
    #     pass
    #
    # def push_right(self):
    #     pass
    #
    def reset(self):
        pass
    
    def __send_command(self, command=""):
        if not isinstance(command, str):
            return False
        
        msg = String()
        msg.data = command
        self.data_sender.publish(msg)


def main():
    gripper_object = RobotGripper()
    gripper_object.initialize()

    # TODO: implement test/sample code
    

    gripper_object.terminate()


if __name__ == '__main__':
    main()