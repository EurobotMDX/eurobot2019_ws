#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import serial

# DEFAULT_PORT = "/dev/serial/by-path/platform-12120000.usb-usb-0:1:1.0"
DEFAULT_PORT = "/dev/serial/by-path/platform-12110000.usb-usb-0:1.1:1.0"
# DEFAULT_PORT = "/dev/serial/by-path/platform-12110000.usb-usb-0:1.2:1.0"
DEFAULT_BAUD = 230400
DEFAULT_LOOP_RATE = 10

class SerialDataHandler(object):
    def __init__(self, default_port=DEFAULT_PORT, default_baud=DEFAULT_BAUD, default_loop_rate=DEFAULT_LOOP_RATE):
        self.port = self.default_port = default_port
        self.baud = self.default_baud = default_baud
        self.loop_rate = self.default_loop_rate = default_loop_rate

        self.rate = None
        self.serial_device = None

        self._data_buffer = ""

        self.range_data_pub = rospy.Publisher('raw_range_data', String, queue_size=10)
        self.msg_subscriber  = rospy.Subscriber('serial_data_handler_msg', String, self.msg_received)
        # self.compass_data_pub = rospy.Publisher('raw_compass_data', String, queue_size=10)
    
    def initialize(self):
        self.port = rospy.get_param('serial_port', self.default_port)
        self.baud = rospy.get_param('serial_baud', self.default_baud)
        self.loop_rate = rospy.get_param('loop_rate', self.default_loop_rate)
        
        if self.serial_device is None:
            try:
                self.serial_device = serial.Serial(self.port, self.baud, timeout=0.1)
            except:
                pass
        
        if self.serial_device is not None:
            rospy.init_node('serial_com_data_handler', anonymous=True)
            self.rate = rospy.Rate(self.loop_rate)
    
    def terminate(self):
        if self.serial_device is not None:
            self.serial_device.close()
    
    def msg_received(self, msg):
        if self.serial_device is None:
            rospy.loginfo("[INFO] Arduino is not connected")
            return

        rospy.loginfo("[INFO] serial manager is sending {} to the arduino",format(msg.data))
        self.serial_device.write(msg.data)

    
    def _read_data_buffer(self):
        if self.serial_device is None:
            return False

        self._data_buffer = self._data_buffer + self.serial_device.read(self.serial_device.in_waiting)

        return True
    
    def get_raw_range_msg_as_str(self):
        if self._data_buffer is None:
            return "[]"
        
        if not "]" in self._data_buffer: return "[]"
        raw_msg_data_as_str = self._data_buffer[:self._data_buffer.rfind(']')]

        if not "[" in raw_msg_data_as_str: return "[]"
        raw_msg_data_as_str = raw_msg_data_as_str[raw_msg_data_as_str.rfind('[') + 1:]

        self._data_buffer = ""
        return "[" + raw_msg_data_as_str + "]"
    
    def run(self):
        while not rospy.is_shutdown():
            self._read_data_buffer()
            
            msg_str = self.get_raw_range_msg_as_str()
            self.range_data_pub.publish(msg_str)
            self.rate.sleep()


def main():
    my_serial_manager = SerialDataHandler()
    my_serial_manager.initialize()
    
    try:
        my_serial_manager.run()
    except rospy.ROSInterruptException:
        pass

    my_serial_manager.terminate()

    

if __name__ == '__main__':
    main()
