#!/usr/bin/env python

from __future__ import division
import rospy
import sys
import odroid_wiringpi
import signal
import thread
from decimal import *
from robot_interface_advanced import AdvancedRobotInterface
from std_msgs.msg import String
import time


# ========================================================== Notes: ========================================================================================== #
# This is Python code for implementation of the line following with the help of the QTR-8RC sensor array (This is not the final code - Work is still in Progress)
# This code makes reading data from the Pololu QTR-8RC IR sensor array possible and with all this data robot will be following any line.
#
#   Here's the documentation about the sensor array that we're using:
#   https://www.pololu.com/docs/pdf/0J12/QTR-8x.pdf
#
#   We're using GPIO pins on the Odroid XU4 mainboard to get data from the Polulu QTR-8RC sensor array
#   Pins within CON10 on Odroid XU4 to which sensor outputs from the Pololu QTR-8RC array sensor will be connected:
#   LEDON pin will be connected to GPIO 7 /EmitterPin
#   IR LED/phototransitors will be connected to GPIO Pins [2, 3, 4, 5, 6, 21, 22, 23]

class qtr_8rc:
    # Class that makes reading values from Pololu QT8-8RC sensor array possible
    # Requires wiringpi https://github.com/WiringPi/WiringPi-Python

    def __init__(self):
        # This function initialises class constants + variables and defines
        # pins through which data from sensors will be received
        self.wp = odroid_wiringpi
        self.wp.wiringPiSetup()

        self.LED_ON_PIN = 7  # This is the emitter pin
        self.SENSOR_PINS = [2, 3, 21, 22, 23, 4, 5, 6]
        self.NUM_SENSORS = len(self.SENSOR_PINS)
        self.CHARGE_TIME = 10  # This command will charge the capacitors located on the QTR-8 RC ArraySensor for 10s This is the delay
        self.READING_TIMEOUT = 1000  # This assumes that reading from the array sensor is black
        self.READINGS_PER_SECOND = 5

        self.sensorValues = []
        self.calibratedMax = []
        self.calibratedMin = []
        self.lastValue = 0
        self.initialise_pins()

    def initialise_pins(self):
        # This function sets up GPIO pins and it also ensures the correct number of elements
        # in the sensor values array and calibration lists to store readings from sensors.

        for pin in self.SENSOR_PINS:
            self.sensorValues.append(0)
            self.calibratedMax.append(0)
            self.calibratedMin.append(0)
            self.wp.pullUpDnControl(pin, self.wp.PUD_DOWN)
        self.wp.pinMode(self.LED_ON_PIN, self.wp.OUTPUT)

    def read_sensors(self):
        # This function follows the Pololu guidance for reading data from the capacitor/sensor discharge:
        # 1. It sets the I/O line to an output and drive it reasonably high.
        # 2. Provides a delay for the sensor output to rise.
        # 3. Make the I/O line an input (high impedance).
        # 4. Measure the time for the voltage to decay by waiting for the I/O line to go low.
        # After finishing all the steps it values from the sensors will be stored in the sensor values list.
        # Higher values will mean darker surfaces.

        for i in range(0, self.NUM_SENSORS):
            self.sensorValues[i] = self.READING_TIMEOUT

        for sensorPin in self.SENSOR_PINS:
            self.wp.pinMode(sensorPin, self.wp.OUTPUT)
            self.wp.digitalWrite(sensorPin, self.wp.HIGH)

        self.wp.delayMicroseconds(self.CHARGE_TIME)

        for sensorPin in self.SENSOR_PINS:
            self.wp.pinMode(sensorPin, self.wp.INPUT)
            # We should ensure that pins are pulled down
            self.wp.digitalWrite(sensorPin, self.wp.LOW)

            startTime = self.wp.micros()
            while self.wp.micros() - startTime < self.READING_TIMEOUT:
                time = self.wp.micros() - startTime
                for i in range(0, self.NUM_SENSORS):
                   if self.wp.digitalRead(self.SENSOR_PINS[i]) == 0 and time < self.sensorValues[i]:
                    self.sensorValues[i] = time

    def print_sensor_values(self, values):
        # This function will output sensor number and it's current recorded sensor value before calibration of the sensors
        for i in range(0, 10):
            self.emitters_on()
            self.read_sensors()
            # self.read_line()
            self.emitters_off()
            print(self.sensorValues)
            time.sleep(0.1)
            #   self.read_sensors()
            #  for i in range(0, len(self.sensorValues)):
            #     print("Sensor{number} Readings:{sensrval1} {sensrval2} {sensrval3} {sensrval4} {sensrval5} {sensrval6} {sensrval7}".format
            #        (number= i, sensrval1=self.sensorValues[i], sensrval2=self.sensorValues[i], sensrval3=self.sensorValues[i],
            #            sensrval4=self.sensorValues[i], sensrval5=self.sensorValues[i], sensrval6=self.sensorValues[i],
            #           sensrval7=self.sensorValues[i]))

    #   print("Sensor:{sensorNo} Reading {valuesf}".format(sensorNo=i, valuesf=values[i]))

    def emitters_on(self):
        # When we call this function LEDON pin (GPIO 7), so that IR LEDs on the array sensor can be turned on.
        # If there is nothing wired to LEDON emitters will always stay on.
        self.wp.digitalWrite(self.LED_ON_PIN, self.wp.HIGH)
        self.wp.delayMicroseconds(20)

    def emitters_off(self):
        # Turns the LEDON pin off so that the IR LEDs can be turned off.
        # If there is nothing wired to LEDON emitters will always be on.
        self.wp.digitalWrite(self.LED_ON_PIN, self.wp.LOW)
        self.wp.delayMicroseconds(20)

    # We can use those two functions above (emiters_on and emiters_off)
    # in order to conserve power consumption on the Sensor_Array


qtr = qtr_8rc()
# while qtr.calibratedMax != 0:
if __name__ == "__main__":
    qtr = qtr_8rc()
    wp = odroid_wiringpi
    # Initialisation of the robot and waiting for pull to start (to be added later)
    print("Team Brainstorm WorldSkills 2019 Robot Line Following Control Centre Developed by Szymon Sebastian Malecki")
    rospy.init_node("eurobot_task_handler", anonymous=False)
    rospy.loginfo("initializing the robot...")
    robot = AdvancedRobotInterface()
    robot.initialize()
    # rospy.loginfo("waiting for pull to start")
    # robot.wait_for_pull_to_start(state=True)

    qtr.emitters_on()
    qtr.read_sensors()
    qtr.emitters_off()
    print("Here're the current values from the sensor before calibration:")
    #  print(qtr.print_sensor_values(qtr.sensorValues))
    print(qtr.print_sensor_values(qtr.sensorValues))

    rospy.loginfo("ctrl-c to terminate")
    qtr.emitters_off()
    rospy.spin()
    rospy.loginfo("terminating....")
    robot.terminate()

"""
                        qtr.emitters_on()
                        print(qtr.read_line())
                        qtr.emitters_off()

                        qtr.emitters_on()
                        qtr.read_sensors()
                        qtr.emitters_off()
                        qtr.print_sensor_values(qtr.sensorValues)
                       # qtr.infinitedisplay(qtr.sensorValues)
                        print(" ")
"""