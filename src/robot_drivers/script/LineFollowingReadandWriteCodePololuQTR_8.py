#!/usr/bin/env python

from __future__ import division
import rospy
import odroid_wiringpi
import signal
import thread
from decimal import *
from robot_interface_advanced import AdvancedRobotInterface
from std_msgs.msg import String
from time import sleep

# ========================================================== Notes: ========================================================================================== #
# This is Python code for implementation of the line following with the help of the QTR-8RC sensor array (This is not the final code - Work is still in Progress)
# This code makes reading data from the Pololu QTR-8RC IR sensor array possible and with all this data robot will be following any line.
#
#   Here's the documentation about the sensor array that we're using:
#   https://www.pololu.com/docs/pdf/0J12/QTR-8x.pdf
#
#   We're using GPIO pins on the Odroid XU4 mainboard to get data from the Polulu QTR-8RC sensor array
#   Pins within CON10 on Odroid XU4 to which sensor outputs from the Pololu QTR-8RC array sensor will be connected:
#   LEDON pin will be connected to GPIO 7
#   IR LED/phototransitors will be connected to GPIO Pins [2, 3, 4, 5, 6, 21, 22, 23]

class qtr_8rc:
    #Initialisation of the robot and waiting for pull to start (to be added later)
    rospy.init_node("eurobot_task_handler", anonymous=False)
    rospy.loginfo("initializing the robot...")
    robot = AdvancedRobotInterface()
    robot.initialize()

   # rospy.loginfo("waiting for pull to start")
   # robot.wait_for_pull_to_start(state=True)

    # Class that makes reading values from Pololu QT8-8RC sensor array possible
    # Requires wiringpi https://github.com/WiringPi/WiringPi-Python

    def __init__(self):
# This function initialises class constants + variables and defines
# pins through which data from sensors will be received
        self.wp = odroid_wiringpi
        self.wp.wiringPiSetupGpio()

        self.LED_ON_PIN = 7
       # self.SENSOR_PINS = [2, 3, 21, 22, 23, 27]
        self.SENSOR_PINS= [2, 3, 4, 5, 6, 21, 22, 23]
        self.NUM_SENSORS = len(self.SENSOR_PINS)
        self.CHARGE_TIME = 10 # This command will charge the capacitors located on the QTR-8 RC ArraySensor for 10s
        self.READING_TIMEOUT = 1000  # This assumes that reading from the array sensor is black

        self.sensorValues = []
        self.calibratedMax = []
        self.calibratedMin = []
        self.lastValue = 0
        self.initialise_pins()

    def initialise_pins(self):
       #This function sets up GPIO pins and it also ensures the correct number of elements
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
      # This function will output sensor number and it's current
      # recorded sensor value before calibration of the sensors

        for i in range(0, self.NUM_SENSORS):
            print("Sensor %d, Reading %d" % (i, values[i]))

    def calibrate_sensors(self):
        # Takes readings across all sensors and sets max and min value readings
        # typical use of this function is to call it several times with delay such that a total of x seconds pass.
        # (e.g. 100 calls, with 20ms delays = 2 seconds for calibration).
        # When running this function it's advised to move the sensor
        # over the line several times in order calibrate contrasting surface.

        for j in range(0, 10):
            self.read_sensors()
            for i in range(0, self.NUM_SENSORS):
                if self.calibratedMax[i] < self.sensorValues[i]:
                   self.calibratedMax[i] = self.sensorValues[i]
                if self.calibratedMin[i] > self.sensorValues[i] and self.sensorValues[i] > 30:
                   self.calibratedMin[i] = self.sensorValues[i]

    def read_line(self):
        # This function reads values from all calibrated sensors on a line.
        # The values range from 0 - 7000, values == 0 and values == 7000 will mean that sensors are not on line or
        # may be outside the line to the extent that they don't detect it or only detect it from the left or right.
        # Values in the range of 0 - 7000 refer to the position of sensor, 3500 referring to centre,
        # lower val to the right and higher to the left (if pins were set up correctly in init).

        self.read_calibrated()
        avg = 0
        sum = 0
        on_line = False

        for i in range(0, self.NUM_SENSORS):
            val = self.sensorValues[i]
            if val > 500: on_line = True
            if val > 50:
                multiplier = i * 1000
                avg += val * multiplier
                sum += val

            if on_line == False:
                if self.lastValue < (self.NUM_SENSORS-1)*1000/2:
                    return 0
                else:
                    return (self.NUM_SENSORS-1)*1000

            self.lastValue = avg/sum
            return self.lastValue

    def initialise_calibration(self):
        # This function resets (inverse) max and min value thresholds prior to calibration
        # so that calibration readings can be correctly stored.
        for i in range(0, self.NUM_SENSORS):
            self.calibratedMax[i] = 0
            self.calibratedMin[i] = self.READING_TIMEOUT

    def read_calibrated(self):
   # This Function will display both uncallibrated and callibrated readings from the Pololu QTR-8 line following sensor.
   # Before doing so it will list all of uncalibrated reading from each sensor on the sensor array and
   # then we will be able to see calibrated values.

        self.read_sensors()
        print("Uncalibrated readings from the sensor:")
        self.print_sensor_values(self.sensorValues)

        for i in range(0, self.NUM_SENSORS):
            denominator = self.calibratedMax[i] - self.calibratedMin[i]
            val = 0

            if denominator != 0:
                val = (self.sensorValues[i] - self.calibratedMin[i]) * 1000 / denominator
            if val < 0:
                 val = 0
            elif val > 1000:
                 val = 1000

            self.sensorValues[i] = val

            print("Calibrated readings:")
            self.print_sensor_values(self.sensorValues)

    def emitters_on(self):
     # When we call this function LEDON pin (GPIO 7), so that IR LEDs on the array sensor can be turned on.
     # If there is nothing wired to LEDON emitters will always stay on.
        self.wp.digitalWrite(self.LED_ON_PIN, self.wp.HIGH)
        self.wp.delayMicroseconds(20)

    def emitters_off(self):
    #Turns the LEDON pin off so that the IR LEDs can be turned off.
    # If there is nothing wired to LEDON emitters will always be on.
        self.wp.digitalWrite(self.LED_ON_PIN, self.wp.LOW)
        self.wp.delayMicroseconds(20)
     #We can use those two functions above (emiters_on and emiters_off)
     # in order to conserve power consumption on the Sensor_Array

    def linecallibration(self):
        if self.lastValue >= 7000:
            print("The robot is not on the line")
            print("You should do something about it ")

qtr = qtr_8rc()
while qtr.calibratedMax != 0:
    if __name__ == "__main__":
        wp = odroid_wiringpi
        try:
            print("Team Brainstorm WorldSkills 2019 Robot Line Following Control Centre Developed by Szymon Sebastian Malecki")
            approve_re_calibration = str(raw_input("Do you want to re-calibrate your Pololu QTR-8 Line Array Sensor before proceeding with the driving? (Y/N): "))
            if approve_re_calibration == 'Y' or approve_re_calibration == 'y':
                approve_re_calibration = True
            elif approve_re_calibration == 'N' or approve_re_calibration == 'n':
                approve_re_calibration = False

            while approve_re_calibration:
                print("Re-calibrating")
                qtr.initialise_calibration()
                qtr.emitters_on()
                for i in range(0, 250):
                    qtr.calibrate_sensors()
                    wp.delayMicroseconds(20)
                qtr.emitters_off()

                print("Re-calibration successfully completed")
                print("He're Max values from the sensor:")
                qtr.print_sensor_values(qtr.calibratedMax)
                print("Re-calibration successfully completed")
                print("He're Minimum values from the sensor:")
                qtr.print_sensor_values(qtr.calibratedMin)
                approval = str(raw_input("Are you happy with the result of the re-calibration (Y/N)? "))
                if approval == 'Y':
                    print("You can now proceed with the driving")

        except Exception as e:
            qtr = qtr_8rc()
            qtr.emitters_off()
            print str(e)
        try:
            while qtr.calibratedMax != 0:
                qtr.emitters_on()
                print qtr.read_line()
                qtr.emitters_off()
        except KeyboardInterrupt:
            qtr.emitters_off()
        except Exception as e:
            print str(e)