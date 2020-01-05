# File: FaulhaberComm by Stanislav Sotnikov
# CCNY Robotics Lab
# Stanislav Sotnikov
# This is an abstraction class for  communicating with Faulhaber Motor Drivers

# "THE BEER-WARE LICENSE":
# stanislav.sotnikov145@gmail.com wrote this file.
# As long as you retain this notice you can do whatever you want with this code.
# If we meet some day, and you think this code is worth it, you can buy me a beer in return.

import time
from math import sin, cos
import serial

TRAVEL_SPEED = 2000
MAX_ACCEL = 100

class FaulhaberComm:

    # Constructor
    def __init__(self, com_port='/dev/ttyS0'):

        # Open serial port
        self._serialport = serial.Serial(
            port=com_port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=5)

        if self._serialport.isOpen():
            print("Faulhaber port is opened!")
            self._port_opened = True
        else:
            raise Exception("Failed to open serial port.")

        # Make sure command confirmations are enabled.
        self._enable_confirmations()

        # Reset positionoing limits
        self.write_and_confirm("{node}APL0\r".format(node=self._ADDR_L))
        self.write_and_confirm("{node}APL0\r".format(node=self._ADDR_R))

        #Load travel speed.
        self.write_and_confirm("{node}SP{v}\r".format(node=self._ADDR_L, v=TRAVEL_SPEED))
        self.write_and_confirm("{node}SP{v}\r".format(node=self._ADDR_R, v=TRAVEL_SPEED))

        #Load acceleration.
        self.write_and_confirm("{node}AC{v}\r".format(node=self._ADDR_L, v=MAX_ACCEL))
        self.write_and_confirm("{node}AC{v}\r".format(node=self._ADDR_R, v=MAX_ACCEL))
        self.write_and_confirm("{node}DEC{v}\r".format(node=self._ADDR_L, v=MAX_ACCEL))
        self.write_and_confirm("{node}DEC{v}\r".format(node=self._ADDR_R, v=MAX_ACCEL))

        # Enable drives.
        self.write_and_confirm("{node}EN\r".format(node=self._ADDR_L))
        self.write_and_confirm("{node}EN\r".format(node=self._ADDR_R))


    # @write_and_confirm sends a command and confirms the "OK" responce.
    # Accessing each motor individually.
    def write_and_confirm(self, string):
        self._serialport.reset_input_buffer()
        self._serialport.write(string.encode())
        responce = self._serialport.read_until()

        # Check if responce is "OK"
        if responce != b"OK\r\n":
            raise Exception("Failed to confirm command.")


    # @write_sync sends a command to both motors simmultaneously
    # by temprorarily disabling asynchronous responses
    # to prevent serial from crashing
    def write_sync(self, string):
        # Disable "OK" responces
        self._serialport.write("ANSW0\r".encode())
  
        # Send command
        self._serialport.write(string.encode())

        # Re-enable responces
        self.write_and_confirm("{node}ANSW2\r".format(node=self._ADDR_R))
        self.write_and_confirm("{node}ANSW2\r".format(node=self._ADDR_L))


    # @set_velocity_right
    # @set_velocity_left
    # Sets continuous motor velocity.
    # Robot keeps moving until velocity is updated
    def set_velocity_right(self, velocity):
        self.write_and_confirm("{node}V{v}\r".format(node=self._ADDR_L, v=self._VSCALE_L*velocity))

    def set_velocity_left(self, velocity):
        self.write_and_confirm("{node}V{v}\r".format(node=self._ADDR_R, v=self._VSCALE_R*velocity))


    # @travel moves the robot in a straight line for the certain distance in millimeters.
    def travel_forward(self, distance):
        self._travel_steps(self._VSCALE_R*self._STEPS_PER_MM*distance, self._VSCALE_L*self._STEPS_PER_MM*distance)

        # Update pose.
        self.pose["translation_x"] += distance*cos(distance)
        self.pose["translation_y"] += distance*sin(distance)

    # @travel moves the robot in a straight line for the certain distance in millimeters.
    def travel_backward(self, distance):
        self._travel_steps(-self._VSCALE_R*self._STEPS_PER_MM*distance, -self._VSCALE_L*self._STEPS_PER_MM*distance)

        # Update pose.
        self.pose["translation_x"] -= distance*cos(distance)
        self.pose["translation_y"] -= distance*sin(distance)

    # @turn_left - Turns left for the set amount of degrees.
    # Requires proper calibration of self._STEPS_PER_DEG
    def turn_left(self, degrees):
        self._travel_steps(self._VSCALE_R*self._STEPS_PER_DEG*degrees, -self._VSCALE_L*self._STEPS_PER_DEG*degrees)
        
        # Update pose.
        self.pose["roatation"] -= degrees
        if self.pose["roatation"] <= -180:
            self.pose["roatation"] += 360

    # @turn_left - Turns left for the set amount of degrees.
    # Requires proper calibration of self._STEPS_PER_DEG
    def turn_right(self, degrees):
        self._travel_steps(-self._VSCALE_R*self._STEPS_PER_DEG*degrees, self._VSCALE_L*self._STEPS_PER_DEG*degrees)

        # Update pose.
        self.pose["roatation"] += degrees
        if self.pose["roatation"] > 180:
            self.pose["roatation"] -= 360


    # @_travel_steps - A helper function that makes motoer travel a certain amount of steps
    # TODO: at long distances encoder overflow occurs, robot stops on overflow
    def _travel_steps(self, right_steps, left_steps):

        # Reset Absolute position to prevent overflow
        self.write_and_confirm("{node}LA0\r".format(node=self._ADDR_L))
        self.write_and_confirm("{node}LA0\r".format(node=self._ADDR_R))
        # Set relative distance for each motor
        self.write_and_confirm("{node}LR{v}\r".format(node=self._ADDR_L, v=int(left_steps)))
        self.write_and_confirm("{node}LR{v}\r".format(node=self._ADDR_R, v=int(right_steps)))

        # Notify when done on right motor.
        self.write_and_confirm("{node}NP\r".format(node=self._ADDR_R))
        
        # Disable confirmations from the left motor.
        self._serialport.write("{node}ANSW0\r".format(node=self._ADDR_L).encode())

        # Start motion synchronously
        # Beware Right motor should still send "OK"
        self._serialport.write("M\r".encode())

        # Wait until motion is finished
        responce = b''
        while b'p' not in responce:
            responce = self._serialport.read_until()

        self._enable_confirmations() 


    def _enable_confirmations(self):
        self._serialport.write("ANSW2\r".encode())
        time.sleep(0.1)
        # There is potentially trash in the buffer.
        self._serialport.reset_input_buffer()


    def _disable_confirmations(self):
        self._serialport.write("ANSW0\r".encode())
        time.sleep(0.1)
        # There is potentially trash in the buffer.
        self._serialport.reset_input_buffer()


    # @stop stops motion. 
    def stop(self):
        self.write_sync("V0\r")


    # Destructor
    def __del__(self):
        self._serialport.write("V0\r".encode())
        self._enable_confirmations()
        self._serialport.close()


    pose = {"roatation"  : 0,
            "translation_x": 0,
            "translation_y": 0,
           }
    # Velocity scaling factors can be either 1 or -1.
    _VSCALE_L = -1
    _VSCALE_R = 1

    # Adress of left and right motors chained on RS232.
    _ADDR_L = 2
    _ADDR_R = 1

    # Encoder steps per degree and millimeter.
    #_STEPS_PER_DEG = 1500
    _STEPS_PER_DEG = 1505
    # Wheel Circumference = 
    # Steps per roatation = 3000
    # Gear ratio = 1:66
    _STEPS_PER_MM = 667
