# File: FaulhaberComm by Stanislav Sotnikov
# CCNY Robotics Lab
# Stanislav Sotnikov
# This is an abstraction class for  communicating with Faulhaber Motor Drivers

# "THE BEER-WARE LICENSE":
# stanislav.sotnikov145@gmail.com wrote this file.
# As long as you retain this notice you can do whatever you want with this code.
# If we meet some day, and you think this code is worth it, you can buy me a beer in return.

import time
import threading
from math import sin, cos
import serial

TRAVEL_SPEED = 2000
MAX_ACCEL = 100

class FaulhaberComm:

    ## Constructor
    def __init__(self, com_port='/dev/ttyS0', positioning=True):

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


        # Start pose estimation thread if enabled.
        if positioning:
            self.positioning_thread = threading.Thread(target=self.update_pose, daemon=True)
            self.positioning_thread.start()

    ## write_and_return method
    # @brief writes a string over serial and returns the answer.
    # @param string String to be written.
    # @return Answer string.
    def write_and_return(self, string):

        with self._mutex:
            self._serialport.reset_input_buffer()
            self._serialport.write(string.encode())
            response = self._serialport.read_until()

        return response
    
    ## write_and_confirm method
    # @brief Sends a command and confirms the "OK" responce. Accessing each motor individually.
    # @param string String to be written.
    def write_and_confirm(self, string):
        response = self.write_and_return(string)

        # Check if responce is "OK"
        if response != b"OK\r\n":
            raise Exception("Failed to confirm command.")

    ## write_sync method.
    # @brief Sends a command to both motors simmultaneously
    #        by temprorarily disabling asynchronous responses
    #        to prevent serial from crashing.
    # @param string String to be written.
    def write_sync(self, string):

        with self._mutex:

            # Disable "OK" responces
            self._serialport.write("ANSW0\r".encode())
    
            # Send command
            self._serialport.write(string.encode())

            # Re-enable responces
            self.write_and_confirm("{node}ANSW2\r".format(node=self._ADDR_R))
            self.write_and_confirm("{node}ANSW2\r".format(node=self._ADDR_L))

    ## set_velocity_right method.
    # @brief Sets continuous motor velocity. Robot keeps moving until velocity is updated.
    # @param velocity Velocity in steps/s
    def set_velocity_right(self, velocity):
        self.write_and_confirm("{node}V{v}\r".format(node=self._ADDR_L, v=self._VSCALE_L*velocity))

    def set_velocity_left(self, velocity):
        self.write_and_confirm("{node}V{v}\r".format(node=self._ADDR_R, v=self._VSCALE_R*velocity))


    ## travel_forward method.
    # @brief Moves the robot in a straight line for the certain distance in millimeters.
    # @param distance Distance to move in mm.
    def travel_forward(self, distance):
        self._travel_steps(self._VSCALE_R*self._STEPS_PER_MM*distance, self._VSCALE_L*self._STEPS_PER_MM*distance)

        # Update pose.
        self.pose["translation_x"] += distance*cos(distance)
        self.pose["translation_y"] += distance*sin(distance)

    ## travel_backward method.
    # @brief Moves the robot in a straight line for the certain distance in millimeters.
    # @param distance Distance to move in mm.
    def travel_backward(self, distance):
        self._travel_steps(-self._VSCALE_R*self._STEPS_PER_MM*distance, -self._VSCALE_L*self._STEPS_PER_MM*distance)

        # Update pose.
        self.pose["translation_x"] -= distance*cos(distance)
        self.pose["translation_y"] -= distance*sin(distance)

    ## turn_left method.
    # @brief Turns left for the set amount of degrees. Requires proper calibration of self._STEPS_PER_DEG
    # @param degrees Amound of degrees to turn.
    def turn_left(self, degrees):
        self._travel_steps(self._VSCALE_R*self._STEPS_PER_DEG*degrees, -self._VSCALE_L*self._STEPS_PER_DEG*degrees)
        '''
        # Update pose.
        self.pose["roatation"] -= degrees
        if self.pose["roatation"] <= -180:
            self.pose["roatation"] += 360
        '''

    def turn_right(self, degrees):
        self._travel_steps(-self._VSCALE_R*self._STEPS_PER_DEG*degrees, self._VSCALE_L*self._STEPS_PER_DEG*degrees)
        '''
        # Update pose.
        self.pose["roatation"] += degrees
        if self.pose["roatation"] > 180:
            self.pose["roatation"] -= 360
        '''

    ## _travel_steps private method
    # @brief A helper function that makes motoer travel a certain amount of steps
    # @TODO: at long distances encoder overflow occurs, robot stops on overflow
    def _travel_steps(self, right_steps, left_steps):

        # Reset Absolute position to prevent overflow
        self.write_and_confirm("{node}LA0\r".format(node=self._ADDR_L))
        self.write_and_confirm("{node}LA0\r".format(node=self._ADDR_R))
        # Set relative distance for each motor
        self.write_and_confirm("{node}LR{v}\r".format(node=self._ADDR_L, v=int(left_steps)))
        self.write_and_confirm("{node}LR{v}\r".format(node=self._ADDR_R, v=int(right_steps)))

        # Notify when done on right motor.
        self.write_and_confirm("{node}NP\r".format(node=self._ADDR_R))
        
        with self._mutex:
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


    ## stop method
    # @brief stops motion.
    def stop(self):
        self.write_sync("V0\r")

    ## read_position_right method
    # @brief Reads right motor's encoder, converts to mm
    def read_position_right(self):
        return int(self.write_and_return("{node}POS\r".format(node=self._ADDR_R)))/self._STEPS_PER_MM
    
    ## read_position_left method
    # @brief Reads left motor's encoder, converts to mm
    def read_position_left(self):
        return int(self.write_and_return("{node}POS\r".format(node=self._ADDR_L)))/self._STEPS_PER_MM

    ## diffdrive method
    # @brief Updates pose based on velocity integration aka differential drive
    # @param v_l Left speed.
    # @param v_r Right speed.
    # @param t Time for integration.
    # @param l Distance between the wheels.
    def diffdrive(self, v_l, v_r, t, l):

        # Straight line, we need to avoid division by zero.
        if v_l == v_r:
            self.pose["translation_x"] = self.pose["translation_x"] + v_l * t * cos(self.pose["roatation"]) 
            self.pose["translation_y"] = self.pose["translation_y"] + v_l * t * sin(self.pose["roatation"])

        # Roatation.
        else:
            R = l/2.0 * ((v_l + v_r) / (v_r - v_l))

            # Instanteneous Center of Roatation.
            ICC_x = self.pose["translation_x"] - R * sin(self.pose["roatation"])
            ICC_y = self.pose["translation_y"] + R * cos(self.pose["roatation"])

            # Compute angular velocity. 
            omega = (v_r - v_l) / l
            
            # computing angle change
            delta_theta = omega * t

            # forward kinematics for differential drive 
            self.pose["translation_x"] = cos(delta_theta)*(self.pose["translation_x"] - ICC_x) - sin(delta_theta)*(self.pose["translation_y"] - ICC_y) + ICC_x
            self.pose["translation_y"] = sin(delta_theta)*(self.pose["translation_x"] - ICC_x) + cos(delta_theta)*(self.pose["translation_y"] - ICC_y) + ICC_y

            self.pose["roatation"] = self.pose["roatation"] + delta_theta
    
    def update_pose(self):
        
        while True:
            encoder_left = self.read_position_left()
            encoder_right = self.read_position_right()

            t = time.time() - self._time
            self._time = t

            velocity_left = (encoder_left - self.prev_encoder["left"])/t
            velocity_right = (encoder_right - self.prev_encoder["right"])/t

            self.prev_encoder["left"] += encoder_left
            self.prev_encoder["right"] += encoder_right

            self.diffdrive(velocity_left, velocity_right, t, 245)

            print(self.pose, end="\r", flush=True)
            time.sleep(0.5)

    ## Destructor
    def __del__(self):
        self._serialport.write("V0\r".encode())
        self._enable_confirmations()
        self._serialport.close()

    _mutex = threading.Lock()
    pose = {"roatation"  : 0,
            "translation_x": 0,
            "translation_y": 0,
            }
    prev_encoder = {"left" : 0,
                    "right" : 0
                    }
    _time = 0

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
