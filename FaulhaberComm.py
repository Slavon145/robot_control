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
import collections
from math import sin, cos, pi

import serial

TRAVEL_SPEED = 1000
MAX_ACCEL = 50

class FaulhaberComm:

    ## Constructor
    def __init__(self, com_port='/dev/ttyUSB0', positioning=True):

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
        
        # Start reading motor driver answers in a thread
        threading.Thread(target=self.receive_answers, daemon=True).start()

        # Make sure command confirmations are enabled.
        self._enable_confirmations()

        # Reset positionoing limits
        self.write_and_confirm("{node}APL0\r".format(node=self._ADDR_L))
        self.write_and_confirm("{node}APL0\r".format(node=self._ADDR_R))
        self.write_and_confirm("{node}HO\r".format(node=self._ADDR_L))
        self.write_and_confirm("{node}HO\r".format(node=self._ADDR_R))

        #Load travel speed.
        self.write_and_confirm("{node}SP{v}\r".format(node=self._ADDR_L, v=TRAVEL_SPEED))
        self.write_and_confirm("{node}SP{v}\r".format(node=self._ADDR_R, v=TRAVEL_SPEED))

        #Load acceleration.
        self.write_and_confirm("{node}AC{v}\r".format(node=self._ADDR_L, v=MAX_ACCEL))
        self.write_and_confirm("{node}AC{v}\r".format(node=self._ADDR_R, v=MAX_ACCEL))
        self.write_and_confirm("{node}DEC{v}\r".format(node=self._ADDR_L, v=MAX_ACCEL))
        self.write_and_confirm("{node}DEC{v}\r".format(node=self._ADDR_R, v=MAX_ACCEL))

        self.stop()
        
        # Enable drives.
        self.write_and_confirm("{node}EN\r".format(node=self._ADDR_L))
        self.write_and_confirm("{node}EN\r".format(node=self._ADDR_R))

        # Start pose estimation thread if enabled.
        if positioning:
            threading.Thread(target=self.update_pose, daemon=True).start()

    ## receive_answers thread
    # @brief reads motor driver responces and adds them into queue
    def receive_answers(self):

        # Reser read buffer when the thread starts
        self._serialport.reset_input_buffer()

        while True:

            with self._read_mutex:
                response = self._serialport.read_until()

                if response:
                    self._answer_queue.append(response)
                    #print(response)

            time.sleep(0.01)


    ## mutex_write method
    # @brief Thread safe write method
    # @param string String to be written.
    def mutex_write(self, string):

        with self._write_mutex:
            self._serialport.write(string.encode())

    ## write_and_return method
    # @brief writes a string over serial and returns the answer.
    # @param string String to be written.
    # @return Answer string.
    def write_and_return(self, string):

        self.mutex_write(string)

        # Wait for the answer
        while not self._answer_queue:
            pass

        return self._answer_queue.popleft()

    
    ## write_and_confirm method
    # @brief Sends a command and confirms the "OK" responce. Accessing each motor individually.
    # @param string String to be written.
    def write_and_confirm(self, string):

        self.mutex_write(string)

        # Check if "OK" is in the queue of responces
        while self._MOTOR_ACK["ok"] not in self._answer_queue:
            pass

        # Delete "OK" from the queue of responces
        self._answer_queue.remove(self._MOTOR_ACK["ok"])
        
    ## write_sync method.
    # @brief Sends a command to both motors simmultaneously
    #        by temprorarily disabling asynchronous responses
    #        to prevent serial from crashing.
    # @param string String to be written.
    def write_sync(self, string):
    
        # Disable "OK" responces on the left motor
        self.mutex_write("{node}ANSW0\r".format(node=self._ADDR_L))

        # Send command
        self.write_and_confirm(string)

        # Re-enable responces on the left motor
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
        self._travel_steps(self._STEPS_PER_M*distance, -self._STEPS_PER_M*distance)

    ## travel_backward method.
    # @brief Moves the robot in a straight line for the certain distance in millimeters.
    # @param distance Distance to move in mm.
    def travel_backward(self, distance):
        self._travel_steps(-self._STEPS_PER_M*distance, self._STEPS_PER_M*distance)

    ## turn_left method.
    # @brief Turns left for the set amount of degrees. Requires proper calibration of self._STEPS_PER_DEG
    # @param degrees Amound of degrees to turn.
    def turn_left(self, degrees):
        self._travel_steps(self._STEPS_PER_DEG*degrees, self._STEPS_PER_DEG*degrees)

    def turn_right(self, degrees):
        self._travel_steps(self._STEPS_PER_DEG*degrees, self._STEPS_PER_DEG*degrees)

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
        
        # Disable confirmations from the left motor.
        self.mutex_write("{node}ANSW0\r".format(node=self._ADDR_L))

        # Start motion synchronously
        # Beware Right motor should still send "p"
        self.mutex_write("M\r")

        # Wait until motion is finished
        while self._MOTOR_ACK["pos_reached"] not in self._answer_queue:
            pass

        # Delete "p" from the queue of responces
        self._answer_queue.remove(self._MOTOR_ACK["pos_reached"])

        self._enable_confirmations()


    def _enable_confirmations(self):
        self.write_and_confirm("{node}ANSW2\r".format(node=self._ADDR_L))
        self.write_and_confirm("{node}ANSW2\r".format(node=self._ADDR_R))


    def _disable_confirmations(self):
        self.mutex_write("ANSW0\r".encode())


    ## stop method
    # @brief stops motion.
    def stop(self):
        self.write_sync("V0\r")

    ## read_position_right method
    # @brief Reads right motor's encoder, converts to meters
    def read_position_right(self):

        # Request position
        self.mutex_write("{node}POS\r".format(node=self._ADDR_R))

        # Wait for responce.
        while True:
            with self._read_mutex:
                for responce in self._answer_queue:
                    if responce not in self._MOTOR_ACK.values():
                        return int(responce)/self._STEPS_PER_M

            time.sleep(0.01)
    
    ## read_position_left method
    # @brief Reads left motor's encoder, converts to mm
    def read_position_left(self):

        # Request position
        self.mutex_write("{node}POS\r".format(node=self._ADDR_L))

        # Wait for responce.
        while True:
            with self._read_mutex:
                for responce in self._answer_queue:
                    if responce not in self._MOTOR_ACK.values():
                        return int(responce)/self._STEPS_PER_M
            time.sleep(0.01)

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
            #encoder_left = self.read_position_left()
            encoder_right = self.read_position_right()

            self.pose["translation_x"] = encoder_right
            #self.pose["translation_y"] = encoder_right
            '''
            t = time.time() - self._time
            self._time = t

            velocity_left = (encoder_left - self.prev_encoder["left"])/t
            velocity_right = (encoder_right - self.prev_encoder["right"])/t

            self.prev_encoder["left"] += encoder_left
            self.prev_encoder["right"] += encoder_right

            self.diffdrive(velocity_left, velocity_right, t, 245)
            '''
            #print(self.pose, end="\r", flush=True)
            print(self.pose)
            time.sleep(0.5)

    ## Destructor
    def __del__(self):
        self.write_sync("V0\r".encode())
        self._enable_confirmations()
        self._serialport.close()
    
    _read_mutex = threading.Lock()
    _write_mutex = threading.Lock()

    pose = {"roatation"  : 0,
            "translation_x": 0,
            "translation_y": 0,
            }
    prev_encoder = {"left" : 0,
                    "right" : 0
                    }
    _time = 0

    _answer_queue = collections.deque(maxlen=5)

    # Used motor acknowledgements
    _MOTOR_ACK = {"ok"          : b"OK\r\n",
                  "pos_reached" : b"p\r\n"
                 }

    # Velocity scaling factors can be either 1 or -1.
    _VSCALE_L = -1
    _VSCALE_R = 1

    # Adress of left and right motors chained on RS232.
    _ADDR_L = 2
    _ADDR_R = 1

    # Steps per roatation = 3000
    # Gear ratio = 1:42

      # Distance between two wheel centers
    _d = 0.26/2
    # Radius of the wheel
    _R = 0.0335

    _STEPS_PER_DEG  = (_d*46*3000)/(360*_R)
    _STEPS_PER_M = (46*3000)/(2*pi*_R)
