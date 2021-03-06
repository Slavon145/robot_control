# robot_control

This API class wraps Faulhaber motor driver communication into functions that directly invoke motor motion.

## Project Structure

There are 2 ways of invoking robot motion through the given set of scripts:

1. User Interface - In this mode the robot is remotely controlled from the user's computer keyboard over TCP.  
  Control keys:  
  *x* - increment speed on both motors.  
  *z* - decrement speed on both motors.  
  *left arrow* - increment speed on the right motor, decrement speed on the left motor (turn left).  
  *right arrow* - increment speed on the left motor, decrement speed on the right motor (turn right).  
  **Note** that the speeds are capped, once the speed limit is reached further increments are ignored.
  
2. Encoder based motion - Robot moves autonomously when appropriate API function is called. Uses encoder to travel a set amount of steps/millimeters or rotate a set amount of steps/degrees.

[FaulhaberComm.py](https://github.com/Slavon145/robot_control/blob/master/FaulhaberComm.py) - Contains the API class to communicate with Faulhaber motor drivers.  
[robot_server.py](https://github.com/Slavon145/robot_control/blob/master/robot_server.py) - A socket server for the remote robot control. Runs on the robot.  
[robot_client.py](https://github.com/Slavon145/robot_control/blob/master/robot_client.py) - A socket client for the remote robot control. Runs on the user's computer.  
[irobot_lib.py](https://github.com/Slavon145/robot_control/blob/master/irobot_lib.py) - Adapter class to implement user interface based motor control.  
[drivingTest_scan.py](https://github.com/Slavon145/robot_control/blob/master/drivingTest_scan.py) - An example of encoder based motion in a zig-zag pattern.   


## Dependencies
* [pyserial](https://pyserial.readthedocs.io/en/latest/) - Provides serial communication with Faulhaber motor drivers.  
* [keyboard](https://pypi.org/project/keyboard/) - Used to controll the robot from user's keyboard.  

## Deployment
Built on a A Raspberry Pi 4. 
** to be continued **
