import FaulhaberComm
import time
robot_control = FaulhaberComm.FaulhaberComm()
print("robot_control initialized!")
# Moves exactly one tile
robot_control.travel_forward(0.5)
robot_control.travel_backward(0.5)
time.sleep(5)
