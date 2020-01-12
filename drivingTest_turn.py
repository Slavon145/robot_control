import FaulhaberComm
import time
robot_control = FaulhaberComm.FaulhaberComm()
print("robot_control initialized!")
# Moves exactly one tile
robot_control.turn_left(90)
time.sleep(5)