import FaulhaberComm

robot_control = FaulhaberComm.FaulhaberComm()
print("robot_control initialized!")
# Moves exactly one tile
robot_control.travel_forward(3000)
