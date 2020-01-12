import time
import FaulhaberComm

if __name__ == "__main__":
    robot_control = FaulhaberComm.FaulhaberComm()
    print("robot_control initialized!")
    # Moves exactly one tile
    robot_control.travel_forward(1)
    robot_control.turn_right(90)

    robot_control.travel_forward(1)
    robot_control.turn_right(90)

    robot_control.travel_forward(1)
    robot_control.turn_right(90)

    robot_control.travel_forward(1)
    robot_control.turn_right(90)
