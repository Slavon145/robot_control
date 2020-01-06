import time
import FaulhaberComm

if __name__ == "__main__":
    robot_control = FaulhaberComm.FaulhaberComm()
    print("robot_control initialized!")
    # Moves exactly one tile
    robot_control.travel_forward(600)
    robot_control.turn_left(45)

    robot_control.travel_forward(100)
    robot_control.turn_right(45)

    robot_control.travel_backward(600)
    #robot_control.turn_right(90)

    #robot_control.travel_forward(150)
    #robot_control.turn_right(90)
