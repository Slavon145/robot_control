#!/usr/bin/env python3
import sys
import socket
import irobot_lib as irobot


PORT = 8000        # Port to listen on (non-privileged ports are > 1023)

def get_ip():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        sock.connect(('10.255.255.255', 1))
        IP = sock.getsockname()[0]
    except:
        IP = '127.0.0.1'
    finally:
        sock.close()
    return IP

if __name__ == '__main__':

    robot_control = irobot.IrobotCommander()
    print("robot_control initialized!")

    my_ip = get_ip()

    if my_ip == "127.0.0.1":

        print("Not connected")
        sys.exit()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

        s.bind((my_ip, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print("Connected.")
# print("Connected by {host}".format(host = socket.gethostbyaddr(addr[0])))
            while True:
                data = conn.recv(512)
                if not data:
                    robot_control.stop_motion()
                    print("Connection closed")
                    break
                robot_control.interface(data.decode())
