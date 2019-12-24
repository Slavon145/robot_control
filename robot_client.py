#!/usr/bin/env python3

import socket
import keyboard


def key_callback(event):
	if(event.event_type == "down"):

		s.sendall(event.name.encode())

HOST = "192.168.1.20"  # The server's hostname or IP address
PORT = 8000        # The port used by the server

keyboard.hook_key('tab', key_callback)
keyboard.hook_key('space', key_callback)
keyboard.hook_key('x', key_callback)
keyboard.hook_key('z', key_callback)
keyboard.hook_key('right', key_callback)
keyboard.hook_key('left', key_callback)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	s.connect((HOST, PORT))
	while True:
		data = s.recv(512)
		if not data:
			print("Connection closed")
			break

		print('Received', repr(data))