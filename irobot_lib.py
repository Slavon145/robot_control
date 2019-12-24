# pylint: skip-file
# This is an an abstraction layer to control Faulhaber gpr cart through robot_server.py

import FaulhaberComm

increment_speed = 5
increment_turn = 1

class IrobotCommander:

	def __init__(self):

		# Speed in range [-100,100]
		# Positive values - fwd; Negative - bkwd
		self._right_speed = 0
		self._left_speed = 0
		self._motor_control = FaulhaberComm.FaulhaberComm()

	def stop_motion(self):

		self._right_speed = 0
		self._left_speed = 0

		self.update_speed()

	# Set right and left speeds to certain values.
	def set_speed(self, new_speed_right, new_speed_left):

		delta_speed_right = new_speed_right - self._right_speed
		delta_speed_left = new_speed_left - self._left_speed

		self.adjust_speed(delta_speed_right, delta_speed_left)


	#  Adds passed increments to right and left motor duty cycles.
	def adjust_speed(self, inc_right, inc_left):

		new_speed_right = self._right_speed + inc_right
		new_speed_left = self._left_speed + inc_left

		# Configure right speed
		# Truncate speeds to be in range [-100, 100].
		if(new_speed_right > 100):
			self._right_speed = 100

		elif(new_speed_right < -100):
			self._right_speed = -100

		# Configure left speed
		# Truncate speeds to be in range [-100, 100].
		else:
			self._right_speed = new_speed_right

		if(new_speed_left > 100):	
			self._left_speed = 100

		elif(new_speed_left < -100):
			self._left_speed = -100			

		else:
			self._left_speed = new_speed_left

		# Update speed motor duty cycle values
		self.update_speed()

	def update_speed(self):
		self._motor_control.set_velocity_right(self._right_speed*100)
		self._motor_control.set_velocity_left(self._left_speed*100)

	def interface(self, key):

		print(key)
		if(key == "space"):
			self.stop_motion()

		if(key == "x"):
			self.adjust_speed(increment_speed, increment_speed)

		if(key == "z"):
			self.adjust_speed(-increment_speed, -increment_speed)

		if(key == "right"):
			self.adjust_speed(increment_turn, -increment_turn)

		if(key == "left"):
			self.adjust_speed(-increment_turn, increment_turn)

	# Destructor
	def __del__(self):
		
		self.stop_motion()