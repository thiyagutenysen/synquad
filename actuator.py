class Default:
	"""Implementation of a standard PID control motor """
	def __init__(self, data):
		self._kp = data['kp']
		self._kd = data['kd']
		pass
	
	def calc_torque(self, motor_pos_commands, motor_vel_commands, current_pos, current_vel):
		applied_motor_torque = self._kp * (motor_pos_commands - current_pos) + self._kd * (motor_vel_commands - current_vel)
		return applied_motor_torque
	
if(__name__ == "__main__"):
	pass
