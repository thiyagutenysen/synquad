import pybullet

class SimulationInterface():
	""" This class controls the robot in pybullet simulation """
	def __init__(self, data):
		self.motor_names = data['motor_names']
		self.reset_pos = data['init_pos']
		self.reset_ori = data['init_ori']
		self.reset_angles = data['init_angles']
		self._client = 0
		self.id = 0
		self._motor_id_list = 0
		pass
	
	def set_pybullet_client(self, cl, id):
		self._client = cl
		self.id = id
		self._motor_id_list = self.buildMotorIdList()
		pass
	
	def buildMotorIdList(self):
		num_joints = self._client.getNumJoints(self.id)
		joint_name_to_id = {}
		for i in range(num_joints):
			joint_info = self._client.getJointInfo(self.id, i)
			joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]
		motor_id_list = [joint_name_to_id[motor_name] for motor_name in self.motor_names]
		return motor_id_list
	
	def apply_torque(self, torques):
		for motor_id, motor_torque in zip(self._motor_id_list, torques):
			self._client.setJointMotorControl2(
				  bodyIndex=self.id,
				  jointIndex=motor_id,
				  controlMode=self._pybullet_client.TORQUE_CONTROL,
				  force=torque)
	
	def get_motor_angles(self):
		motor_ang = [self._client.getJointState(self.id, motor_id)[0] for motor_id in self._motor_id_list]
		return motor_ang

	def get_motor_velocities(self):
		motor_vel = [self._pybullet_client.getJointState(self.stoch2, motor_id)[1] for motor_id in self._motor_id_list]
		return motor_vel
	
	def reset(self):
		self._client.resetBasePositionAndOrientation(self.id, self.reset_pos, self.reset_ori)
		self._client.resetBaseVelocity(self.id, [0, 0, 0], [0, 0, 0])
		i = 0
		for motor_id in self._motor_id_list:
			self._client.resetJointState(self.id,motor_id,targetValue = self.reset_angles[i], targetVelocity=0)
			i=i+1
		pass



	

