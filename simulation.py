import pybullet

class SimulationInterface():
	""" This class controls the robot in pybullet simulation """
	def __init__(self, data):
		self.name = data['name']
		self.motor_names = data['motor_names']
		self.reset_pos = data['init_pos']
		self.reset_ori = data['init_ori']
		self.reset_angles = data['init_angles']
		self._client = 0
		self.id = 0
		self._motor_id_list = 0
		pass
	
	def set_pybullet_client(self, cl, r_id):
		self._client = cl
		self.id = r_id
		self._motor_id_list = self.buildMotorIdList()
		pass
	
	def buildMotorIdList(self):
		num_joints = self._client.getNumJoints(self.id)
		joint_name_to_id = {}
		for i in range(num_joints):
			joint_info = self._client.getJointInfo(self.id, i)
			joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]
		motor_id_list = [joint_name_to_id[motor_name] for motor_name in self.motor_names]
		self.add_constraints(joint_name_to_id)
		return motor_id_list
	
	def apply_torque(self, torques):
		for motor_id, motor_torque in zip(self._motor_id_list, torques):
			self._client.setJointMotorControl2(
				  bodyIndex=self.id,
				  jointIndex=motor_id,
				  controlMode=self._client.TORQUE_CONTROL,
				  force=motor_torque)
	
	def get_motor_angles(self):
		motor_ang = [self._client.getJointState(self.id, motor_id)[0] for motor_id in self._motor_id_list]
		return motor_ang

	def get_motor_velocities(self):
		motor_vel = [self._client.getJointState(self.id, motor_id)[1] for motor_id in self._motor_id_list]
		return motor_vel
	
	def reset(self):
		self._client.resetBasePositionAndOrientation(self.id, self.reset_pos, self.reset_ori)
		self._client.resetBaseVelocity(self.id, [0, 0, 0], [0, 0, 0])
		self._client.resetDebugVisualizerCamera(1,0,0, [0, 0, 0])
		i = 0
		for motor_id in self._motor_id_list:
			self._client.resetJointState(self.id,motor_id,targetValue = self.reset_angles[i], targetVelocity=0)
			i=i+1
		num_joints = self._client.getNumJoints(self.id)
		# for motor_id in self._motor_id_list:
		# 	self._client.setJointMotorControl2(
		# 					  bodyIndex=self.id,
		# 					  jointIndex=motor_id,
		# 					  controlMode=self._client.VELOCITY_CONTROL,
		# 					  targetVelocity=0,
		# 					  force=0)

		for i in range(num_joints):
			joint_info = self._client.getJointInfo(self.id, i)
			self._client.setJointMotorControl2(
							  bodyIndex=self.id,
							  jointIndex=joint_info[0],
							  controlMode=self._client.VELOCITY_CONTROL,
							  targetVelocity=0,
							  force=0)
		pass
	
	def on_rack(self):
		self._client.createConstraint(
				self.id, -1, -1, -1, self._client.JOINT_FIXED,
				[0, 0, 0], [0,0,0], self.reset_pos)
		pass

	def add_constraints(self, joint_name_to_id):
		if(self.name=='Stoch'):
			legs = ['fl_', 'fr_', 'bl_', 'br_']
			for leg in legs:
				c = self._client.createConstraint(
						self.id, joint_name_to_id[leg + "lower_hip_joint"],
						self.id, joint_name_to_id[leg + "lower_knee_joint"],
						self._client.JOINT_POINT2POINT, [0, 0, 0],
						[0.014, 0, 0.076], [0.0,0.0,-0.077])
				self._client.changeConstraint(c, maxForce=200)
		pass



	
