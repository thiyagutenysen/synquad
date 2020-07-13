import sys, os
sys.path.append(os.path.realpath('../..'))
import numpy as np
import gym
import os
from gym import utils, spaces
import pdb
import pybRL.envs.walking_controller as walking_controller
import time
import math

import pybullet
import pybRL.envs.bullet_client as bullet_client
import pybullet_data
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

Stoch = False
Laikago=True
if(Stoch):
	INIT_POSITION = [0, 0, 0.29]
	INIT_ORIENTATION = [0, 0, 0, 1]
if(Laikago):
	INIT_POSITION = [0, 0, 0.5]
	INIT_ORIENTATION = [0, 0.5, 0.5, 0]

LEG_POSITION = ["fl_", "bl_", "fr_", "br_"]
KNEE_CONSTRAINT_POINT_RIGHT = [0.014, 0, 0.076] #hip
KNEE_CONSTRAINT_POINT_LEFT = [0.0,0.0,-0.077] #knee
RENDER_HEIGHT = 720 #360
RENDER_WIDTH = 960 #480
PI = np.pi
no_of_points = 100
def constrain_theta(theta):
	theta = np.fmod(theta, 2*no_of_points)
	if(theta < 0):
		theta = theta + 2*no_of_points
	return theta
class Stoch2Env(gym.Env):

	def __init__(self,
				 render = False,
				 on_rack = False,
				 gait = 'trot',
				 phase = [0,no_of_points,no_of_points,0],
				 action_dim = 12,
				 obs_dim = 2,
				 scale = 1.0,
				 roc = 0.3,
				 stairs = True):

		self._is_stairs = stairs
		self.scale = scale #INITIAL ONE FOR TRAINING
		self._is_render = render
		self._on_rack = on_rack
		if self._is_render:
			self._pybullet_client = bullet_client.BulletClient(connection_mode=pybullet.GUI)
		else:
			self._pybullet_client = bullet_client.BulletClient()

		self._theta = 0
		self._theta0 = 0
		self._update_action_every = 1.  # update is every 50% of the step i.e., theta goes from 0 to pi/2
		self._frequency = 2.5 #change back to 1
		# self._frequency = 2.8 #change back to 1
		self.frequency_weight = 1
		self.prev_yaw = 0
		self._kp = 20
		self._kd = 2
		self.dt = 0.005 # LET ME CHANGE, was 0.001
		self._frame_skip = 25 # Working ratio is 5* self._dt
		self._n_steps = 0
		self._action_dim = action_dim

		self._obs_dim = obs_dim

		self.action = np.zeros(self._action_dim)

		self._last_base_position = [0, 0, 0]
		self._distance_limit = float("inf")

		self._xpos_previous = 0.0
		self._walkcon = walking_controller.WalkingController(gait_type=gait,
															 phase=phase, scale = self.scale)

		self._cam_dist = 1.0
		self._cam_yaw = 0.0
		self._cam_pitch = 0.0

		self.avg_vel_per_step = 0
		self.avg_omega_per_step = 0

		self.linearV = 0
		self.angV = 0

		self.radius = roc
		## Gym env related mandatory variables
		observation_high = np.array([10.0] * self._obs_dim)
		observation_low = -observation_high
		self.observation_space = spaces.Box(observation_low, observation_high)

		action_high = np.array([1] * self._action_dim)
		self.action_space = spaces.Box(-action_high, action_high)
		self.hard_reset()
		if(self._is_stairs):
			boxHalfLength = 0.06
			boxHalfWidth = 2.5
			boxHalfHeight = 0.02
			sh_colBox = self._pybullet_client.createCollisionShape(self._pybullet_client.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
			boxOrigin = 0.15
			n_steps = 30
			for i in range(n_steps):
				block=self._pybullet_client.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,basePosition = [boxOrigin + i*2*boxHalfLength,0,boxHalfHeight + i*2*boxHalfHeight],baseOrientation=[0.0,0.0,0.0,1])
			x = 1


	def hard_reset(self):
		self._pybullet_client.resetSimulation()
		self._pybullet_client.setPhysicsEngineParameter(numSolverIterations=int(300))
		self._pybullet_client.setTimeStep(self.dt/self._frame_skip)

		plane = self._pybullet_client.loadURDF("%s/plane.urdf" % pybullet_data.getDataPath())
		self._pybullet_client.changeVisualShape(plane,-1,rgbaColor=[1,1,1,0.9])
		self._pybullet_client.setGravity(0, 0, -9.8)
		if(Stoch):
			model_path = os.path.realpath('../..')+'/pybRL/envs/stoch_two_abduction_urdf/urdf/stoch_two_abduction_urdf.urdf'
		if(Laikago):
			model_path = os.path.realpath('../..')+'/pybRL/envs/laikago/laikago_toes.urdf'
		print(model_path)
		self.stoch2 = self._pybullet_client.loadURDF(model_path, INIT_POSITION, INIT_ORIENTATION)

		self._joint_name_to_id, self._motor_id_list, self._motor_id_list_obs_space = self.BuildMotorIdList()

		num_legs = 4
		for i in range(num_legs):
			self.ResetLeg(i, add_constraint=True)
		self.ResetPoseForAbd()
		if self._on_rack:
			self._pybullet_client.createConstraint(
				self.stoch2, -1, -1, -1, self._pybullet_client.JOINT_FIXED,
				[0, 0, 0], INIT_ORIENTATION, INIT_POSITION)

		self._pybullet_client.resetBasePositionAndOrientation(self.stoch2, INIT_POSITION, INIT_ORIENTATION)
		self._pybullet_client.resetBaseVelocity(self.stoch2, [0, 0, 0], [0, 0, 0])

		self._pybullet_client.resetDebugVisualizerCamera(self._cam_dist, self._cam_yaw, self._cam_pitch, [0, 0, 0])
		self.SetFootFriction(0.6)


	def reset(self):
		self._pybullet_client.resetBasePositionAndOrientation(self.stoch2, INIT_POSITION, INIT_ORIENTATION)
		self._pybullet_client.resetBaseVelocity(self.stoch2, [0, 0, 0], [0, 0, 0])

		num_legs = 4
		for i in range(num_legs):
			self.ResetLeg(i, add_constraint=False)
		self.ResetPoseForAbd
		self._pybullet_client.resetDebugVisualizerCamera(self._cam_dist, self._cam_yaw, self._cam_pitch, [0, 0, 0])
		self._n_steps = 0
		return self.GetObservationReset()


	def CurrentVelocities(self):
	
		current_w = self.GetBaseAngularVelocity()[2]
		current_v = self.GetBaseLinearVelocity()
		radial_v = math.sqrt(current_v[0]**2 + current_v[1]**2)
		return radial_v, current_w

	

	def do_simulation(self, action, n_frames, callback=None):
		omega = 2 * no_of_points * self._frequency  #Maybe remove later
		energy_spent_per_step = 0
		self.action = action
		cost_reference = 0
		ii = 0
		angle_data = []
		counter = 0
		sum_V = 0
		sum_W = 0
		current_theta = self._theta
		while(np.abs(self._theta - current_theta) <= no_of_points * self._update_action_every):
			current_angle_data = np.concatenate(([self._theta],self.GetMotorAngles()))
			angle_data.append(current_angle_data)
			abd_m_angle_cmd, leg_m_angle_cmd, d_spine_des, leg_m_vel_cmd= self._walkcon.transform_action_to_motor_joint_command_bezier(self._theta,action, self.radius)
			self._theta = constrain_theta(omega * self.dt + self._theta)
			# print(self._theta)
			qpos_act = np.array(self.GetMotorAngles())
			m_angle_cmd_ext = np.array(leg_m_angle_cmd + abd_m_angle_cmd)
			m_vel_cmd_ext = np.zeros(12)
			counter = counter+1
			current_v, current_w = self.CurrentVelocities()
			sum_V = sum_V + current_v
			sum_W = sum_W + current_w
			for _ in range(n_frames):
				ii = ii + 1
				applied_motor_torque = self._apply_pd_control(m_angle_cmd_ext, m_vel_cmd_ext)
				self._pybullet_client.stepSimulation()
				joint_power = np.multiply(applied_motor_torque, self.GetMotorVelocities()) # Power output of individual actuators
				joint_power[ joint_power < 0.0] = 0.0 # Zero all the negative power terms
				energy_spent = np.sum(joint_power) * self.dt/n_frames
				energy_spent_per_step += energy_spent

		self.avg_vel_per_step = sum_V/counter
		self.avg_omega_per_step = sum_W/counter
		self._n_steps += 1
		return energy_spent_per_step, cost_reference, angle_data

	def render(self, mode="rgb_array", close=False):
		if mode != "rgb_array":
			return np.array([])

		base_pos, _ = self.GetBasePosAndOrientation()
		view_matrix = self._pybullet_client.computeViewMatrixFromYawPitchRoll(
				cameraTargetPosition=base_pos,
				distance=self._cam_dist,
				yaw=self._cam_yaw,
				pitch=self._cam_pitch,
				roll=0,
				upAxisIndex=2)
		proj_matrix = self._pybullet_client.computeProjectionMatrixFOV(
				fov=60, aspect=float(RENDER_WIDTH)/RENDER_HEIGHT,
				nearVal=0.1, farVal=100.0)
		(_, _, px, _, _) = self._pybullet_client.getCameraImage(
				width=RENDER_WIDTH, height=RENDER_HEIGHT, viewMatrix=view_matrix,
				projectionMatrix=proj_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

		rgb_array = np.array(px).reshape(RENDER_WIDTH, RENDER_HEIGHT, 4)
		rgb_array = rgb_array[:, :, :3]
		return rgb_array


	def _apply_pd_control(self, motor_commands, motor_vel_commands):
		qpos_act = self.GetMotorAngles()
		qvel_act = self.GetMotorVelocities()
		applied_motor_torque = self._kp * (motor_commands - qpos_act) + self._kd * (motor_vel_commands - qvel_act)

		for motor_id, motor_torque in zip(self._motor_id_list, applied_motor_torque):
			self.SetMotorTorqueById(motor_id, motor_torque)
		return applied_motor_torque



	def GetObservationReset(self):
		"""
		Resets the robot and returns the base position and Orientation with a random error
		:param : None, should be called in the reset function if an error in initial pos is desired
		:return : Initial state with an error.
		Robot starts in the same position, only it's readings have some error.
		"""
		motor_angles = self.GetMotorAngles()
		obs = np.array(motor_angles)
		#obs = np.concatenate((motor_angles[self.radius],[self.scale])).ravel()
		return obs

	def GetMotorAngles(self):
		motor_ang = [self._pybullet_client.getJointState(self.stoch2, motor_id)[0] for motor_id in self._motor_id_list]
		return motor_ang
	def GetMotorVelocities(self):
		motor_vel = [self._pybullet_client.getJointState(self.stoch2, motor_id)[1] for motor_id in self._motor_id_list]
		return motor_vel
	def GetMotorTorques(self):
		motor_torq = [self._pybullet_client.getJointState(self.stoch2, motor_id)[3] for motor_id in self._motor_id_list]
		return motor_torq
	def GetBasePosAndOrientation(self):
		position, orientation = (self._pybullet_client.getBasePositionAndOrientation(self.stoch2))
		return position, orientation

	def GetDesiredMotorAngles(self):
		_, leg_m_angle_cmd, _, _ = self._walkcon.transform_action_to_motor_joint_command(self._theta,self.action)

		return leg_m_angle_cmd

	def GetBaseAngularVelocity(self):
		basevelocity= self._pybullet_client.getBaseVelocity(self.stoch2)
		return basevelocity[1] #world AngularVelocity vec3, list of 3 floats

	def GetBaseLinearVelocity(self):
		basevelocity= self._pybullet_client.getBaseVelocity(self.stoch2)
		return basevelocity[0] #world linear Velocity vec3, list of 3 floats



	def SetMotorTorqueById(self, motor_id, torque):
		self._pybullet_client.setJointMotorControl2(
				  bodyIndex=self.stoch2,
				  jointIndex=motor_id,
				  controlMode=self._pybullet_client.TORQUE_CONTROL,
				  force=torque)
	def BuildMotorIdList(self):
		num_joints = self._pybullet_client.getNumJoints(self.stoch2)
		joint_name_to_id = {}
		for i in range(num_joints):
			joint_info = self._pybullet_client.getJointInfo(self.stoch2, i)
			joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

		#adding abduction
		MOTOR_NAMES = [ "motor_fl_hip_joint",
						"motor_fl_knee_joint",
						"motor_fr_hip_joint",
						"motor_fr_knee_joint",
						"motor_bl_hip_joint",
						"motor_bl_knee_joint",
						"motor_br_hip_joint",
						"motor_br_knee_joint",
						"motor_fl_abd_joint",
						"motor_fr_abd_joint",
						"motor_bl_abd_joint",
						"motor_br_abd_joint"]
		motor_id_list = [joint_name_to_id[motor_name] for motor_name in MOTOR_NAMES]

		return joint_name_to_id, motor_id_list, motor_id_list_obs_space
	def ResetLeg(self, leg_id, add_constraint):
		leg_position = LEG_POSITION[leg_id]
		self._pybullet_client.resetJointState(
				  self.stoch2,
				  self._joint_name_to_id["motor_" + leg_position + "knee_joint"], # motor
				  targetValue = 0, targetVelocity=0)
		self._pybullet_client.resetJointState(
					self.stoch2,
					self._joint_name_to_id["motor_" + leg_position + "hip_joint"], # motor
					targetValue = 0, targetVelocity=0)
		if(Stoch):
			self._pybullet_client.resetJointState(
				  self.stoch2,
				  self._joint_name_to_id[leg_position + "lower_knee_joint"],
				  targetValue = 0, targetVelocity=0)
			self._pybullet_client.resetJointState(
					self.stoch2,
					self._joint_name_to_id[leg_position + "lower_hip_joint"],
					targetValue = 0, targetVelocity=0)

			if add_constraint:
				c = self._pybullet_client.createConstraint(
					self.stoch2, self._joint_name_to_id[leg_position + "lower_hip_joint"],
					self.stoch2, self._joint_name_to_id[leg_position + "lower_knee_joint"],
					self._pybullet_client.JOINT_POINT2POINT, [0, 0, 0],
					KNEE_CONSTRAINT_POINT_RIGHT, KNEE_CONSTRAINT_POINT_LEFT)

				self._pybullet_client.changeConstraint(c, maxForce=200)

		# set the upper motors to zero
		self._pybullet_client.setJointMotorControl2(
							  bodyIndex=self.stoch2,
							  jointIndex=(self._joint_name_to_id["motor_" + leg_position + "knee_joint"]),
							  controlMode=self._pybullet_client.VELOCITY_CONTROL,
							  targetVelocity=0,
							  force=0)
		self._pybullet_client.setJointMotorControl2(
							  bodyIndex=self.stoch2,
							  jointIndex=(self._joint_name_to_id["motor_"+ leg_position + "hip_joint"]),
							  controlMode=self._pybullet_client.VELOCITY_CONTROL,
							  targetVelocity=0,
							  force=0)

		# set the lower joints to zero
		if(Stoch):
			self._pybullet_client.setJointMotorControl2(
								bodyIndex=self.stoch2,
								jointIndex=(self._joint_name_to_id[leg_position + "lower_hip_joint"]),
								controlMode=self._pybullet_client.VELOCITY_CONTROL,
								targetVelocity=0,
								force=0)
			self._pybullet_client.setJointMotorControl2(
								bodyIndex=self.stoch2,
								jointIndex=(self._joint_name_to_id[leg_position + "lower_knee_joint"]),
								controlMode=self._pybullet_client.VELOCITY_CONTROL,
								targetVelocity=0,
								force=0)


	def ResetPoseForAbd(self):
		self._pybullet_client.resetJointState(
			self.stoch2,
			self._joint_name_to_id["motor_fl_abd_joint"],
			targetValue = 0, targetVelocity = 0)
		self._pybullet_client.resetJointState(
			self.stoch2,
			self._joint_name_to_id["motor_fr_abd_joint"],
			targetValue = 0, targetVelocity = 0)
		self._pybullet_client.resetJointState(
			self.stoch2,
			self._joint_name_to_id["motor_bl_abd_joint"],
			targetValue = 0, targetVelocity = 0)
		self._pybullet_client.resetJointState(
			self.stoch2,
			self._joint_name_to_id["motor_br_abd_joint"],
			targetValue = 0, targetVelocity = 0)

		#Set control mode for each motor and initial conditions
		self._pybullet_client.setJointMotorControl2(
			bodyIndex = self.stoch2,
			jointIndex = (self._joint_name_to_id["motor_fl_abd_joint"]),
			controlMode = self._pybullet_client.VELOCITY_CONTROL,
			force = 0,
			targetVelocity = 0
		)
		self._pybullet_client.setJointMotorControl2(
			bodyIndex = self.stoch2,
			jointIndex = (self._joint_name_to_id["motor_fr_abd_joint"]),
			controlMode = self._pybullet_client.VELOCITY_CONTROL,
			force = 0,
			targetVelocity = 0
		)
		self._pybullet_client.setJointMotorControl2(
			bodyIndex = self.stoch2,
			jointIndex = (self._joint_name_to_id["motor_bl_abd_joint"]),
			controlMode = self._pybullet_client.VELOCITY_CONTROL,
			force = 0,
			targetVelocity = 0
		)
		self._pybullet_client.setJointMotorControl2(
			bodyIndex = self.stoch2,
			jointIndex = (self._joint_name_to_id["motor_br_abd_joint"]),
			controlMode = self._pybullet_client.VELOCITY_CONTROL,
			force = 0,
			targetVelocity = 0
		)



if(__name__ == "__main__"):
	
	env = Stoch2Env(render=True, stairs = False,on_rack=False, gait = 'trot', roc = 100, scale = 1.0, phase = [0, 100, 100, 0])
	action = [-0.5,1,1,1,1,-0.5]*2
	states = []
	env.reset()
	angles = []
	for i in np.arange(1000):
		time.sleep(0.01)
		env.simulate_command(np.ones(12)*0.0, np.zeros(12))
		# cstate, _, _, angle = env.step(action)
		pass

