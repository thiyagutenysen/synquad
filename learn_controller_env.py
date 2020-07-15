import gym
from gym import error, spaces, utils
import numpy as np
from gym.utils import seeding
from world import World
from dataclasses import dataclass
from typing import List
import math
@dataclass
class TargetTrack:
    """Class for keeping the details of a trajectory"""
    Junctionpts: List[List[float]]
    curves: List[str]



class Learn_ControllerEnv(gym.Env):
		metadata = {'render.modes': ['human']}
		def __init__(self,
				robot,
				simulation_steps,
				render=False):
			self.world = World(render=render)
			self.robot = self.world.load_robot(robot, on_rack = False)
			self.sim_steps = simulation_steps
			self.training_trajectory = TargetTrack(Junctionpts=[[0,0,0],[1,0,0],[1.707,0.707,0],[2.707,0,0],[3.5,0,0]],
	                          						curves=['straight','straight','straight','straight'])
			self._last_base_position = [0,0,0]



			

		def step(self,action):
			transformed_action = action
			self.robot.controller.weights = transformed_action[0:6]
			self.robot.controller.omega =  transformed_action[6]
			
			straight = [1,1]
			str_k = ord('i')

			stop = [1,0]
			sto_k = ord('k')

			left = [-0.001,1]
			l_k = ord('j')

			right = [0,1]
			r_k = ord('l')

			jc = [1,1]

			for s in range(int(self.sim_steps)):
				keys = env.world._pybullet_client.getKeyboardEvents()

				if(len(keys)>0):
					if str_k in keys and keys[str_k]&self.world._pybullet_client.KEY_WAS_TRIGGERED:
						jc = straight
					elif sto_k in keys and keys[sto_k]&self.world._pybullet_client.KEY_WAS_TRIGGERED:
						jc = stop					
					elif r_k in keys and keys[r_k]&self.world._pybullet_client.KEY_WAS_TRIGGERED:
						jc = right			
					elif l_k in keys and keys[l_k]&self.world._pybullet_client.KEY_WAS_TRIGGERED:
						jc = left
				print("command_recieved:",jc)
				self.robot.joy_input = jc
				self.world.sim(self.robot)
			pass
		
		def drawTrack(self):

			for i in range(len(self.training_trajectory.Junctionpts)-1):
				if(self.training_trajectory.curves[i] =='straight'):
					self.world._pybullet_client.addUserDebugLine(self.training_trajectory.Junctionpts[i],self.training_trajectory.Junctionpts[i+1],
															lineColorRGB=[1,1,0],lineWidth=10)
				elif(self.training_trajectory.curves[i] =='phellipse'):
					diameter = np.linalg.norm(np.subtract(np.array(self.training_trajectory.Junctionpts[i]),np.array(self.training_trajectory.Junctionpts[i+1])))
					ellipse_centre = 0.5*np.add(np.array(self.training_trajectory.Junctionpts[i]),np.array(self.training_trajectory.Junctionpts[i+1]))
					print(ellipse_centre)
					current_pt = self.training_trajectory.Junctionpts[i+1]
					resolution = 100
					for j in range(resolution):
						next_point = np.add(ellipse_centre,np.array([(diameter/2)*math.cos(j*np.pi/resolution),0.5*math.sin(j*np.pi/resolution),0]))
						print("np",next_point)
						self.world._pybullet_client.addUserDebugLine(current_pt,next_point,lineColorRGB=[1,1,0],lineWidth=10)
						current_pt = next_point

				elif(self.training_trajectory.curves[i] =='nhellipse'):
					diameter = np.linalg.norm(np.subtract(np.array(self.training_trajectory.Junctionpts[i]),np.array(self.training_trajectory.Junctionpts[i+1])))
					ellipse_centre = 0.5*np.add(np.array(self.training_trajectory.Junctionpts[i]),np.array(self.training_trajectory.Junctionpts[i+1]))
					print(ellipse_centre)
					current_pt = self.training_trajectory.Junctionpts[i+1]
					resolution = 100
					for j in range(resolution):
						next_point = np.add(ellipse_centre,np.array([(diameter/2)*math.cos(j*np.pi/resolution),-0.5*math.sin(j*np.pi/resolution),0]))
						print("np",next_point)
						self.world._pybullet_client.addUserDebugLine(current_pt,next_point,lineColorRGB=[1,1,0],lineWidth=10)
						current_pt = next_point
		


			



		def calculate_reward(self):
			pos, ori = self.world._pybullet_client.getBasePositionAndOrientation(self.world.robot_id)
			RPY = np.round(self.world._pybullet_client.getEulerFromQuaternion(ori),4)
			roll_reward = np.exp(-45 * ((RPY[0]-self.support_plane_estimated_roll)) ** 2)
			pitch_reward = np.exp(-45 * ((RPY[1]-self.support_plane_estimated_pitch)) ** 2)
			yaw_reward = np.exp(-30 * (RPY[2] ** 2))

			#0 - x_dis,1 - y_dis,2 - z_dis,
			distance_travelled = np.subtract(np.array(pos),np.array(self._last_base_position))
			self._last_base_position = pos

			reward = round(yaw_reward, 4) + round(pitch_reward, 4) + round(roll_reward, 4)\
																+round(height_reward,4) + 100 * round(distance_travelled[0], 4)\
																+ 100 * round(distance_travelled[1], 4)+ 100 * round(distance_travelled[2], 4)
			return reward

		def GetObservation(self):
			pos, ori = self.world._pybullet_client.getBasePositionAndOrientation(self.world.robot_id)
			RPY = np.round(self.world._pybullet_client.getEulerFromQuaternion(ori),4)

			for val in RPY:
				self.obs_queue.append(val)
			obs = np.concatenate((self.obs_queue, [self.support_plane_estimated_roll,self.support_plane_estimated_pitch])).ravel()


			return obs
					
		def reset(self):
			'''
			1.reset simulation parameters
			'''
			initial_state = self.step([omega_zero_action,radius_zero_action,default_kp,defauly_kd])[0]
			#print("reset")
			return initial_state
					
		def close(self):
			'''
			kill env 
			'''
			print("close")


if(__name__ == "__main__"):
	env = Learn_ControllerEnv(robot='stoch',simulation_steps=10e20,render=False)
	env.drawTrack()
	env.step(action=np.array([1,1,1,1,1,1,2.5]))
