import gym
from gym import error, spaces, utils
import numpy as np
from gym.utils import seeding
from world import World



class Learn_ControllerEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self,
  			   robot,

  			   simulation_steps,
  			   target_velocity,
  			   render=False):
  	self.world = World(render=render)
	self.robot = world.load_robot(robot, on_rack = False)
	self.sim_steps = simulation_steps
  self.training_trajectory = []
	# while True:
	# 	world.sim(robot)
	# 	pass



   

  def step(self,action):
  	transformed_action = action
  	self.robot.controller.weights = transformed_action[0:6]
  	#self.robot.joy_input = action[6:8]
  	
  	for s in range(self.simulation_steps):
  		  	self.world.sim(self.robot)



  	pass


  def calculate_reward(self,avg_vel):

    weight_matrix = np.array([0.5,0.5])
    exp_weight = [-6,-6]
    abs_error = np.absolute(avg_vel - self.target_velocity)

    exponential_error = np.exp(exp_weight[0]*abs_error[0]+exp_weight[1]*abs_error[1])

    return(exponential_error)

def calculate_reward(self, action, energy_spent_per_step, cost_reference):



    pos, ori = self.GetBasePosAndOrientation()

    RPY = np.round(self.world._pybullet_client.getEulerFromQuaternion(ori),4)

    roll_reward = np.exp(-45 * ((RPY[0]-self.support_plane_estimated_roll)) ** 2)
    pitch_reward = np.exp(-45 * ((RPY[1]-self.support_plane_estimated_pitch)) ** 2)
    yaw_reward = np.exp(-30 * (RPY[2] ** 2))
    step_distance_x = (x - x_l)
    if done:
      reward = 0
    else:
      reward = round(yaw_reward, 4) + round(pitch_reward, 4) + round(roll_reward, 4)\
              +round(height_reward,4) + 100 * round(step_distance_x, 4)
    return reward

def GetObservation(self):
    pos, ori = self.GetBasePosAndOrientation()
    RPY = self._pybullet_client.getEulerFromQuaternion(ori)
    RPY = np.round(RPY, 5)
    self.inverse = False
    
    
    if RPY[2] < -0.005:
      RPY[2] = RPY[2] * -1
      self.inverse = True
    
    
    

    # Appending current_yaw #and Yaw Rate in the queue
    for val in RPY:
      self.obs_queue.append(val)

    self.obs_support_roll.append(self.support_plane_estimated_roll)
    self.obs_support_pitch.append(self.support_plane_estimated_pitch)

    roll_run_avg = 0
    pitch_run_avg = 0

    for i in range(int(len(self.obs_support_roll))):
      roll_run_avg =roll_run_avg+self.obs_support_roll[i] 
      pitch_run_avg=pitch_run_avg+self.obs_support_pitch[i] 

    if(self._n_steps >=len(self.obs_support_roll)):
      roll_run_avg = roll_run_avg / len(self.obs_support_roll)
      pitch_run_avg = pitch_run_avg / len(self.obs_support_pitch)
    else:
      roll_run_avg = self.support_plane_estimated_roll
      pitch_run_avg = self.support_plane_estimated_pitch

    #obs = np.concatenate((self.obs_queue, [roll_run_avg,pitch_run_avg])).ravel()
    obs = np.concatenate((self.obs_queue, [self.support_plane_estimated_roll,self.support_plane_estimated_pitch])).ravel()


    return obs
    
  def reset(self):
    '''
    1.reset simulation parameters
    '''

    initial_state = self.step([omega_zero_action,radius_zero_action,default_kp,defauly_kd])[0]
    #print("reset")
    return initial_state
    
  def render(self, mode='human'):
    '''
    visulaize a frame (need to chk if this function is required)
    print("render")
    '''
  def close(self):
    '''
    kill env 
    '''
    raisim_dll._close()
    print("close")