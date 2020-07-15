import sys, os
import json
import numpy as np
from simulation import SimulationInterface
from actuator import Default
from inverse_kinematics import InverseKinematics, FiveBarLinkage, SpatialLinkage
from controller import Controller, ClassicPositionController
ik_name_to_class={'Five-Bar':FiveBarLinkage,'Spatial_Linkage':SpatialLinkage}
motor_name_to_class={'Default':Default}
controller_name_to_class={'Classic':ClassicPositionController}
class QuadrupedRobot():
    """
    The goal of this class is to simply store data pertaining to the quadruped robot.
    """
    def __init__(self, data):
        self.name = data['name']
        self.urdf_path = os.path.realpath('')+ data['urdf_path']
        self.init_pos = data['init_pos']
        self.init_ori = data['init_ori']
        self.sim = SimulationInterface(data)
        self.motor = motor_name_to_class[data['motor_model']['type']](data['motor_model'])
        self.ik = ik_name_to_class[data['IK']['type']](data['IK'])
<<<<<<< HEAD
        self.controller = controller_name_to_class[data['controller']['type']](data['controller'])
        self.joy_input = [0.5,1.5]
=======
        self.controller = controller_name_to_class[data['controller']['type']](self.parse_controller(data['controller']))
        self.joy_input = [1,1]
        self.motor_commands_pos = 0
        self.motor_commands_vel = 0
>>>>>>> master
        pass
    
    def apply_pd_control(self, des_pos=0, des_vel=0):
        if(des_pos == 0 and des_vel == 0):
            des_pos = self.motor_commands_pos
            des_vel = self.motor_commands_vel
        current_ang = self.sim.get_motor_angles()
        current_vel = self.sim.get_motor_velocities()
        motor_torque = self.motor.calc_torque(des_pos, des_vel, current_ang, current_vel)
        self.sim.apply_torque(motor_torque)
        pass
    
    def update_control_step(self, inputs = 0):
        if(inputs==0):
            inputs = self.joy_input
        pts = self.controller.command(inputs)
        final_pos = self.ik.solve(pts)
        self.motor_commands_pos = np.array(final_pos)
        self.motor_commands_vel = np.zeros(12)
        pass

    def parse_controller(self, data):
        if(data['step_height']=="Automatic"):
            data['step_height'] = round(self.ik._forward_2D(self.init_pos[0], self.init_pos[1])[0],2)
            pass
        if(data['step_length']=="Automatic"):
            sh = data['step_height']
            ll = self.ik.link_lengths[0] + self.ik.link_lengths[1]
            sl = np.sqrt(ll**2  - sh**2)
            data['step_length'] = round(0.5*sl,2)
            pass
        if(data['pts'] == "Automatic"):
            sh = data['step_height']
            sl = data['step_length']
            data['pts']=[[-sl/2, -sh], [-sl, -sh],[-sl/2, -0.6*sh],[sl/2, -0.6*sh],[sl, -sh],[sl/2, -sh]]
            pass
        print(data)
        return data
    

if(__name__ == "__main__"):
    with open("robots/stoch_two_abduction_urdf/stoch.json") as f:
        data = json.load(f)
    stoch = QuadrupedRobot(data)
    print(stoch.motor._kd)