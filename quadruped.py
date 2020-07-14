import sys, os
import json
import numpy as np
from simulation import SimulationInterface
from actuator import Default
from inverse_kinematics import InverseKinematics, FiveBarLinkage, SpatialLinkage
from controller import Controller
ik_name_to_class={'Five-Bar':FiveBarLinkage,'Spatial_Linkage':SpatialLinkage}
motor_name_to_class={'Default':Default}
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
        self.controller = Controller(data)
        pass
    
    def apply_pd_control(self, des_pos, des_vel):
        current_ang = self.sim.get_motor_angles()
        current_vel = self.sim.get_motor_velocities()
        motor_torque = self.motor.calc_torque(des_pos, des_vel, current_ang, current_vel)
        self.sim.apply_torque(motor_torque)
        pass
    
    def apply_control_step(self, input = 0):
        pts = self.controller.command()
        final_pos = self.ik.solve(pts)
        # final_pos = [motor_hip,motor_knee]*4+[motor_abd]*4
        final_pos = np.array(final_pos)
        vel = np.zeros(12)
        self.apply_pd_control(final_pos, vel)
        pass
    

if(__name__ == "__main__"):
    with open("robots/stoch_two_abduction_urdf/stoch.json") as f:
        data = json.load(f)
    stoch = QuadrupedRobot(data)
    print(stoch.motor._kd)