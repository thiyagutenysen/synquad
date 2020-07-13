import sys, os
import json
from simulation import SimulationInterface
from actuator import Default
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
        if(data['motor_model']['type'] == "Default"):
            self.motor = Default(data['motor_model'])
        pass
    
    def apply_pd_control(self, des_pos, des_vel):
        current_ang = self.sim.get_motor_angles()
        current_vel = self.sim.get_motor_vel()
        motor_torque = self.motor.calc_torque(des_pos, des_vel, current_ang, current_vel)
        self.sim.apply_torque(motor_torque)
        pass
    

if(__name__ == "__main__"):
    with open("robots/stoch_two_abduction_urdf/stoch.json") as f:
        data = json.load(f)
    stoch = QuadrupedRobot(data)
    print(stoch.motor._kd)