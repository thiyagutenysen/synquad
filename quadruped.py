import sys, os
import json
class QuadrupedRobot():
    def __init__(self, data):
        self.name = data['name']
        self.urdf_path = os.path.realpath('')+ data['urdf_path']
        self.init_pos = data['init_pos']
        self.init_ori = data['init_ori']
        self.id = 0
        self.pyb=0
        pass

if(__name__ == "__main__"):
    with open("stoch.json") as f:
        data = json.load(f)
    stoch = QuadrupedRobot(data)
    print(stoch.init_pos)