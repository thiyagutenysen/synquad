import numpy as np
import json
class InverseKinematics():
    def __init__(self,data):
        self.link_lengths = data['link_lengths']
        self.motor_offsets = data['motor_offsets']
        self.motor_scale = data['motor_directions']
        pass
    def solve(self, x,y,z):
        """ This function will return the angles, given a certain x,y,z value. It is an inverse kinematics solver"""
        pass

class FiveBarLinkage(InverseKinematics):
    def __init__(self,data):
        super(FiveBarLinkage, self).__init__(data)
        self.name = data['type']
        pass

    def _inverse_stoch2(self, x,y):

        l1 =    self.link_lengths[0]
        l2 =    self.link_lengths[1]
        l4 =    self.link_lengths[2]
        l5 =    self.link_lengths[3]
        le =    self.link_lengths[5]
        tq1 =   self.link_lengths[6]
        tq2 =   self.link_lengths[7]
        delta = self.link_lengths[4]
        xb = [[0,0],[0,0]]
        yb = [[0,0],[0,0]]
        phid = [0,0];psi = [0,0]; theta = [0,0]
        R_base = [[0,0],[0.035,0]]
        xb[0] = R_base[0][0];xb[1] = R_base[1][0]
        yb[0] = R_base[0][1];yb[1] = R_base[1][1]
        l3 = np.sqrt((x-xb[0])**2+(y-yb[0])**2)
        theta[0] = np.arctan2((y-yb[0]),(x-xb[0]))
        zeta = (l3**2 - l1**2 -l2**2)/(2*l1*l2)
        zeta = np.sign(zeta) if abs(zeta) > 1 else zeta
        phid[0] = np.arccos(zeta)
        psi[0] = np.arctan2(l2*np.sin(phid[0]),(l1+l2*np.cos(phid[0])))
        q1 = theta[0] - psi[0]
        q2 = q1 + phid[0]
        xm = l1*np.cos(q1)+l2*np.cos(q2)
        ym = l1*np.sin(q1)+l2*np.sin(q2)
        xi = (xm+xb[0])
        yi = (ym+yb[0])

        xi = xb[0] + l1*np.cos(q1) + 0.04*np.cos(q2-tq1)
        yi = yb[0] + l1*np.sin(q1) + 0.04*np.sin(q2-tq1)
        R = [xi,yi]
        l6 = np.sqrt(((xi-xb[1])**2+(yi-yb[1])**2))
        theta[1] = np.arctan2((yi-yb[1]),(xi-xb[1]))
        Zeta = (l6**2 - l4**2 - l5**2)/(2*l5*l4)
        leg = 'left'
        Zeta = np.sign(Zeta) if abs(Zeta) > 1 else Zeta
        phid[1] = np.arccos(Zeta)
        psi[1] = np.arctan2(l5*np.sin(phid[1]),(l4+l5*np.cos(phid[1])))
        q3 = theta[1]+psi[1]
        q4 = q3-phid[1]
        xm = l4*np.cos(q3)+l5*np.cos(q4)+xb[1]
        ym = l4*np.sin(q3)+l5*np.sin(q4)+yb[1]

        if Zeta == 1:
            [q1, q2] = self._inverse_new(xm,ym,delta,Leg)

        return [q3, q1, q4, q2]

    def _inverse_new(self, xm,ym,delta):

        l1 = self.link_lengths[0]
        l2 = self.link_lengths[1]-self.link_lengths[4]
        l4 = self.link_lengths[2]
        l5 = self.link_lengths[3]
        delta = Leg[4]
        xb = [[0,0],[0,0]]
        yb = [[0,0],[0,0]]
        phid = [0,0];psi = [0,0]; theta = [0,0]
        R_base = [[1,0],[-1,0]]
        xb[0] = R_base[0][0];xb[1] = R_base[1][0]
        yb[0] = R_base[0][1];yb[1] = R_base[1][1]
        l3 = np.sqrt((xm-xb[0])**2+(ym-yb[0])**2)
        theta[0] = np.arctan2((ym-yb[0]),(xm-xb[0]))
        zeta = (l3**2 - l1**2 -l2**2)/(2*l1*l2)
        zeta = np.sign(zeta) if abs(zeta) > 1 else zeta
        phid[0] = np.arccos(zeta)
        psi[0] = np.arctan2(l2*np.sin(phid[0]),(l1+l2*np.cos(phid[0])))
        q1 = theta[0] + psi[0]
        q2 = q1 - phid[0]
        xm = l1*np.cos(q1)+l2*np.cos(q2)
        ym = l1*np.sin(q1)+l2*np.sin(q2)

        return [q1,q2]
    
    def solve(self, x,y,z):
        theta = np.arctan2(z,-y)
        new_coords = np.array([x,-y/np.cos(theta) - 0.035,z])
        motor_knee, motor_hip, _, _ = self._inverse_stoch2(new_coords[0], -new_coords[1])
        motor_hip = motor_hip + self.motor_offsets[0]
        motor_knee = motor_knee + self.motor_offsets[1]
        return [motor_hip, motor_knee, theta]


if(__name__ == "__main__"):
    robot_name_to_json = {
	'stoch':'robots/stoch_two_abduction_urdf/stoch.json',
	'laikago':'robots/laikago/laikago.json',
	'mini_cheetah':'robots/mini_cheetah/mini_cheetah.json',
	'hyq':'robots/hyq/hyq.json',
	'minitaur':'robots/minitaur/minitaur.json'
    }
    with open(robot_name_to_json['stoch'])as f:
        data = json.load(f)
    ik = FiveBarLinkage(data['IK'])
    print(ik.solve(0.05, - 0.243, 0))
