import numpy as np
import json
from numpy import cos,sin,sqrt
def constrain_theta(thetas):
    new_thetas = []
    for theta in thetas:
        theta = np.fmod(theta, 2*np.pi)
        if(theta>np.pi):
            theta = theta - 2*np.pi
        if(theta < -np.pi):
            theta = theta + 2*np.pi
        new_thetas.append(theta)
    return new_thetas

class InverseKinematics():
    def __init__(self,data):
        self.link_lengths = data['link_lengths']
        self.motor_offsets = data['motor_offsets']
        self.motor_scale = data['motor_dir']
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
    
    def solve(self, pts):
        motor_abd = []
        motor_ang = []
        leg_list=['FL','FR','BL','BR']
        for leg in leg_list:
            x,y,z = pts[leg]
            theta = np.arctan2(z,-y)
            new_coords = np.array([x,-y/np.cos(theta) - 0.035,z])
            motor_knee, motor_hip, _, _ = self._inverse_stoch2(new_coords[0], -new_coords[1])
            motor_hip = (motor_hip + self.motor_offsets[leg]['hip'])*self.motor_scale[leg]['hip']
            motor_knee = (motor_knee + self.motor_offsets[leg]['knee'])*self.motor_scale[leg]['knee']
            theta = (theta + self.motor_offsets[leg]['abd'])*self.motor_scale[leg]['abd']
            motor_ang.append(motor_hip)
            motor_ang.append(motor_knee)
            motor_abd.append(theta)
        final_motor_angles = motor_ang+motor_abd
        return final_motor_angles

class SpatialLinkage(InverseKinematics):
    def __init__(self,data):
        super(SpatialLinkage, self).__init__(data)
        self.sol_branch = data['sol_branch']
        self.name = data['type']
        pass
    
    def _inverse_2D(self,x,y,sol_branch = 1):
        l1 = self.link_lengths[0]
        l2 = self.link_lengths[1]
        t1 = (-4*l2*y + sqrt(16*l2**2*y**2 - 4*(-l1**2 + l2**2 - 2*l2*x + x**2 + y**2)*(-l1**2 + l2**2 + 2*l2*x + x**2 + y**2)))/(2.*(l1**2 - l2**2 - 2*l2*x - x**2 - y**2))
        t2 = (-4*l2*y - sqrt(16*l2**2*y**2 - 4*(-l1**2 + l2**2 - 2*l2*x + x**2 + y**2)*(-l1**2 + l2**2 + 2*l2*x + x**2 + y**2)))/(2.*(l1**2 - l2**2 - 2*l2*x - x**2 - y**2))
        if(sol_branch):
            t = t2
        else:
            t = t1
        th12 = np.arctan2(2*t,(1-t**2))
        th1 = np.arctan2(y - l2*sin(th12), x - l2*cos(th12))
        th2 = th12 - th1
        return [th1,th2]
    
    def _forward_2D(self,th1,th2):
        l1 = self.link_lengths[0]
        l2 = self.link_lengths[1]
        x = l1 * cos(th1) + l2 * cos(th1+th2)
        y = l1 * sin(th1) + l2 * sin(th1+th2)
        return [x,y]

    def solve(self, pts):
        motor_ang=[]
        motor_abd = []
        leg_list = ['FL','FR','BL','BR']
        for leg in leg_list:
            x,y,z=pts[leg]
            theta = np.arctan2(z,-y)
            new_coords = np.array([x,y/np.cos(theta),z])
            motor_hip, motor_knee = self._inverse_2D(new_coords[0], new_coords[1],sol_branch=self.sol_branch[leg])
            motor_hip =  (motor_hip + self.motor_offsets[leg]['hip'])*self.motor_scale[leg]['hip']
            motor_knee =  (motor_knee+self.motor_offsets[leg]['knee'])*self.motor_scale[leg]['knee']
            theta = (theta+self.motor_offsets[leg]['abd'])*self.motor_scale[leg]['abd']
            motor_ang.append(motor_hip)
            motor_ang.append(motor_knee)
            motor_abd.append(theta)
        final_motor_angles = motor_ang+motor_abd
        return final_motor_angles

    

if(__name__ == "__main__"):
    robot_name_to_json = {
	'stoch':'robots/stoch_two_abduction_urdf/stoch.json',
	'laikago':'robots/laikago/laikago.json',
	'mini_cheetah':'robots/mini_cheetah/mini_cheetah.json',
	'hyq':'robots/hyq/hyq.json',
	'minitaur':'robots/minitaur/minitaur.json'
    }
    with open(robot_name_to_json['mini_cheetah'])as f:
        data = json.load(f)
    # ik = FiveBarLinkage(data['IK'])
    # print(ik.solve(0.05, - 0.243, 0))
    ik = SpatialLinkage(data['IK'])
    ik.link_lengths = np.array([0.25,0.25])
    th1 = 0
    th2 = 0
    x,y = ik._forward_2D(th1,th2)
    th1_, th2_ = ik._inverse_2D(x,y)
    newx, newy = ik._forward_2D(th1_, th2_)
    print(x,y,newx,newy)
    th1_, th2_,th3_ = ik.solve(0.0,-0.5,0.0)
    th1, th2 = ik._inverse_2D(0.0,-0.001)
    print(th1_,th2_,th3_)
    print(th1,th2)

    pass