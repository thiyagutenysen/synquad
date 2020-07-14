import numpy as np
import matplotlib.pyplot as plt
no_of_points = 100
def constrain_theta(theta):
	theta = np.fmod(theta, 2*no_of_points)
	if(theta < 0):
		theta = theta + 2*no_of_points
	return theta
class Controller():
	def __init__(self, data):
		self._theta = 0
		self.body_length = data['body_length']
		self.body_width = data['body_width']
		self.dt = 0.005
		pass
	def command(self, args = 0):
		"""For now a very simple Circular Controller"""
		omega = 1
		a = 0.068*1.5
		b = 0.04*2
		# a = 0.15
		# b = 0.15
		x = -1*np.cos(np.pi*self._theta/no_of_points)*a
		y = np.sin(np.pi*self._theta/no_of_points)*b-0.243
		if(y<-0.243):
			y = -0.243
		self._theta = constrain_theta(0.1 + self._theta)
		#For HyQ use 0.05, rest use 0.1
		final = [0.4, -0.1, 0.0]
		final = [0.0,-0.01,-0.1]
		final_pts= {'FL':final,'FR':final,'BL':final,'BR':final}
		return final_pts

	def reset():
		self._theta = 0
		pass
class leg_data:
	def __init__(self, name):
		self.name = name
		self.motor_hip = 0.0
		self.motor_knee = 0.0
		self.motor_abduction = 0.0
		self.x = 0.0
		self.y = 0.0
		self.radius = 0.0
		self.theta = 0.0
		self.phi = 0.0
		self.gamma = 0.0
		self.b = 1.0
		self.step_length = 0.0
		pass

class ClassicPositionController(Controller):
	def __init__(self, data):
		super(ClassicPositionController, self).__init__(data)
		self.front_left = leg_data('FL')
		self.front_right = leg_data('FR')
		self.back_left = leg_data('BL')
		self.back_right = leg_data('BR')
		self._pts = np.array(data['pts'])
		self.step_length = data['step_length']
		self.weights = np.array([1,1,1,1,1,1])
		self.no_of_points = 100
		self._phase = data['phase']
		self.step_height = data['step_height']
		pass

	def _update_leg_theta(self,theta):
		""" Depending on the gait, the theta for every leg is calculated"""
		def constrain_theta(theta):
			theta = np.fmod(theta, 2*self.no_of_points)
			if(theta < 0):
				theta = theta + 2*self.no_of_points
			return theta
		self.front_right.theta = constrain_theta(theta+self._phase['FR'])
		self.front_left.theta = constrain_theta(theta+self._phase['FL'])
		self.back_right.theta = constrain_theta(theta+self._phase['BR'])
		self.back_left.theta = constrain_theta(theta+self._phase['BL'])
		pass
	
	def _update_leg_phi(self, radius):
		if(radius >= 0):
			self.front_left.phi =  np.arctan2(self.body_length/2, radius + self.body_width/2)
			self.front_right.phi = -np.arctan2(self.body_length/2, radius - self.body_width/2)
			self.back_left.phi = -np.arctan2(self.body_length/2, radius + self.body_width/2)
			self.back_right.phi =  np.arctan2(self.body_length/2, radius - self.body_width/2)
		  
		if(radius<0):
			newr = -1*radius
			self.front_right.phi =  np.arctan2(self.body_length/2, newr + self.body_width/2)
			self.front_left.phi = -np.arctan2(self.body_length/2, newr - self.body_width/2)
			self.back_right.phi = -np.arctan2(self.body_length/2, newr + self.body_width/2)
			self.back_left.phi =  np.arctan2(self.body_length/2, newr - self.body_width/2)
	
	def _update_leg_step_length(self, step_length, radius):
		if(abs(radius) <= 0.12):
			self.front_right.step_length = step_length
			self.front_left.step_length = step_length 
			self.back_right.step_length = step_length 
			self.back_left.step_length = step_length 
			return

		if(radius >= 0):
			self.front_right.step_length = step_length * (radius - self.body_width/2)/radius
			self.front_left.step_length = step_length * (radius + self.body_width/2)/radius
			self.back_right.step_length = step_length * (radius - self.body_width/2)/radius
			self.back_left.step_length = step_length * (radius + self.body_width/2)/radius
			return

		if(radius < 0):
			newr = radius*-1
			self.front_left.step_length = step_length * (newr- self.body_width/2)/newr
			self.front_right.step_length = step_length * (newr + self.body_width/2)/newr
			self.back_left.step_length = step_length * (newr - self.body_width/2)/newr
			self.back_right.step_length = step_length *(newr + self.body_width/2)/newr
			return 
	
	def drawBezier(self, points, weights, t):
		newpoints = np.zeros(points.shape)
		def drawCurve(points, weights, t):
			# print("ent1")
			if(points.shape[0]==1):
				return [points[0,0]/weights[0], points[0,1]/weights[0]]
			else:
				newpoints=np.zeros([points.shape[0]-1, points.shape[1]])
				newweights=np.zeros(weights.size)
				for i in np.arange(newpoints.shape[0]):
					x = (1-t) * points[i,0] + t * points[i+1,0]
					y = (1-t) * points[i,1] + t * points[i+1,1]
					w = (1-t) * weights[i] + t*weights[i+1]
					newpoints[i,0] = x
					newpoints[i,1] = y
					newweights[i] = w
				return drawCurve(newpoints, newweights, t)
		for i in np.arange(points.shape[0]):
			newpoints[i]=points[i]*weights[i]

		if(t<1):
			return drawCurve(newpoints, weights, t)
		if(t>=1):
			return [points[-1,0]+ (t-1)*(points[0,0] - points[-1,0]), -1*self.step_height]
	

	def command(self, args=0):
		radius, scale = args
		legs = [self.front_left, self.front_right, self.back_left, self.back_right]
		pts = {}
		x= []
		y = []
		z= []
		step_length = self.step_length
		weights = self.weights
		if radius == 1.0:
			radius = 10000
		self._update_leg_phi(radius)
		self._update_leg_step_length(step_length, radius)
		self._update_leg_theta(self._theta)
		for leg in legs:
			tau = leg.theta/self.no_of_points
			for i in range (weights.shape[0]):
				if weights[i] == 0:
					weights[i] = 1e-2
			new_pts = self._pts.copy()
			new_pts[0,0] = -leg.step_length/2 
			new_pts[-1,0] = leg.step_length/2 
			x,y = self.drawBezier(new_pts, weights, tau)
			leg.x, leg.y, leg.z = np.array([[np.cos(leg.phi),0,np.sin(leg.phi)],[0,1,0],[-np.sin(leg.phi),0, np.cos(leg.phi)]])@np.array([x,y,0])
			leg.x = leg.x * scale 
			leg.z = leg.z * scale 
			pts[leg.name] = [leg.x, leg.y, leg.z]	
		self._theta = constrain_theta(0.2 + self._theta)
		return pts
	

if(__name__ == "__main__"):
	data = {'body_length':10, 'body_width':10, 'phase':{'FL':0,'FR':0,'BL':0,'BR':0}}
	c = ClassicPositionController(data)
	xs = []
	ys = []
	for i in range(10000):
		pts = c.command(1,0.5)
		xs.append(pts['FL'][0])
		ys.append(pts['FL'][1])
	plt.figure()
	plt.plot(xs,ys,'r')
	plt.show()
	pass