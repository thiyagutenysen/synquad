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
	def command(self):
		"""For now a very simple Circular Controller"""
		omega = 1
		a = 0.068*1.5
		b = 0.04*2
		a = 0.15
		b = 0.15
		x = -1*np.cos(np.pi*self._theta/no_of_points)*a
		y = np.sin(np.pi*self._theta/no_of_points)*b-0.5
		# if(y<-0.243):
		# 	y = -0.243
		self._theta = constrain_theta(0.1 + self._theta)
		#For HyQ use 0.05, rest use 0.1
		final = [0.4, -0.1, 0.0]
		final = [x,y,0]
		final_pts= {'FL':final,'FR':final,'BL':final,'BR':final}
		return final_pts

	def reset():
		self._theta = 0
		pass

if(__name__ == "__main__"):
	data = {'body_length':0, 'body_width':0}
	c = Controller(data)
	xs = []
	ys = []
	for i in range(100):
		x,y,z = c.command()
		xs.append(x)
		ys.append(y)
	plt.figure()
	plt.plot(xs,ys,'r')
	plt.show()
	pass