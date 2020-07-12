import pybullet
import pybullet_data
import bullet_client
import time
class World:
	def __init__(self, render = True, gravity=-9.8, frame_count=25):
		if(render):
			self._pybullet_client = bullet_client.BulletClient(pybullet.GUI)
		else:
			self._pybullet_client = bullet_client.BulletClient(pybullet.GUI)   
		self._pybullet_client.setPhysicsEngineParameter(numSolverIterations=int(300))
		self._pybullet_client.setTimeStep(0.0002)
		plane = self._pybullet_client.loadURDF("%s/plane.urdf" % pybullet_data.getDataPath())
		self._pybullet_client.changeVisualShape(plane,-1,rgbaColor=[1,1,1,0.9])
		self._pybullet_client.setGravity(0, 0, gravity)
		self.frames = frame_count
		pass

	def sim(self):
		for i in range(self.frames):
			self._pybullet_client.stepSimulation()
			time.sleep(1./240.)


if(__name__ == "__main__"):
	world = World()
	world.sim()
