import pybullet
import pybullet_data
import bullet_client
import time
from quadruped import QuadrupedRobot
import json
robot_name_to_json = {
	'stoch':'robots/stoch_two_abduction_urdf/stoch.json',
	'laikago':'robots/laikago/laikago.json',
	'mini_cheetah':'robots/mini_cheetah/mini_cheetah.json',
	'hyq':'robots/hyq/hyq.json',
	'minitaur':'robots/minitaur/minitaur.json',
	'littleDog':'robots/littledog/littleDog.json',
	'anymal':'robots/anymal/anymal.json'
}
class World:
	def __init__(self, render = True, gravity=-9.8, frame_count=25):
		if(render):
			self._pybullet_client = bullet_client.BulletClient(pybullet.GUI)
		else:
			self._pybullet_client = bullet_client.BulletClient(pybullet.GUI)   
		self._pybullet_client.setPhysicsEngineParameter(numSolverIterations=int(300))
		self._pybullet_client.setTimeStep(0.0002)
		plane = self._pybullet_client.loadURDF("%s/plane.urdf" % pybullet_data.getDataPath())
		self._pybullet_client.changeDynamics(plane,-1, lateralFriction=0.5)
		self._pybullet_client.changeVisualShape(plane,-1,rgbaColor=[1,1,1,0.9])

		self._pybullet_client.setGravity(0, 0, gravity)
		self.frames = frame_count
		pass

	def sim(self, robot):
		robot.update_control_step()
		for i in range(self.frames):
			robot.apply_pd_control()
			self._pybullet_client.stepSimulation()

		pass

	def load_robot(self, name, on_rack = False):
		json_path = robot_name_to_json[name]
		with open(json_path) as f:
			data = json.load(f)
		robot = QuadrupedRobot(data)
		self.robot_id = self._pybullet_client.loadURDF(robot.urdf_path, robot.init_pos, robot.init_ori)
		robot.sim.set_pybullet_client(self._pybullet_client, self.robot_id)
		# FOOT_LINK_ID = [3,8,14,19]
		# for link_id in FOOT_LINK_ID:
		# 	self._pybullet_client.changeDynamics(
		# 	robot.sim.id, link_id, lateralFriction=0.6)
		robot.sim.reset()
		if(on_rack):
			robot.sim.on_rack()
		return robot
	


if(__name__ == "__main__"):
	world = World()
	robot = world.load_robot('anymal', on_rack = False)
	while True:
		world.sim(robot)
		pass
