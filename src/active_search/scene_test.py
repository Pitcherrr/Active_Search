import pybullet as p
import pybullet_data

from scene_loader import *

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(fixedTimeStep=60, numSubSteps=4)
p.setGravity(0.0, 0.0, -9.81)

curr_scene = Scene()

env_yaml = YamlScene("scene01.yaml")

env_yaml.generate(np.random)

