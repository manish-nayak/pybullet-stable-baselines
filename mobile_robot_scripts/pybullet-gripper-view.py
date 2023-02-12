import pybullet as p
import time
import pybullet_data
import os

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
#boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
boxId = p.loadSDF("kuka_iiwa/model.sdf")
#boxId = p.loadURDF("racecar/racecar_differential.urdf",startPos, startOrientation)

# boxId = p.loadSDF(os.path.join(os.path.dirname(__file__),"../urdf/model.sdf"),startPos, startOrientation)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

# python -m pybullet_envs.examples.minitaur_gym_env_example
# python -m pybullet_envs.examples.enjoy_TF_HumanoidBulletEnv_v0_2017may
