import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])

objUid = p.loadURDF("../urdf/r2d2.urdf",startPos, startOrientation)
jointIndex = 5

numJoint = p.getNumJoints(objUid)
jointInfo = p.getJointInfo(objUid, jointIndex)

maxForce = 0
mode = p.VELOCITY_CONTROL
p.setJointMotorControl2(objUid, jointIndex,
 	controlMode=mode, force=maxForce)

_link_name_to_index = {p.getBodyInfo(objUid)[0].decode('UTF-8'):-1,}
for _id in range(p.getNumJoints(objUid)):
	_name = p.getJointInfo(objUid, _id)[12].decode('UTF-8')
	_link_name_to_index[_name] = _id

maxForce = 500
targetVel = 50
p.setJointMotorControl2(bodyUniqueId=objUid, 
					jointIndex=2, 
					controlMode=p.VELOCITY_CONTROL,
					targetVelocity = targetVel,
force = maxForce)

p.setJointMotorControl2(bodyUniqueId=objUid, 
					jointIndex=3, 
					controlMode=p.VELOCITY_CONTROL,
					targetVelocity = targetVel,
force = maxForce)

p.setJointMotorControl2(bodyUniqueId=objUid, 
					jointIndex=6, 
					controlMode=p.VELOCITY_CONTROL,
					targetVelocity = targetVel/2,
force = maxForce)

p.setJointMotorControl2(bodyUniqueId=objUid, 
					jointIndex=7, 
					controlMode=p.VELOCITY_CONTROL,
					targetVelocity = targetVel/2,
force = maxForce)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(objUid)
print(cubePos,cubeOrn)
p.disconnect()

