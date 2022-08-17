import pybullet as p
import time
import pybullet_data
import os

# def applyAction(motorCommands,speedMultiplierRight,speedMultiplierLeft,robotUniqueId):
#     targetVelocityRight = motorCommands[0] * speedMultiplierRight
#     targetVelocityLeft = motorCommands[1] * speedMultiplierLeft

#     right_front_wheeel = 2
#     right_back_wheel = 3
#     left_front_wheel = 6
#     left_back_wheel = 7

#     rightWheels = [right_front_wheeel, right_back_wheel]
#     leftWheels = [left_front_wheel, left_back_wheel]

#     for i in range(p.getNumJoints(robotUniqueId)):
#         print(p.getJointInfo(robotUniqueId,i))
#         print(p.getJointState(robotUniqueId,i))
#     for motor in rightWheels:
#       p.setJointMotorControl2(robotUniqueId,
#                                     motor,
#                                     p.VELOCITY_CONTROL,
#                                     targetVelocity=targetVelocityRight,
#                                     force=maxForce)
#     for motor in leftWheels:
#       #print("Left wheel motor id is {}".format(motor))
#       p.setJointMotorControl2(robotUniqueId,
#                                     motor,
#                                     p.VELOCITY_CONTROL,
#                                     targetVelocity=targetVelocityLeft,
#                                     force=maxForce)

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])

objUid = p.loadURDF(os.path.join(os.path.dirname(__file__),"../urdf/r2d2.urdf"),startPos, startOrientation)
# jointIndex = 5

# numJoint = p.getNumJoints(objUid)
# jointInfo = p.getJointInfo(objUid, jointIndex)

maxForce = 0
mode = p.VELOCITY_CONTROL
# p.setJointMotorControl2(objUid, jointIndex,
#  	controlMode=mode, force=maxForce)

_link_name_to_index = {p.getBodyInfo(objUid)[0].decode('UTF-8'):-1,}
for _id in range(p.getNumJoints(objUid)):
	_name = p.getJointInfo(objUid, _id)[12].decode('UTF-8')
	_link_name_to_index[_name] = _id
	# print(p.getJointState(objUid,_id))
	# p.enableJointForceTorqueSensor(objUid,_id,enableSensor = True)

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
motorCommands = [1, 1]
# applyAction(motorCommands,targetVel,targetVel/2,objUid)

for i in range (10000):
    # applyAction(motorCommands,targetVel,targetVel/2,objUid)
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(objUid)
print(cubePos,cubeOrn)
p.disconnect()



