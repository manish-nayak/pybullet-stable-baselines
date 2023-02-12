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

objUid = p.loadSDF(os.path.join(os.path.dirname(__file__),"kuka_iiwa/model.sdf"))[0]
_link_name_to_index = {p.getBodyInfo(objUid)[0].decode('UTF-8'):-1,}
link_arr = [];
for _id in range(p.getNumJoints(objUid)):
	_name = p.getJointInfo(objUid, _id)[12].decode('UTF-8')
	_link_name_to_index[_name] = _id
	joint_low_limit = p.getJointInfo(objUid, _id)[8]
	joint_upper_limit = p.getJointInfo(objUid, _id)[9]
	p.addUserDebugParameter(_name,joint_low_limit,joint_upper_limit,0)

maxForce = 500
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
while True:
	joint_indices = []
	targetPosArr = []
	maxForceArr = []
	p.configureDebugVisualizer(p.COV_ENABLE_VR_RENDER_CONTROLLERS,1)

	for _id in range(p.getNumJoints(objUid)):
		joint_indices.append(_id)
		joint_pos = p.readUserDebugParameter(_id)
		targetPosArr.append(joint_pos)
		maxForceArr.append(maxForce)
		print(p.getJointInfo(objUid, _id)[14])
		print(p.getJointInfo(objUid, _id)[15])
		print("-------------Link No. "+str(_id)+"-------------")
	p.setJointMotorControlArray(bodyIndex=objUid,
								jointIndices = joint_indices,
								controlMode = p.POSITION_CONTROL,
								targetPositions = targetPosArr,
								forces = maxForceArr)
	print(p.getJointInfo(objUid, _id)[14])
	print(p.getJointInfo(objUid, _id)[15])
	print("-------------New Step----------")
	

	p.stepSimulation()
	time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(objUid)
print(cubePos,cubeOrn)
p.disconnect()

# python -m pybullet_envs.examples.minitaur_gym_env_example
# python -m pybullet_envs.examples.enjoy_TF_HumanoidBulletEnv_v0_2017may
