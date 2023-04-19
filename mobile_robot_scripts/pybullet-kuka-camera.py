import pybullet as p
import time
import pybullet_data
import os

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])

RENDER_HEIGHT = 720
RENDER_WIDTH = 960

objUid = p.loadURDF(os.path.join(os.path.dirname(__file__),"kuka_iiwa/model_camera.urdf"), startPos, startOrientation)
_link_name_to_index = {p.getBodyInfo(objUid)[0].decode('UTF-8'):-1,}
link_arr = [];
for _id in range(p.getNumJoints(objUid)):
	_name = p.getJointInfo(objUid, _id)[1].decode('UTF-8')
	_link_name_to_index[_name] = _id
	joint_low_limit = p.getJointInfo(objUid, _id)[8]
	joint_upper_limit = p.getJointInfo(objUid, _id)[9]
	joint_type = p.getJointInfo(objUid, _id)[2]
	# if(joint_type != 4):
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
		print("-------------Joint No. "+str(_id)+"-------------")
		name = list(_link_name_to_index.keys())[_id + 1]
		print(name)
		print(p.getJointInfo(objUid, _id)[14])
		print(p.getJointInfo(objUid, _id)[15])
		print(p.getJointState(objUid, _id)[0])
		print(p.getLinkState(objUid, _id)[0] )
		print(p.getLinkState(objUid, _id)[1])
		print(p.getLinkState(objUid, _id)[2])
		print(p.getLinkState(objUid, _id)[3])
		print(p.getLinkState(objUid, _id)[4])
		print(p.getLinkState(objUid, _id)[5])

		robotPos, robotOrn = p.getBasePositionAndOrientation(objUid)
		linkPos = p.getLinkState(objUid, _id)[0]
		linkOrn = p.getLinkState(objUid, _id)[1]
		invRobotPos, invRobotOrn = p.invertTransform(robotPos, robotOrn)
		linkPosInRobot, linkOrnInRobot = p.multiplyTransforms(invRobotPos, invRobotOrn, linkPos, linkOrn)
		print(linkPosInRobot)
		print(linkOrnInRobot)


	p.setJointMotorControlArray(bodyIndex=objUid,
								jointIndices = joint_indices,
								controlMode = p.POSITION_CONTROL,
								targetPositions = targetPosArr,
								forces = maxForceArr)
	# print(p.getJointInfo(objUid, _id)[14])
	# print(p.getJointInfo(objUid, _id)[15])
	print("-------------New Step----------")
	
	cam_link_id = 7
	cam_focus_link_id = 8
	cam_eye_pos = p.getLinkState(objUid, cam_link_id)[0]
	cam_target_pos = p.getLinkState(objUid, cam_focus_link_id)[0]
	cam_orn = p.getLinkState(objUid, cam_link_id)[1]
	cam_up_vector = p.rotateVector(cam_orn, [0,0,1])

	view_matrix = p.computeViewMatrix(cameraEyePosition = cam_eye_pos,
									cameraTargetPosition = cam_target_pos,
									cameraUpVector = [0,0,1])

	# view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=cam_target_pos,
    #                                                         distance=0.5,
    #                                                         yaw=1.57,
    #                                                         pitch=1.57,
    #                                                         roll=1.57,
    #                                                         upAxisIndex=2)

	proj_matrix = p.computeProjectionMatrixFOV(fov=60,
							aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
							nearVal=0.1,
							farVal=100.0)

	(_, _, px, _, _) = p.getCameraImage(width=RENDER_WIDTH,
												height=RENDER_HEIGHT,
												viewMatrix=view_matrix,
												projectionMatrix=proj_matrix,
												renderer=p.ER_BULLET_HARDWARE_OPENGL)


	delta_x_cam = 0.025
	delta_y_cam = 0.5
	line_start_right = [delta_x_cam, 0, 0]
	line_start_left = [-delta_x_cam, 0, 0]
	line_end_right =[delta_x_cam, delta_y_cam, 0]
	line_end_left = [-delta_x_cam, delta_y_cam, 0]
	inertialQuaternion = p.getQuaternionFromEuler([0,0,0])

	[line_start_right_coord, line_start_right_orn] = p.multiplyTransforms(cam_eye_pos, cam_orn, line_start_right, inertialQuaternion)
	[line_start_left_coord, line_start_left_orn] = p.multiplyTransforms(cam_eye_pos, cam_orn, line_start_left, inertialQuaternion)
	[line_end_right_coord, line_end_right_orn] = p.multiplyTransforms(cam_eye_pos, cam_orn, line_end_right, inertialQuaternion)
	[line_end_left_coord, line_end_left_orn] = p.multiplyTransforms(cam_eye_pos, cam_orn, line_end_left, inertialQuaternion)


	line1 = p.addUserDebugLine(line_start_right_coord, line_end_right_coord, lineColorRGB = [1,0,0])
	line2 = p.addUserDebugLine(line_start_left_coord, line_end_left_coord, lineColorRGB = [1,0,0])

	p.stepSimulation()
	
	p.removeUserDebugItem(line1)
	p.removeUserDebugItem(line2)
	
	time.sleep(1./240.)
p.disconnect()

# python -m pybullet_envs.examples.minitaur_gym_env_example
# python -m pybullet_envs.examples.enjoy_TF_HumanoidBulletEnv_v0_2017may
