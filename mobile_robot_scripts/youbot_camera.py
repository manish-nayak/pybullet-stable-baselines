import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import numpy as np
import copy
import math
import pybullet_data

RENDER_HEIGHT = 720
RENDER_WIDTH = 960

class Youbot_Cam:

    def __init__(self, urdfRootPath = pybullet_data.getDataPath(), timeStep=0.01, physicsClientId = 0):
        self.urdfRootPath = urdfRootPath
        self.timeStep = timeStep
        self.maxVelocity = 1.0
        self.maxForce = 500
        self.useInversKinematics = 0
        self.egoMotionJointForce = 1e+6
        self.cameraLinkIndex = 21
        self.cameraFocusIndex = 22
        self.movableJoints = [16,17,18]
        self.endEffectorId = 20
        self.camLinkid = 21
        self.camFocusId = 22
        self.width = 341
        self.height = 256

        self.reset(physicsClientId) #Resetting the Robot

    def reset(self, physicsClientId):
        startPos = [0,0,0]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        flags = p.URDF_USE_SELF_COLLISION
        self.youbotCamUid  = p.loadURDF(os.path.join(currentdir, "youbot_description/youbot.urdf"), basePosition=startPos, baseOrientation=startOrientation, flags = flags, physicsClientId = physicsClientId)
        self.numJoints = p.getNumJoints(self.youbotCamUid)

        self.link_name_to_index = {p.getBodyInfo(self.youbotCamUid)[0].decode('UTF-8'):-1,}
        self.jointLowerLimit = []
        self.jointUpperLimit = []
        self.jointType = []

        for _id in range(self.numJoints):
            _name = p.getJointInfo(self.youbotCamUid, _id)[1].decode('UTF-8')
            self.link_name_to_index[_name] = _id
            self.jointLowerLimit.append(p.getJointInfo(self.youbotCamUid, _id)[8])
            self.jointUpperLimit.append(p.getJointInfo(self.youbotCamUid, _id)[9])
            self.jointType.append(p.getJointInfo(self.youbotCamUid, _id)[2])
        
        jointRefPos = [1.17, -3.19, 0.25]
        for jointIndex, jointVal in enumerate(self.movableJoints):
            p.resetJointState(self.youbotCamUid, jointVal, jointRefPos[jointIndex])
            p.setJointMotorControl2(self.youbotCamUid,
                              jointVal,
                              p.POSITION_CONTROL,
                              targetPosition=jointRefPos[jointIndex],
                              force=self.maxForce)

    def applyAction(self, motorCommands):

        # Consider only forward kinematics for now
        # joint2cmd = motorCommands[0]
        # joint3cmd = motorCommands[1]
        # joint4cmd = motorCommands[2]

        # p.setJointMotorControl2(self.youbotCamUid,
        #                       16,
        #                       p.VELOCITY_CONTROL,
        #                       targetVelocity=joint2cmd,
        #                       force=self.maxForce)

        # p.setJointMotorControl2(self.youbotCamUid,
        #                       17,
        #                       p.VELOCITY_CONTROL,
        #                       targetVelocity=joint3cmd,
        #                       force=self.maxForce)

        # p.setJointMotorControl2(self.youbotCamUid,
        #                       18,
        #                       p.VELOCITY_CONTROL,
        #                       targetVelocity=joint4cmd,
        #                       force=self.maxForce)

        forceArr = [self.maxForce, self.maxForce, self.maxForce]
        p.setJointMotorControlArray(bodyIndex=self.youbotCamUid,
								jointIndices = self.movableJoints,
								controlMode = p.VELOCITY_CONTROL,
								targetVelocities = motorCommands,
								forces = forceArr)

        for jointId in range(self.numJoints):
            if jointId not in self.movableJoints:
                p.setJointMotorControl2(self.youbotCamUid,
                                        jointId,
                                        p.POSITION_CONTROL,
                                        targetPosition=0,
                                        force=self.egoMotionJointForce)


    def getObservation(self):

        cam_eye_pos = p.getLinkState(self.youbotCamUid, self.camLinkid)[0]
        cam_target_pos = p.getLinkState(self.youbotCamUid, self.camFocusId)[0]
        # cam_orn = p.getLinkState(self.youbotCamUid, self.camLinkid)[1]
        # cam_up_vector = p.rotateVector(cam_orn, [0,0,1])

        view_matrix = p.computeViewMatrix(cameraEyePosition = cam_eye_pos,
                                        cameraTargetPosition = cam_target_pos,
                                        cameraUpVector = [0,0,1])


        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                aspect=float(self.width) / self.height,
                                nearVal=0.1,
                                farVal=10.0)

        (_, _, img_arr, _, _) = p.getCameraImage(width=self.width,
                                            height=self.height,
                                            viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        np_img_arr = np.reshape(img_arr, (self.height, self.width, 4))
        self._observation = np_img_arr

        return self._observation