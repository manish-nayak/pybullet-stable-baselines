import os
import copy
import math

import numpy as np


class R2D2:

  def __init__(self, bullet_client, urdfRootPath='', timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self._p = bullet_client
    self.reset()

  def reset(self):
    # robot = self._p.loadURDF(os.path.join(self.urdfRootPath, "r2d2.urdf"),
    #                        [0, 0, .5],
    #                        useFixedBase=False)
    robot = self._p.loadURDF(os.path.join(os.path.dirname(__file__),"../urdf/r2d2_modified.urdf"),
                           [0, 0, .5],
                           useFixedBase=False)
                           
           
    self.robotUniqueId = robot
    # for i in range(self._p.getNumJoints(robot)):
    # 	  print(self._p.getJointInfo(robot,i))

    for wheel in range(self._p.getNumJoints(robot)):
      # self._p.setJointMotorControl2(robot,
      #                               wheel,
      #                               self._p.VELOCITY_CONTROL,
      #                               targetVelocity=0,
      #                               force=0)
      # self._p.setJointMotorControl2(robot,
      #                               wheel,
      #                               self._p.VELOCITY_CONTROL,
      #                               force=0)                              
      self._p.getJointInfo(robot, wheel)
      # self._p.enableJointForceTorqueSensor(robot,wheel,enableSensor = True)
      # if(wheel == 10):
      #   self._p.changeDynamics(robot, wheel, linearDamping=0, angularDamping=0, maxJointVelocity = 0.1)
    

    right_front_wheeel = 2
    right_back_wheel = 3
    left_front_wheel = 6
    left_back_wheel = 7

    self.rightWheels = [right_front_wheeel, right_back_wheel]
    self.leftWheels = [left_front_wheel, left_back_wheel]
    self.maxForce = 350
    self.nMotors = 2 #4 Total
    self.speedMultiplierRight = 40.
    self.speedMultiplierLeft = 40.0

  def getActionDimension(self):
    return self.nMotors

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = []
    pos, orn = self._p.getBasePositionAndOrientation(self.robotUniqueId)

    observation.extend(list(pos))
    observation.extend(list(orn))

    return observation

  def applyAction(self, motorCommands):
    targetVelocityRight = motorCommands[0] * self.speedMultiplierRight
    targetVelocityLeft = motorCommands[1] * self.speedMultiplierLeft
    # for i in range(self._p.getNumJoints(self.robotUniqueId)):
    #   print(self._p.getJointInfo(self.robotUniqueId,i))
    #   print(self._p.getJointState(self.robotUniqueId,i))
      # self._p.changeDynamics(self.robotUniqueId, i, linearDamping=0, angularDamping=0)
    for motor in self.rightWheels:
      self._p.setJointMotorControl2(self.robotUniqueId,
                                    motor,
                                    self._p.VELOCITY_CONTROL,
                                    targetVelocity=targetVelocityRight,
                                    force=self.maxForce)
    for motor in self.leftWheels:
      # print("Left wheel motor id is {}".format(motor))
      self._p.setJointMotorControl2(self.robotUniqueId,
                                    motor,
                                    self._p.VELOCITY_CONTROL,
                                    targetVelocity=targetVelocityLeft,
                                    force=self.maxForce)

    # for motor in [0,1,4,5,8,9,10,11,12,13,14]:
    #   #print("Left wheel motor id is {}".format(motor))
    #   self._p.setJointMotorControl2(self.robotUniqueId,
    #                                 motor,
    #                                 self._p.VELOCITY_CONTROL,
    #                                 targetVelocity=0*targetVelocityLeft,
    #                                 force=0)