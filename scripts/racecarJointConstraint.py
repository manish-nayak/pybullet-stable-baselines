#a mimic joint can act as a gear between two joints
#you can control the gear ratio in magnitude and sign (>0 reverses direction)

import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", 0, 0, -1)
car = p.loadURDF("racecar/racecar_differential.urdf", [0, 0, 0.2])
for i in range(p.getNumJoints(car)):
  print(p.getJointInfo(car, i))
  p.setJointMotorControl2(car, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

c = p.createConstraint(car,
                                 9,
                                 car,
                                 11,
                                 jointType=p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car,
                              10,
                              car,
                              13,
                              jointType=p.JOINT_GEAR,
                              jointAxis=[0, 1, 0],
                              parentFramePosition=[0, 0, 0],
                              childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,
                              9,
                              car,
                              13,
                              jointType=p.JOINT_GEAR,
                              jointAxis=[0, 1, 0],
                              parentFramePosition=[0, 0, 0],
                              childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,
                              16,
                              car,
                              18,
                              jointType=p.JOINT_GEAR,
                              jointAxis=[0, 1, 0],
                              parentFramePosition=[0, 0, 0],
                              childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car,
                              16,
                              car,
                              19,
                              jointType=p.JOINT_GEAR,
                              jointAxis=[0, 1, 0],
                              parentFramePosition=[0, 0, 0],
                              childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,
                              17,
                              car,
                              19,
                              jointType=p.JOINT_GEAR,
                              jointAxis=[0, 1, 0],
                              parentFramePosition=[0, 0, 0],
                              childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,
                              1,
                              car,
                              18,
                              jointType=p.JOINT_GEAR,
                              jointAxis=[0, 1, 0],
                              parentFramePosition=[0, 0, 0],
                              childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
c = p.createConstraint(car,
                              3,
                              car,
                              19,
                              jointType=p.JOINT_GEAR,
                              jointAxis=[0, 1, 0],
                              parentFramePosition=[0, 0, 0],
                              childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, -10)
  time.sleep(0.01)
#p.removeConstraint(c)
