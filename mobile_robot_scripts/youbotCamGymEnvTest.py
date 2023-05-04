#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from youbotCamGymEnv import youbotCamGymEnv
import time


def main():

  environment = youbotCamGymEnv(renders=True, isDiscrete=False)

  motorsIds = []
 
  dv = 1
  motorsIds.append(environment._p.addUserDebugParameter("Joint2", -dv, dv, 0))
  motorsIds.append(environment._p.addUserDebugParameter("Joint3", -dv, dv, 0))
  motorsIds.append(environment._p.addUserDebugParameter("Joint4", -dv, dv, 0))
  
  done = False
  while (not done):

    action = []
    for motorId in motorsIds:
      action.append(environment._p.readUserDebugParameter(motorId))

    state, reward, done, info = environment.step(action)
    obs = environment.getEnvObservation()
    
    contact = environment._p.getContactPoints(environment._youbot_cam.youbotCamUid, environment._youbot_cam.youbotCamUid, 
                                        )
    closestPoints = environment._p.getClosestPoints(environment.duckUid, environment._youbot_cam.youbotCamUid, 1000, -1,
                                        environment._youbot_cam.endEffectorId)
    print("Closest Points for ",len(closestPoints))
    print(closestPoints[0])
    print(closestPoints[1])
    print(closestPoints[2])
    print(closestPoints[3])
    print(closestPoints[4])

if __name__ == "__main__":
  main()
