#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from r2d2GymEnv import r2d2GymEnv
isDiscrete = False


def main():

  environment = r2d2GymEnv(renders=True, isDiscrete=isDiscrete)
  environment.reset()

  targetVelocitySliderRight = environment._p.addUserDebugParameter("wheelVelocityRight", -1, 1, 0)
  targetVelocitySliderLeft = environment._p.addUserDebugParameter("wheelVelocityLeft", -1, 1, 0)

  while (True):
    targetVelocityRight = environment._p.readUserDebugParameter(targetVelocitySliderRight)
    targetVelocityLeft = environment._p.readUserDebugParameter(targetVelocitySliderLeft)
    if (isDiscrete):
      discreteAction = 0
      if (targetVelocityRight < -0.33):
        discreteAction = 0
      else:
        if (targetVelocityRight > 0.33):
          discreteAction = 6
        else:
          discreteAction = 3
      if (targetVelocityLeft > -0.17):
        if (targetVelocityLeft > 0.17):
          discreteAction = discreteAction + 2
        else:
          discreteAction = discreteAction + 1
      action = discreteAction
    else:
      action = [targetVelocityRight, targetVelocityLeft]
    state, reward, done, info = environment.step(action)
    obs = environment.getExtendedObservation()
    print("obs")
    print(obs)


if __name__ == "__main__":
  main()
