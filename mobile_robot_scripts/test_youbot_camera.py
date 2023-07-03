#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from youbotCamGymEnv import youbotCamGymEnv
import datetime
from stable_baselines3 import ppo
from stable_baselines3.common.env_checker import check_env 
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv


def main():  

  env = youbotCamGymEnv(renders=True, isDiscrete=False)
  model = ppo.PPO.load(os.path.join(currentdir,"youbot_camera_trajectory"))
  obs = env.reset()

  for i in range(1000):
      action, _states = model.predict(obs, deterministic=True)
      obs, reward, done, info = env.step(action)
      # print("reward is ", reward)
      env.render(mode='human')
      if done:
        obs = env.reset()

  env.close()

if __name__ == '__main__':
  main()
