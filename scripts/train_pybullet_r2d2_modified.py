#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from r2d2GymEnv import r2d2GymEnv
import datetime
from stable_baselines3 import ppo
from stable_baselines3.common.env_checker import check_env

def main():

  env = r2d2GymEnv(renders=False, isDiscrete=False)
# It will check your custom environment and output additional warnings if needed
  check_env(env)

  model = ppo.PPO("MlpPolicy", env, verbose=1).learn(40000)
  print("############Training completed################")
  model.save(os.path.join(currentdir,"r2d2_heading_stadium"))
  obs = env.reset()
  env.close()

  # env2 = r2d2GymEnv(renders=True, isDiscrete=False)
  # obs = env2.reset()
  # for i in range(1000):
  #     action, _states = model.predict(obs, deterministic=True)
  #     obs, reward, done, info = env2.step(action)
  #     #print("reward is ", reward)
  #     env2.render(mode='human')
  #     if done:
  #       obs = env2.reset()

  # env2.close()

if __name__ == '__main__':
  main()
