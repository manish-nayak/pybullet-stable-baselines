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

  env = youbotCamGymEnv(renders=False, isDiscrete=False)
# It will check your custom environment and output additional warnings if needed
  check_env(env)
  n_procs = 4
  # list of envs 
  num_envs = 3
  envs = [lambda: youbotCamGymEnv(renders=False, isDiscrete=False) for i in range(num_envs)]