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
from stable_baselines3.common.callbacks import ProgressBarCallback



def main():  

  env = youbotCamGymEnv(renders=False, isDiscrete=False)
# It will check your custom environment and output additional warnings if needed
  check_env(env)
  n_procs = 4
  # list of envs 
  num_envs = 3
  envs = [lambda: youbotCamGymEnv(renders=False, isDiscrete=False) for i in range(num_envs)]

  # Vec Env 
  train_env = SubprocVecEnv(envs)

  model = ppo.PPO("CnnPolicy", train_env, tensorboard_log="/tmp/ppo/", verbose=1)
  model.learn(total_timesteps = 10000, progress_bar=False)
  print("############Training completed################")
  model.save(os.path.join(currentdir,"youbot_camera_trajectory"))
  
  # del model

  # env = youbotCamGymEnv(renders=True, isDiscrete=False)
  model = ppo.PPO.load(os.path.join(currentdir,"youbot_camera_trajectory"))
  # obs = env.reset()

  # for i in range(1000):
  #     action, _states = model.predict(obs, deterministic=True)
  #     obs, reward, done, info = env.step(action)
  #     # print("reward is ", reward)
  #     env.render(mode='human')
  #     if done:
  #       obs = env.reset()

  # env.close()

if __name__ == '__main__':
  main()
