#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from r2d2GymEnv import r2d2GymEnv
import datetime
from stable_baselines3 import ppo
#from gym.wrappers.monitoring.video_recorder import VideoRecorder
from stable_baselines3.common.vec_env import VecVideoRecorder, DummyVecEnv
import imageio
import numpy as np

def main():

  env = r2d2GymEnv(renders=False, isDiscrete=False)
  model = ppo.PPO.load(os.path.join(currentdir,"r2d2_heading"), env=env)

  obs = env.reset()
  after_training = os.path.join(currentdir, "videos/after_training.mp4")

  # video = VideoRecorder(env, after_training)
  images = []
  img = model.env.render(mode='rgb_array')


  for i in range(1000):
      action, _states = model.predict(obs, deterministic=True)
      obs, reward, done, info = env.step(action)
      # print("reward is ", reward)
      # env.render()
      img = model.env.render(mode='rgb_array')
      images.append(img)
      # video.capture_frame()
      if done:
        obs = env.reset()

  env.close()
  # video.close()
  imageio.mimsave(os.path.join(currentdir,'videos/r2d2_tracker.gif'), [np.array(img) for i, img in enumerate(images) if i%2 == 0], fps=15)
  np.save(os.path.join(currentdir,'videos/images_arr'),images)

if __name__ == '__main__':
  main()
