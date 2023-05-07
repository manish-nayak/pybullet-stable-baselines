import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
import youbot_camera
import random
import pybullet_data
from pkg_resources import parse_version

maxSteps = 1000

RENDER_HEIGHT = 720
RENDER_WIDTH = 960


class youbotCamGymEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self,
                urdfRoot=pybullet_data.getDataPath(),
                actionRepeat=1,
                isEnableSelfCollision=True,
                renders=False,
                isDiscrete=False):
        self._timeStep = 1. / 240.
        self._urdfRoot = urdfRoot
        self._actionRepeat = actionRepeat
        self._isEnableSelfCollision = isEnableSelfCollision
        self._observation = []
        self._envStepCounter = 0
        self._renders = renders
        self._width = 341
        self._height = 256
        self._isDiscrete = isDiscrete
        self.terminated = 0
        self._p = p
        if self._renders:
            cid = p.connect(p.SHARED_MEMORY)
            if (cid < 0):
                p.connect(p.GUI)
            p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])
        else:
            p.connect(p.DIRECT)
        #timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "kukaTimings.json")
        self.seed()
        self.reset()
        observationDim = len(self._youbot_cam.getObservation())
        #print("observationDim")
        #print(observationDim)

        observation_high = np.array([np.finfo(np.float32).max] * observationDim)
        if (self._isDiscrete):
            self.action_space = spaces.Discrete(3)
        else:
            action_dim = 3
            self._action_bound = 5
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
        self.observation_space = spaces.Box(low=0,
                                            high=255,
                                            shape=(self._height, self._width, 4),
                                            dtype=np.uint8)
        self.viewer = None

    def reset(self):
        self.terminated = 0
        p.resetSimulation()
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(self._timeStep)

        self.planeId = p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"))
        self.tableUid = p.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"), basePosition = [0.6,0,0.0], globalScaling = 0.3)
        self.jengaUid = p.loadURDF(os.path.join(self._urdfRoot, "jenga/jenga.urdf"),[0.7,0,0.2], globalScaling = 0.6)
        p.changeVisualShape(self.jengaUid,-1,rgbaColor=[0.58,0.29,0,1])
        self.duckUid = p.loadURDF(os.path.join(self._urdfRoot, "duck_vhacd.urdf"), basePosition = [0.5,-0.1,0.2], globalScaling = 0.75)
        p.changeDynamics(self.duckUid,-1,linearDamping=1, angularDamping=1, rollingFriction=0.1, spinningFriction=0.1)

        self.mugUid = p.loadURDF(os.path.join(self._urdfRoot, "objects/mug.urdf"), basePosition = [0.5,0.1,0.2], globalScaling = 0.5)
        bikeOrn = p.getQuaternionFromEuler([1.57,0,0])
        self.bikeUid = p.loadURDF(os.path.join(self._urdfRoot, "bicycle/bike.urdf"), basePosition = [1, 0.3, 0.1], baseOrientation = bikeOrn, globalScaling = 0.25)


        p.setGravity(0, 0, -10)
        self._youbot_cam = youbot_camera.Youbot_Cam(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
        self._envStepCounter = 0
        p.stepSimulation()
        self._observation = self._youbot_cam.getObservation()
        self.defaultSelfContact = len( p.getContactPoints(self._youbot_cam.youbotCamUid, self._youbot_cam.youbotCamUid) )

        return np.array(self._observation)

    def __del__(self):
        p.disconnect()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        for i in range(self._actionRepeat):
            self._youbot_cam.applyAction(action)
            p.stepSimulation()
            if self._termination():
                break
            self._envStepCounter += 1
        self._observation = self._youbot_cam.getObservation()
        if self._renders:
            time.sleep(self._timeStep)

        done = self._termination()
        reward = self._reward()

        return np.array(self._observation), reward, done, {}

    def render(self, mode='human', close=False):
        if mode != "rgb_array":
            return np.array([])
        return self._youbot_cam.getObservation()

    def getEnvObservation(self):
        return self._observation

    def _termination(self):
        state = p.getLinkState(self._youbot_cam.youbotCamUid, self._youbot_cam.endEffectorId)
        actualEndEffectorPos = state[0]

        if (self.terminated or self._envStepCounter > maxSteps):
            self._observation = self._youbot_cam.getObservation()
            return True
        maxDist = 0.5
        closestPoints = p.getClosestPoints(self.duckUid, self._youbot_cam.youbotCamUid, maxDist, -1,
                                        self._youbot_cam.endEffectorId)
        closestDist = closestPoints[0][8]
        minDist = 0.02

        currentSelfContact = len( p.getContactPoints(self._youbot_cam.youbotCamUid, self._youbot_cam.youbotCamUid) )
        self.hit = currentSelfContact>self.defaultSelfContact
        
        if (closestDist<minDist or currentSelfContact>self.defaultSelfContact):  
            self.terminated = 1
            print(self.defaultSelfContact, currentSelfContact)
            self._observation = self._youbot_cam.getObservation()                
            return True
        return False


    def _reward(self):

        #rewards is height of target object
        blockPos, blockOrn = p.getBasePositionAndOrientation(self.duckUid)
        closestPoints = p.getClosestPoints(self.duckUid, self._youbot_cam.youbotCamUid, 1000, -1,
                                        self._youbot_cam.endEffectorId)

        reward = -1000
        numPt = len(closestPoints)
        # print(numPt)
        # print(closestPoints)

        if (numPt > 0):
            reward = -closestPoints[0][8] * 10
            # print("reward: ", reward)

        
        closest_dist = closestPoints[0][8]
        max_dist = 0.005

        if (closest_dist < max_dist):  
            reward = reward + 1000

        if (self.hit == True):
            reward = -100

        return reward

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = reset
        _seed = seed
        _step = step
