import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import math
import random
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
from pybullet_utils import bullet_client as bc

maxSteps = 200

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
        self._physics_client_id = -1
        # self._p = p
        if self._renders:
            # self.cid = p.connect(p.SHARED_MEMORY)
            self._pybullet_client = bc.BulletClient(connection_mode=p.SHARED_MEMORY)
            # if (self.cid < 0):
            #     self.cid = p.connect(p.GUI)
            if (self._pybullet_client < 0):
                self._pybullet_client = bc.BulletClient(connection_mode = p.GUI)
            self._pybullet_client.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])
        else:
            # self.cid = p.connect(p.DIRECT)
            self._pybullet_client = bc.BulletClient(p.DIRECT)
            print("Client Id during Init - ", self._physics_client_id)
        #timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "kukaTimings.json")
        self._physics_client_id = self._pybullet_client._client
            
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
        print("reset Beginning cid is", self._physics_client_id)
        self._pybullet_client.resetSimulation()
        self._pybullet_client.setPhysicsEngineParameter(numSolverIterations=150)
        self._pybullet_client.setTimeStep(self._timeStep)

        self.planeId = self._pybullet_client.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"))
        self.tableUid = self._pybullet_client.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"), basePosition = [0.6,0,0.0], globalScaling = 0.3)
        self.jengaUid = self._pybullet_client.loadURDF(os.path.join(self._urdfRoot, "jenga/jenga.urdf"),[0.7,0,0.2], globalScaling = 0.6)
        self._pybullet_client.changeVisualShape(self.jengaUid,-1,rgbaColor=[0.58,0.29,0,1])
        
        xpos = 0.5 + 0.05 * random.random()
        ypos = -0.1 + 0.025 * random.random()
        ang = 3.1415925438 * random.random()
        orn = self._pybullet_client.getQuaternionFromEuler([0, 0, ang])

        self.duckUid = self._pybullet_client.loadURDF(os.path.join(self._urdfRoot, "duck_vhacd.urdf"), basePosition = [xpos,ypos,0.2], baseOrientation = orn, globalScaling = 0.75)
        self._pybullet_client.changeDynamics(self.duckUid,-1,linearDamping=1, angularDamping=1, rollingFriction=0.1, spinningFriction=0.1)

        self.mugUid = self._pybullet_client.loadURDF(os.path.join(self._urdfRoot, "objects/mug.urdf"), basePosition = [0.5,0.1,0.2], globalScaling = 0.5)
        bikeOrn = self._pybullet_client.getQuaternionFromEuler([1.57,0,0])
        self.bikeUid = self._pybullet_client.loadURDF(os.path.join(self._urdfRoot, "bicycle/bike.urdf"), basePosition = [1, 0.3, 0.1], baseOrientation = bikeOrn, globalScaling = 0.25)


        self._pybullet_client.setGravity(0, 0, -10)
        self._youbot_cam = youbot_camera.Youbot_Cam(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
        self._envStepCounter = 0
        self._pybullet_client.stepSimulation()
        self._observation = self._youbot_cam.getObservation()
        self.defaultSelfContact = len( self._pybullet_client.getContactPoints(self._youbot_cam.youbotCamUid, self._youbot_cam.youbotCamUid) )

        return np.array(self._observation)

    def __del__(self):
        print("Delete function cid - ", self._physics_client_id)
        connection_state = self._pybullet_client.getConnectionInfo(self._physics_client_id)
        print("Connection State - ", connection_state)
        if self._pybullet_client.isConnected(self._physics_client_id):
            print("Delete the Client")
            self._pybullet_client.disconnect()
            # self.cid = -1
        # p.disconnect()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        for i in range(self._actionRepeat):
            self._youbot_cam.applyAction(action)
            self._pybullet_client.stepSimulation()
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
        state = self._pybullet_client.getLinkState(self._youbot_cam.youbotCamUid, self._youbot_cam.endEffectorId)
        actualEndEffectorPos = state[0]

        if (self.terminated or self._envStepCounter > maxSteps):
            self._observation = self._youbot_cam.getObservation()
            return True
        maxDist = 0.5
        closestPoints = self._pybullet_client.getClosestPoints(self.duckUid, self._youbot_cam.youbotCamUid, maxDist, -1,
                                        self._youbot_cam.endEffectorId)
        closestDist = 100
        if(len(closestPoints)>0):
            closestDist = closestPoints[0][8]
        minDist = 0.02

        currentSelfContact = len( self._pybullet_client.getContactPoints(self._youbot_cam.youbotCamUid, self._youbot_cam.youbotCamUid) )
        self.hit = currentSelfContact>self.defaultSelfContact
        
        if (closestDist<minDist or currentSelfContact>self.defaultSelfContact):  
            self.terminated = 1
            print("Termination of epsiode", self.defaultSelfContact, currentSelfContact)
            self._observation = self._youbot_cam.getObservation()                
            return True
        return False


    def _reward(self):

        #rewards is height of target object
        blockPos, blockOrn = self._pybullet_client.getBasePositionAndOrientation(self.duckUid)
        closestPoints = self._pybullet_client.getClosestPoints(self.duckUid, self._youbot_cam.youbotCamUid, 1000, -1,
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
            print("Maximum Reward Attained")

        if (self.hit == True):
            reward = -100

        return reward

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = reset
        _seed = seed
        _step = step
