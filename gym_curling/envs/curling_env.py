# Main
import gym
from gym import error, spaces, utils
from gym.utils import seeding

import os
import pybullet as p
import pybullet_data as pd
import math
import numpy as np
import random


class CurlingEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40,
                                     cameraTargetPosition=[0.55, -0.35, 0.2])  
        ## ? continuous space with # axes
        ## Action space:
        ## Observation space:
        self.action_space = spaces.Box(np.array([-1]*4), np.array([1]*4))
        self.observation_space = spaces.Box(np.array([-1]*3), np.array([1]*3))

    def step(self, action):
        pass

    def reset(self):
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setGravity(0, 0, -10)
        urdfRootPath = pd.getDataPath()
        

    def render(self, mode='human'):
        pass

    def close(self):
        pass
