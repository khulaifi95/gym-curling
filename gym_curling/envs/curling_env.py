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
        self.action_space = spaces.Box(np.array([-1]*3), np.array([1]*3))
        self.observation_space = spaces.Box(np.array([-1]*3), np.array([1]*3))

    def step(self, action):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        orientation = p.getQuaternionFromEuler([0.,-math.pi,math.pi/2.])
        dv = 0.005
        dx = action[0] * dv
        dy = action[1] * dv
        dz = action[2] * dv
        fingers = action[3]

        currentPose = p.getLinkState(self.pandaUid, 11)
        currentPosition = currentPose[0]
        newPosition = [currentPosition[0] + dx,
                       currentPosition[1] + dy,
                       currentPosition[2] + dz]
        jointPoses = p.calculateInverseKinematics(self.pandaUid,11,newPosition, orientation)

        p.setJointMotorControlArray(self.pandaUid, list(range(7))+[9,10], p.POSITION_CONTROL, list(jointPoses)+2*[fingers])

        p.stepSimulation()

        state_object, _ = p.getBasePositionAndOrientation(self.objectUid)
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
        if state_object[2]>0.45:
            reward = 1
            done = True
        else:
            reward = 0
            done = False
        info = state_object
        observation = state_robot + state_fingers
        return observation, reward, done, info

    def reset(self):
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setGravity(0, 0, -10)
        urdfRootPath = pd.getDataPath()

        planeUid = p.loadURDF("Plane.urdf", [0, 0, -0.65])

        self.curlingUid = p.loadURDF("Curling.urdf", useFixedBase=True)

        tableUid = p.loadURDF("table.urdf"), [0.5, 0, -0.65]
        tryUid = p.loadURDF("traybox.urdf"), [0.65, 0, 0]

        state_object = [random.uniform(
            0.5, 0.8), random.uniform(-0.2, 0.2), 0.05]
        self.objectUid

    def render(self, mode='human'):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0.7, 0, 0.05],
            distance=0.7, yaw=90, pitch=-70,
            roll=0, upAxisIndex=2)

        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=float(960)/720,
                                                   nearVal=0.1, farVal=100.0)

        (_, _, px, _, _) = p.getCameraImage(width=960, height=720,
                                            viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720, 960, 4))
        rgb_array = rgb_array[:, :, :3]

        return rgb_array

    def close(self):
        p.disconnect()
