import gymnasium as gym
import numpy as np
import math

class PendulumSimulation:
    def __init__(self, render=False, mass=1.0, length=1.0):
        render_mode = "human" if render else None
        self.env = gym.make("Pendulum-v1", render_mode=render_mode)
        
        self.env.unwrapped.m = mass
        self.env.unwrapped.l = length
        
    def reset(self, initial_state=None):
        observation, info = self.env.reset()
        if initial_state is not None:
            self.env.unwrapped.state = np.array(initial_state, dtype=np.float32)
            theta, thetadot = self.env.unwrapped.state
            observation = np.array([np.cos(theta), np.sin(theta), thetadot], dtype=np.float32)
        return self._get_state(observation)

    def step(self, torque):
        action = np.array([torque], dtype=np.float32)
        observation, reward, terminated, truncated, info = self.env.step(action)
        theta, theta_dot = self._get_state(observation)
        return theta, theta_dot, reward, terminated or truncated

    def _get_state(self, observation):
        cos_theta, sin_theta, theta_dot = observation
        theta = math.atan2(sin_theta, cos_theta)
        return theta, theta_dot

    def close(self):
        self.env.close()