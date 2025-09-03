from agents.local_planner import CustomPlanner
from lib.env_setup.base_env import CarBaseEnv
from lib.scenarios.pedestrian_crossing import PedestrianCrossingScenario

import time
import math
import gymnasium as gym
from gymnasium import spaces
import carla
import numpy as np
import random


class AgentWithV2V(CarBaseEnv, gym.Env):
    def __init__(self):
       super().__init__()

    #    self.set_sync()
       #!!! TODO: add local planner control to observation space
       self.observation_space = spaces.Dict({
            "velocity": spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32),  # vx, vy
        })
       
       self.action_space = spaces.Box(low=np.array([-1.0, 0.0, 0.0]), high=np.array([1.0, 1.0, 1.0]), dtype=np.float64)

    def planner_control(self):
        return self.planner.run_step(debug=True)
    
    def _get_obs(self):
        try:
            # Get velocity
            velocity = self.car.get_velocity()
            vx = np.clip(velocity.x / 50.0, -1.0, 1.0)
            vy = np.clip(velocity.y / 50.0, -1.0, 1.0)

            obs = {
                "velocity": np.array([vx, vy], dtype=np.float32),
            }

            return obs

        except Exception as e:
            print(e)
            return {
                "velocity": np.zeros((2,), dtype=np.float32),
            }
 
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.cleanup()
        self.collision_data.clear()

        self.spawn_car()
        route = PedestrianCrossingScenario(self.client, 1)
        self.planner = CustomPlanner(self.car, route=route)

        return self._get_obs(), {}
    
    def step(self, action):
        rl_control = carla.VehicleControl(
            steer=float(action[0]),
            throttle=float(action[1]),
            brake=float(action[2])
        )

        planner_control = self.planner.run_step(debug=False)

        # Blend RL + planner
        alpha = 0.5  # 0 = planner only, 1 = RL only
        final_control = carla.VehicleControl(
            steer=planner_control.steer * (1 - alpha) + rl_control.steer * alpha,
            throttle=planner_control.throttle * (1 - alpha) + rl_control.throttle * alpha,
            brake=rl_control.brake  # usually RL decides braking fully
        )

        self.apply_control(planner_control)
        self.world.tick()

        v = self.car.get_velocity()
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2) * 3.6

        done = False
        reward = 0

        if self.collision_data:
            reward = 100
            done = True


        obs = self._get_obs()
        # print(f'step {self.step_count} obs: {obs}')

        return obs, reward, done, False, {}
    
    
    
    def close(self):
        self.cleanup()

        print(f'cleaned up')
 

