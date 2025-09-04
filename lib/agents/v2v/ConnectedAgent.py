from agents.local_planner import CustomPlanner
from env_setup.v2v_channel import V2PChannel
from lib.env_setup.base_env import CarBaseEnv
from lib.scenarios.pedestrian_crossing import PedestrianCrossingScenario

import time
import math
import gymnasium as gym
from gymnasium import spaces
import carla
import numpy as np
import random


class V2PEnv(CarBaseEnv, gym.Env):
    """
    Ego vehicle receives V2P messages from pedestrian.
    Obs: [ego_speed, ped_rx, ped_ry, ped_speed, v2p_valid]
    Action: [throttle (-1..1), steer (-1..1)]
    """
    metadata = {"render_modes": []}

    def __init__(self):
        super().__init__()
        self.set_sync()

        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0,1.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(5,), dtype=np.float32)

        # V2P channel
        self.radio = V2PChannel(latency_steps=2, drop_prob=0.1, range_m=50.0)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.cleanup()
        self.clear_image_queue()
        self.collision_data.clear()

        self.spawn_car()
        self.planner = CustomPlanner(self.car, target_speed=5.0)

        # TODO: add scenario

        for _ in range(5):
            self.world.tick()
            self.radio.tick()

        return self._get_obs(), {}
    
    def get_planner_control(self):
        return self.planner.run_step(debug=True)
    

    def step(self, action):
        rl_control = carla.VehicleControl(
            steer=float(action[0]),
            throttle=float(action[1]),
            brake=float(action[2])
        )
        planner_control = self.get_planner_control()
        # Blend RL + planner
        alpha = 0.5  # 0 = planner only, 1 = RL only
        final_control = carla.VehicleControl(
            steer=planner_control.steer * (1 - alpha) + rl_control.steer * alpha,
            throttle=planner_control.throttle * (1 - alpha) + rl_control.throttle * alpha,
            brake=rl_control.brake  # usually RL decides braking fully
        )

        self.apply_control(planner_control)

        # Pedestrian broadcasts V2P
        ped_tf = self.ped.get_transform()
        ped_vel = self.ped.get_velocity()
        ped_speed = math.hypot(ped_vel.x, ped_vel.y)
        self.radio.broadcast(self.ped.id, {"x": ped_tf.location.x,
                                           "y": ped_tf.location.y,
                                           "speed": ped_speed})

        # Tick sim
        self.world.tick()
        self.radio.tick()

        obs = self._get_obs()
        if self.collision_data:
            reward = -100
            done = True
            
        truncated = False
        return obs, reward, done, truncated, {}

    def _get_obs(self):
        ego_tf = self.ego.get_transform()
        ego_vel = self.ego.get_velocity()
        ego_speed = math.hypot(ego_vel.x, ego_vel.y)

        msgs = self.radio.receive((ego_tf.location.x, ego_tf.location.y))
        if msgs:
            _, m = msgs[-1]
            rx, ry = m["rx"], m["ry"]
            speed = m["speed"]
            valid = 1.0
        else:
            rx, ry, speed, valid = 0.0, 0.0, 0.0, 0.0

        return np.array([ego_speed, rx, ry, speed, valid], dtype=np.float32)


    def close(self):
        self.cleanup()
 

