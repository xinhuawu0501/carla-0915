from lib.agents.local_planner import CustomPlanner
from lib.env_setup.car import Car, Sensor
from lib.env_setup.carla_env import CarlaEnv
from lib.env_setup.scenario_manager import ScenarioManager
from lib.util.image_processing import process_semantic_img
from lib.scenarios.pedestrian_crossing import PedestrianCrossingScenario
from lib.constants.camera import IMG_X, IMG_Y

import time
import random
import math
import gymnasium as gym
from gymnasium import spaces
import carla
import numpy as np
import queue



class AgentWithSensor(Car, gym.Env):
    def __init__(self):
       self.env = CarlaEnv()
       super().__init__(self.env.world)

       self.env.set_sync()

       self.observation_space = spaces.Dict({
            # "planner_control": spaces.Box(low=np.array([-1.0, 0.0, 0.0]), high=np.array([1.0, 1.0, 1.0]), dtype=np.float64),
            "image": spaces.Box(low=0, high=1, shape=(3, IMG_Y, IMG_X), dtype=np.float32),
            "velocity": spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32),  # vx, vy
        })
       
       self.action_space = spaces.Box(low=np.array([-1.0, 0.0, 0.0]), high=np.array([1.0, 1.0, 1.0]), dtype=np.float64)

    def _get_obs(self):
        try:
            img = self.get_raw_img_from_q(Sensor.SEMANTIC)
            img_arr = process_semantic_img(img)
            transposed_img = np.transpose(img_arr, (2, 0, 1))  # to (3, 600, 800)
            
            # Get velocity
            velocity = self.car.get_velocity()
            vx = np.clip(velocity.x / 50.0, -1.0, 1.0)
            vy = np.clip(velocity.y / 50.0, -1.0, 1.0)

            obs = {
                # "planner_control": 
                "image": transposed_img.astype(np.float32) / 255.0,
                "velocity": np.array([vx, vy], dtype=np.float32),
            }

            # self.debug_semantic(obs)

            return obs

        except queue.Empty:
            print('empty queue')
            return {
                "image": np.zeros((3, IMG_Y, IMG_X), dtype=np.float32),
                "velocity": np.zeros((2,), dtype=np.float32),
            }
        
    def setup_scenario(self):
        self.scenario = ScenarioManager(self.env)
        ego_target_speed = self.scenario.ego_target_speed
        ego_route = self.scenario.ego_route

        sp = ego_route[0].transform
        self.spawn_self(spawn_point=sp, sensor_options={Sensor.RGB: True, Sensor.SEMANTIC: False})
        
        if self.car is None:
            raise RuntimeError("Car failed to spawn. Check spawn point or blueprint.")

        self.planner = CustomPlanner(self.car, route=ego_route, target_speed=ego_target_speed)
        self.scenario.run_scenario(ego=self, planner=self.planner)

        while True:
            control = self.get_planner_control()
            self.car.apply_control(control)

            # self.display_img(img_type=Sensor.RGB)

            if self.env.is_sync:
                self.env.world.tick()
                self.scenario.tick()
            else:
                self.env.world.wait_for_tick()  
 
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.env.cleanup()
        self.cleanup()

        self.setup_scenario()

        self.world.tick()
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
        self.env.cleanup()
        self.cleanup()

        print(f'cleaned up')
 

