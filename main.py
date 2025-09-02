from lib.env_setup.base_env import CarBaseEnv
from lib.PythonAPI.carla.agents.navigation.local_planner import LocalPlanner
from lib.util.image_processing import process_rgb_img, cv_display, process_semantic_img
from lib.scenarios.pedestrian_crossing import PedestrianCrossingScenario
import time
import math
import gymnasium as gym
from gymnasium import spaces
import carla
import numpy as np
import queue
import cv2
import random


class CustomPlanner(LocalPlanner):
    def __init__(self, vehicle, map_inst=None, route=None):
        opt_dict = {
            'target_speed': 20.0,     # km/h
            'sampling_resolution': 2.0
        }
        super().__init__(vehicle, opt_dict, map_inst)

        if route:
            self.set_global_plan(route)

    def run_step(self, debug=False):
        """
        Compute next vehicle control using LocalPlanner.
        """
        control = super().run_step(debug=debug)
        return control
    
    def set_route(self, route: carla.WayPoint[]): # type: ignore
        self.set_global_plan(route)

IMG_Y = 600
IMG_X = 800

CITYSCAPES_PALETTE = {
    0: np.array([0, 0, 0]),         # Unlabeled
    1: np.array([70, 70, 70]),      # Buildings
    4: np.array([220, 20, 60]),     # Pedestrians
    10: np.array([0, 0, 142]),      # Vehicles
    12: np.array([128, 64, 128]),   # Road
    13: np.array([244, 35, 232]),   # Sidewalk
    14: np.array([220, 220, 0]),    # Traffic Signs
}

CITYSCAPES_LABEL = {
    0: 'unlabeled',         # Unlabeled
    1: 'buildings',      # Buildings
    4: 'pedestrian',     # Pedestrians
    10: 'vehicle',      # Vehicles
    12: 'road',   # Road
    13: 'sidewalk',   # Sidewalk
    14: 'traffic sign',    # Traffic Signs
}

# Helper: convert RGB image to class ID map
def rgb_to_class_id(img_rgb):
    """
    img_rgb: HWC uint8
    returns: HxW int32 with class IDs
    """
    class_map = np.zeros((img_rgb.shape[0], img_rgb.shape[1]), dtype=np.int32)
    for class_id, color in CITYSCAPES_PALETTE.items():
        mask = np.all(img_rgb == color, axis=2)
        class_map[mask] = class_id
    return class_map

class AgentWithSensor(CarBaseEnv, gym.Env):
    def __init__(self):
       super().__init__()

    #    self.set_sync()
       #!!! TODO: add local planner control to observation space
       self.observation_space = spaces.Dict({
            "image": spaces.Box(low=0, high=1, shape=(3, IMG_Y, IMG_X), dtype=np.float32),
            "velocity": spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32),  # vx, vy
        })
       
       self.action_space = spaces.Box(low=np.array([-1.0, 0.0, 0.0]), high=np.array([1.0, 1.0, 1.0]), dtype=np.float64)

    def planner_control(self):
        return self.planner.run_step(debug=True)
    
    def _get_obs(self):
        try:
            img = self.get_semantic_img()
            img_arr = process_semantic_img(img)
            transposed_img = np.transpose(img_arr, (2, 0, 1))  # to (3, 600, 800)
            
            # Get velocity
            velocity = self.car.get_velocity()
            vx = np.clip(velocity.x / 50.0, -1.0, 1.0)
            vy = np.clip(velocity.y / 50.0, -1.0, 1.0)

            obs = {
                "image": transposed_img.astype(np.float32) / 255.0,
                "velocity": np.array([vx, vy], dtype=np.float32),
            }

            self.debug_semantic(obs)

            return obs

        except queue.Empty:
            print('empty queue')
            return {
                "image": np.zeros((3, IMG_Y, IMG_X), dtype=np.float32),
                "velocity": np.zeros((2,), dtype=np.float32),
            }
    
    def debug_semantic(self, obs):
        """
        obs: dict containing 'image' key (C,H,W) float32 normalized [0,1]
        """
        try:
            semantic_img = obs['image']  # (C,H,W), normalized
            # Convert to HWC uint8 for display
            img_display = np.transpose(semantic_img, (1, 2, 0))  # HWC
            img_display = (img_display * 255).astype(np.uint8)
            img_display = cv2.cvtColor(img_display, cv2.COLOR_RGB2BGR)
            
            # Display the semantic image
            cv_display(img_display)
            
            # Convert RGB to class IDs
            class_map = rgb_to_class_id(img_display)
            unique_classes = np.unique(class_map)         
            print("Detected semantic classes:", [k for k in unique_classes])
            
            # Detect pedestrian presence
            if 4 in unique_classes:
                print("Pedestrian detected! Switching to RL control.")
                self.pedestrian_detected = True
            else:
                self.pedestrian_detected = False

        except Exception as e:
            print("debug_semantic error:", e)

    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.cleanup()
        self.clear_image_queue()
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
 



def init():
    env = AgentWithSensor()
    world = env.world
    client = env.client
   
    try:
        env.reset()
        car = env.car

        while True:
            control = env.planner.run_step(debug=False)
            env.car.apply_control(control)
            if env.is_sync:
                env.world.tick()
            else:
                env.world.wait_for_tick()  
       
    except KeyboardInterrupt:
        pass
    finally:
        env.cleanup()

init()

