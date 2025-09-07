from enum import Enum
import carla
import random
from lib.env_setup.carla_env import CarlaEnv
import yaml
import os

def load_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class ScenarioManager:
    def __init__(self, env: CarlaEnv):
        self.env = env

        self.config = load_yaml(os.path.join(os.getcwd(), 'lib/env_setup/config.yaml'))
        self.traffic_size = random.randint(10, 50)
        self.walker_speed = 1 + random.random() # between 1 and 2 m/s

        # self.env.spawn_NPC_cars(self.traffic_size)

        # log current scenario
        self.load_weather()
        self.log_current_scenario()

    def load_weather(self):
        visibility = self.config["visibility"]

        weather = random.choice(visibility['low'])
        precipitation, fog_density, sun_altitude_angle, cloudiness = (
            float(weather.get(k, 0.0)) for k in ['precipitation', 'fog_density', 'sun_altitude_angle', 'cloudiness']
        )

        print(weather['id'])

        self.env.change_weather(cloudiness=cloudiness, precipitation=precipitation, fog_density=fog_density, sun_altitude_angle=sun_altitude_angle)

    def run_scenario(self):
        target_cw = random.choice(self.env.get_all_crosswalk_polygons())
        lanes = self.env._get_lanes_passing_crosswalk(target_cw)
        print(lanes)

        self.ego_route = random.choice(lanes['turning'])
        self.env.draw_waypoints(self.ego_route)


    def log_current_scenario(self):
        weather = self.env.world.get_weather()
        cur_map = self.env.world_map.name

        print(f'Loading {cur_map}\n')