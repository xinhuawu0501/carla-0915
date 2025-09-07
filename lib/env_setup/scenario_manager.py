from enum import Enum
import random
from lib.env_setup.carla_env import CarlaEnv


class ScenarioManager:
    def __init__(self, env: CarlaEnv):
        self.env = env
        env.change_weather(rain=random.randint(0, 100), 
                           fog_density=random.randint(0, 100), 
                           cloud=random.randint(0, 100))
        
        self.traffic_size = random.randint(10, 50)
        self.walker_speed = 1 + random.random() # between 1 and 2 m/s

        # self.env.spawn_NPC_cars(self.traffic_size)

        # log current scenario
        self.log_current_scenario()

    def run_scenario(self):
        target_cw = random.choice(self.env.get_all_crosswalk_polygons())
        lanes = self.env._get_lanes_passing_crosswalk(target_cw)
        print(lanes)

        self.ego_route = random.choice(lanes['turning'])
        self.env.draw_waypoints(self.ego_route)


    def log_current_scenario(self):
        weather = self.env.world.get_weather()
        cur_map = self.env.world_map.name

        print(f'Loading {cur_map}\nWeather condition: {weather}')