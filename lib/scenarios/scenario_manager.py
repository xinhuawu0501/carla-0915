from enum import Enum
import random
from env_setup.carla_env import CarlaEnv

scenarios = []

class ScenarioManager:
    def __init__(self, env: CarlaEnv):
        self.env = env

        self.scenario = random.choice(scenarios)
        env.change_weather(rain=random.randint(0, 100), 
                           fog_density=random.randint(0, 100), 
                           cloud=random.randint(0, 100))
        
        self.traffic_size = random.randint(10, 50)
        self.walker_speed = 1 + random.random() # between 1 and 2 m/s

        self.env.spawn_NPC_cars(self.traffic_size)
        self.ego_route = self.get_ego_route()

        # log current scenario
        self.log_current_scenario()

    def get_ego_route(self):
        return []

    def log_current_scenario(self):
        weather = self.env.world.get_weather()
        cur_map = self.env.world_map.name

        print(f'Loading {cur_map}\nWeather condition: {weather}')