from enum import Enum
import carla
import random
from lib.agents.local_planner import CustomPlanner
from lib.env_setup.car import Car
from lib.env_setup.carla_env import CarlaEnv
import yaml
import os
import time

from lib.env_setup.pedestrian import Pedestrian

def load_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class ScenarioManager:
    def __init__(self, env: CarlaEnv):
        self.env = env
        self.world = self.env.world
        self.settings = self.world.get_settings()
        self.sidewalks = self.env.get_sidewalks()

        self.config = load_yaml(os.path.join(os.getcwd(), 'lib/env_setup/config.yaml'))
        self.traffic_size = random.randint(10, 50)
        self.walker_speed = 1 + random.random() # between 1 and 2 m/s
        self.ego_target_speed = 10

        # self.env.spawn_NPC_cars(self.traffic_size)
        self.get_actor_routes()

        # log current scenario
        self.load_weather()
        self.log_current_scenario()

    def set_ego(self, ego: Car, ego_planner: CustomPlanner):
        self.egp = ego
        self.ego_planner = ego_planner

    def get_actor_routes(self):
        try:
            target_polygon = random.choice(self.env.get_all_crosswalk_polygons())
            lanes = self.env._get_lanes_passing_crosswalk(target_polygon)
            # calculate ego vehicle route
            self.ego_route = random.choice(lanes['turning'])
            collision_wp = self.ego_route[int(len(self.ego_route) / 2)]

           # determine which side in crosswalk polygon
            closest_cw = min(target_polygon, key=lambda loc: self.env.get_distance(loc, collision_wp.transform.location))
            opposite_cw = max(target_polygon, key=lambda loc: self.env.get_distance(loc, closest_cw))
        
            # calculate walker route
            closest_sidewalk_wp = min(self.sidewalks, key=lambda sw: self.env.get_distance(sw.transform.location, closest_cw))
            closest_sidewalk_wp_loc = closest_sidewalk_wp.transform.location
            opposite_sidewalk_wp = min(self.sidewalks, key=lambda sw: self.env.get_distance(sw.transform.location, opposite_cw))
            opposite_sidewalk_loc = opposite_sidewalk_wp.transform.location

            self.walker_route = [closest_sidewalk_wp_loc, closest_cw, collision_wp.transform.location, opposite_cw, opposite_sidewalk_loc]
            
            # debugging util
            self.env.draw_locations(self.walker_route, 'walker route')
            self.env.draw_waypoints(self.ego_route)
            self.env.draw_waypoints([collision_wp], 'collision')
            self.env.move_spectator_to_loc(collision_wp.transform.location)
        except Exception as e:
            print(e)           

    def get_route_len(self, route: list[carla.Waypoint]):
        d = 0

        try:
            for i in range(len(route) - 1):
                cur = route[i].transform.location
                next = route[i + 1].transform.location
                d += self.env.get_distance(cur, next)
            
            return d
        except Exception as e:
            print(e)
            return d

    def run_scenario(self, ego: Car, planner: CustomPlanner):
        try:
            # calculate ego's estimated time to crosswalk
            mid_i = int(len(self.ego_route) / 2)
            ego_d_to_cw = self.get_route_len(self.ego_route[:mid_i + 1])
            ego_time_to_cw = ego_d_to_cw / (self.ego_target_speed * 1000 / 60 / 60) # to m / s

            # calculate walker time to crosswalk
            walker_d_to_cw = self.env.get_distance(self.walker_route[0], self.walker_route[2])
            walker_time_to_cw = walker_d_to_cw / self.walker_speed # sec
            
            self.ped = Pedestrian(self.env.world, route=self.walker_route, speed=self.walker_speed)
            fixed_delta_seconds = self.settings.fixed_delta_seconds

            print(f'ego_time_to_cw: {ego_time_to_cw}\nwalker_time_to_cw: {walker_time_to_cw}')

            #TODO fix walker too slow issue
            walker_delay = max(0, ego_time_to_cw - walker_time_to_cw) 
            delay_tick = int(walker_delay / fixed_delta_seconds)
            self.walker_start_frame = self.world.get_snapshot().frame + delay_tick
            print(self.walker_start_frame)
          
        except Exception as e:
            print(f'run scenario fail: {e}')

    def tick(self):
        """Call this once per env.step() to update scenario logic"""
        frame = self.world.get_snapshot().frame
        if self.ped.walker and self.walker_start_frame and frame >= self.walker_start_frame:
            if not self.ped.has_started:  
                self.ped.start_walker(self.walker_route)
                print(f'walker starts at {frame}')


    def load_weather(self):
        visibility = self.config["visibility"]

        weather = random.choice(visibility['high'])
        precipitation, fog_density, sun_altitude_angle, cloudiness = (
            float(weather.get(k, 0.0)) for k in ['precipitation', 'fog_density', 'sun_altitude_angle', 'cloudiness']
        )

        print(weather['id'])

        self.env.change_weather(cloudiness=cloudiness, precipitation=precipitation, fog_density=fog_density, sun_altitude_angle=sun_altitude_angle)



    def log_current_scenario(self):
        weather = self.env.world.get_weather()
        cur_map = self.env.world_map.name

        print(f'Loading {cur_map}\n')