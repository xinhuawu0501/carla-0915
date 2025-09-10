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
from lib.util.transform import get_direction

def load_yaml(file_path):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)

class ScenarioManager:
    def __init__(self, env: CarlaEnv):
        self.env = env
        self.world = self.env.world
        self.settings = self.world.get_settings()
        self.sidewalks = self.env.get_sidewalks()

        #TODO: move scenario config to config.yaml
        self.config = load_yaml(os.path.join(os.getcwd(), 'lib/env_setup/config.yaml'))
        self.traffic_size = random.randint(10, 50)
        self.walker_max_speed = 1.4 + random.uniform(-0.6, 0.6) #between 0.8 and 2 m/s
        self.ego_target_speed = 50
        self.num_of_walker = 5

        self.walker_start_frame = []
        self.cur_walker_i = 0

        # self.env.spawn_NPC_cars(self.traffic_size)

        # log current scenario
        self.load_weather()
        self.log_current_scenario()

    def set_ego(self, ego: Car, ego_planner: CustomPlanner):
        self.ego = ego
        self.ego_planner = ego_planner

    def get_actor_routes(self):
        try:
            #TODO: 
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
            # self.env.draw_locations(self.walker_route, 'walker route')
            # self.env.draw_waypoints(self.ego_route)
            # self.env.draw_waypoints([collision_wp], 'collision')
            self.env.move_spectator_to_loc(collision_wp.transform.location)
        except Exception as e:
            print(e)           

    def get_route_len(self, route: list[carla.Waypoint] | list[carla.Location]):
        d = 0

        try:
            for i in range(len(route) - 1):
                # Waypoint obj
                if hasattr(route[i], 'transform'):
                    cur = route[i].transform.location
                    next = route[i + 1].transform.location
                    d += self.env.get_distance(cur, next)
                else: # Location obj
                    cur = route[i]
                    next = route[i + 1]
                    d += self.env.get_distance(cur, next)
            
            return d
        except Exception as e:
            print(e)
            return d

    def run_scenario(self):
        try:
            # get ego and walker route
            self.get_actor_routes()

            # calculate ego's estimated time to crosswalk
            ego_collision_point_i = int(len(self.ego_route) / 2)
            ego_d_to_cw = self.get_route_len(self.ego_route[:ego_collision_point_i + 1])
            ego_speed = self.ego_target_speed * 1000 / 60 / 60 # to m / s
            
            buffer = random.uniform(0.1, 0.3)
            ego_time_to_cw = ego_d_to_cw / (ego_speed - buffer) 

            # calculate walker time to crosswalk
            walker_collision_point_i = 2
            walker_d_to_cw = self.get_route_len(self.walker_route[:walker_collision_point_i + 1])
            walker_time_to_cw = walker_d_to_cw / self.walker_max_speed # sec
            
            fixed_delta_seconds = self.settings.fixed_delta_seconds

            walker_delay = ego_time_to_cw - walker_time_to_cw
            walker_spawn_interval = 6 # sec

            # for debugging
            ego_arr_frame = self.world.get_snapshot().frame + int(ego_time_to_cw / fixed_delta_seconds)

            for i in range(self.num_of_walker):
                j = i - int(self.num_of_walker / 2)
                delay = walker_delay + walker_spawn_interval * j
                delay_tick = int(delay / fixed_delta_seconds)
                start_frame = self.world.get_snapshot().frame + delay_tick + 1 / fixed_delta_seconds
                self.walker_start_frame.append(start_frame)

            print(f'walker starts at {self.walker_start_frame} frames; ego arrived at {ego_arr_frame} frame')
  
        except Exception as e:
            print(f'run scenario fail: {e}')

    def tick(self):
        """Call this once per env.step() to update scenario logic"""
        frame = self.world.get_snapshot().frame
        print(frame)

        if self.cur_walker_i < self.num_of_walker and frame >= self.walker_start_frame[self.cur_walker_i]:
            print(f'spawn {self.cur_walker_i} walker at frame {frame}')
            ped = Pedestrian(self.env.world, route=self.walker_route, max_speed=self.walker_max_speed)
 
            ped.set_manual_control()
            direction = get_direction(self.walker_route[0], self.walker_route[-1])

            ped.apply_manual_control(direction=direction)
            #walker_speed_diff = self.walker_max_speed - self.ped.get_speed()

            self.cur_walker_i += 1
        
        self.world.tick()


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