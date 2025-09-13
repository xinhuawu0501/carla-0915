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
    

class LOC_TYPE(Enum):
    INTERSECTION='4_way_intersection'
    T_JUNCTION='t_junction'
    UNSIGNALIZED_MIN_JUNCTION='unsignalized_minor_junction'


junction_map = {
    LOC_TYPE.INTERSECTION: [189],
    LOC_TYPE.T_JUNCTION: [23, 532, 468, 719],
    LOC_TYPE.UNSIGNALIZED_MIN_JUNCTION: [134, 895, 664]
}

class ScenarioManager:
    def __init__(self, env: CarlaEnv):
        self.env = env
        self.world = self.env.world
        self.settings = self.world.get_settings()
        self.sidewalks = self.env.get_sidewalks()

        self.load_scenario_config()

        self.walker_start_frame = []
        self.walkers = []
        self.cur_walker_i = 0
        self._ego_route = None

        self.log_current_scenario()

    def get_actor_routes(self):
        try:
            target_junction_id = random.choice(junction_map[self.cur_location_type])

            cws_in_junctions = self.env.junction_wps_map.get(target_junction_id)
            target_polygon_i = random.choice(list(cws_in_junctions.keys()))
            target_polygon = self.env.all_cw_polygons[target_polygon_i]
     
            lanes = self.env._get_lanes_passing_crosswalk(junction_id=target_junction_id, polygon_id=target_polygon_i)

            # calculate ego vehicle route
            self._ego_route = random.choice(lanes['turning'])
            self.collision_wp = next((wp for wp in self._ego_route if self.env.wp_is_in_polygon(wp=wp, target_polygon=target_polygon)), self._ego_route[len(self._ego_route) // 2])
           
           # determine which side in crosswalk polygon
            closest_cw = min(target_polygon, key=lambda loc: self.env.get_distance(loc, self.collision_wp.transform.location))
            opposite_cw = max(target_polygon, key=lambda loc: self.env.get_distance(loc, closest_cw))
        
            # calculate walker route
            closest_sidewalk_wp = min(self.sidewalks, key=lambda sw: self.env.get_distance(sw.transform.location, closest_cw))
            closest_sidewalk_wp_loc = closest_sidewalk_wp.transform.location
            opposite_sidewalk_wp = min(self.sidewalks, key=lambda sw: self.env.get_distance(sw.transform.location, opposite_cw))
            opposite_sidewalk_loc = opposite_sidewalk_wp.transform.location

            self.walker_route = [closest_sidewalk_wp_loc, closest_cw, self.collision_wp.transform.location, opposite_cw, opposite_sidewalk_loc]
            
            self.env.move_spectator_to_loc(self.collision_wp.transform.location)

            return self.collision_wp
        except Exception as e:
            print(f'fail to get actor route: {e}')           

    def get_route_len(self, route: list[carla.Waypoint] | list[carla.Location]):
        d = 0

        try:
            for i in range(len(route) - 1):
                # Waypoint obj for car
                if hasattr(route[i], 'transform'):
                    cur = route[i].transform.location
                    next = route[i + 1].transform.location
                    d += self.env.get_distance(cur, next)
                else: # Location obj for walker
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
            ego_collision_point_i = self._ego_route.index(self.collision_wp)

            ego_d_to_cw = self.get_route_len(self.ego_route[:ego_collision_point_i + 1])
            ego_speed = self.ego_target_speed * 1000 / 60 / 60 # to m / s
            
            buffer = 0 #random.uniform(0.1, 0.3)
            ego_time_to_cw = ego_d_to_cw / (ego_speed - buffer) 

            # calculate walker time to crosswalk
            walker_collision_point_i = 2
            walker_d_to_cw = self.get_route_len(self.walker_route[:walker_collision_point_i + 1])
            walker_time_to_cw = walker_d_to_cw / self.walker_max_speed # sec
            
            fixed_delta_seconds = self.settings.fixed_delta_seconds

            walker_delay = ego_time_to_cw - walker_time_to_cw
            walker_spawn_interval = 5 # sec

            if walker_delay < 0:
                print('delay', walker_delay)
            
            target_start_frame = self.world.get_snapshot().frame + int(walker_delay / fixed_delta_seconds)

            for i in range(self.num_of_walker):
                j = i - self.num_of_walker // 2
                start_frame = max(self.world.get_snapshot().frame, target_start_frame + (walker_spawn_interval / fixed_delta_seconds) * j)
                if start_frame not in self.walker_start_frame:
                    self.walker_start_frame.append(start_frame)
            
            # update number of walker 
            self.num_of_walker = len(self.walker_start_frame)
            
            # print(f'walker starts at {self.walker_start_frame} frames')
  
        except Exception as e:
            print(f'run scenario fail: {e}')

    def tick(self):
        """Call this once per env.step() to update scenario logic"""
        frame = self.world.get_snapshot().frame

        if self.cur_walker_i < self.num_of_walker and frame >= self.walker_start_frame[self.cur_walker_i]:
            ped = Pedestrian(self.env.world, route=self.walker_route, max_speed=self.walker_max_speed)
            self.walkers.append(ped.walker)
 
            ped.set_manual_control()
            direction = get_direction(self.walker_route[0], self.walker_route[-1])

            ped.apply_manual_control(direction=direction)
            #walker_speed_diff = self.walker_max_speed - self.ped.get_speed()

            self.cur_walker_i += 1
        
        self.world.tick()


    def load_weather(self, visibility):
        weather = random.choice(visibility['high'])
        precipitation, fog_density, sun_altitude_angle, cloudiness = (
            float(weather.get(k, 0.0)) for k in ['precipitation', 'fog_density', 'sun_altitude_angle', 'cloudiness']
        )

        print(weather['id'])

        self.env.change_weather(cloudiness=cloudiness, precipitation=precipitation, fog_density=fog_density, sun_altitude_angle=sun_altitude_angle)

    def load_scenario_config(self):
        try:
            config = load_yaml(os.path.join(os.getcwd(), 'lib/env_setup/config.yaml'))
            
            config_env = config["env"]
            visibility = config_env["visibility"]    
            scenario_options = config["scenarios"]
            self.cur_scenario = random.choice(scenario_options)

            location_types = self.cur_scenario["location_types"]
            self.cur_location_type = LOC_TYPE(random.choice(location_types))

            walker_config = self.cur_scenario['pedestrian']
            self.walker_max_speed = random.choice(walker_config['max_speed'])
            self.num_of_walker = random.choice(walker_config["num_of_pedestrian"])

            ego_config = self.cur_scenario['ego']
            self.ego_target_speed = random.choice(ego_config["target_speed"])

            self.load_weather(visibility=visibility)
        except Exception as e:
            print(f'fail to load scenario: {e}')


    def log_current_scenario(self):
        weather = self.env.world.get_weather()
        cur_map = self.env.world_map.name

        print(f'Loading {cur_map}\n')
        print(f'Scenario: {self.cur_scenario["name"]}\nLocation type: {self.cur_location_type}\nEgo target speed {self.ego_target_speed}\nWalker max speed {self.walker_max_speed}')
        
    def cleanup(self):
        self.env.batch_destroy_actors(self.walkers)
        self.walkers.clear()
        
    @property
    def ego_route(self) -> list:
        if not self._ego_route:
            raise NotImplementedError
        return self._ego_route