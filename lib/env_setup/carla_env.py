import carla
import random
from shapely import Polygon, Point

from lib.env_setup.pedestrian import Pedestrian
from lib.util.transform import get_yaw_diff


class CarlaEnv():
    is_sync = False
    npc_car_list = []
    npc_walkers = []
    
    def __init__(self):
        self.client = carla.Client("localhost", 2000) # type: ignore
        self.client.set_timeout(5.0)
        self.world = self.client.load_world('Town10HD')
        self.world_bp = self.world.get_blueprint_library()
        self.world_map = self.world.get_map()
        self.spawn_points = self.world_map.get_spawn_points()
        self.spectator = self.world.get_spectator()
        self.vehicle_bps = self.world_bp.filter('vehicle.*.*')
        self.crosswalks = self.get_all_crosswalk()
        
    def move_spectator_to_loc(self, location):
        spec_trans = location 
        spec_trans.z += 25.0
        rot = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
        self.spectator.set_transform(carla.Transform(spec_trans, rot))

    def set_sync(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = True 
        settings.fixed_delta_seconds = 0.05  # Optional: fixed simulation step (e.g., 20 FPS)
        self.world.apply_settings(settings)
        self.is_sync = True

# =========== npc util ===================================
    def spawn_NPC_cars(self, num_of_car=50):
        for i in range(num_of_car):
            c = self.world.try_spawn_actor(random.choice(self.vehicle_bps), random.choice(self.spawn_points))
            if c is not None:
                if hasattr(self, 'TM_PORT'): c.set_autopilot(True, self.TM_PORT)
                else: c.set_autopilot(True)

                self.npc_car_list.append(c)

                print(f'spawned {c.id}')

    def spawn_npc_walkers(self, num_of_walker=10, speed=None, route=[]):
        for i in range(num_of_walker):
            ped = Pedestrian(self.world, route=route, max_speed=speed)
            walker = ped.walker
            if walker:
                self.npc_walkers.append(walker)
        
        print(f'spawned {len(self.npc_walkers)} npc walkers')
        self.world.set_pedestrians_cross_factor(1.0)

# ======== traffic light management ===================
    def create_tm(self):
        self.TM_PORT = 8000
        self.tm = self.client.get_trafficmanager(self.TM_PORT)
        self.tm.set_hybrid_physics_mode(True)

    def get_all_traffic_lights(self):
        self.traffic_lights = self.world.get_actors().filter('traffic.traffic_light')
        return self.traffic_lights

    def draw_traffic_lights(self):
        self.get_all_traffic_lights()

        for i,tl in enumerate(self.traffic_lights):
            loc = tl.get_transform().location
            
            self.world.debug.draw_string(
                loc,
                text=str(tl.id),
                color=carla.Color(r=255, g=0, b=0),
                life_time=1000.0,
                persistent_lines=True
            )

    def change_traffic_light(self, id):
        tl = self.world.get_actors().find(id)
        tl.freeze(True) #prevent automatic update
        tl.set_state(carla.TrafficLightState.Yellow)
        print(f'turning {id} from {tl.get_state()} to y')

# ======== draw string in carla simulation ===================
    def draw_spawn_points(self):
        for i, sp in enumerate(self.spawn_points):
            loc = sp.location + carla.Location(z=0.5)  # lift above ground
            # Draw a label
            self.world.debug.draw_string(
                loc,
                str(i),
                color=carla.Color(255, 255, 0),  # yellow
                life_time=60.0
            )

    def draw_waypoints(self, wps, strg=''):
        for wp in wps:
            loc = wp.transform.location + carla.Location(z=0.2)
            self.world.debug.draw_string(
                loc,
                strg + str(wp.id),
                color=carla.Color(255, 255, 0),
                life_time=600.0
            )

    def draw_locations(self, locations: list[carla.Location], displayed_str=''):
        for loc in locations:
            self.world.debug.draw_string(
                loc + carla.Location(z=0.2),
                displayed_str + str(loc),
                color=carla.Color(255, 0, 0),
                life_time=60.0
            )
    
    #=========== crosswalk & sidewalk utilities ===================================================#
    def get_all_crosswalk(self) -> list[carla.Location]:
        self.crosswalks = self.world_map.get_crosswalks()
        return self.crosswalks
    
    def get_all_crosswalk_polygons(self):
        polygons = []
        
        for i in range(0, len(self.crosswalks), 5):
            cw_group = self.crosswalks[i:i + 5]
            polygons.append(cw_group)

        return polygons
    
    def _get_crosswalk_polygon(self, crosswalk_point) -> list[carla.Location]:
        try:
            polygons = self.get_all_crosswalk_polygons()
            target_group = list(filter(lambda group: crosswalk_point in group, polygons))[0]
            for p in target_group:
                p.z = 0

            return target_group
            
            
        except Exception as e:
            print(f'_get_crosswalk_polygon err: {e}')
            return []
        
    def generate_navigation_from_wp(self, wp, before_wp=20.0, after_wp=20.0, d=5.0):
        nav = []

        try:
            curr = wp
            before_steps = int(before_wp/d)
            after_steps = int(after_wp/d)
            for _ in range(before_steps):
                prevs = curr.previous(d)
                if not prevs:
                    break
                curr = prevs[0]
                nav.insert(0, curr)  

            nav.append(wp)

            curr = wp
            for _ in range(after_steps):
                nexts = curr.next(d)
                if not nexts:
                    break
                curr = nexts[0]
                nav.append(curr)

            if len(nav) < before_steps + after_steps + 1:
                raise Exception("Navigation path too short")

        except Exception as e:
            print(f"[WARN] Failed to generate navigation: {e}")

        return nav

    def _get_lanes_passing_crosswalk(self, target_polygon: list[carla.Location]):
        lanes_to_crosswalk = {'straight': [], 'turning': []}
        try:
            if not hasattr(self, 'intersections'):
                self.get_all_intersections()
                

            if not hasattr(self, 'crosswalks'):
                self.get_all_crosswalk()

            crosswalk_polygon = Polygon([(pt.x, pt.y) for pt in target_polygon])
            wps_in_crosswalk_polygon = [wp for wp in self.intersections if crosswalk_polygon.contains(Point(wp.transform.location.x, wp.transform.location.y))]

            for i, wp in enumerate(wps_in_crosswalk_polygon):
                before = 30
                after = 30
                d = 5
                route = self.generate_navigation_from_wp(wp, before_wp=before, after_wp=after, d=d)

                if not len(route): continue
                start = route[0]
                yaw_diff = get_yaw_diff(wp.transform, start.transform)
                
                if yaw_diff > 30:
                    lanes_to_crosswalk['turning'].append(route)
                else:
                    lanes_to_crosswalk['straight'].append(route)
        except Exception as e:
            print(f'_get_lanes_passing_crosswalk error: {e}')
        
        return lanes_to_crosswalk


    def _get_closest_crosswalk_point_from_wp(self, crosswalk_pol, wp)->carla.Location:
        loc = wp.transform.location
        min_d = 1000000
        result = None

        for p in crosswalk_pol:
            d = self.get_distance(p, loc)
            if d < min_d:
                min_d = d
                result = p

        return result
    
    def get_sidewalks(self, x_range=(-300, 300), y_range=(-300, 300), step=2.0) -> list[carla.Waypoint]:
        sidewalk_wps = []
        seen = set()

        for x in range(int(x_range[0]), int(x_range[1]), int(step)):
            for y in range(int(y_range[0]), int(y_range[1]), int(step)):
                loc = carla.Location(x=float(x), y=float(y), z=0.0) # type: ignore
                wp = self.world_map.get_waypoint(
                    loc,
                    project_to_road=False,
                    lane_type=carla.LaneType.Sidewalk # type: ignore
                )
                if wp and (wp.road_id, wp.lane_id, round(wp.s, 1)) not in seen:
                    sidewalk_wps.append(wp)
                    seen.add((wp.road_id, wp.lane_id, round(wp.s, 1)))
    
        return sidewalk_wps
    
# =========== other utils =============================================
    def generate_wp(self, distance=2.0):
        self.wps = self.world_map.generate_waypoints(distance=distance)
        return self.wps
   
    def get_distance(self, a: carla.Location, b: carla.Location):
        return a.distance(b) # float (meters)
    
    def get_available_map(self):
        maps = self.client.get_available_maps()
        return maps

    def get_all_intersections(self, draw_str=False):
        waypoints = self.generate_wp(2.0)

        self.intersections = [wp for wp in waypoints if wp.is_intersection]
        if draw_str:
            self.draw_waypoints(self.intersections)
        return self.intersections
 
    def change_weather(self,sun_altitude_angle=70.0, precipitation=0.0, cloudiness=0.0, fog_density=0.0):
        weather = carla.WeatherParameters(
        cloudiness=cloudiness,        # 0-100
        fog_density = fog_density,
        precipitation=precipitation,
        sun_altitude_angle=sun_altitude_angle
        )
        self.world.set_weather(weather)

    def cleanup(self):
        try:   
            actors = self.world.get_actors()
            for a in actors:
                type_id = a.type_id

                if type_id.startswith('controller.ai'):
                    a.stop()

                elif type_id.startswith('sensor.'):
                    if a.is_listening:
                        a.stop() 

                if type_id.startswith('sensor.') or type_id.startswith('vehicle.') or type_id.startswith('walker.'):
                    print(f'destroy {type_id} with id {a.id}')
                    a.destroy()
        
            
            if self.is_sync:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)

        except Exception as e:
            print(f'fail to clean up: {e}')


