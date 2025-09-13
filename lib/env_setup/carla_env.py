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

        self.all_cw_polygons = self._get_all_crosswalk_polygons()
        self.junction_wps_map = self.get_all_wp_in_junctions()
        
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
    
    def _get_all_crosswalk_polygons(self):
        polygons = []
        self.get_all_crosswalk()

        for i in range(0, len(self.crosswalks), 5):
            cw_group = self.crosswalks[i:i + 5]
            polygons.append(cw_group)

        return polygons
    
    def get_all_wp_in_junctions(self):
        waypoints = self.generate_wp(2.0)
        junction_wps_map = {}

        for wp in waypoints:
            if wp.is_intersection:
                junction = wp.get_junction()

                if not junction_wps_map.get(junction.id):
                    junction_wps_map[junction.id] = {} 

                # only store wps in crosswalks 
                for i, p in enumerate(self.all_cw_polygons):
                    if self.wp_is_in_polygon(wp=wp, target_polygon=p):
                        if not junction_wps_map[junction.id].get(i):
                            junction_wps_map[junction.id][i] = []

                        junction_wps_map[junction.id][i].append(wp)
                        
        return junction_wps_map
        
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
    
    def wp_is_in_polygon(self, target_polygon: list[carla.Location], wp: carla.Waypoint) -> bool:
        pol = Polygon([(pt.x, pt.y) for pt in target_polygon])
        p = Point(wp.transform.location.x, wp.transform.location.y)
        return pol.contains(p)


    def _get_lanes_passing_crosswalk(self, junction_id: int, polygon_id: int, len_before=50.0, len_after=50.0, d=5.0):
        lanes_to_crosswalk = {'straight': [], 'turning': []}
        try:
            polygons_in_junction = self.junction_wps_map.get(junction_id)
            wps_in_crosswalk_polygon = polygons_in_junction.get(polygon_id)
       
            for i, wp in enumerate(wps_in_crosswalk_polygon):
                route = self.generate_navigation_from_wp(wp, before_wp=len_before, after_wp=len_after, d=d)
                if not route:
                    continue

                start = route[0]
                yaw_diff = get_yaw_diff(wp.transform, start.transform)
                
                if yaw_diff > 30:
                    lanes_to_crosswalk['turning'].append(route)
                else:
                    lanes_to_crosswalk['straight'].append(route)
        except Exception as e:
            print(f'_get_lanes_passing_crosswalk error: {e}')
        
        return lanes_to_crosswalk

    
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


 
    def change_weather(self,sun_altitude_angle=70.0, precipitation=0.0, cloudiness=0.0, fog_density=0.0):
        weather = carla.WeatherParameters(
        cloudiness=cloudiness,        # 0-100
        fog_density = fog_density,
        precipitation=precipitation,
        sun_altitude_angle=sun_altitude_angle
        )
        self.world.set_weather(weather)

    def batch_destroy_actors(self, actor_list):
        '''
        for destroying npc walker and cars. Do not use for sensors.
        '''
        try:
            result = self.client.apply_batch_sync([carla.command.DestroyActor(x.id) for x in actor_list])
            
            for r in result:
                result_str = f'successfully destroyed {r.actor_id}' if not r.error else f'Error destroying {r.actor_id} with error {r.error}'
                print(result_str)
            
            return result
        except Exception as e:
            print(e)

    def cleanup(self):
        try:   
            if self.is_sync:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)

        except Exception as e:
            print(f'fail to clean up: {e}')


