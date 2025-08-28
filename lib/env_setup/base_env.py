import carla
import time
import random
import math
import numpy as np
import queue
from lib.util.image_processing import cv_display, process_rgb_img, process_semantic_img
import cv2
from shapely import Polygon, Point

class CarBaseEnv():
    is_sync = False
    collision_data = []
    
    def __init__(self):
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.load_world('Town10HD')
        self.world_bp = self.world.get_blueprint_library()
        self.world_map = self.world.get_map()
        self.spawn_points = self.world_map.get_spawn_points()
        self.spectator = self.world.get_spectator()

      
    def spawn_car(self, sp=None, sensor_options=None,IMG_X=800, IMG_Y=600):
        self.vehicle_bps = self.world_bp.filter('vehicle.*.*')
        self.vehicle_bp = self.vehicle_bps[0]

        if not sp:
            self.car_sp = random.choice(self.spawn_points)
        
        if sensor_options is None:
            sensor_options={'semantic': True, 'rgb': False, 'colsen': True}


        self.car = self.world.spawn_actor(self.vehicle_bp, self.car_sp)
        print(f'spawned {self.car.id}')

        cam_transform = carla.Transform(carla.Location(x=-4, z=3), carla.Rotation(pitch=-15))

        if sensor_options.get('rgb'):
            self.image_queue = queue.Queue()
            self.cam_bp = self.world_bp.find('sensor.camera.rgb')
            self.cam_bp.set_attribute('image_size_x', str(IMG_X))
            self.cam_bp.set_attribute('image_size_y', str(IMG_Y))
            self.cam_bp.set_attribute('fov', '110')

            self.rgb_cam = self.world.spawn_actor(self.cam_bp, cam_transform, attach_to=self.car)
            self.rgb_cam.listen(lambda img: self.image_queue.put(img))

        if sensor_options.get('semantic'):
            self.semantic_img_queue = queue.Queue()
            cam_bp = self.world_bp.find('sensor.camera.semantic_segmentation')
            cam_bp.set_attribute('image_size_x', str(IMG_X))
            cam_bp.set_attribute('image_size_y', str(IMG_Y))
            cam_bp.set_attribute('fov', '90')
            self.semantic_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.car)
            self.semantic_cam.listen(lambda img: self.semantic_img_queue.put(img))

        if sensor_options.get('colsen'):
            self.colsen_bp = self.world_bp.find('sensor.other.collision')
            self.colsen = self.world.spawn_actor(self.colsen_bp, carla.Transform(), attach_to=self.car)
            self.colsen.listen(lambda event: self.collision_data.append(event))

        time.sleep(3.0)


    def apply_control(self, control):
        self.car.apply_control(control)

    def clear_image_queue(self):
        try:
            if hasattr(self, 'image_queue'):
                while not self.image_queue.empty():
                    try:
                        self.image_queue.get_nowait()
                    except queue.Empty:
                        break
            if hasattr(self, 'semantic_img_queue'):
                while not self.semantic_img_queue.empty():
                    try:
                        self.semantic_img_queue.get_nowait()
                    except queue.Empty:
                        break
        except Exception as e:
            print(e)
        
    def move_spectator_to_loc(self, location):
        spec_trans = location 
        spec_trans.z += 2.0
        rot = carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
        self.spectator.set_transform(carla.Transform(spec_trans, rot))
    
    def get_rgb_img(self):
        try:
            image = self.image_queue.get(block=False)
            return image
        except queue.Empty:
            print('rgb empty')

    def display_rgb(self, raw_image):
        try:
            arr = process_rgb_img(raw_image)
            cv_display(arr)
        except Exception as e:
            print(e)

    def get_semantic_img(self):
        try:
            image = self.semantic_img_queue.get(block=False)
            return image
        except queue.Empty:
            print('semantic empty')

    def display_semantic(self, raw_image):
        try:
            arr = process_semantic_img(raw_image)
            cv_display(arr)
        except Exception as e:
            print(e)


    def set_sync(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = True 
        settings.fixed_delta_seconds = 0.05  # Optional: fixed simulation step (e.g., 20 FPS)
        self.world.apply_settings(settings)
        self.is_sync = True

    def is_at_traffic_light(self):
        is_at = self.car.is_at_traffic_light()
        if is_at:
            print(f'is at traffic light: {is_at}')
            print(f'light: {self.car.get_traffic_light_state()}')

    def create_tm(self):
        self.TM_PORT = 8000
        self.tm = self.client.get_trafficmanager(self.TM_PORT)
        self.tm.set_hybrid_physics_mode(True)

    def spawn_NPC_cars(self, num_of_car=50):
        for i in range(num_of_car):
            c = self.world.try_spawn_actor(random.choice(self.vehicle_bps), random.choice(self.spawn_points))
            if c is not None:
                if self.TM_PORT: c.set_autopilot(True, self.TM_PORT)
                else: c.set_autopilot(True)

                print(f'spawned {c.id}')

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

    def generate_wp(self, distance=2.0):
        self.wps = self.world_map.generate_waypoints(distance=distance)
        return self.wps
   

    def draw_wp(self, wps):
        for wp in wps:
            loc = wp.transform.location + carla.Location(z=0.2)
            self.world.debug.draw_string(
                loc,
                'wp'+str(wp.id),
                color=carla.Color(255, 255, 0),
                life_time=60.0
            )

    def create_route(self):
        waypoints = []
        self.wps = self.world_map.generate_waypoints(distance=5.0)

        
        start = self.wps[0]
        print(start)
        
        waypoints = [start] + start.next_until_lane_end(2.0)
        
        route_ind = [36, 37, 32, 23, 128, 70, 130, 124]
        route = []
        for ind in route_ind:
            route.append(self.spawn_points[ind].location)
        # self.spectator.set_transform(self.spawn_points[route_ind[0]])

        self.spawn_car(self.spawn_points[route_ind[0]])
        self.car.set_autopilot(True, self.TM_PORT)
        self.tm.set_path(self.car, route)

        for i,s in enumerate(self.spawn_points):
            self.world.debug.draw_string(
                s.location,
                text=str(i),
                color=carla.Color(r=255, g=0, b=0),
                life_time=1000.0,
                persistent_lines=True
            )

    def draw_locations(self, locations: list[carla.Location], displayed_str=''):
        for loc in locations:
            self.world.debug.draw_string(
                loc + carla.Location(z=0.2),
                displayed_str + str(loc),
                color=carla.Color(255, 0, 0),
                life_time=60.0
            )

    def get_yaw_from_to(self, from_loc, to_loc):
        dx = to_loc.x - from_loc.x
        dy = to_loc.y - from_loc.y
        yaw = math.degrees(math.atan2(dy, dx))  # atan2 gives angle in radians
        return yaw
    
    def get_direction(self, from_loc: carla.Location, to_loc: carla.Location) -> carla.Vector3D:
        dx = to_loc.x - from_loc.x
        dy = to_loc.y - from_loc.y
        dz = to_loc.z - from_loc.z
        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        if length == 0:
            return carla.Vector3D(0.0, 0.0, 0.0)
        return carla.Vector3D(x=dx/length, y=dy/length, z=dz/length)
    
    #=========== crosswalk utilities ===================================================#
    def get_all_crosswalk(self, draw_str=False) -> list[carla.Location]:
        self.crosswalks = self.world_map.get_crosswalks()
        if draw_str:
            self.draw_locations(self.crosswalks, displayed_str='cw')
        return self.crosswalks
    
    def get_locations_in_crosswalk(self, crosswalk_point) -> list[carla.Location]:
        try:
            indexes = [index for index, loc in enumerate(self.crosswalks) if loc == crosswalk_point ]
            print(indexes)
            return self.crosswalks[indexes[0]:indexes[1]]
        except Exception as e:
            print(e)
            return []
    
    def get_opposite_point_in_crosswalk(self, entry):
        index = self.crosswalks.index(entry)
        exit = self.crosswalks[index + 3]
        return exit
    
    def get_all_intersections(self, draw_str=False):
        waypoints = self.generate_wp(2.0)

        self.intersections = [wp for wp in waypoints if wp.is_intersection]
        if draw_str:
            self.draw_wp(self.intersections)
        return self.intersections
    
    def get_wp_leading_to_crosswalk(self, cw=None):
        lanes_to_crosswalk = []
        if not hasattr(self, 'intersections'):
            self.get_all_intersections(draw_str=False)

        if not hasattr(self, 'crosswalks'):
            self.get_all_crosswalk(draw_str=False)

        if not cw:
            cw = random.choice(self.crosswalks)

        loc_in_cw = self.get_locations_in_crosswalk(cw)
        crosswalk_polygon = Polygon([(pt.x, pt.y) for pt in loc_in_cw])
        wps_in_crosswalk_polygon = [wp for wp in self.intersections if crosswalk_polygon.contains(Point(wp.transform.location.x, wp.transform.location.y))]

        self.draw_wp(wps_in_crosswalk_polygon)



    def spawn_walker(self, spawn_point=None):
        try:
            self.get_all_crosswalk()

            route = []
            self.draw_spawn_points()
            
            entry = random.choice(self.crosswalks)
            self.get_opposite_point_in_crosswalk(entry)
            self.move_spectator_to_loc(entry)

            walker_bps = self.world_bp.filter('walker.pedestrian.*')
            bp = random.choice(walker_bps)
            # sp = spawn_point if spawn_point else random.choice(self.spawn_points)
            yaw = self.get_yaw_from_to(entry, exit)
            sp = carla.Transform(entry, carla.Rotation(yaw=yaw))
            self.walker = self.world.spawn_actor(bp, sp)

            control = carla.WalkerControl()
            control.direction = self.get_direction(entry, exit)
            control.speed = 5
            self.walker.apply_control(control)

    

        except Exception as e:
            print(f'fail to spawn walker: {e}')
   
    def change_weather(self, rain=0.0, cloud=0.0):
        weather = carla.WeatherParameters(
        cloudiness=cloud,        # 0-100
        precipitation=rain,     # 0-100
        precipitation_deposits=0.0,  # 0-100
        wind_intensity=0.0,    # 0-100
        sun_azimuth_angle=90.0,  # 0-360
        sun_altitude_angle=75.0  # -90 to 90
        )
        self.world.set_weather(weather)

    def get_available_map(self):
        maps = self.client.get_available_maps()
        return maps

    def cleanup(self):
        try:
            actors = self.world.get_actors()
            for a in actors:
                type_id = a.type_id

                if type_id.startswith('sensor.') or type_id.startswith('controller.ai'):
                    a.stop()

                if type_id.startswith('sensor.') or type_id.startswith('vehicle.') or type_id.startswith('walker.'):
                    print(f'destroy {type_id} with id {a.id}')
                    a.destroy()
        
            # if self.npc_car_list:
            #     self.client.apply_batch([carla.command.DestroyActor(x) for x in self.npc_car_list])
            
            if self.is_sync:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
            cv2.destroyAllWindows()

        except Exception as e:
            print(f'fail to clean up: {e}')


