import carla
import time
import random
import math
import numpy as np
import queue
from lib.constants.camera import CITYSCAPES_PALETTE
from lib.util.image_processing import cv_display, process_rgb_img, process_semantic_img
import cv2
from shapely.geometry import Polygon, Point


class CarBaseEnv():
    is_sync = False
    collision_data = []
    walker_list = []
    walker_controller_list = []
    
    def __init__(self):
        self.client = carla.Client("localhost", 2000) # type: ignore
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
        spec_trans.z += 20.0
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

    # Helper: convert RGB image to class ID map
    def rgb_to_class_id(self, img_rgb):
        """
        img_rgb: HWC uint8
        returns: HxW int32 with class IDs
        """
        class_map = np.zeros((img_rgb.shape[0], img_rgb.shape[1]), dtype=np.int32)
        for class_id, color in CITYSCAPES_PALETTE.items():
            mask = np.all(img_rgb == color, axis=2)
            class_map[mask] = class_id
        return class_map

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
            class_map = self.rgb_to_class_id(img_display)
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
   

    def draw_wp(self, wps, strg=''):
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
    
    def get_distance(self, a: carla.Location, b: carla.Location):
        return a.distance(b)

    def get_closest_spawn_point(self, target_location: carla.Location) -> carla.Transform:
        closest_spawn = min(
            self.spawn_points,
            key=lambda sp: sp.location.distance(target_location)
        )

        return closest_spawn
    
    #=========== crosswalk utilities ===================================================#
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
          
    def get_all_intersections(self, draw_str=False):
        waypoints = self.generate_wp(2.0)

        self.intersections = [wp for wp in waypoints if wp.is_intersection]
        if draw_str:
            self.draw_wp(self.intersections)
        return self.intersections
 
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

        except Exception as e:
            print(f'fail to clean up: {e}')


