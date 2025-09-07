import carla
import random
import queue
from enum import Enum
import numpy as np
import cv2

from lib.constants.camera import CITYSCAPES_PALETTE, IMG_X, IMG_Y
from lib.util.image_processing import cv_display, process_rgb_img, process_semantic_img

class Sensor(Enum):
    RGB = 'RGB'
    SEMANTIC = 'SEMANTIC'
    COLSEN = 'COLSEN'

DEFAULT_SENSOR_OPTION = {
    Sensor.RGB: False,
    Sensor.SEMANTIC: True,
    Sensor.COLSEN: True
}

DEFAULT_CAM_ATTR = {
    'x': IMG_X,
    'y': IMG_Y,
    'fov': 110
}

class Car:
    collision_data = []
    sensors = []

    def __init__(
            self, 
            world):
        self.world = world
        self.world_bp = self.world.get_blueprint_library()
        self.vehicle_bps = self.world_bp.filter('vehicle.*.*')
        self.all_sps = self.world.get_map().get_spawn_points()

        self.image_queue = queue.Queue()
        self.semantic_img_queue = queue.Queue()

        self.car = None
        
    def spawn_self(self, spawn_point=None, sensor_options=DEFAULT_SENSOR_OPTION, cam_attr=DEFAULT_CAM_ATTR):
        try:
            self.vehicle_bp = self.vehicle_bps[0]
            sp = spawn_point if spawn_point else random.choice(self.all_sps)
        
            self.car = self.world.spawn_actor(self.vehicle_bp, sp)
            print(f'spawned {self.car.id}')

            cam_transform = carla.Transform(carla.Location(x=-4, z=3), carla.Rotation(pitch=-15))

            if sensor_options.get(Sensor.RGB):
                self.cam_bp = self.world_bp.find('sensor.camera.rgb')
                self.cam_bp.set_attribute('image_size_x', str(cam_attr['x']))
                self.cam_bp.set_attribute('image_size_y', str(cam_attr['y']))
                self.cam_bp.set_attribute('fov', str(cam_attr['fov']))

                self.rgb_cam = self.world.spawn_actor(self.cam_bp, cam_transform, attach_to=self.car)
                self.rgb_cam.listen(lambda img: self.image_queue.put(img))
                self.sensors.append(self.rgb_cam)

            if sensor_options.get(Sensor.SEMANTIC):
                cam_bp = self.world_bp.find('sensor.camera.semantic_segmentation')
                cam_bp.set_attribute('image_size_x', str(cam_attr['x']))
                cam_bp.set_attribute('image_size_y', str(cam_attr['y']))
                cam_bp.set_attribute('fov', str(cam_attr['fov']))
                self.semantic_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.car)
                self.semantic_cam.listen(lambda img: self.semantic_img_queue.put(img))
                self.sensors.append(self.semantic_cam)

            if sensor_options.get(Sensor.COLSEN):
                self.colsen_bp = self.world_bp.find('sensor.other.collision')
                self.colsen = self.world.spawn_actor(self.colsen_bp, carla.Transform(), attach_to=self.car)
                self.colsen.listen(lambda event: self.on_collision(event))
                self.sensors.append(self.colsen)

        except Exception as e:
            print(f'spawn car failed {e}')
    
    def on_collision(self, event):
        self.collision_data.append(event)

        if event.other_actor.type_id.startswith('walker'):
            print(f'collided with {event.other_actor.type_id} {event.other_actor.id}')

    def get_raw_img_from_q(self, img_type = Sensor.SEMANTIC):
        try:
            raw_img = None
            if img_type == Sensor.SEMANTIC:
                raw_img = self.semantic_img_queue.get(block=False)
            elif img_type == Sensor.RGB:
                raw_img = self.image_queue.get(block=False)
            return raw_img
        except queue.Empty:
            print(f'{img_type} queue empty')

    def display_img(self, img_type = Sensor.SEMANTIC):
        try:
            raw = self.get_raw_img_from_q(img_type=img_type)
            processed = None
            if img_type == Sensor.SEMANTIC:
                processed = process_semantic_img(raw)
            elif img_type == Sensor.RGB:
                processed = process_rgb_img(raw)

            cv_display(processed)
        except Exception as e:
            print(e)

    def clear_all_q(self):
        try:
            while not self.image_queue.empty():
                try:
                    self.image_queue.get_nowait()
                except queue.Empty:
                    break

            while not self.semantic_img_queue.empty():
                try:
                    self.semantic_img_queue.get_nowait()
                except queue.Empty:
                    break
        except Exception as e:
            print(e)

    def is_at_traffic_light(self):
        if not self.car:
            return False
        
        is_at = self.car.is_at_traffic_light()
        if is_at:
            print(f'is at traffic light: {is_at}')
            print(f'light: {self.car.get_traffic_light_state()}')
        
        return is_at
    
    def apply_control(self, control: carla.VehicleControl):
        if self.car:
            self.car.apply_control(control)

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

    
    def cleanup(self):
        self.clear_all_q()
        print(f'{len(self.collision_data)} collisions happened')
        self.collision_data.clear()
