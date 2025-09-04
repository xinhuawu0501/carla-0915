import carla
import random
import queue
from enum import Enum

from constants.camera import IMG_X, IMG_Y
from util.image_processing import cv_display, process_rgb_img, process_semantic_img

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
            world, 
            sensor_options=DEFAULT_SENSOR_OPTION, 
            sp=None, 
            cam_attr=DEFAULT_CAM_ATTR):
        self.world = world
        self.sensor_options = sensor_options
        self.vehicle_bps = self.world_bp.filter('vehicle.*.*')
        self.all_sps = self.world.get_spawn_points()

        self.cam_attr = cam_attr
        self.sp = sp
        self.car = None

        self.image_queue = queue.Queue
        self.semantic_img_queue = queue.Queue
        
        self.spawn_self()

    def spawn_self(self):
        try:
            self.vehicle_bp = self.vehicle_bps[0]

            if not self.sp:
                self.sp = random.choice(self.all_sps)
            
            self.car = self.world.spawn_actor(self.vehicle_bp, self.car_sp)
            print(f'spawned {self.car.id}')

            cam_transform = carla.Transform(carla.Location(x=-4, z=3), carla.Rotation(pitch=-15))

            if self.sensor_options.get(Sensor.RGB):
                self.cam_bp = self.world_bp.find('sensor.camera.rgb')
                self.cam_bp.set_attribute('image_size_x', str(self.cam_attr.x))
                self.cam_bp.set_attribute('image_size_y', str(self.cam_attr.y))
                self.cam_bp.set_attribute('fov', str(self.cam_attr.fov))

                self.rgb_cam = self.world.spawn_actor(self.cam_bp, cam_transform, attach_to=self.car)
                self.rgb_cam.listen(lambda img: self.image_queue.put(img))
                self.sensors.append(self.rgb_cam)

            if self.sensor_options.get(Sensor.SEMANTIC):
                self.semantic_img_queue = queue.Queue()
                cam_bp = self.world_bp.find('sensor.camera.semantic_segmentation')
                cam_bp.set_attribute('image_size_x', str(self.cam_attr.x))
                cam_bp.set_attribute('image_size_y', str(self.cam_attr.y))
                cam_bp.set_attribute('fov', str(self.cam_attr.fov))
                self.semantic_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.car)
                self.semantic_cam.listen(lambda img: self.semantic_img_queue.put(img))
                self.sensors.append(self.semantic_cam)

            if self.sensor_options.get(Sensor.COLSEN):
                self.colsen_bp = self.world_bp.find('sensor.other.collision')
                self.colsen = self.world.spawn_actor(self.colsen_bp, carla.Transform(), attach_to=self.car)
                self.colsen.listen(lambda event: self.collision_data.append(event))
                self.sensors.append(self.colsen)

        except Exception as e:
            print(f'spawn car failed {e}')


    def get_raw_img_from_q(self, img_type = Sensor.SEMANTIC):
        try:
            if img_type == Sensor.SEMANTIC:
                raw_img = self.semantic_img_queue.get(block=False)
            elif img_type == Sensor.RGB:
                raw_img = self.image_queue.get(block=False)
            return raw_img
        except queue.Empty:
            print(f'{img_type} queue empty')

    def display_img(self, raw, img_type = Sensor.SEMANTIC):
        try:
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
        is_at = self.car.is_at_traffic_light()
        if is_at:
            print(f'is at traffic light: {is_at}')
            print(f'light: {self.car.get_traffic_light_state()}')
        
        return is_at
    
    def apply_control(self, control: carla.VehicleControl):
        self.car.apply_control(control)
    
    def close(self):
        for sensor in self.sensors:
            sensor.stop()
            sensor.destroy()
        
        self.car.destroy()
