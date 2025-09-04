import carla
import random
import queue

from constants.camera import IMG_X, IMG_Y

DEFAULT_SENSOR_OPTION = {
    'rgb': False,
    'semantic': True,
    'colsen': True
}

DEFAULT_CAM_ATTR = {
    'x': IMG_X,
    'y': IMG_Y,
    'fov': 110
}

class CarBaseAgent:
    collision_data = []

    def __init__(
            self, 
            world, 
            sensor_options=DEFAULT_SENSOR_OPTION, 
            sp=None, 
            route=None, 
            cam_attr=DEFAULT_CAM_ATTR):
        self.world = world
        self.sensor_options = sensor_options
        self.vehicle_bps = self.world_bp.filter('vehicle.*.*')
        self.all_sps = self.world.get_spawn_points()

        self.route = route
        self.cam_attr = cam_attr
        self.sp = sp
        self.car = None

        self.image_queue = queue.Queue
        self.semantic_img_queue = queue.Queue
        
    def spawn_self(self):
        try:
            self.vehicle_bp = self.vehicle_bps[0]

            if not self.sp:
                self.sp = random.choice(self.all_sps)
            
            self.car = self.world.spawn_actor(self.vehicle_bp, self.car_sp)
            print(f'spawned {self.car.id}')

            cam_transform = carla.Transform(carla.Location(x=-4, z=3), carla.Rotation(pitch=-15))

            if self.sensor_options.get('rgb'):
                self.cam_bp = self.world_bp.find('sensor.camera.rgb')
                self.cam_bp.set_attribute('image_size_x', str(self.cam_attr.x))
                self.cam_bp.set_attribute('image_size_y', str(self.cam_attr.y))
                self.cam_bp.set_attribute('fov', str(self.cam_attr.fov))

                self.rgb_cam = self.world.spawn_actor(self.cam_bp, cam_transform, attach_to=self.car)
                self.rgb_cam.listen(lambda img: self.image_queue.put(img))

            if self.sensor_options.get('semantic'):
                self.semantic_img_queue = queue.Queue()
                cam_bp = self.world_bp.find('sensor.camera.semantic_segmentation')
                cam_bp.set_attribute('image_size_x', str(self.cam_attr.x))
                cam_bp.set_attribute('image_size_y', str(self.cam_attr.y))
                cam_bp.set_attribute('fov', str(self.cam_attr.fov))
                self.semantic_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.car)
                self.semantic_cam.listen(lambda img: self.semantic_img_queue.put(img))

            if self.sensor_options.get('colsen'):
                self.colsen_bp = self.world_bp.find('sensor.other.collision')
                self.colsen = self.world.spawn_actor(self.colsen_bp, carla.Transform(), attach_to=self.car)
                self.colsen.listen(lambda event: self.collision_data.append(event))
        except Exception as e:
            print(f'spawn car failed {e}')


