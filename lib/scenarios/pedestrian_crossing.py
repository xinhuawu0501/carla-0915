from lib.env_setup.carla_env import CarlaEnv
from lib.env_setup.pedestrian import Pedestrian
from lib.scenarios.base_scenario import DEFAULT_WEATHER, BaseScenario
from lib.util.transform import get_yaw_diff
import carla
import random
import time
from shapely import Polygon, Point


'''
pick a random crosswalk and let walker(s) walk through the crosswalk; return a route for ego vehicle to follow
'''
class PedestrianCrossingScenario():
    def __init__(self, env: CarlaEnv) -> None:
        self.env = env
        

    
    
    

        
    

    


        