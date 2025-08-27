from lib.env_setup.base_env import CarBaseEnv
from lib.PythonAPI.carla.agents.navigation.local_planner import LocalPlanner

import time

class CustomPlanner(LocalPlanner):
    def __init__(self, vehicle, map_inst=None):
        opt_dict = {
            'target_speed': 20.0,     # km/h
            'sampling_resolution': 2.0
        }
        super().__init__(vehicle, opt_dict, map_inst)

    def run_step(self, debug=False):
        """
        Compute next vehicle control using LocalPlanner.
        """
        control = super().run_step(debug=debug)
        return control

class AgentWithSensor(CarBaseEnv):
    def __init__(self):
       super().__init__()

       self.set_sync()
       print(self.world)

    def planner_control(self):
        return self.planner.run_step(debug=True)
    
    def reset(self):
        self.spawn_car()
        self.planner = CustomPlanner(self.car)



  

def init():
    env = AgentWithSensor()
 
    try:
        env.reset()
        while True:
            env.display_rgb()
            env.car.apply_control(env.planner_control())
            env.world.tick()
        
       
    except KeyboardInterrupt:
        pass
    finally:
        env.cleanup()

init()

