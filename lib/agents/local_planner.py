from lib.PythonAPI.carla.agents.navigation.local_planner import LocalPlanner
import carla

class CustomPlanner(LocalPlanner):
    def __init__(self, vehicle, map_inst=None, route=None):
        opt_dict = {
            'target_speed': 10.0,     # km/h
            'sampling_resolution': 2.0
        }
        super().__init__(vehicle, opt_dict, map_inst)
        self.opt_dict = opt_dict

        if route:
            self.set_route(route)

    def run_step(self, debug=False):
        """
        Compute next vehicle control using LocalPlanner.
        """
        control = super().run_step(debug=debug)
        return control
    
    def set_route(self, route: list[carla.Waypoint]):
        speed = self.opt_dict.get('target_speed', 20.0)
        plan = [(wp, speed) for wp in route]

        # print(plan)
        self.set_global_plan(plan)
  