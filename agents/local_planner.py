from lib.PythonAPI.carla.agents.navigation.local_planner import LocalPlanner

class CustomPlanner(LocalPlanner):
    def __init__(self, vehicle, map_inst=None, route=None):
        opt_dict = {
            'target_speed': 20.0,     # km/h
            'sampling_resolution': 2.0
        }
        super().__init__(vehicle, opt_dict, map_inst)

        if route:
            self.set_global_plan(route)

    def run_step(self, debug=False):
        """
        Compute next vehicle control using LocalPlanner.
        """
        control = super().run_step(debug=debug)
        return control
    
    def set_route(self, route: carla.WayPoint[]): # type: ignore
        self.set_global_plan(route)