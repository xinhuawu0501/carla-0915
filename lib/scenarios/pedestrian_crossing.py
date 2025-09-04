from lib.env_setup.pedestrian import Pedestrian
from lib.scenarios.base_scenario import BaseScenario
from lib.util.transform import get_yaw_diff
import carla
import random
import time
from shapely import Polygon, Point


'''
pick a random crosswalk and let walker(s) walk through the crosswalk; return a route for ego vehicle to follow
'''
class PedestrianCrossingScenario(BaseScenario):
    walker_list = []
    def __init__(self, world, weather, num_of_walker= 1, speed=1.0):
        super().__init__(world=world)
        self.crosswalks = self.world_map.get_crosswalks()
        self.target_crosswalk = random.choice(self.crosswalks)
        self.num_of_walker = num_of_walker
        self.speed = speed
        self.sidewalks = self.get_sidewalks()

        self.spawn_walkers()

    def get_possible_car_routes(self):
        return self._get_lanes_passing_crosswalk()

    def spawn_walkers(self):
        try:
            route = self.get_walker_route()
            self.move_spectator_to_loc(route[0])

            for i in range(self.num_of_walker):
                ped = Pedestrian(self.world, route=route, speed=self.speed)
                walker = ped.walker
                #TODO: spawn walker every [] seconds?
                # time.sleep(5)
                if walker:
                    self.walker_list.append(walker)
        except Exception as e:
            print(f'spawn walker failed in pedestrian crossing scenario: {e}')
      

    def get_walker_route(self):
        route = []
        polygon = self._get_crosswalk_polygon(self.target_crosswalk)

        #walker entry should be on sidewalk
        cw_entry = polygon[0]
        sidewalk_location = [wp.transform.location for wp in self.sidewalks]
        spawn_point = self.get_closest_loc(cw_entry, sidewalk_location)
        cw_exit = polygon[-2]
        route.append(spawn_point)
        route.append(cw_entry)
        route.append(cw_exit)
        #TODO:  after crossing, keep walking on sidewalk randomly
        return route
    
    def draw_locations(self, locations: list[carla.Location], displayed_str=''):
        for loc in locations:
            self.world.debug.draw_string(
                loc + carla.Location(z=0.2),
                displayed_str + str(loc),
                color=carla.Color(255, 0, 0),
                life_time=100.0
            )

    def is_same_location(self, a, b, tol=0.05):
        return (abs(a.x - b.x) < tol and
                abs(a.y - b.y) < tol and
                abs(a.z - b.z) < tol)
    
    def __print_loc(self, loc):
        print(f'Location: {loc.x} {loc.y} {loc.z}')

    def _get_crosswalk_polygon(self, crosswalk_point) -> list[carla.Location]:
        try:
            self.draw_locations(self.crosswalks)
            self.move_spectator_to_loc(crosswalk_point)

            polygons = self.get_all_crosswalk_polygons()
            target_group = list(filter(lambda group: crosswalk_point in group, polygons))[0]
            return target_group
            
            
        except Exception as e:
            print(f'_get_crosswalk_polygon err: {e}')
            return []
        
    def _get_lanes_passing_crosswalk(self):
        lanes_to_crosswalk = {'straight': [], 'turning': []}
        try:
            if not hasattr(self, 'intersections'):
                self.get_all_intersections()

            if not hasattr(self, 'crosswalks'):
                self.get_all_crosswalk()

            loc_in_cw = self._get_crosswalk_polygon(self.target_crosswalk)

            crosswalk_polygon = Polygon([(pt.x, pt.y) for pt in loc_in_cw])
            wps_in_crosswalk_polygon = [wp for wp in self.intersections if crosswalk_polygon.contains(Point(wp.transform.location.x, wp.transform.location.y))]

            for i, wp in enumerate(wps_in_crosswalk_polygon):
                prev = wp.previous(10.0)[0]
                nxt = wp.next(10.0)[0]

                route = [prev, wp, nxt]
                yaw_diff = get_yaw_diff(prev.transform, nxt.transform)
                if yaw_diff > 30:
                    lanes_to_crosswalk['turning'].append(route)
                else:
                    lanes_to_crosswalk['straight'].append(route)
        except Exception as e:
            print(f'_get_lanes_passing_crosswalk error: {e}')
        
        return lanes_to_crosswalk


    
    

        
    

    


        