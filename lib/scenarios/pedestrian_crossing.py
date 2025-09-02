from lib.env_setup.pedestrian import Pedestrian
import carla
import random
import time
from shapely.geometry import Polygon, Point


'''
pick a random crosswalk and let walker(s) walk through the crosswalk; return a route for ego vehicle to follow
'''
class PedestrianCrossingScenario:
    walker_list = []
    def __init__(self, client, num_of_walker= 1, speed=1.4):
        self.client = client
        self.world = self.world.client
        self.world_map = self.world.get_map()
        self.crosswalks = self.world_map.get_crosswalks()
        self.target_crosswalk = random.choice(self.crosswalks)
        self.num_of_walker = num_of_walker
        self.speed = speed
        self.sidewalks = self._get_sidewalks()

        self.spawn_walkers()
        car_route_options = self._get_lanes_passing_crosswalk()
        return car_route_options


    def spawn_walkers(self):
        route = self.get_walker_route()

        for i in range(self.num_of_walker):
            walker = Pedestrian(self.client, route=route, speed=self.speed)
            #TODO: spawn walker every [] seconds?
            time.sleep(5)
            if walker:
                self.walker_list.append(walker)
      

    def get_walker_route(self):
        route = []
        polygon = self._get_crosswalk_polygon(self.target_crosswalk)
        cw_entry = random.choice(polygon)
        cw_exit = self._get_opposite_point_in_crosswalk(cw_entry, polygon)
        route.append(cw_entry)
        route.append(cw_exit)
        #TODO:  after crossing, keep walking on sidewalk randomly
        return route

    def _get_crosswalk_polygon(self, crosswalk_point) -> list[carla.Location]:
        try:
            indexes = [index for index, loc in enumerate(self.crosswalks) if loc == crosswalk_point ]
            succeed = len(indexes) > 1
            if not succeed:
                ind = 0
                while ind < len(self.crosswalks):
                    for i in range(ind + 1, len(self.crosswalks)):
                        if self.crosswalks[i] == self.crosswalks[ind]:
                            return self.crosswalks[ind:i]


            return self.crosswalks[indexes[0]:indexes[1]]
        except Exception as e:
            print(e)
            return []
        
    def _get_opposite_point_in_crosswalk(self, entry, polygon):
        index = self.crosswalks.index(entry)
        exit = -1
        for i in range(index + 1, len(polygon)):
            cw = polygon[i]
            if cw == entry:
                exit = cw
                break
        return exit
    
    def _get_sidewalks(self, x_range=(-300, 300), y_range=(-300, 300), step=2.0):
        sidewalk_wps = []
        seen = set()

        for x in range(int(x_range[0]), int(x_range[1]), int(step)):
            for y in range(int(y_range[0]), int(y_range[1]), int(step)):
                loc = carla.Location(x=float(x), y=float(y), z=0.0) # type: ignore
                wp = self.world_map.get_waypoint(
                    loc,
                    project_to_road=False,
                    lane_type=carla.LaneType.Sidewalk # type: ignore
                )
                if wp and (wp.road_id, wp.lane_id, round(wp.s, 1)) not in seen:
                    sidewalk_wps.append(wp)
                    seen.add((wp.road_id, wp.lane_id, round(wp.s, 1)))
        
        for s in sidewalk_wps:
            print(s.lane_type)
        return sidewalk_wps
    
    def _get_all_intersections(self, draw_str=False):
        waypoints = self.world_map.generate_waypoints(distance=2.0)

        self.intersections = [wp for wp in waypoints if wp.is_intersection]
        return self.intersections
    
    def _get_all_crosswalk(self, draw_str=False) -> list[carla.Location]:
        self.crosswalks = self.world_map.get_crosswalks()
        return self.crosswalks
    
    def _get_lanes_passing_crosswalk(self):
        lanes_to_crosswalk = []
        if not hasattr(self, 'intersections'):
            self._get_all_intersections(draw_str=False)

        if not hasattr(self, 'crosswalks'):
            self._get_all_crosswalk(draw_str=False)


        loc_in_cw = self._get_crosswalk_polygon(self.target_crosswalk)

        crosswalk_polygon = Polygon([(pt.x, pt.y) for pt in loc_in_cw])
        wps_in_crosswalk_polygon = [wp for wp in self.intersections if crosswalk_polygon.contains(Point(wp.transform.location.x, wp.transform.location.y))]

        for i, wp in enumerate(wps_in_crosswalk_polygon):
            prev = wp.previous(10.0)[0]
            nxt = wp.next(10.0)[0]

            route = [prev, wp, nxt]
            lanes_to_crosswalk.append(route)

        return lanes_to_crosswalk

    
    
    

        
    

    


        