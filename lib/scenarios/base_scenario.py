import carla

DEFAULT_WEATHER=carla.WeatherParameters(
        cloudiness=0,        # 0-100
        precipitation=0,     # 0-100
        precipitation_deposits=0.0,  # 0-100
        wind_intensity=0.0,    # 0-100
        sun_azimuth_angle=90.0,  # 0-360
        sun_altitude_angle=75.0  # -90 to 90
        )

class BaseScenario:
    def __init__(self, client, weather=DEFAULT_WEATHER):
        self.client = client
        self.world = client.world
        self.world_map = self.world.get_map()

        self.world.set_weather(weather)

    def get_sidewalks(self, x_range=(-300, 300), y_range=(-300, 300), step=2.0):
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
    
    def get_all_intersections(self):
        waypoints = self.world_map.generate_waypoints(distance=2.0)

        self.intersections = [wp for wp in waypoints if wp.is_intersection]
        return self.intersections
    
    def get_all_crosswalk(self) -> list[carla.Location]:
        self.crosswalks = self.world_map.get_crosswalks()
        return self.crosswalks
    
    def get_closest_loc(self, target_loc, loc_list):
        closest_loc = min(
            loc_list,
            key=lambda loc: loc.distance(target_loc)
        )
        
        return closest_loc

