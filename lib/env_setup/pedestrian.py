import random
import carla

DEFAULT_SPEED=1.4

class Pedestrian:
    def __init__(self, client, route=[], speed=DEFAULT_SPEED) -> None:
        self.client = client
        self.world = self.client.world
        self.bp = self.world.get_blueprint_library()
        self.walker_bps = self.bp.filter("walker.pedestrian.*")
        self.world_map = self.world.get_map()
        self.sidewalks = self.get_sidewalks()

        #if no specified spawn point and route, let walker wander randomly
        self.spawn_walker(route=route, speed=speed) 

    def spawn_walker(self, route=[], speed=DEFAULT_SPEED):
        try:
            bp = random.choice(self.walker_bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
    
            sp_wp = route[0] if route else random.choice(self.sidewalks)
            spawn_transform = sp_wp.transform
            spawn_transform.location.z += 0.5  # lift to avoid collision
        
            self.walker = self.world.try_spawn_actor(bp, spawn_transform)

            self.world.tick()

            # Spawn AI controller
            controller_bp = self.bp.find("controller.ai.walker")
            self.controller = self.world.spawn_actor(controller_bp, carla.Transform(), attach_to=self.walker) # type: ignore
            self.controller.start()
            self.controller.set_max_speed(speed)

            if len(route) < 1:
                target_loc = self.world.get_random_location_from_navigation()
                self.controller.go_to_location(target_loc)
                print(f"✅ Walker spawned at {spawn_transform.location}, moving to {target_loc}")

            else:
                for location in route:
                    self.controller.go_to_location(location)
                print(f"✅ Walker spawned at {spawn_transform.location}, moving to {route[-1]}")

            return self.walker, self.controller

        except Exception as e:
            print(e)
            return None


    def get_sidewalks(self, x_range=(-300, 300), y_range=(-300, 300), step=2.0):
        carla_map = self.world_map
        sidewalk_wps = []
        seen = set()

        for x in range(int(x_range[0]), int(x_range[1]), int(step)):
            for y in range(int(y_range[0]), int(y_range[1]), int(step)):
                loc = carla.Location(x=float(x), y=float(y), z=0.0) # type: ignore
                wp = carla_map.get_waypoint(
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



