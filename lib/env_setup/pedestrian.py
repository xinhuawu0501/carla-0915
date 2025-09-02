import random
import carla

DEFAULT_SPEED=1.4

class Pedestrian:
    def __init__(self, client, route, speed=DEFAULT_SPEED) -> None:
        self.client = client
        self.world = self.client.world
        self.bp = self.world.get_blueprint_library()
        self.walker_bps = self.bp.filter("walker.pedestrian.*")
        self.world_map = self.world.get_map()

        #if no specified spawn point and route, let walker wander randomly
        self.spawn_walker(route=route, speed=speed)
        if self.walker:
            return self.walker
        else:
            return None 

    def spawn_walker(self, route, speed=DEFAULT_SPEED):
        try:
            bp = random.choice(self.walker_bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
    
            sp_wp = route[0]
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




