import random
import carla

DEFAULT_SPEED=1.0

class Pedestrian:
    def __init__(self, world, route=[], speed=DEFAULT_SPEED) -> None:
        self.world = world
        self.bp = self.world.get_blueprint_library()
        self.walker_bps = self.bp.filter("walker.pedestrian.*")
        self.world_map = self.world.get_map()

        #if no specified spawn point and route, let walker wander randomly
        self.speed = speed
        self.walker = self._spawn_walker(route=route)
        self.has_started = False

    def _spawn_walker(self, route=[]):
        try:
            bp = random.choice(self.walker_bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")

            if len(route) < 1:
                self.spawn_points = self.world_map.get_spawn_points()
                sp = random.choice(self.spawn_points)
                sp_loc = sp.location
            else:
                sp_loc = route[0]

            sp_loc.z = 0.5  # lift to avoid collision
            spawn_transform = carla.Transform(sp_loc, carla.Rotation())
            walker = self.world.spawn_actor(bp, spawn_transform)

            self.world.tick()

            return walker
        except Exception as e:
            print(f'_spawn_walker err {e}')
            return None
        
    
    def start_walker(self, route=[]):
        try:
            # Spawn AI controller
            controller_bp = self.bp.find("controller.ai.walker")
            self.controller = self.world.spawn_actor(controller_bp, carla.Transform(), attach_to=self.walker) # type: ignore
            self.controller.start()
            self.controller.set_max_speed(self.speed)

            if len(route):
                destination = route[-1]
                for location in route:
                    self.controller.go_to_location(location)
                print(f"✅ Walker moving to {route[-1]}")

            else:
                destination = self.world.get_random_location_from_navigation()
                self.controller.go_to_location(destination)
                print(f"✅ Walker moving to {destination}")

            self.has_started = True

            #TODO: stop walker after reaching destination
        except Exception as e:
            print(e)





