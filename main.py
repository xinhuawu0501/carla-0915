from lib.env_setup.base_env import CarBaseEnv
import time

def init():
    env = CarBaseEnv()
 
    try:
        env.set_sync()
        env.spawn_car()
        env.car.set_autopilot(True)
        while True:
            env.display_rgb()
            env.world.tick()
        
       
    except KeyboardInterrupt:
        pass
    finally:
        env.cleanup()

init()

