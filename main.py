from agents.sensor.AgentWithSensor import AgentWithSensor

def init():
    env = AgentWithSensor()
    world = env.world
    client = env.client
   
    try:
        env.reset()
        car = env.car

        while True:
            control = env.get_planner_control()
            env.car.apply_control(control)
            if env.is_sync:
                env.world.tick()
            else:
                env.world.wait_for_tick()  
       
    except KeyboardInterrupt:
        pass
    finally:
        env.cleanup()

init()

