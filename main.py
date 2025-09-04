from lib.agents.sensor.AgentWithSensor import AgentWithSensor

def init():
    env = AgentWithSensor()

   
    try:
        env.reset()
        env.move_spectator_to_loc(env.car.get_transform().location)

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

