from lib.agents.sensor.AgentWithSensor import AgentWithSensor

def init():
    env = AgentWithSensor()
    world = env.world
    client = env.client
   
    try:
        env.setup_scenario()
        car = env.car
        # env.move_spectator_to_loc(car.get_transform().location)

        while True:
            control = env.planner.run_step(debug=False)
            env.car.apply_control(control)
            env.display_semantic()

            if env.is_sync:
                env.world.tick()
            else:
                env.world.wait_for_tick()  
       
    except KeyboardInterrupt:
        pass
    finally:
        env.cleanup()

init()

