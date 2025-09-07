from lib.agents.sensor.AgentWithSensor import AgentWithSensor

def init():   
    agent = AgentWithSensor()

    try:
        agent.reset()

        env = agent.env
        car = agent.car

        if car is None:
            raise RuntimeError("Car failed to spawn. Check spawn point or blueprint.")

        env.move_spectator_to_loc(car.get_transform().location)

        while True:
            control = agent.get_planner_control()
            car.apply_control(control)

            # agent.display_img()

            if env.is_sync:
                env.world.tick()
            else:
                env.world.wait_for_tick()  
       
    except KeyboardInterrupt:
        pass
    finally:
        agent.close()

init()

