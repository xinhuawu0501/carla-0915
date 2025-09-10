from lib.agents.sensor.AgentWithSensor import AgentWithSensor
from lib.env_setup.car import Sensor

def init():   
    agent = AgentWithSensor()
    step = 0
    round = 50
    num_of_col = 0
    try:
        for _ in range(round):
            agent.reset()
            step = 0
            while True:
                if step >= 300:
                    break
                control = agent.get_planner_control()
                agent.car.apply_control(control)

                agent.display_img(img_type=Sensor.RGB)

                if agent.env.is_sync:
                    agent.scenario.tick()
                else:
                    #async mode
                    agent.env.world.wait_for_tick()  
                step += 1
            if agent.collision_data:
                num_of_col += 1
            agent.cleanup()
        
        print(f'collision count {num_of_col} in {round} simulation')
       
    except KeyboardInterrupt:
        pass
    finally:
        agent.close()

init()

