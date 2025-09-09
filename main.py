from lib.agents.sensor.AgentWithSensor import AgentWithSensor
from lib.env_setup.car import Sensor

def init():   
    agent = AgentWithSensor()

    try:
        agent.reset()
       
    except KeyboardInterrupt:
        pass
    finally:
        agent.close()

init()

