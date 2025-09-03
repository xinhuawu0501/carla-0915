import carla

DEFAULT_WEATHER=carla.WeatherParameters(
        cloudiness=0,        # 0-100
        precipitation=0,     # 0-100
        precipitation_deposits=0.0,  # 0-100
        wind_intensity=0.0,    # 0-100
        sun_azimuth_angle=90.0,  # 0-360
        sun_altitude_angle=75.0  # -90 to 90
        )

class BaseScenario:
    def __init__(self, client, weather=DEFAULT_WEATHER):
        self.client = client
        self.world = client.world
        self.world_map = self.world.get_map()

        self.world.set_weather(weather)

