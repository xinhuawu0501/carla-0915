import math
import carla

def get_yaw_from_two_locations(from_loc, to_loc):
    dx = to_loc.x - from_loc.x
    dy = to_loc.y - from_loc.y
    yaw = math.degrees(math.atan2(dy, dx))  # atan2 gives angle in radians
    return yaw

def get_yaw_diff(transform_a, transform_b):
    ya = transform_a.rotation.yaw % 360
    yb = transform_b.rotation.yaw % 360
    diff = abs(ya - yb)

    return min(diff, 360 - diff) # handle cases like 359 deg and 2 deg



def get_direction(from_loc: carla.Location, to_loc: carla.Location) -> carla.Vector3D:
    dx = to_loc.x - from_loc.x
    dy = to_loc.y - from_loc.y
    dz = to_loc.z - from_loc.z
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    if length == 0:
        return carla.Vector3D(0.0, 0.0, 0.0)
    return carla.Vector3D(x=dx/length, y=dy/length, z=dz/length)
