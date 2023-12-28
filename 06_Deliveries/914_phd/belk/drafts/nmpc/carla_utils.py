# carla_utils.py
import carla
import random, math
import numpy as np

def initialize_vehicle(world, vehicle_type='model3', spawn_point=None):
    """
    Initialize and spawn a vehicle in the CARLA world.

    Parameters:
    - world: CARLA world object
    - vehicle_type: String, type of the vehicle to spawn (default: 'model3' - Tesla Model 3)
    - spawn_point: carla.Transform, specific spawn point (optional)

    Returns:
    - vehicle: Spawned CARLA vehicle object
    """
    blueprint_library = world.get_blueprint_library()
    vehicle_model = blueprint_library.filter(vehicle_type)[0]

    if spawn_point is None:
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = random.choice(spawn_points)

    vehicle = world.spawn_actor(vehicle_model, spawn_point)
    return vehicle

def get_vehicle_state(vehicle):
    """
    Retrieve the current state of the vehicle.

    Parameters:
    - vehicle: CARLA vehicle object

    Returns:
    - state: Dict containing position (x, y) and heading (theta) of the vehicle
    """
    carla_control = carla.VehicleControl()
    loc = vehicle.get_location()
    rot = vehicle.get_transform().rotation.yaw * 0.017453293 #convert to rad
    steer_angle = carla_control.steer*1.2217
    # return {
    #     'x': loc.x,
    #     'y': loc.y,
    #     'theta': rot  # Note: Depending on your setup, you might need to convert this to radians
    # }
    return [loc.x, loc.y, rot, steer_angle]

def apply_control_to_vehicle(vehicle, control):
    """
    Apply control commands to the CARLA vehicle.

    Parameters:
    - vehicle: CARLA vehicle object
    - control: Dict containing control commands (throttle, steer, etc.)
    """
    # carla_control = carla.VehicleControl(throttle=control.get('throttle', 0),
    #                                      steer=control.get('steer', 0),
    #                                      brake=control.get('brake', 0),
    #                                      hand_brake=False,
    #                                      reverse=False,
    #                                      manual_gear_shift=False,
    #                                      gear=0)
    carla_control = carla.VehicleControl()
    carla_control.throttle = control.get('throttle', 0.0)
    carla_control.steer = control.get('steer', 0.0)
    carla_control.brake = control.get('brake', 0.0)
    vehicle.apply_control(carla_control)

def apply_target_speed(vehicle, speed=0):
    vehicle.set_target_velocity(carla.Vector3D(-4,0,0))


def update_camera_view(vehicle, spectator):
    ego_t = vehicle.get_transform()
    spect_location = ego_t.location + carla.Location(z=45)
    spect_rotation = carla.Rotation(yaw=ego_t.rotation.yaw, pitch = -85)
    spect_t = carla.Transform(spect_location, spect_rotation)
    spectator.set_transform(spect_t)
    pass

def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * vel.length()

def teleport(vehicle, to_state):
    if vehicle is None:
        return
    if abs(to_state[0]) <= 0.001:
        return
    #teleport vehicle
    vehicle_t = vehicle.get_transform()
    vehicle_t.location.x = float(to_state[0])
    vehicle_t.location.y = float(to_state[1])
    # vehicle_t.rotation.yaw = np.rad2deg(float(to_state[2]))
    print(' Teleporting to : ', vehicle_t)
    vehicle.set_transform(vehicle_t)

def draw_waypoints(world, waypoints, z=0.5, lifespan=1.0, is_line=False):
    """
    Draw a list of waypoints at a certain height given in z.

        :param world: carla.world object
        :param waypoints: list or iterable container with the waypoints to draw
        :param z: height in meters
    """
    for wpt in waypoints:
        wpt_t = wpt.transform
        begin = wpt_t.location + carla.Location(z=z)
        angle = math.radians(wpt_t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        if is_line:
            world.debug.draw_point(begin, size=0.2, color=carla.Color(0,100,50,150), life_time=lifespan)
        else:
            world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=lifespan)