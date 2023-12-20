# carla_utils.py
import carla
import random

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
    loc = vehicle.get_location()
    rot = vehicle.get_transform().rotation.yaw
    return {
        'x': loc.x,
        'y': loc.y,
        'theta': rot  # Note: Depending on your setup, you might need to convert this to radians
    }

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


def update_camera_view(vehicle, spectator):
    ego_t = vehicle.get_transform()
    spect_location = ego_t.location + carla.Location(z=45)
    spect_rotation = carla.Rotation(yaw=ego_t.rotation.yaw, pitch = -85)
    spect_t = carla.Transform(spect_location, spect_rotation)
    spectator.set_transform(spect_t)
    pass