# main.py
import carla
from nmpc_controller import NMPCController
from carla_utils import initialize_vehicle, apply_control_to_vehicle, get_vehicle_state, update_camera_view
from trajectory import ReferenceTrajectory

def main():
    # Initialize CARLA and NMPC
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    vehicle = initialize_vehicle(world)

    spectator = world.get_spectator()

    nmpc = NMPCController()
    ref_trajectory = ReferenceTrajectory()

    try:
        for step in range(100):  # Number of simulation steps
            current_state = get_vehicle_state(vehicle)
            ref_point = ref_trajectory.get_ref_point(step)
            control = nmpc.compute_control(current_state, ref_point)
            print(control)
            apply_control_to_vehicle(vehicle, control)
            update_camera_view(vehicle, spectator)
    finally:
        print("Simulation ended")
        vehicle.destroy()

if __name__ == "__main__":
    main()
