# main.py
import carla
from nmpc_controller import NMPCController
from carla_utils import *
from trajectory import ReferenceTrajectory
import logging, time


class Simulation:
    def __init__(self) -> None:

        self.done = False
        self.world=None
        self.spectator = None
        self.ego_vehicle = None

        pass

    def _init_world(self, sync = True):
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()

        if sync:
            settings = self.world.get_settings()
            if not settings.synchronous_mode:
                    settings.synchronous_mode = True
                    settings.fixed_delta_seconds = 0.05
                    settings.rendering = True
            self.world.apply_settings(settings)

    def _update_camera_bird_view(self):
        ego_t = self.ego_vehicle.get_transform()
        spect_location = ego_t.location + carla.Location(z=45)
        spect_rotation = carla.Rotation(yaw=ego_t.rotation.yaw, pitch = -85)
        spect_t = carla.Transform(spect_location, spect_rotation)
        spectator.set_transform(spect_t) 

    def setup(self):
        pass

    def run_step(self):
        self._update_camera_bird_view()
        return self.done

    def teardown(self):
        pass



def main():
    ###########
    log_level = logging.DEBUG
    logging.basicConfig(format='%(asctime)s %(levelname)-8s: %(message)s', level=log_level, datefmt='%Y-%m-%d %H:%M:%S')

    # logging.info('listening to server %s:%s', args.host, args.port)

    ###################
    #Prepare the simulation
    simulation = Simulation()
    simulation.setup()

    # Initialize CARLA and NMPC
    world = init_world()
    vehicle = initialize_vehicle(world)

    spectator = world.get_spectator()

    nmpc = NMPCController()
    ref_trajectory = ReferenceTrajectory()

    try:
        # for step in range(1000):  # Number of simulation steps
        while True:
            done = simulation.run_step()
            if done:
                logging.info("Goal Reached")
                time.sleep(2)
                logging.info("Program Exit")
                break
                 
            step=0
            current_state = get_vehicle_state(vehicle)
            ref_point = ref_trajectory.get_ref_point(step)
            # control = nmpc.compute_control(current_state, ref_point)
            # print(control)
            # apply_control_to_vehicle(vehicle, control)
            apply_target_speed(vehicle, speed=20)
            update_camera_view(vehicle, spectator)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as e:
        print(f'Exception occured due to {e}')
    finally:
        print("Simulation ended")
        vehicle.destroy()

if __name__ == "__main__":
    main()
