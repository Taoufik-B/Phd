# main.py
import carla
from nmpc_controller import NMPCController
from carla_utils import *
from trajectory import ReferenceTrajectory
import logging, time, sys
import numpy as np




class Simulation:
    def __init__(self) -> None:
        self.N = 10
        self.done = False
        self.world=None
        self.spectator = None
        self.ego_vehicle = None
        self.setting = None
        self.actor_list = []
        self.controller = None
        self.iteration = 0
        self.target_state = [0,0,0]
        self.map = "Town05"
        self._init_world()

    def _init_world(self, sync = True):
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        # self.world = client.get_world()
        self.world = client.load_world(self.map)
        # client.reload_world()

        if sync:
            self.settings = self.world.get_settings()
            settings = self.world.get_settings()
            if not settings.synchronous_mode:
                    settings.synchronous_mode = True
                    settings.fixed_delta_seconds = 0.4
                    settings.rendering = True
            self.world.apply_settings(settings)
        
        self.spectator = self.world.get_spectator()
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

    def _update_camera_bird_view(self):
        ego_t = self.ego_vehicle.get_transform()
        spect_location = ego_t.location + carla.Location(z=45)
        spect_rotation = carla.Rotation(yaw=ego_t.rotation.yaw, pitch = -85)
        spect_t = carla.Transform(spect_location, spect_rotation)
        self.spectator.set_transform(spect_t)

    def _spawn_vehicle(self, spawn_point, filter = "vehicle.*", role="hero"):
        blueprint_library = self.world.get_blueprint_library()
        bp_vehicle = blueprint_library.filter(filter).find('vehicle.audi.etron')
        bp_vehicle.set_attribute('role_name', role)
        # set color
        bp_vehicle.set_attribute('color', '48, 80, 114')

        while self.ego_vehicle is None:
            # if not self.map.get_spawn_points():
            #     print('There are no spawn points available in your map/town.')
            #     print('Please add some Vehicle Spawn Point to your UE4 scene.')
            #     sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            # for i,p in enumerate(spawn_points):
            #     print(i,p)
            # spawn_point_t = carla.Transform()
            # spawn_point_t.location.x = 7.086513
            # spawn_point_t.location.y = -200.483002
            # spawn_point_t.rotation.yaw = np.rad2deg(spawn_point[2])
            # print(spawn_point_t)
            self.ego_vehicle = self.world.try_spawn_actor(bp_vehicle, spawn_points[266])
            if self.ego_vehicle:
                self.actor_list.append(self.ego_vehicle)
                #teleport vehicle
                spawn_point_t = carla.Transform()
                spawn_point_t.location.x = spawn_point[0]
                spawn_point_t.location.y = spawn_point[1]
                spawn_point_t.rotation.yaw = np.rad2deg(spawn_point[2])
                self.ego_vehicle.set_transform(spawn_point_t)

    def setup(self, controller, spawn_point):
        self.controller = controller
        self._spawn_vehicle(spawn_point)

        pass

    def run_step(self, ref_trajectory):
        self._update_camera_bird_view()
        current_state = get_vehicle_state(self.ego_vehicle)
        if current_state[0] == 0:
            current_state=ref_trajectory.x0
        

        target_state  = ref_trajectory.get_ref_points(self.iteration, self.N)

        # print(current_state, target_state)
        current_speed=get_speed(self.ego_vehicle)
        self.controller.current_speed=current_speed
        control, u = self.controller.compute_control(current_state, target_state, self.iteration)
        apply_control_to_vehicle(self.ego_vehicle, control)
        # apply_target_speed(self.ego_vehicle)
        # print("Current State: ",current_state)
        # control = carla.VehicleAckermannControl(steer=control['steer'], steer_speed=0.0, speed=control['speed'], acceleration=0.0, jerk=0.0)
        # self.ego_vehicle.apply_ackermann_control(control)

        self.controller.run_step(u)

        # print(self.ego_vehicle.get_physics_control())

        # print("Current speed: ",current_speed)
        if current_state[0]-target_state[0,0] < 15.0:
            self.iteration += 1
        self.done = self.iteration == 80
        self.world.tick()

        return self.done

    def teardown(self):
        if self.settings:
            self.rendering = False
            self.world.apply_settings(self.settings)
            self.settings = None
        for actor in self.actor_list:
            actor.destroy()



def main():
    ###########
    log_level = logging.DEBUG
    logging.basicConfig(format='%(asctime)s %(levelname)-8s: %(message)s', level=log_level, datefmt='%Y-%m-%d %H:%M:%S')

    # logging.info('listening to server %s:%s', args.host, args.port)

    ###################
    #Prepare the simulation
    ref_trajectory = ReferenceTrajectory()
    simulation = Simulation()
    
    nmpc = NMPCController(ref_trajectory)
    
    simulation.setup(nmpc, ref_trajectory.x0)

    # Initialize CARLA and NMPC


    try:
        # for step in range(1000):  # Number of simulation steps
        logging.info("Simulation Start")
        while True:
            simulation.run_step(ref_trajectory)
            if simulation.done:
                logging.info("Goal Reached")
                logging.info("Program Exit")
                break
                 
            # step=0
            # current_state = get_vehicle_state(vehicle)
            # ref_point = ref_trajectory.get_ref_point(step)
            # control = nmpc.compute_control(current_state, ref_point)
            # print(control)
            # apply_control_to_vehicle(vehicle, control)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as e:
        print(f'Exception occured due to {e}')
    finally:
        print("Simulation ended")
        simulation.teardown()

if __name__ == "__main__":
    main()
