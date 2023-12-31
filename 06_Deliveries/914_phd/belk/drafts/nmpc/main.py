# main.py
import sys, glob, os

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


import carla
import logging, time
import numpy as np
from nmpc_controller import NMPCController
from trajectory import ReferenceTrajectory
from carla_utils import *
from visualization import *

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
        self.map_name = '/Game/Carla/Maps/Town05'
        self.vehicle_position = np.zeros((0,4))
        self.controls = np.zeros((0,2))
        self._init_world()

    def _init_world(self, sync = True):
        client = carla.Client(CARLA_SERVER_IP, 2000)
        client.set_timeout(10.0)
        # print(client.get_available_maps())
        self.world = client.get_world()
        # self.world = client.load_world(self.map_name)

        if sync:
            self.settings = self.world.get_settings()
            settings = self.world.get_settings()
            if not settings.synchronous_mode:
                    settings.synchronous_mode = True
                    settings.fixed_delta_seconds = 0.1
                    settings.rendering = True
            self.world.apply_settings(settings)
            # client.reload_world(False)
            # time.sleep(3)
        
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

            spawn_points = self.map.get_spawn_points()
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
        # if current_state[0] == 0:
        #     current_state=ref_trajectory.x0
        

        target_state  = ref_trajectory.get_ref_points(self.iteration, self.N)

        # print(current_state, target_state)
        current_speed=get_speed(self.ego_vehicle)
        self.controller.current_speed=current_speed
        control, u = self.controller.compute_control(current_state, target_state, self.iteration)
        # apply_control_to_vehicle(self.ego_vehicle, control)
        # apply_target_speed(self.ego_vehicle)
        # control = carla.VehicleAckermannControl(steer=control['steer'], steer_speed=control['steer_rate'], speed=control['speed'], acceleration=0.0, jerk=0.0)
        # self.ego_vehicle.apply_ackermann_control(control)

        ### TEST Teleport
        # print(self.controller.X0[:,0])
        teleport(self.ego_vehicle, self.controller.X0[:,1])

        self.controller.run_step(u)

        self.controls = np.vstack((self.controls, [control['speed'], control['steer']]))

        self.vehicle_position=np.vstack((self.vehicle_position,current_state))

        # print(self.ego_vehicle.get_physics_control())
        # if np.linalg.norm(current_state[0:2]-target_state[0:2]) <= 5.0:
        # if np.linalg.norm(current_state[0:1]-target_state[0,0:1]) <= 5.0:
        if current_state[0]-target_state[0,0] <= 5.0 and current_state[1]-target_state[0,1] <= 5.0 :
            self.iteration += 1
            self.done = self.iteration == len(ref_trajectory.path)-self.N
        self.world.tick()

        return self.done

    def teardown(self):
        if self.settings:
            self.rendering = False
            self.world.apply_settings(self.settings)
            self.settings = None
        for actor in self.actor_list:
            actor.destroy()
        np.save('vp.npy', self.vehicle_position)
        np.save('controls.npy', self.controls)


def run_dry_simulation(nmpc,ref_trajectory, N):
    t=[]
    reference = ref_trajectory.get_reference()
    mpciter=0
    t0 = 0
    v_ref = 20
    delta_ref = 0
    distance = 0.1

    print(ref_trajectory.size)
    try:
        # while (mpciter < ref_trajectory.size):
        while (True):
            # if mpciter+N > ref_trajectory.size:
            #     break
            # else:
            #     ref_point = ref_trajectory.get_next_wps(mpciter, N)
            # if len(ref_point) < N:
            #     break
            u_opt  = nmpc.compute_control(mpciter, v_ref, delta_ref)
            nmpc.run_step(u_opt)
            # distance = math.sqrt((ref_point[0,0]-nmpc.x0[0])**2 + (ref_point[0,1]-nmpc.x0[1])**2)
            # print("Current Distance",distance)
            # print(ref_point[1,0:2])
            # print(nmpc.x0[0:2])
            # if distance < 5:
            #     print("Goal Reached at iteration: ", mpciter)
            distance = np.linalg.norm(ref_trajectory.xs[0:2]-nmpc.x0[0:2])
            distance_p = np.linalg.norm(ref_trajectory.xs[0:2]-nmpc.p_history[0:2,0,mpciter])
            logging.info(f"Distance iteration {mpciter} : {distance}")
            mpciter += 1     
            t.append(t0)
            if distance < 0.5:
                break
            if distance_p <0.5:
                break
            # if mpciter > 500:
            #     break
            
        simulate(ref_trajectory.path, nmpc.p_history, nmpc.x_history, nmpc.u_opt_history, t, nmpc.dt, nmpc.N,reference, False)
    except Exception as e:
        print(e)
    finally:
        print("Simulation ended")


def main():
    ###########
    # logging
    log_level = logging.INFO
    logging.basicConfig(format='%(asctime)s %(levelname)-8s: %(message)s', level=log_level, datefmt='%Y-%m-%d %H:%M:%S')
    ###################
    # config
    # CARLA_SERVER_IP = '192.168.1.136'
    CARLA_SERVER_IP = 'localhost'

    ###################
    # NMPC parameters
    N = 25                      # horizon
    dt = 0.05                    # delta time
    Q = [15, 15, 0.5, 0.01]    # states
    R = [0.05, 0.5]             # controls


    # Model Parameters
    L = 3
    activate_rk4 = True

    ###################
    # Trajectory
    trajectory = ReferenceTrajectory()


    ###################
    # Prepare the simulation
    nmpc = NMPCController(trajectory, Q, R, L, dt, N, activate_rk4)
    # carla_simulation = Simulation()
    
    
    # carla_simulation.setup(nmpc, trajectory)
    try:
        logging.info("simulation Start")
        run_dry_simulation(nmpc,trajectory, N)
    #     while True:
    #         st = time.time()
    #         carla_simulation.run_step()
    #         et = time.time()
    #         print("World_tick elapsed_Time(s): ", et-st)
    #         if carla_simulation.done:
    #             logging.info("Goal Reached")
    #             logging.info("Program Exit")
    #             break
        
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as e:
        print(f'Exception occured due to {e}')
    finally:
        # carla_simulation.teardown()
        print("Simulation ended")

if __name__ == "__main__":
    main()
