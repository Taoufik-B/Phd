import sys, glob, os

# try:
print(sys.version_info.major,
    sys.version_info.minor,
    os.name)

print('dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    7,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))
sys.path.append(glob.glob('simu/dist/carla-*%d.7-%s.egg' % (
    sys.version_info.major,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
print("Try Carla import", sys.path)
# except IndexError:
#     pass


import carla
import numpy as np
from carla_utils import *   


CARLA_SERVER_IP = 'localhost'


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

    def run_step(self, X0,u):
        self._update_camera_bird_view()
        # apply control/ 
        # x is the state vector [x,y,psi,delta]
        # u is the control vector u[:,0]
        teleport(self.ego_vehicle, X0[:,0]) 
        # get state
        current_state = get_vehicle_state(self.ego_vehicle)

        # update p0 and shift controls
        self.controller.update_mpc(current_state,u)

        self.world.tick()

        return self.done
    # def run_step(self, ref_trajectory):
    #     self._update_camera_bird_view()
    #     current_state = get_vehicle_state(self.ego_vehicle)
    #     # if current_state[0] == 0:
    #     #     current_state=ref_trajectory.x0
        

    #     target_state  = ref_trajectory.get_ref_points(self.iteration, self.N)

    #     # print(current_state, target_state)
    #     current_speed=get_speed(self.ego_vehicle)
    #     self.controller.current_speed=current_speed
    #     control, u = self.controller.compute_control(current_state, target_state, self.iteration)
    #     # apply_control_to_vehicle(self.ego_vehicle, control)
    #     # apply_target_speed(self.ego_vehicle)
    #     # control = carla.VehicleAckermannControl(steer=control['steer'], steer_speed=control['steer_rate'], speed=control['speed'], acceleration=0.0, jerk=0.0)
    #     # self.ego_vehicle.apply_ackermann_control(control)

    #     ### TEST Teleport
    #     # print(self.controller.X0[:,0])
    #     teleport(self.ego_vehicle, self.controller.X0[:,1])

    #     self.controller.run_step(u)

    #     self.controls = np.vstack((self.controls, [control['speed'], control['steer']]))

    #     self.vehicle_position=np.vstack((self.vehicle_position,current_state))

    #     # print(self.ego_vehicle.get_physics_control())
    #     # if np.linalg.norm(current_state[0:2]-target_state[0:2]) <= 5.0:
    #     # if np.linalg.norm(current_state[0:1]-target_state[0,0:1]) <= 5.0:
    #     if current_state[0]-target_state[0,0] <= 5.0 and current_state[1]-target_state[0,1] <= 5.0 :
    #         self.iteration += 1
    #         self.done = self.iteration == len(ref_trajectory.path)-self.N
    #     self.world.tick()

    #     return self.done

    def teardown(self):
        if self.settings:
            self.rendering = False
            self.world.apply_settings(self.settings)
            self.settings = None
        for actor in self.actor_list:
            actor.destroy()
        # np.save('data/vp.npy', self.vehicle_position)
        # np.save('data/controls.npy', self.controls)