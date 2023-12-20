import casadi as ca
import numpy as np
import logging
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

import carla

import argparse
import sys
import time

import _thread
from timeit import timeit


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * vel.length()



class VehicleNMPCController():
    """
    VehicleNMPCController is the ....
    """


    def __init__(self, vehicle, offset=0, max_throttle=0.75, max_brake=0.3,
                 max_steering=0.8):
        """
        Constructor method.
        """
        # Vehicle parameters
        L = 2.5  # Wheelbase of the vehicle
        dt = 0.1  # Time step
        N = 20    # Prediction horizon

        # NMPC parameters
        Q = np.diag([1, 1, 0.5])  # State cost
        R = np.diag([0.1, 0.1])   # Control cost

        # Define symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        n_states = states.size()[0]

        v = ca.SX.sym('v')
        delta = ca.SX.sym('delta')
        controls = ca.vertcat(v, delta)
        n_controls = controls.size()[0]

        # State update equations of the kinematic bicycle model
        rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), v/L * ca.tan(delta))
        f = ca.Function('f', [states, controls], [rhs])

        # Setup NMPC problem
        U = ca.SX.sym('U', n_controls, N)
        P = ca.SX.sym('P', n_states + n_states)
        X = ca.SX.sym('X', n_states, N+1)
        obj = 0  # Objective function
        g = []  # Constraints vector

        # Fill the NMPC problem
        for k in range(N):
            st = X[:, k]
            con = U[:, k]
            obj += ca.mtimes([(st-P[:3]).T, Q, (st-P[:3])])  # State cost
            obj += ca.mtimes([con.T, R, con])  # Control cost
            st_next = X[:, k+1]
            f_value = f(st, con)
            st_next_euler = st + (dt * f_value)
            g.append(st_next - st_next_euler)

        # Create an NLP solver
        OPT_variables = ca.vertcat(ca.reshape(U, -1, 1))
        nlp_prob = {'f': obj, 'x': OPT_variables, 'g': ca.vertcat(*g), 'p': P}
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

                # Define the reference path or trajectory
        self.ref_trajectory = np.array(...)  # Replace with actual reference trajectory

        # Initialize variables
        self.x0 = np.array([0, 0, 0])  # Initial state
        self.u0 = np.zeros((N, 2))     # Initial control
        self.X0 = ca.repmat(self.x0, 1, N+1)


        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()

    def run_step(self):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.

            :param target_speed: 
            :param waypoint: 
            :return: 
        """

        # Simulation loop
        for i in range(100):  # Number of simulation steps
            # Current reference point
            ref_point = self.ref_trajectory[i, :]
            
            # Solve the NMPC optimization problem
            sol = self.solver(x0=X0, p=ca.vertcat(x0, ref_point), lbg=0, ubg=0)
            u = np.array(sol['x']).reshape(N, n_controls)

            # Apply the first control input
            x0 = f(x0, u[0, :]).full().flatten()
            
            # Shift the control input for the next step
            u0 = np.vstack([u[1:, :], np.zeros(n_controls)])

        control = carla.VehicleControl()
        control.steer += 0.01
        control.throttle = 0.6
       

        return control





# To import a basic agent
class BasicAgent(object):
    def __init__(self, vehicle) -> None:
        self._vehicle = vehicle
        self._init_controller()
        self.done = False
        self.xs = None

        pass

    def set_destination(self, destination) -> None:
        self.xs = destination
        pass

    
    def _init_controller(self):
        """Controller initialization"""
        # self._vehicle_controller = VehiclePIDController(self._vehicle,
        #                                                 args_lateral=self._args_lateral_dict,
        #                                                 args_longitudinal=self._args_longitudinal_dict,
        #                                                 offset=self._offset,
        #                                                 max_throttle=self._max_throt,
        #                                                 max_brake=self._max_brake,
        #                                                 max_steering=self._max_steer)

        self._vehicle_controller = VehicleNMPCController(self._vehicle)

        # Compute the current vehicle waypoint
        # current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

    def run_step(self):
        control = self._vehicle_controller.run_step()
        return control
    
    def is_done(self):
        return self.done
    

# TODO adding typing parameters and returns

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class World(object):
    def __init__(self, carla_World, args) -> None:
        self.world = carla_World
        self.sync = args.sync
        self.actor_role_name = args.rolename
        self._actor_filter = args.filter
        self.actor_list = []
        self.ego_vehicle= None
        self.spectator = None
        self.virtual_sensor = None
        self.agent = None
        self.original_settings = None
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

    def setup(self):
        if self.sync:
            self.original_settings = self.world.get_settings()
            settings = self.world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                settings.rendering = True
            self.world.apply_settings(settings)


    def prepare(self):
        blueprint_library = self.world.get_blueprint_library()
        bp_vehicle = blueprint_library.filter(self._actor_filter).find('vehicle.audi.etron')
        bp_vehicle.set_attribute('role_name', self.actor_role_name)
        # set color
        bp_vehicle.set_attribute('color', '48, 80, 114')
        # customize the world
        # Toggle all buildings off
        # self.world.unload_map_layer(carla.MapLayer.All)
        # self.world.load_map_layer(carla.MapLayer.Ground)

        # spawn the ego vehicle
        while self.ego_vehicle is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = spawn_points[26]
            x0 = spawn_point
            xs = spawn_points[27]
            self.ego_vehicle = self.world.try_spawn_actor(bp_vehicle, spawn_point)
            if self.ego_vehicle:
                self.actor_list.append(self.ego_vehicle)

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        virtual_sensor_position = carla.Transform(carla.Location(x=-4, z=3.0), carla.Rotation(pitch=15.0))
        self.virtual_sensor = self.world.spawn_actor(camera_bp, virtual_sensor_position, attach_to=self.ego_vehicle, attachment_type=carla.AttachmentType.SpringArmGhost)
        # self.virtual_sensor = self.world.spawn_actor(camera_bp, virtual_sensor_position, attach_to=self.ego_vehicle)
        self.actor_list.append(self.virtual_sensor)
        # adding the spectators
        self.spectator = self.world.get_spectator()
        self.agent = BasicAgent(self.ego_vehicle)
        # destination = random.choice(spawn_points).location
        destination = spawn_points[140].location
        self.waypoints_plan = self.agent.set_destination(destination)
        # Draw the spawn point locations as numbers in the map
        # for i, spawn_point in enumerate(spawn_points):
        #     self.world.debug.draw_string(spawn_point.location, str(i), life_time=120)
        #     # self.world.debug.draw_point(spawn_point.location, life_time=0)
        #     pass
        pass


    def run_step(self):
        # self.spectator.set_transform(self.virtual_sensor.get_transform())
        self._follow_ego_bird_view()
        v=get_speed(self.ego_vehicle)
        self.world.debug.draw_string(self.ego_vehicle.get_location(),  'Speed: % 2.0f km/h' % v, color=carla.Color(5,150,200))
        agent_control = self.agent.run_step()
        ## -------------- Sim with sync mode --------------------
        # agent_control = carla.VehicleControl()
        # agent_control.throttle = 0.8
        # time.sleep(0.04)
        ## ------------------------------------------------------
        self.ego_vehicle.apply_control(agent_control)
        if self.sync:
            self.world.tick()
        else:    
            self.world.wait_for_tick()

        return self.ego_vehicle.get_transform()

    def is_done(self):
        return self.agent.is_done()

    def _follow_ego_bird_view(self):
        ego_t = self.ego_vehicle.get_transform()
        spect_location = ego_t.location + carla.Location(z=20)
        spect_rotation = carla.Rotation(yaw=ego_t.rotation.yaw, pitch = -85)
        spect_t = carla.Transform(spect_location, spect_rotation)
        self.spectator.set_transform(spect_t)
        pass

    def run_step_animation(self, name):
        path = Path(waypoints=self.waypoints_plan)
        uego = UpdateEgoVeh(path, ego_veh=self.ego_vehicle)
        anim = FuncAnimation(uego.fig, uego, interval = 100, blit=True)
        plt.show()

        pass
        

    def teardown(self):
        if self.original_settings:
            self.original_settings.rendering = False
            self.world.apply_settings(self.original_settings)
            self.original_settings = None
        for actor in self.actor_list:
            actor.destroy()



# ==============================================================================
# -- run() --------------------------------------------------------------------
# ==============================================================================

def run(args):
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        sim_world = client.get_world()
        if args.sync:
            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)
            if args.autopilot and not sim_world.get_settings().synchronous_mode:
                print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        world = World(sim_world, args)
        world.setup()
        if args.sync:
            client.reload_world(False) # reload map keeping the world settings
            pass
        world.prepare()

        run_anim_id = None

        if args.animate:
            run_anim_id = _thread.start_new_thread(world.run_step_animation, ("Plot Animation", ))
        # else:
        time.sleep(2)
        while True:
            print(timeit(lambda: world.run_step(), number=1))
            if world.is_done():
                logging.info("Goal Reached")
                # if _thread.exit()
                time.sleep(2)
                logging.info("Program Exit")
                break
    except Exception as e:
        print(f'Exception occured due to {e}')

    finally:
        if world is not None:
            world.teardown()





# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        default=False,
        action='store_true',
        help='Activate synchronous mode execution')
    argparser.add_argument(
        '--animate',
        default=False,
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(asctime)s %(levelname)-8s: %(message)s', level=log_level, datefmt='%Y-%m-%d %H:%M:%S')

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        run(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')



if __name__ == '__main__':
    main()