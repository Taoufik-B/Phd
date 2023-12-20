import carla
import logging
import argparse
import sys
import json
import time
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from json import JSONEncoder
import _thread
from timeit import timeit

# To import a basic agent
from agents.motion.basic_agent import BasicAgent
from agents.libs.misc import get_speed
from agents.libs.plot import Path, UpdateEgoVeh

# TODO adding typing parameters and returns

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

# documentation example
class example:
    """
    Summary
    -------
    This class implements the 2D Kinematic Bicycle Model for vehicle dynamics

    Attributes
    ----------
    dt (float) : discrete time period [s]
    wheelbase (float) : vehicle's wheelbase [m]
    max_steer (float) : vehicle's steering limits [rad]

    Methods
    -------
    __init__(wheelbase: float, max_steer: float, delta_time: float=0.05)
        initialises the class

    update(x, y, yaw, velocity, acceleration, steering_angle)
        updates the vehicle's state using the kinematic bicycle model
    """
    
    def method_doc(self):
        """
        Summary
        -------
        Updates the vehicle's state using the kinematic bicycle model

        Parameters
        ----------
        x (int) : vehicle's x-coordinate [m]
        y (int) : vehicle's y-coordinate [m]
        yaw (int) : vehicle's heading [rad]
        velocity (int) : vehicle's velocity in the x-axis [m/s]
        acceleration (int) : vehicle's accleration [m/s^2]
        steering_angle (int) : vehicle's steering angle [rad]

        Returns
        -------
        new_x (int) : vehicle's x-coordinate [m]
        new_y (int) : vehicle's y-coordinate [m]
        new_yaw (int) : vehicle's heading [rad]
        new_velocity (int) : vehicle's velocity in the x-axis [m/s]
        steering_angle (int) : vehicle's steering angle [rad]
        angular_velocity (int) : vehicle's angular velocity [rad/s]
        """
        pass


class VehiclePhysicsControlEncoder(JSONEncoder):
        def default(self, o):
            return o.__dict__

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
            spawn_point = spawn_points[23]
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
        self.agent = BasicAgent(self.ego_vehicle, target_speed=70)
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
        agent_control = self.agent.run_step(debug=True)
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
        return self.agent.done()

    def _follow_ego_bird_view(self):
        ego_t = self.ego_vehicle.get_transform()
        spect_location = ego_t.location + carla.Location(z=45)
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