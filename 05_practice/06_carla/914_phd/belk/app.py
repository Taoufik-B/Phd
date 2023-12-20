import carla
import logging
import argparse
import sys
import random
import math
from matplotlib import pyplot as plt

# To import a basic agent
from agents.motion.basic_agent import BasicAgent
from agents.tools.misc import get_speed

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class World(object):
    SPECT_PITCH = carla.Rotation(pitch=-90)
    def __init__(self, carla_World, args) -> None:
        self.world = carla_World
        self.sync = args.sync
        self.actor_role_name = args.rolename
        self._actor_filter = args.filter
        self.actor_list = []
        self.ego_vehicle = None
        self.spectator = None
        self.virtual_sensor = None
        self.agent = None
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.start()

    def start(self):
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
        # set autopilot
        # self.ego_vehicle.set_autopilot(True)
        # To start a behavior agent with an aggressive profile
        self.agent = BasicAgent(self.ego_vehicle, target_speed=70)
        # destination = random.choice(spawn_points).location
        destination = spawn_points[140].location
        self.agent.set_destination(destination, animate=False)
        # Draw the spawn point locations as numbers in the map
        for i, spawn_point in enumerate(spawn_points):
            # self.world.debug.draw_string(spawn_point.location, str(i), life_time=120)
            # self.world.debug.draw_point(spawn_point.location, life_time=0)
            pass
        pass

    def tick(self):
        # self.spectator.set_transform(self.virtual_sensor.get_transform())
        self.follow_ego_bird_view()
        v=get_speed(self.ego_vehicle)
        self.world.debug.draw_string(self.ego_vehicle.get_location(),  'Speed: % 2.0f km/h' % v, color=carla.Color(5,150,200))
        self.ego_vehicle.apply_control(self.agent.run_step(debug=True, animate=False))
        # print(get_speed(self.ego_vehicle))
        self.world.wait_for_tick()

    def follow_ego_bird_view(self):
        ego_t = self.ego_vehicle.get_transform()
        spect_location = ego_t.location + carla.Location(z=45)
        spect_rotation = carla.Rotation(yaw=ego_t.rotation.yaw, pitch = -85)
        spect_t = carla.Transform(spect_location, spect_rotation)
        self.spectator.set_transform(spect_t)
        pass
        

    def destroy(self):
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
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                settings.rendering = True
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)
        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")
        
        world = World(sim_world, args)
        
        while True:
            world.tick()
    except Exception as e:
        print(f'Exception occured due to {e}')

    finally:
        if original_settings:
            original_settings.rendering = False
            sim_world.apply_settings(original_settings)

        if world is not None:
            actors_list = world.get_actors()
            for actor in actors_list:
                actor.destroy()
            world.destroy()





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
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        run(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')



if __name__ == '__main__':

    main()