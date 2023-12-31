# pylint: skip-file
from csv import reader
from dataclasses import dataclass
from math import radians

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from agents.motion.model import KinematicBicycleModel
from agents.motion.controller import StanleyController
from agents.libs import CarDescription, generate_cubic_spline
from agents.tools.misc import get_waypoints_xy, write_trajectory_file


class Simulation:

    def __init__(self):

        fps = 50.0

        self.dt = 1/fps
        self.map_size_x = 70
        self.map_size_y = 40
        self.frames = 2500
        self.loop = False


class Path:

    def __init__(self, waypoints=None):
        ds = 0.05
        if waypoints is None:
            # Get path to waypoints.csv
            with open('agents/data/waypoints_racetrack.csv', newline='') as f:
                rows = list(reader(f, delimiter=','))

            x, y = [[float(i) for i in row] for row in zip(*rows[1:])]
        else:
            x, y= get_waypoints_xy(waypoints)
            print(x, y)
            write_trajectory_file(x, y)
        self.px, self.py, self.pyaw, _ = generate_cubic_spline(x, y, ds)


class Car:

    def __init__(self, init_x, init_y, init_yaw, px, py, pyaw, delta_time):

        # Model parameters
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw
        self.delta_time = delta_time
        self.time = 0.0
        self.velocity = 0.0
        self.wheel_angle = 0.0
        self.angular_velocity = 0.0
        max_steer = radians(33)
        wheelbase = 2.96



        # Acceleration parameters
        target_velocity = 50.0
        self.time_to_reach_target_velocity = 5.0
        self.required_acceleration = target_velocity / self.time_to_reach_target_velocity

        # Tracker parameters
        self.px = px
        self.py = py
        self.pyaw = pyaw
        self.k = 8.0
        self.ksoft = 1.0
        self.kyaw = 0.01
        self.ksteer = 0.0
        self.crosstrack_error = None
        self.target_id = None

        # Description parameters
        self.colour = 'black'
        overall_length = 4.97
        overall_width = 1.964
        tyre_diameter = 0.4826
        tyre_width = 0.265
        axle_track = 1.7
        rear_overhang = 0.5 * (overall_length - wheelbase)

        self.tracker = StanleyController(self.k, self.ksoft, self.kyaw, self.ksteer, max_steer, wheelbase, self.px, self.py, self.pyaw)
        self.kinematic_bicycle_model = KinematicBicycleModel(wheelbase, max_steer, self.delta_time)
        self.description = CarDescription(overall_length, overall_width, rear_overhang, tyre_diameter, tyre_width, axle_track, wheelbase)

    
    def get_required_acceleration(self):

        self.time += self.delta_time
        return self.required_acceleration
    

    def plot_car(self):
        
        return self.description.plot_car(self.x, self.y, self.yaw, self.wheel_angle)


    def drive(self):
        
        acceleration = 0 if self.time > self.time_to_reach_target_velocity else self.get_required_acceleration()
        self.wheel_angle, self.target_id, self.crosstrack_error = self.tracker.stanley_control(self.x, self.y, self.yaw, self.velocity, self.wheel_angle)
        self.x, self.y, self.yaw, self.velocity, _, _ = self.kinematic_bicycle_model.update(self.x, self.y, self.yaw, self.velocity, acceleration, self.wheel_angle)

        print(f"Cross-track term: {self.crosstrack_error}{' '*10}", end="\r")


@dataclass
class Fargs:
    ax: plt.Axes
    sim: Simulation
    path: Path
    car: Car
    car_outline: plt.Line2D
    front_right_wheel: plt.Line2D
    front_left_wheel: plt.Line2D
    rear_right_wheel: plt.Line2D
    rear_left_wheel: plt.Line2D
    rear_axle: plt.Line2D
    annotation: plt.Annotation
    target: plt.Line2D
   

class Animation:
    def __init__(self, waypoints=None):
        self.sim  = Simulation()
        self.path = Path(waypoints=waypoints)
        self.car  = Car(self.path.px[0], 
                        self.path.py[0], 
                        self.path.pyaw[0], 
                        self.path.px, 
                        self.path.py, 
                        self.path.pyaw, 
                        self.sim.dt)
        self.fig = plt.figure()
        self.fargs = None
        self.interval = self.sim.dt * 10**3
        self._plot()
        pass

    def _plot(self):
        

        self.fig = plt.figure()
        ax = plt.axes()
        ax.set_aspect('equal')

        # road = plt.Circle((0, 0), 50, color='gray', fill=False, linewidth=30)
        # ax.plot(path.px+3, path.py, '-', color='black')
        # ax.plot(path.px-3, path.py, '-', color='black')
        ax.plot(self.path.px, self.path.py, '-', color='black', linewidth=10, alpha=0.6)
        ax.plot(self.path.px, self.path.py, '--', color='gold')

        empty              = ([], [])
        target,            = ax.plot(*empty, '+r')
        car_outline,       = ax.plot(*empty, color=self.car.colour)
        front_right_wheel, = ax.plot(*empty, color=self.car.colour)
        rear_right_wheel,  = ax.plot(*empty, color=self.car.colour)
        front_left_wheel,  = ax.plot(*empty, color=self.car.colour)
        rear_left_wheel,   = ax.plot(*empty, color=self.car.colour)
        rear_axle,         = ax.plot(self.car.x, self.car.y, '+', color=self.car.colour, markersize=2)
        annotation         = ax.annotate(f'{self.car.x:.1f}, {self.car.y:.1f}', xy=(self.car.x, self.car.y + 5), color='black', annotation_clip=False)

        self.fargs = [Fargs(
            ax=ax,
            sim=self.sim,
            path=self.path,
            car=self.car,
            car_outline=car_outline,
            front_right_wheel=front_right_wheel,
            front_left_wheel=front_left_wheel,
            rear_right_wheel=rear_right_wheel,
            rear_left_wheel=rear_left_wheel,
            rear_axle=rear_axle,
            annotation=annotation,
            target=target
        )]
        plt.grid()
        plt.show()
        pass

    def _animate(self, frame):

        ax                = self.fargs.ax
        sim               = self.fargs.sim
        path              = self.fargs.path
        car               = self.fargs.car
        car_outline       = self.fargs.car_outline
        front_right_wheel = self.fargs.front_right_wheel
        front_left_wheel  = self.fargs.front_left_wheel
        rear_right_wheel  = self.fargs.rear_right_wheel
        rear_left_wheel   = self.fargs.rear_left_wheel
        rear_axle         = self.fargs.rear_axle
        annotation        = self.fargs.annotation
        target            = self.fargs.target

        # Camera tracks car
        # ax.set_xlim(car.x - sim.map_size_x, car.x + sim.map_size_x)
        # ax.set_ylim(car.y - sim.map_size_y, car.y + sim.map_size_y)

        # Drive and draw car
        car.drive()
        outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = car.plot_car()
        car_outline.set_data(*outline_plot)
        front_right_wheel.set_data(*fr_plot)
        rear_right_wheel.set_data(*rr_plot)
        front_left_wheel.set_data(*fl_plot)
        rear_left_wheel.set_data(*rl_plot)
        rear_axle.set_data(car.x, car.y)

        # Show car's target
        target.set_data(path.px[car.target_id], path.py[car.target_id])
        # target.set_data(car.x, car.y)

        # Annotate car's coordinate above car
        annotation.set_text(f'{car.x:.1f}, {car.y:.1f}')
        annotation.set_position((car.x, car.y + 5))

        plt.title(f'{sim.dt*frame:.2f}s', loc='right')
        plt.xlabel(f'Speed: {car.velocity:.2f} m/s', loc='left')
        # plt.savefig(f'image/visualisation_{frame:03}.png', dpi=300)

        return car_outline, front_right_wheel, rear_right_wheel, front_left_wheel, rear_left_wheel, rear_axle, target,

    def run_step(self):
        _ = FuncAnimation(self.fig, self._animate, frames=self.sim.frames, init_func=lambda: None, interval=self.interval, repeat=self.sim.loop)

        pass

def main():
    pass
    
    # sim  = Simulation()
    # path = Path()
    # car  = Car(path.px[0], path.py[0], path.pyaw[0], path.px, path.py, path.pyaw, sim.dt)

    # interval = sim.dt * 10**3

    # fig = plt.figure()
    # ax = plt.axes()
    # ax.set_aspect('equal')

    # # road = plt.Circle((0, 0), 50, color='gray', fill=False, linewidth=30)
    # # ax.plot(path.px+3, path.py, '-', color='black')
    # # ax.plot(path.px-3, path.py, '-', color='black')
    # ax.plot(path.px, path.py, '-', color='black', linewidth=10, alpha=0.6)
    # ax.plot(path.px, path.py, '--', color='gold')

    # empty              = ([], [])
    # target,            = ax.plot(*empty, '+r')
    # car_outline,       = ax.plot(*empty, color=car.colour)
    # front_right_wheel, = ax.plot(*empty, color=car.colour)
    # rear_right_wheel,  = ax.plot(*empty, color=car.colour)
    # front_left_wheel,  = ax.plot(*empty, color=car.colour)
    # rear_left_wheel,   = ax.plot(*empty, color=car.colour)
    # rear_axle,         = ax.plot(car.x, car.y, '+', color=car.colour, markersize=2)
    # annotation         = ax.annotate(f'{car.x:.1f}, {car.y:.1f}', xy=(car.x, car.y + 5), color='black', annotation_clip=False)

    # fargs = [Fargs(
    #     ax=ax,
    #     sim=sim,
    #     path=path,
    #     car=car,
    #     car_outline=car_outline,
    #     front_right_wheel=front_right_wheel,
    #     front_left_wheel=front_left_wheel,
    #     rear_right_wheel=rear_right_wheel,
    #     rear_left_wheel=rear_left_wheel,
    #     rear_axle=rear_axle,
    #     annotation=annotation,
    #     target=target
    # )]

    # _ = FuncAnimation(fig, animate, frames=sim.frames, init_func=lambda: None, fargs=fargs, interval=interval, repeat=sim.loop)
    # # anim.save('animation.gif', writer='imagemagick', fps=50)
    
    # plt.grid()
    # plt.show()


if __name__ == '__main__':
    main()
