import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from time import time


def simulate(trajectory, cat_states, cat_controls, t, step_horizon, N, reference, save=False):
    def create_triangle(state=[0,0,0], h=0.5, w=0.25, update=False):
        x, y, th = state
        triangle = np.array([
            [h, 0   ],
            [0,  w/2],
            [0, -w/2],
            [h, 0   ]
        ]).T
        rotation_matrix = np.array([
            [cos(th), -sin(th)],
            [sin(th),  cos(th)]
        ])

        coords = np.array([[x, y]]) + (rotation_matrix @ triangle).T
        if update == True:
            return coords
        else:
            return coords[:3, :]
        
    def create_circle(center=[0,0], radius =0.5, num_points=100 ):
        x_center,y_center=center

        theta = np.linspace(0, 2 * np.pi, num_points)  # Angle from 0 to 2*pi.
        x = x_center + radius * np.cos(theta)  # X coordinates.
        y = y_center + radius * np.sin(theta)  # Y coordinates.

        return np.array([x, y])


        pass

    def init():
        return path, horizon, current_state, target_state,

    def animate(i):
        # get variables
        x = cat_states[0, 0, i]
        y = cat_states[1, 0, i]
        th = cat_states[2, 0, i]
        

        # update path
        if i == 0:
            path.set_data(np.array([]), np.array([]))
        x_new = np.hstack((path.get_xdata(), x))
        y_new = np.hstack((path.get_ydata(), y))
        path.set_data(x_new, y_new)

        # update horizon
        x_new = cat_states[0, :, i]
        y_new = cat_states[1, :, i]
        horizon.set_data(x_new, y_new)

        # update current_state
        current_state.set_xy(create_triangle([x, y, th], update=True))

        # update target_state
        # xy = target_state.get_xy()
        # target_state.set_xy(xy)            

        return path, horizon, current_state, target_state,

    # create figure and axes
    fig, ax = plt.subplots(figsize=(6, 6))
    min_scale = min(reference[0], reference[1], reference[3], reference[4]) - 50
    max_scale = max(reference[0], reference[1], reference[3], reference[4]) + 20
    # ax.set_xlim(left = min_scale, right = max_scale)
    # ax.set_ylim(bottom = min_scale, top = max_scale)

    # create lines:
    #   path
    path, = ax.plot([], [], 'k', linewidth=2)
    #   horizon
    horizon, = ax.plot([], [], 'x-g', alpha=0.8)

    trajectory_plot, = ax.plot(trajectory[:,0], trajectory[:,1], 'x-r', alpha=0.3)
    #   current_state
    current_triangle = create_triangle(reference[:3])
    current_state = ax.fill(current_triangle[:, 0], current_triangle[:, 1], color='r')
    current_state_1 = ax.fill(current_triangle[:, 0], current_triangle[:, 1], color='g')
    current_state = current_state[0]
    #   target_state
    target_triangle = create_triangle(reference[3:])
    target_state = ax.fill(target_triangle[:, 0], target_triangle[:, 1], color='b')
    target_state = target_state[0]
    # Trying obstacles spawning
    # obstacle_circle = create_circle([1.5,3], 0.25)
    # obstacle_state  = ax.plot(obstacle_circle[0], obstacle_circle[1], color='g')
    # obstacle_state = obstacle_state[0]
    # obstacle_circle_2 = create_circle([5,5], 1.25)
    # obstacle_state_2  = ax.plot(obstacle_circle_2[0], obstacle_circle_2[1], color='g')
    # obstacle_state_2 = obstacle_state_2[0]
    sim = animation.FuncAnimation(
        fig=fig,
        func=animate,
        init_func=init,
        frames=len(t),
        interval=N,
        blit=True,
        repeat=True
    )
    plt.show()

    if save == True:
        sim.save('./animation' + str(time()) +'.gif', writer='ffmpeg', fps=30)

    return

