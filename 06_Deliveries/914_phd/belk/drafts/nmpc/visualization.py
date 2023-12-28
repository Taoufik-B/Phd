# visualization.py
import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from time import time
import logging

log_level = logging.NOTSET

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
        return path_p, horizon_p, current_state, target_state, velocity_p, delta_p

    def animate(i):
        # get variables
        x = cat_states[0, 0, i]
        y = cat_states[1, 0, i]
        th = cat_states[2, 0, i]

        if i == len(t):
            v = cat_controls[0,0,i-1]
            delta = cat_controls[1,0,i-1]
        else:
            v = cat_controls[0,0,i]
            delta = cat_controls[1,0,i]
        # v = cat_controls[0,0,i]
        # delta = cat_controls[1,0,i]
        

        # update path
        if i == 0:
            path_p.set_data(np.array([]), np.array([]))
            velocity_p.set_data(np.array([]), np.array([]))
            delta_p.set_data(np.array([]), np.array([]))


        x_new = np.hstack((path_p.get_xdata(), x))
        y_new = np.hstack((path_p.get_ydata(), y))
        path_p.set_data(x_new, y_new)



        # update horizon
        x_new = cat_states[0, :, i]
        y_new = cat_states[1, :, i]
        horizon_p.set_data(x_new, y_new)

        # update controls
        vx_new = np.hstack((velocity_p.get_xdata(), i))
        vy_new = np.hstack((velocity_p.get_ydata(), v))
        velocity_p.set_data(vx_new,vy_new)
       
        deltax_new = np.hstack((delta_p.get_xdata(), i))
        deltay_new = np.hstack((delta_p.get_ydata(), delta))
        delta_p.set_data(deltax_new,deltay_new)


        # update current_state
        current_state.set_xy(create_triangle([x, y, th], update=True))

        # update target_state
        # xy = target_state.get_xy()
        # target_state.set_xy(xy)            

        return path_p, horizon_p, current_state, target_state, velocity_p, delta_p

    # create figure and axes
    fig = plt.figure(figsize=(6,6), layout='constrained')
    axs = fig.subplot_mosaic([["speed", "path"],
                              ["delta", "path"]])

    
    # create lines:
    #   path
    axs["path"].set_title("Path")
    axs["path"].set_xlabel("x position (m)")
    axs["path"].set_ylabel("y position (m)")
    
    
    path_p, = axs["path"].plot([], [], 'k', linewidth=2)
    #   horizon
    horizon_p, = axs["path"].plot([], [], 'x-g', alpha=0.8)

    #controls
    velocity_p, = axs["speed"].plot([], [], 'g', alpha=0.8)
    delta_p, = axs["delta"].plot([], [], 'b', alpha=0.8)

    velocity_p.axes.set_xlim(xmin=0, xmax=len(t))
    velocity_p.axes.set_ylim(ymin=-10, ymax=30)
    delta_p.axes.set_xlim(xmin=0, xmax=len(t))
    delta_p.axes.set_ylim(ymin=-4, ymax=4)


    trajectory_plot, = axs["path"].plot(trajectory[:,0], trajectory[:,1], 'ob', alpha=0.3)
    #   current_state
    current_triangle = create_triangle(reference[:3])
    current_state = axs["path"].fill(current_triangle[:, 0], current_triangle[:, 1], color='r')
    current_state = current_state[0]
    #   target_state
    target_triangle = create_triangle(reference[4:-1])
    target_state = axs["path"].fill(target_triangle[:, 0], target_triangle[:, 1], color='b')
    target_state = target_state[0]

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
        sim.save('./figures/fig' + str(time()) +'.gif', writer='ffmpeg', fps=30)

    return

