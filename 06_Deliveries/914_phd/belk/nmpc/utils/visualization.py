# visualization.py
import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from time import time
import logging

log_level = logging.NOTSET

def simulate(trajectory, params, cat_states, cat_controls, t, step_horizon, N, reference, scenario, save=False):
    def create_triangle(state=[0,0,0], h=3, w=1.25, update=False):
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
        return path_p, horizon_p, params_p, current_state, target_state, velocity_p, vref_p, delta_p, yaw_v_p, yaw_ref_p, phi_p

    def animate(i):
        # get variables
        x = cat_states[0, 0, i]
        y = cat_states[1, 0, i]
        psi = cat_states[2, 0, i]
        delta = cat_states[3, 0, i]

        # params
        xp = params[0, 0, i]
        yp = params[1, 0, i]
        yawp = params[2, 0, i]


        if i == len(t):
            v = cat_controls[0,0,i-1]
            phi = cat_controls[1,0,i-1]
        else:
            v = cat_controls[0,0,i]
            phi = cat_controls[1,0,i]
        # v = cat_controls[0,0,i]
        # delta = cat_controls[1,0,i]
        

        # update path
        if i == 0:
            path_p.set_data(np.array([]), np.array([]))
            params_p.set_data(np.array([]), np.array([]))
            velocity_p.set_data(np.array([]), np.array([]))
            vref_p.set_data(np.array([]), np.array([]))
            delta_p.set_data(np.array([]), np.array([]))
            yaw_ref_p.set_data(np.array([]), np.array([]))
            yaw_v_p.set_data(np.array([]), np.array([]))
            phi_p.set_data(np.array([]), np.array([]))


        x_new = np.hstack((path_p.get_xdata(), x))
        y_new = np.hstack((path_p.get_ydata(), y))
        path_p.set_data(x_new, y_new)

        # update horizon
        x_new = cat_states[0, :, i]
        y_new = cat_states[1, :, i]
        horizon_p.set_data(x_new, y_new)

        # params
        x_new = params[0,:,i]
        y_new = params[1,:,i]
        vref_new = params[4,0,i]
        params_p.set_data(x_new, y_new)

        # update controls
        vx_new = np.hstack((velocity_p.get_xdata(), i))
        vy_new = np.hstack((velocity_p.get_ydata(), v))
        velocity_p.set_data(vx_new,vy_new)
        
        vx_new = np.hstack((vref_p.get_xdata(), i))
        vy_new = np.hstack((vref_p.get_ydata(), vref_new))
        vref_p.set_data(vx_new,vy_new)
       
        deltax_new = np.hstack((delta_p.get_xdata(), i))
        deltay_new = np.hstack((delta_p.get_ydata(), delta))
        delta_p.set_data(deltax_new,deltay_new)
        
        x_new= np.hstack((yaw_ref_p.get_xdata(), i))
        y_new= np.hstack((yaw_ref_p.get_ydata(), yawp))
        yaw_ref_p.set_data(x_new,y_new)

        x_new= np.hstack((yaw_v_p.get_xdata(), i))
        y_new = np.hstack((yaw_v_p.get_ydata(), psi))
        yaw_v_p.set_data(x_new,y_new)
        #phi
        x_new= np.hstack((phi_p.get_xdata(), i))
        y_new = np.hstack((phi_p.get_ydata(), phi))
        phi_p.set_data(x_new,y_new)
        # deltax_new = np.hstack((delta_p.get_xdata(), i))
        # deltay_new = np.hstack((delta_p.get_ydata(), delta))
        # delta_p.set_data(deltax_new,deltay_new)


        # update current_state
        current_state.set_xy(create_triangle([x, y, psi], update=True))

        # update target_state
        # xy = target_state.get_xy()
        # target_state.set_xy(xy)            

        return path_p, params_p, horizon_p, current_state, target_state, velocity_p, vref_p, delta_p,yaw_ref_p, yaw_v_p, phi_p

    # create figure and axes
    fig = plt.figure(figsize=(6,6), layout='constrained')
    axs = fig.subplot_mosaic([
                            ["yaw", "path"],
                            ["delta", "path"],
                            ["speed", "rmse"]
                            ])

    
    #   path
    axs["path"].set_title("Path")
    axs["path"].set_xlabel("x position (m)")
    axs["path"].set_ylabel("y position (m)")
    #   Heading angle
    axs["yaw"].set_title("Heading angle")
    axs["yaw"].set_xlabel("frames")
    axs["yaw"].set_ylabel("psi angle (rad)")
    #   Heading angle
    axs["delta"].set_title("Steering angle")
    axs["delta"].set_xlabel("frames")
    axs["delta"].set_ylabel("delta angle (rad)")
    #   velocity
    axs["speed"].set_title("Velocity")
    axs["speed"].set_xlabel("frames")
    axs["speed"].set_ylabel("velocity (m/s)")

    # for direction in ["left", "right", "bottom", "top"]:
    # hides borders
    for ax in axs:
        axs[ax].spines['right'].set_visible(False)
        axs[ax].spines['top'].set_visible(False)
    
    
    path_p, = axs["path"].plot([], [], 'k', linewidth=2, label="Reference path")
    #   horizon
    horizon_p, = axs["path"].plot([], [], 'x-g', alpha=0.4, label="MPC horizon")
    # params tracking trajectory
    params_p, = axs["path"].plot([], [], 'x-',color='orange', alpha=0.6, label='Parametrized trajectory')

    #controls
    velocity_p, = axs["speed"].step([], [], '--b', alpha=0.8, label="Vehicle Velocity")
    vref_p, = axs["speed"].step([], [], '--k', alpha=0.4, label="Reference Velocity")
    axs["speed"].legend(loc="upper right")

    delta_p, = axs["delta"].plot([], [], 'b', alpha=0.8, label="Vehicle Steering Angle (rad)")
    phi_p, = axs["delta"].step([], [], '--k', alpha=0.4, label="Vehicle Steering rate (rad/s)")
    axs["delta"].legend(loc="upper right")

    yaw_v_p, = axs["yaw"].plot([], [], 'b', alpha=0.8, label="Vehicle Heading Angle")
    yaw_ref_p, = axs["yaw"].plot([], [], '--k', alpha=0.4, label="Reference Heading Angle")
    axs["yaw"].legend(loc="upper right")

    ## to do computation
    rmse_X = np.linalg.norm(cat_states[:3,0,:]-params[:3,0,:])
    print(rmse_X)
    rmse_x, = axs["rmse"].plot([], [], 'b', alpha=0.8, label="RMSE x")



    velocity_p.axes.set_xlim(xmin=0, xmax=len(t))
    velocity_p.axes.set_ylim(ymin=0, ymax=25)
    velocity_p.axes.set_autoscaley_on(True)
    axs["delta"].axes.set_xlim(xmin=0, xmax=len(t))
    axs["delta"].axes.set_ylim(ymin=-1,ymax=1)
    yaw_ref_p.axes.set_xlim(xmin=0, xmax=len(t))
    yaw_ref_p.axes.set_ylim(ymin=-4, ymax=4)
    yaw_v_p.axes.set_xlim(xmin=0, xmax=len(t))
    yaw_v_p.axes.set_ylim(ymin=-4, ymax=4)


    trajectory_plot, = axs["path"].plot(trajectory[:,0], trajectory[:,1], 'ob', alpha=0.3)
    #   current_state
    current_triangle = create_triangle(reference[:3])
    current_state = axs["path"].fill(current_triangle[:, 0], current_triangle[:, 1], color='m', label='Vehicle position')
    current_state = current_state[0]
    #   target_state
    target_triangle = create_triangle(reference[4:-1])
    target_state = axs["path"].fill(target_triangle[:, 0], target_triangle[:, 1], color='b', label='Target vehicle position')
    target_state = target_state[0]
    axs["path"].legend(loc="center right")

    sim = animation.FuncAnimation(
        fig=fig,
        func=animate,
        init_func=init,
        frames=len(t),
        interval=100,
        blit=True,
        repeat=True
    )

    fig.suptitle(scenario, fontsize=24)
    plt.show()
    # plt.legend()


    if save == True:
        print("Saving Annimation for Scenario %s" % scenario)
        sim.save(f'./figures/fig_{scenario}.gif', writer='ffmpeg', fps=30)
        plt.savefig(f'./figures/fig_{scenario}.png')
 

if __name__ == '__main__':
    from trajectory import ReferenceTrajectory
    scenario = 'sc_00#003'
    path = 'data'
    # plot title
    x = np.load(f'{path}/x_{scenario}.npy')
    u = np.load(f'{path}/u_{scenario}.npy')
    p = np.load(f'{path}/p_{scenario}.npy')
    trajectory = ReferenceTrajectory(f'maps/carla_town05_02012024.wps', 4)
    N = 5
    dT = 0.5
    print((p.shape))
    simulate( trajectory=trajectory.path
            ,params=p
            ,cat_states=x
            ,cat_controls=u
            ,t=x[0,0,:]
            ,step_horizon=dT
            ,N=N
            ,reference=trajectory.get_reference()
            ,scenario=scenario
            ,save=False
            )
    pass