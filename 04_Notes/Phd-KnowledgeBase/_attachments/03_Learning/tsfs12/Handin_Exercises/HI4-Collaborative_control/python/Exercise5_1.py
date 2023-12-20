#!/usr/bin/env python
# coding: utf-8

# %% TSFS12 Hand-in exercise 4: Collaborative control

# Initial imports

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collab_functions import CreateAgent, multi_agent_ode


# Activate matplotlib for notebook. Plots must be in external windows due to animations.


# %% Define agent dynamics, control, and measurement functions


# You need to follow the functions f1, h1, g1 to define your f2, h2, g2 functions here
# You need to define new functions f2, g2, h2 here, according to the description of Exercise 5.1
def f1(t, x, u, mdlpar):
    """Dynamic function for single integrator model

    Input arguments are:
       t - time
       x - state vector of the agent (position)
       u - the control input u,
       mdlpar - dictionary with parameters of the model

    Output:
       dxdt - state-derivative
    """
    return u


def g1(y, xref, ctrlpar):
    """Control function

    Compute the control signal, in this case it is a P-controller which gives a control signal
    proportional to the vector between the current position and the reference position.

    Input arguments:
      y - measurement y,
      xref - the reference vector xref (in this case the desired position of the agent)
      ctrlpar - dictionary which contains parameters used by the controller (in this case the proportional gain k).

    Output argument:
      Control signal
    """

    k = ctrlpar["k"]
    return k * (xref - y)


def h1(x, measpar):
    """Measurement function

     The measurement function gives the measurements available to the
     agent.

     Input arguments:
         x - the complete state vector of all agents in the multi-agent system
         measpar - dictionary containing parameters used in the function. In
                   this case the indices in the state vector x of the states measured by the
                   agent stored in key idx.

    Output:
        y - The measurement vector
    """
    meas_idx = measpar["idx"]
    return x[meas_idx]


# %% Create all agents

# Create all agents, specifying models, parameters, controllers, the measurement structure, and references for controllers.

# Think about how to update the model parameters here, as you need to include wind force
modelparam = {"m": 1}
# Think about how to update this according to f2, g2, h2
controller_1 = {"k": 1}
controller_2 = {"k": 10}
# Think about how to update this according to f2, g2, h2
simple_formation_measurement_index = [
    {"idx": [0, 1]},
    {"idx": [2, 3]},
    {"idx": [4, 5]},
    {"idx": [6, 7]},
]
# Think about how to update this according to f2, g2, h2
simple_formation_references = [
    lambda t: [np.cos(2 * t), np.sin(2 * t)],
    lambda t: [2 * np.cos(-t), 2 * np.sin(-t)],
    lambda t: [3, 2],
    lambda t: [5, 4],
]
# Think about how to update this according to f2, g2, h2
agents = [
    CreateAgent(
        f1,
        modelparam,
        g1,
        controller_2,
        h1,
        simple_formation_measurement_index[0],
        simple_formation_references[0],
    ),
    CreateAgent(
        f1,
        modelparam,
        g1,
        controller_1,
        h1,
        simple_formation_measurement_index[1],
        simple_formation_references[1],
    ),
    CreateAgent(
        f1,
        modelparam,
        g1,
        controller_1,
        h1,
        simple_formation_measurement_index[2],
        simple_formation_references[2],
    ),
    CreateAgent(
        f1,
        modelparam,
        g1,
        controller_2,
        h1,
        simple_formation_measurement_index[3],
        simple_formation_references[3],
    ),
]
n = 4  # Number of states for each agent, you have 4 states here


# %% Simulate the multi-agent system

# This part should not need to be modified except specifying the initial state.

x0 = np.array(
    [1, 0, 2, 0, 3, 0, 4, 0]
)  # Think about how to update this according to f2, g2, h2
t = np.arange(0, 10, 0.05)
x = odeint(lambda x, t: multi_agent_ode(x, t, agents, n), x0, t)


# Animate the simulated agents.

fig, ax = plt.subplots(num=20, clear=True)
ax.axis([-2, 6, -3, 6])
# You need to update the plot function to plot trajectory
m = tuple([ax.plot(np.nan, np.nan, "bo")[0] for k in range(len(agents))])


def animate(i):
    for idx, mi in enumerate(m):
        mi.set_xdata(x[i, 0 + n * idx])
        mi.set_ydata(x[i, 1 + n * idx])
    return m


ani = animation.FuncAnimation(
    fig, animate, interval=0, frames=x.shape[0], blit=True, save_count=50, repeat=False
)

# %%
plt.show()
