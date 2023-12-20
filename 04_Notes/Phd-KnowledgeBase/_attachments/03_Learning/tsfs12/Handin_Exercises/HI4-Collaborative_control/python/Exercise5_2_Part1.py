#!/usr/bin/env python
# coding: utf-8

# %% TSFS12 Hand-in exercise 4: Collaborative control

# Initial imports

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collab_functions import multi_agent_ode, CreateAgent


# Activate matplotlib for notebook. Plots must be in external windows due to animations.


# %% Define agent dynamics, control, and measurement functions

agent_idx = [range(4 * i, 4 * (i + 1)) for i in range(0, 8)]

measurement_graph = [
    agent_idx[0],  # Agent 1
    np.dstack(
        (
            np.array([agent_idx[1], agent_idx[0]]),  # Agent 2
            np.array([agent_idx[1], agent_idx[2]]),
        )
    ),
    np.dstack(
        (
            np.array([agent_idx[2], agent_idx[1]]),  # Agent 3
            np.array([agent_idx[2], agent_idx[3]]),
        )
    ),
    np.dstack(
        (
            np.array([agent_idx[3], agent_idx[2]]),  # Agent 4
            np.array([agent_idx[3], agent_idx[4]]),
        )
    ),
    np.dstack(
        (
            np.array([agent_idx[4], agent_idx[3]]),  # Agent 5
            np.array([agent_idx[4], agent_idx[5]]),
        )
    ),
    np.dstack(
        (
            np.array([agent_idx[5], agent_idx[4]]),  # Agent 6
            np.array([agent_idx[5], agent_idx[6]]),
        )
    ),
    np.dstack(
        (
            np.array([agent_idx[6], agent_idx[5]]),  # Agent 7
            np.array([agent_idx[6], agent_idx[7]]),
        )
    ),
    np.dstack((np.array([agent_idx[7], agent_idx[6]]),)),  # Agent 8
]

formation_measurement_index = [{"idx": mi} for mi in measurement_graph]

formation_references = [
    lambda t: np.array([0, 4, 0, 0]).reshape(4, -1),
    lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
    lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
    lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
    lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
    lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
    lambda t: np.array([[0, 0], [1, -1], [0, 0], [0, 0]]),
    lambda t: np.array([0, 1, 0, 0]).reshape(4, -1),
]


# Firstly create your state, measurememt, and control functions, and assign them correctly to the corresponding agents. Note you have two types of agents here.

# Then the implementation is similar to what you did in Exercise 5.1

# %%
plt.show()
