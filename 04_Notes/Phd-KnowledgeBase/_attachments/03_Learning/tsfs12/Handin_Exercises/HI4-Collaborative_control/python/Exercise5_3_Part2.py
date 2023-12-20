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

agent_idx = ...  # Define your idx here

measurement_graph = [...]  # Define your measurement pairs here

formation_measurement_index = [{"idx": mi} for mi in measurement_graph]

formation_references = [...]  # Define your reference here


# Firstly create your state, measurememt, and control functions, and assign them correctly to the corresponding agents. Note you have two types of agents here.

# Then the implementation is similar to what you did in Exercise 5.2

# %%
plt.show()
