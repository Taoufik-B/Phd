import matplotlib.pyplot as plt
import numpy as np
import math



trajectory = np.load('wps.npy')
vehicle_trajectory =  np.load('vp.npy')
controls =  np.load('controls.npy')

distance = [0,]
for i,p in enumerate(vehicle_trajectory):
    if i+1 >= len(vehicle_trajectory):
        break
    distance.append(math.sqrt((vehicle_trajectory[i+1,0]-vehicle_trajectory[i,0])**2 + (vehicle_trajectory[i+1,1]-vehicle_trajectory[i,1])**2 ))

# print(distance)


fig, ax = plt.subplots(figsize=(6, 6))
fig_1, ax1 = plt.subplots(figsize=(6, 6))
trajectory_plot, = ax.plot(trajectory[:,0], trajectory[:,1], 'x-r', alpha=0.3)
vehicle_plot, = ax.plot(vehicle_trajectory[:,0], vehicle_trajectory[:,1], 'b', alpha=0.3, linewidth=5)


# speed_plot, = ax1.plot(np.linspace(0,len(controls), len(controls)), controls[:,0], 'b', alpha=0.3, linewidth=5)
# speed_plot, = ax1.plot(vehicle_trajectory[:,0], controls[:,0], 'b', alpha=0.3, linewidth=5)
# steering_plot, = ax1.plot(np.linspace(0,len(controls), len(controls)), controls[:,1], 'b', alpha=0.3, linewidth=5)
steering_plot, = ax1.plot(vehicle_trajectory[:,0], controls[:,1], 'b', alpha=0.3, linewidth=5)
# steering_plot, = ax1.plot(distance, controls[:,1], 'b', alpha=0.3, linewidth=5)

plt.show()