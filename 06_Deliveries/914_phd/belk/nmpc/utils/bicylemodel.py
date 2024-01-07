import matplotlib.pyplot as plt
import numpy as np

def draw_bicycle(ax, rear_wheel_center, wheelbase, steering_angle, bicycle_length):
    """
    Draws a simple kinematic bicycle model.
    :param ax: Matplotlib axis object
    :param rear_wheel_center: Tuple (x, y) for the center of the rear wheel
    :param wheelbase: Distance between the centers of the front and rear wheels
    :param steering_angle: Steering angle in radians
    :param bicycle_length: Total length of the bicycle
    """
    rear_x, rear_y = rear_wheel_center
    front_x = rear_x + wheelbase * np.cos(steering_angle)
    front_y = rear_y + wheelbase * np.sin(steering_angle)

    # Draw the bicycle body (line from rear to front wheel)
    ax.plot([rear_x, front_x], [rear_y, front_y], 'k-')

    # Draw the wheels as circles
    wheel_radius = wheelbase / 4
    rear_wheel = plt.Circle((rear_x, rear_y), wheel_radius, color='black', fill=False)
    front_wheel = plt.Circle((front_x, front_y), wheel_radius, color='black', fill=False)

    ax.add_artist(rear_wheel)
    ax.add_artist(front_wheel)

    # Optionally, extend the line to represent the total length of the bicycle
    extended_front_x = rear_x + bicycle_length * np.cos(steering_angle)
    extended_front_y = rear_y + bicycle_length * np.sin(steering_angle)
    ax.plot([front_x, extended_front_x], [front_y, extended_front_y], 'k--')

# Example usage
fig, ax = plt.subplots()
draw_bicycle(ax, rear_wheel_center=(0, 0), wheelbase=2, steering_angle=np.pi/6, bicycle_length=3)

# Set equal aspect ratio and limits for better visualization
ax.set_aspect('equal', 'box')
ax.set_xlim(-1, 4)
ax.set_ylim(-2, 2)
plt.show()
