# trajectory.py
import numpy as np

class ReferenceTrajectory:
    def __init__(self):
        """
        Initialize the reference trajectory.
        """
        # Define a simple example trajectory
        # Format: [x, y, theta]
        self.trajectory_points = np.array([
            [0, 0, 0],
            [10, 0, 0],
            [20, 10, 45],
            [30, 20, 45],
            # ... Add more points as needed
        ])

    def get_ref_point(self, step):
        """
        Return the reference point for the given step.

        Parameters:
        - step: int, the current step in the simulation

        Returns:
        - ref_point: Array, the reference point [x, y, theta]
        """
        # Loop the trajectory if the step exceeds the length
        index = step % len(self.trajectory_points)
        return self.trajectory_points[index]
