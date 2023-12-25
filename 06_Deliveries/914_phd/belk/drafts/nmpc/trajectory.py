# trajectory.py
import numpy as np
import casadi as ca

    
class ReferenceTrajectory:
    def __init__(self) -> None:
        self.x0 = None
        self.xs = None
        self.path = None
        self._load()
        self.size = len(self.path)

    def _load(self):
        self.path = np.load("./wps.npy")
        self.path = self.path[1:,]
        self.path[:,2] = np.deg2rad(self.path[:,2])
        self.x0 = self.path[0,:]
        self.xs = self.path[-1,:]

    
    def get_ref_points(self, step, horizon):
        return self.path[step:step+horizon,:]
    
    def get_reference(self):
        return ca.vertcat(self.x0, self.xs).full()[:,0]
