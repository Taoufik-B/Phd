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
        self.path = np.load("./data/wps.npy")
        self.path = self.path[1:,]
        # self.path = self.path[1::4,]
        self.path[:,2] = np.deg2rad(self.path[:,2])
        self.path = np.hstack((self.path, np.zeros((len(self.path),1))))
        self.x0 = self.path[0,:]
        self.xs = self.path[-1,:]

    def get_fake_ref_points(self, step, horizon):
        array = self.path[step:step+1,:]
        print(self.path[step:step+2,:])
        print(array)
        print(np.repeat(array, horizon, axis=0))
        print(np.repeat(array, horizon, axis=1))
        return np.repeat(array, horizon, axis=0)

    
    def get_ref_points(self, step, horizon):
        return self.path[step:step+horizon,:]
    
    def get_reference(self):
        return ca.vertcat(self.x0, self.xs).full()[:,0]
