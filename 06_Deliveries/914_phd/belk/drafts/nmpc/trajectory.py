# trajectory.py
import numpy as np
import casadi as ca

from typing import Any, Tuple
from scipy.interpolate import CubicSpline

def initialise_cubic_spline(x: Any, y: Any, ds: float, bc_type: str) -> Tuple[CubicSpline, np.ndarray]:

    distance = np.concatenate((np.zeros(1), np.cumsum(np.hypot(np.ediff1d(x), np.ediff1d(y)))))
    points = np.array([x, y]).T
    s = np.arange(0, distance[-1], ds)
    
    try:
        cs = CubicSpline(distance, points, bc_type=bc_type, axis=0, extrapolate=False)
        
    except ValueError as e:
        raise ValueError(f"{e} If you are getting a sequence error, do check if your input dataset contains consecutive duplicate(s).")
 
    return cs, s

def generate_cubic_spline(x: Any, y: Any, ds: float=0.05, bc_type: str='natural') -> Tuple[np.ndarray, ...]:
    
    cs, s = initialise_cubic_spline(x, y, ds, bc_type)

    dx, dy = cs.derivative(1)(s).T
    yaw = np.arctan2(dy, dx)

    ddx, ddy = cs.derivative(2)(s).T
    curvature = (ddy*dx - ddx*dy) / ((dx*dx + dy*dy)**1.5)

    # cx, cy = cs(s).T

    return cs(s), yaw, curvature

    
class ReferenceTrajectory:
    def __init__(self) -> None:
        self.path = None
        self.trajectory = None
        self._load()
        self._generate_spline()
        self.x0 = self.cs(0)
        self.xs = self.cs(len(self.x)-1)
        self.size = len(self.x)


    def _load(self):
        self.path = np.load("./data/wps.npy")
        self.path = self.path[1:,]
        self.path = self.path[1::4,]
        self.path[:,2] = np.deg2rad(self.path[:,2])
        self.path = np.hstack((self.path, np.zeros((len(self.path),1))))

    
    def _generate_spline(self):
        self.x=self.path[:,0]
        self.y=self.path[:,1]
        self.psi=self.path[:,2]
        self.delta=self.path[:,3]
        # distance = np.concatenate((np.zeros(1), np.cumsum(np.hypot(np.ediff1d(self.x), np.ediff1d(self.y)))))
        points = np.array([self.x, self.y, self.psi, self.delta]).T
        # s = np.arange(0, distance[-1], dt)
    
        try:
            self.cs = CubicSpline(range(0,len(self.x)), points, bc_type="natural", axis=0, extrapolate=False)
            # dx, dy = cs.derivative(1)(s).T
            # self.yaw = np.arctan2(dy, dx)

            # ddx, ddy = cs.derivative(2)(s).T
            # self.curvature = (ddy*dx - ddx*dy) / ((dx*dx + dy*dy)**1.5)
            # self.cx, self.cy = cs(s).T
            # self.trajectory = np.array([self.cx,self.cy,self.yaw,np.zeros(len(self.cx))]).T
        
        except ValueError as e:
            raise ValueError(f"{e} If you are getting a sequence error, do check if your input dataset contains consecutive duplicate(s).")
 

    def get_fake_ref_points(self, step, horizon):
        array = self.path[step:step+1,:]
        return np.repeat(array, horizon, axis=0)

    
    def get_ref_points(self, step, horizon):
        return self.path[step:step+horizon,:]
    
    # def get_next_wp(self, step, dt):
    #     if step == 0:
    #         return self.path[step:step+1,0]
    #     xp = self.path[step:step+2,0]
    #     yp = self.path[step:step+2,1]
    #     cx,cy, cyaw, kp = generate_cubic_spline(xp,yp,dt)

    #     return np.array([cx,cy,cyaw,np.zeros(len(cx))]).T
    #     # return cx, cy, cyaw
    
    def get_reference(self):
        return ca.vertcat(self.x0, self.xs).full()[:,0]
    
    def get_next_wps(self, step, horizon):
        return self.trajectory[step*horizon:(step+1)*horizon,:]
    
    def get_next_wp(self, step):
        if step >= self.size:
            print("STEEEEEEEEEEEEEEEEP", step)
            return self.cs(self.size-1)
        else:
            return self.cs(step)


def main():
    
    from matplotlib import pyplot as plt

    trajectory = ReferenceTrajectory()
    px,py,cpsi, cdelta = trajectory.cs(range(160)).T
    # py = trajectory.cy

    x = trajectory.x
    y = trajectory.y
    psi = trajectory.psi

    # pyaw = trajectory.yaw
    # pk = trajectory.curvature

    ## vehicle model
    v = 5
    L=2.8
    xv=[x[0]]
    yv=[y[0]]
    for i,pv in enumerate(psi):     
        xv.append(v*np.cos(pv)+xv[i])
        yv.append(v*np.sin(pv)+yv[i])
    # yawv = v/L*tan()

    fig, ax = plt.subplots(1, 3, figsize=(15, 5))
    plt.style.use('seaborn-pastel')

    ax[0].set_box_aspect(1)
    ax[0].set_title('Geometry')
    ax[0].plot(px, py, c='m')
    # ax[0].plot(x, y, c='b')
    # ax[0].plot(xv, yv, 'x-g')
    # ax[0].plot(x, y, c='red')
    # ax[0].plot(px+3, py, c='m')
    # ax[0].plot(px-3, py, c='m')

    ax[1].set_box_aspect(1)
    ax[1].set_title('Yaw')
    # ax[1].plot(pyaw, c='m')
    # ax[1].plot(psi, c='b')

    ax[2].set_box_aspect(1)
    ax[2].set_title('Curvature')
    # ax[2].plot(pk, c='m')
    
    plt.show()

if __name__ == '__main__':
    main()

