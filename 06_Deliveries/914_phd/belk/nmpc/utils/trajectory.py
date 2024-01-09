# trajectory.py
import numpy as np
import casadi as ca

from scipy.interpolate import CubicSpline

    
class ReferenceTrajectory:
    def __init__(self, wps_file, wps_factor) -> None:
        self.path = None
        self.trajectory = None
        self._load(wps_file, wps_factor)
        self._generate_spline()
        self.x0 = self.cs(0)
        self.xs = self.cs(len(self.x)-1)
        self.us = [0,0]
        self.size = len(self.x)


    def _load(self, wps_file, wps_factor):
        self.path = np.load(wps_file)
        self.path = self.path[1:,]
        self.path = self.path[1::wps_factor,]
        self.path[:,2] = np.deg2rad(self.path[:,2])
        self.path = np.hstack((self.path, np.zeros((len(self.path),1))))

    
    def _generate_spline(self):
        self.x=self.path[:,0]
        self.y=self.path[:,1]
        self.psi=self.path[:,2]
        self.delta=self.path[:,3]
        points = np.array([self.x, self.y, self.psi, self.delta]).T    
        try:
            self.cs = CubicSpline(range(0,len(self.x)), points, bc_type="natural", axis=0, extrapolate=False)
        
        except ValueError as e:
            raise ValueError(f"{e} If you are getting a sequence error, do check if your input dataset contains consecutive duplicate(s).")
    
    def get_reference(self):
        return ca.vertcat(self.x0, self.xs).full()[:,0] 
    
    def get_next_wp(self, step):
        if step >= self.size:
            return self.cs(self.size-1)
        else:
            return self.cs(step)
    def get_tracking_wps(self, step, N, dT):
        p_x_ref = np.ndarray((4,0))
        p_u_ref = np.ndarray((2,0))
        for k in range(N):
            t_predict = (step+k)*dT
            cx = self.get_next_wp(t_predict)
            cx[2] = np.clip(cx[2], -np.pi, np.pi)
            np.nan_to_num(cx,False, self.xs)
            cu = self.get_fd_wp(t_predict)
            print(f"us :: {self.us}")
            np.nan_to_num(cu,False)
            p_x_ref = ca.horzcat(p_x_ref, cx)
            p_u_ref = ca.horzcat(p_u_ref, cu)
        return p_x_ref, p_u_ref
        
        
    def get_fd_wp(self, step):
        v_vect = self.cs.derivative(1)(step)[0:2]
        phi_ref = self.cs.derivative(1)(step)[2]
        return [float(ca.norm_2(v_vect)), float(phi_ref)]


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

