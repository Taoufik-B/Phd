from time import time
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from simulation_code import simulate



class NMPCController:
    def __init__(self, dt, N, ref_trajectory, Q, R) -> None:
        """
        Initialize the NMPC controller.

        Parameters:
        - dt: float, time step
        - N: int, prediction horizon
        """
        self.dt = dt
        self.N = N
        self.L =2.8

        self.ref_trajectory = ref_trajectory

        self.Q, self.R = Q,R
        
        # Define the NMPC problem
        self._setup_mpc()


        pass
    def _setup_mpc(self):
        """
        Setup the NMPC optimization problem
        """
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')

        
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')

        # State vector and control inputs
        states = ca.vertcat(x, y, theta)
        controls = ca.vertcat(v, omega)

        n_states = states.numel()
        n_controls = controls.numel()

        # State update equation
        rhs = ca.vertcat(
            v*ca.cos(theta),
            v*ca.sin(theta),
            v/self.L*ca.tan(omega)
            # omega
        )
        self.f = ca.Function('f', [states, controls], [rhs])
        
        # Decision variables
        # P = ca.SX.sym('P', n_states*2) # parameters (Initial and reference trajectory)
        n_ref = n_controls+n_states
        P = ca.SX.sym('P', n_states+self.N*(n_ref)) # parameters (Initial and reference trajectory)
        U = ca.SX.sym('U', n_controls, self.N) # controls

        # Matrix of the states over the optimization problem
        X = ca.SX.sym('X', n_states, self.N+1)


        # Define the objective function
        obj = 0
        # Define the weighing matrices
        Q = np.diag((5, 10, 0.05))  # states
        R = np.diag((0.5, 0.05))    # controls
        # Compute the solution
        g=[]

        

        g = ca.vertcat(g,X[:,0]-P[0:3]) #initial condition constraint

        for k in range(self.N):
            st_next_euler = X[:,k] + self.dt*self.f(X[:,k], U[:,k])
            st_next = X[:,k+1] 
            # Compute constraints
            g = ca.vertcat(g, st_next - st_next_euler)
            # Compute the objective function
            state_error = X[:,k] - P[n_states+n_ref*k:n_states+n_ref*k+n_states]
            # con = U[:,k]
            con = U[:,k] - P[n_states+n_ref*k+n_states:n_states+n_ref*k+n_states+n_controls]
            obj += state_error.T@self.Q@state_error# State cost
            obj += con.T@self.R@con # Control cost


        # Setting Optimization variables
        Opt_Vars = ca.vertcat(X.reshape((-1,1)),U.reshape((-1,1)))

        
        # Define the NMPC optimization problem
        self.nlp = {'f': obj, 'x': Opt_Vars, 'g': g, 'p': P}
        opts = {
            'ipopt': # interior point optimizer
            {
                'max_iter':100,
                'print_level':0,
                'acceptable_tol':1e-8,
                'acceptable_obj_change_tol':1e-6
            },
            'print_time':0,
        }
        
        self.solver = ca.nlpsol('solver', 'ipopt', self.nlp, opts)


        lb_states = [-ca.inf, -ca.inf, -ca.inf]
        ub_states = [ca.inf, ca.inf, ca.inf]

        lb_controls = [0, -ca.pi/4]
        ub_controls = [20, ca.pi/4]

        g_bounds = [0, 0, 0]


        self.x0 = self.ref_trajectory.x0
        self.xs = self.ref_trajectory.xs

        self.u0 = ca.DM.zeros((n_controls, self.N))
        self.X0 = ca.DM([x for x in self.x0]*(self.N+1))



        self.args = {
            'lbx' : lb_states*(self.N+1)+lb_controls*self.N,
            'ubx' : ub_states*(self.N+1)+ub_controls*self.N,
            'lbg' : g_bounds*(self.N+1),
            'ubg' : g_bounds*(self.N+1),
            'p': ca.DM.zeros((n_states+self.N*n_ref,1)),
            'x0': ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))
        }

        self.u_cl = [] # history of controls
        self.xx = ca.DM.zeros((n_states, 162)) # contains the history of states (simulation time)
        self.xx1 = np.zeros((n_states, self.N+1, 162)) # contains the history of states (simulation time)

        self.xx[:,0] = self.x0

    def compute_control(self, mpciter, ref_point):
        """
        Compute the control action based on the current state and the reference trajectory.

        Parameters:
        - current_state: Dict, current state of the vehicle
        - ref_trajectory: Array, reference trajectory points

        Returns:
        - control: Dict, calculated control commands
        """
        # Convert current_state to the format required by the NMPC problem
        # current_state_vector = ca.DM([current_state['x'], current_state['y'], current_state['theta']])

        # Formulate the problem parameters

        self.args['p'][0:3] = self.x0

        for k in range(self.N):
            t_predict = mpciter*self.dt + k*self.dt
            # x_ref = 0.5*t_predict
            x_ref = ref_point[k,0]
            # y_ref = 3.25
            # y_ref = ca.exp(5-t_predict)+ca.cos(t_predict+4)+4
            y_ref = ref_point[k,1]
            theta_ref = ref_point[k,2]
            u_ref = 10
            omega_ref = 0
            # if x_ref >= 12:
            #     x_ref = 12
            #     y_ref = 3
            #     theta_ref = 0
            #     u_ref = 0
            #     omega_ref = 0
            self.args['p'][5*k+3:5*k+3+3] = [x_ref, y_ref, theta_ref]
            self.args['p'][5*k+3+3:5*k+3+3+2] = [u_ref, omega_ref]
        print(ref_point)
        

        self.args['x0'] = ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))


        # Solve the NMPC optimization problem
        sol = self.solver(**self.args)
        # print(sol['x'])
        u_opt = sol['x'][3*(self.N+1):].reshape((2,self.N))
        self.X0 = sol['x'][:3*(self.N+1)].reshape((3,self.N+1))
        self.xx1[:,:,mpciter] = self.X0

        return u_opt
        

    def run_step(self, t0, u):
        f_value = self.f(self.x0, u[:, 0])
        self.x0  = ca.DM.full(self.x0 + (self.dt * f_value))
        self.u0[:,:-1] = u[:,1:]
        # print(self.X0)
        self.X0[:,:-1]=self.X0[:,1:]

        return  t0

class ReferenceTrajectory:
    def __init__(self) -> None:
        self.x0 = None
        self.xs = None
        self.path = None

    def load(self):
        self.path = np.load("./wps.npy")
        self.path[:,2] = np.deg2rad(self.path[:,2])
        self.x0 = self.path[0,:]
        self.xs = self.path[-1,:]

    def get_ref_point(self, step):
        return self.path[step,:]
    
    def get_ref_point(self, step, horizon):
        return self.path[step:step+horizon,:]
    
    def get_reference(self):
        return ca.vertcat(self.x0, self.xs).full()[:,0]



def main():
    sim_time = 40
    dt = 0.5
    N = 10
    # N = 3

    Q = np.diag((1, 5, 0.1))  # states
    R = np.diag((0.5, 0.05))    # controls

    mpciter = 0
    t = []
    t0=0


    ref_trajectory = ReferenceTrajectory()
    ref_trajectory.load()
    nmpc = NMPCController(dt, N, ref_trajectory, Q, R)
    x0=ref_trajectory.get_ref_point(0,0)
    xs=ref_trajectory.get_ref_point(162,0)
    reference = ref_trajectory.get_reference()

    try:
        while (mpciter+N < 162):
            # current_state = get_vehicle_state(vehicle)
            ref_point = ref_trajectory.get_ref_point(mpciter, N)
            control = nmpc.compute_control(mpciter, ref_point)
            t.append(t0)
            t0 = nmpc.run_step(t0, control)
            mpciter += 1
            
        simulate(ref_trajectory.path,nmpc.xx1, nmpc.u_cl, t, nmpc.dt, nmpc.N,reference, True)
        pass
    finally:
        print("Simulation ended")


if __name__ == "__main__":
    main()