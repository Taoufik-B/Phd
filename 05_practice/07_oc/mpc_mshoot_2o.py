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
            omega
        )
        self.f = ca.Function('f', [states, controls], [rhs])
        
        # Decision variables
        P = ca.SX.sym('P', n_states*2) # parameters (Initial and reference trajectory)
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
            state_error = X[:,k] - P[3:6]
            con = U[:,k]
            obj += state_error.T@self.Q@state_error# State cost
            obj += con.T@self.R@con # Control cost

        # Obstacle avoidance constraints 
        for k in range(self.N+1):
            # g=ca.vertcat(g, -ca.norm_2(X[0:1,k]-[1.5,3])+(1.75))        
            # g=ca.vertcat(g, -np.linalg.norm(X[0:1,k].full()-[1.5,3])+(1.75)) 
            # o1 radius 0.25 [1.5,3]       
            g=ca.vertcat(g, -ca.sqrt((X[0,k]-1.5)**2 +(X[1,k]-3)**2)+(0.30))
            # o2 radius 1.5 [4,6]
            # print(X[:,k])        
        # Get optimal trajectory
        # self.ff = ca.Function('ff', [U, P], [X])
        print(g[3*(self.N+1):])

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


        v_max = 0.6
        v_min = -v_max
        omega_max = ca.pi/4
        omega_min = -omega_max

        x_ub = ca.inf
        y_ub = ca.inf
        theta_ub = ca.inf

        self.x0 = self.ref_trajectory.x0
        self.xs = self.ref_trajectory.xs

        self.u0 = ca.DM.zeros((n_controls, self.N))
        self.X0 = ca.DM([self.x0.full()[:,0]]*(self.N+1))

        self.args = {
            'lbx' : [-x_ub, -y_ub, -theta_ub]*(self.N+1)+[v_min, omega_min]*self.N,
            'ubx' : [x_ub, y_ub, theta_ub]*(self.N+1)+[v_max, omega_max]*self.N,
            'lbg' : [0,0,0]*(self.N+1)+[-ca.inf]*(self.N+1),
            'ubg' : [0,0,0]*(self.N+1)+[0.0]*(self.N+1),
            'p': ca.vertcat(self.x0,self.xs),
            'x0': ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))
        }

        self.u_cl = [] # history of controls
        self.xx = ca.DM.zeros((n_states, 200)) # contains the history of states (simulation time)
        self.xx1 = np.zeros((n_states, self.N+1, 200)) # contains the history of states (simulation time)

        self.xx[:,0] = self.x0

    def compute_control(self, mpciter):
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

        self.args['p'] = ca.vertcat(self.x0,self.xs)
        self.args['x0'] = ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))

        # print(self.args['p'].size())
        # print(self.args['x0'].numel(), self.X0.numel(), self.X0)
        # print(len(self.args['lbx']))
        # print(len(self.args['ubx']))
        # print(len(self.args['lbg']))
        # print(len(self.args['ubg']))


        # Solve the NMPC optimization problem
        sol = self.solver(**self.args)
        # print(sol['x'])
        u_opt = sol['x'][3*(self.N+1):].reshape((2,self.N))
        self.X0 = sol['x'][:3*(self.N+1)].reshape((3,self.N+1))
        self.xx1[:,:,mpciter] = self.X0
        # print(self.X0)
        # Compute optimal solution tajectory
        # ff_value = self.ff(u_opt, self.args['p'])
        # self.xx1[:,:,mpciter] = ff_value

        return u_opt
        

    def run_step(self, t0, u):
        f_value = self.f(self.x0, u[:, 0])
        self.x0  = ca.DM.full(self.x0 + (self.dt * f_value))
        self.u0[:,:-1] = u[:,1:]
        # print(self.X0)
        self.X0[:,:-1]=self.X0[:,1:]

        return  t0

class ReferenceTrajectory:
    def __init__(self, x0, xs) -> None:
        self.x0 = ca.DM(x0)
        self.xs = ca.DM(xs)

    def get_ref_point(self, step):
        return step
    
    def get_reference(self):
        return ca.vertcat(self.x0, self.xs).full()[:,0]



def main():
    # Config
    x0 = [0,0,0]
    xs = [9,8,0]
    sim_time = 40
    dt = 0.2
    N = 25

    Q = np.diag((1, 5, 0.1))  # states
    R = np.diag((0.5, 0.05))    # controls

    mpciter = 0
    t = []
    t0=0

    ref_trajectory = ReferenceTrajectory(x0, xs)
    reference = ref_trajectory.get_reference()
    nmpc = NMPCController(dt, N, ref_trajectory, Q, R)

    try:
        while (ca.norm_2(nmpc.x0 - nmpc.xs) > 1e-2) and (mpciter < (sim_time / nmpc.dt)):
            # current_state = get_vehicle_state(vehicle)
            # ref_point = ref_trajectory.get_ref_point(step)
            control = nmpc.compute_control(mpciter)
            t.append(t0)
            t0 = nmpc.run_step(t0, control)
            mpciter += 1
        print(nmpc.X0)
        print(nmpc.x0)
        # print(nmpc.xx1[:,:,mpciter-1])
            
        simulate(nmpc.xx1, nmpc.u_cl, t, nmpc.dt, nmpc.N,reference, True)
    finally:
        print("Simulation ended")


if __name__ == "__main__":
    main()