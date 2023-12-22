# nmpc_controller.py
import casadi as ca
import numpy as np

class NMPCController:
    def __init__(self, L=2.8, dt=0.05, N=5):
        """
        Initialize the NMPC controller.

        Parameters:
        - L: float, wheelbase of the vehicle
        - dt: float, time step
        - N: int, prediction horizon
        """
        self.L = L
        self.dt = dt
        self.N = N

        # Define the NMPC problem
        self._setup_nmpc()

    def _setup_nmpc(self):
        """
        Sets up the NMPC optimization problem.
        """
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        # v = ca.SX.sym('v')
        v = 10
        delta = ca.SX.sym('delta')

        # State vector and control inputs
        states = ca.vertcat(x, y, theta)
        # controls = ca.vertcat(v, delta)
        controls = ca.vertcat(delta)

        n_states = states.numel()
        n_controls = controls.numel()

        n_ref = n_states + n_controls

        # State update equations (kinematic bicycle model)
        rhs = ca.vertcat(v * ca.cos(np.deg2rad(theta))
                         , v * ca.sin(np.deg2rad(theta))
                         , v/self.L * ca.tan(delta*ca.pi)
                         )
        self.f = ca.Function('f', [states, controls], [rhs])


        # Prediction horizon
        U = ca.SX.sym('U', n_controls, self.N)  # Control matrix (2 controls: v, delta)
        # U = ca.SX.sym('U', 1, self.N)  # Control matrix (2 controls: v, delta)
        P = ca.SX.sym('P', n_states+n_states)  # Parameters (current state + reference trajectory)

        # Matrix of the states over the optimization problem
        X = ca.SX.sym('X', n_states, self.N+1)

        # Objective function and constraints
        # Cost weights
        self.Q = np.diag([1.0, 1.0, 0.5])  # State cost
        # R = np.diag([0.1, 0.1])       # Control cost
        self.R = [0.1]      # Control cost

        # Define the objective function
        obj = 0
 
        # Define constraints (if any)
        g = []

        g = ca.vertcat(g,X[:,0]-P[0:3]) #initial condition constraint

        for k in range(self.N):
            st_next_euler = X[:,k] + self.dt*self.f(X[:,k], U[:,k])
            st_next = X[:,k+1] 
            # Compute constraints
            g = ca.vertcat(g, st_next - st_next_euler)
            # Compute the objective function
            state_error = X[:,k] - P[3:6]
            # state_error = X[:,k] - P[n_ref*k+3:n_ref*k+3+3]
            con = U[:,k]
            # con = U[:,k] - P[n_ref*k+3+3:n_ref*k+3+3+2]
            obj += state_error.T@self.Q@state_error# State cost
            obj += con.T@self.R@con # Control cost

        # Setting Optimization variables
        Opt_Vars = ca.vertcat(X.reshape((-1,1)),U.reshape((-1,1)))

        # Define the NMPC optimization problem
        # self.nlp = {'f': obj, 'x': ca.reshape(U, -1, 1), 'g': ca.vertcat(*g), 'p': P}
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
        
        self.x0 =ca.DM([55,130,180])
        self.u0 = ca.DM.zeros((n_controls, self.N))
        self.X0 = ca.DM([self.x0.full()[:,0]]*(self.N+1))
        
        control_min = [-1]
        state_min = [-ca.inf,-ca.inf,-ca.pi]
        control_max = [1]
        state_max = [ca.inf,ca.inf,ca.pi]

        constraints_b = [0,0,0]
        self.args = {
            'lbx' : state_min*(self.N+1)+control_min*self.N,
            'ubx' : state_max*(self.N+1)+control_max*self.N,
            'lbg' : constraints_b*(self.N+1),
            'ubg' : constraints_b*(self.N+1),
            'p': ca.DM.zeros((n_states+n_states,1)),
            'x0': ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))
        }



    def compute_control(self, current_state, ref_trajectory):
        """
        Compute the control action based on the current state and the reference trajectory.

        Parameters:
        - current_state: Dict, current state of the vehicle
        - ref_trajectory: Array, reference trajectory points

        Returns:
        - control: Dict, calculated control commands
        """
        # Convert current_state to the format required by the NMPC problem
        current_state_vector = ca.DM([current_state['x'], current_state['y'], current_state['theta']])

        # Formulate the problem parameters
        self.args['p'] = ca.vertcat(
            current_state_vector,
            ca.reshape(ref_trajectory, -1, 1)
        )

        # Solve the NMPC optimization problem
        sol = self.solver(**self.args)
        # print(sol)
        u_opt = sol['x'][3*(self.N+1):].reshape((1,self.N))
        self.X0 = sol['x'][:3*(self.N+1)].reshape((3,self.N+1))
        # u_opt = ca.reshape(sol['x'].full(), self.N, 1)

        print(u_opt)
        print(self.X0)

        # Extract the first set of control actions
        # throttle_value = float(max(u_opt[0, 0], 0))
        steer_value = float(u_opt[0, 0])

        # print(steer_value)

        # Return the control commands (as a dictionary)
        return {'throttle': 0.8, 'steer': steer_value, 'brake': 0}