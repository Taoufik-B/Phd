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
        v = ca.SX.sym('v')
        delta = ca.SX.sym('delta')

        # State vector and control inputs
        states = ca.vertcat(x, y, theta)
        controls = ca.vertcat(v, delta)

        # State update equations (kinematic bicycle model)
        rhs = ca.vertcat(v * ca.cos(theta)
                         , v * ca.sin(theta)
                         , v/self.L * ca.tan(delta)
                         )
        self.f = ca.Function('f', [states, controls], [rhs])

        # Objective function and constraints
        # Cost weights
        Q = np.diag([1.0, 1.0, 0.5])  # State cost
        R = np.diag([0.1, 0.1])       # Control cost

        # Prediction horizon
        U = ca.SX.sym('U', 2, self.N)  # Control matrix (2 controls: v, delta)
        P = ca.SX.sym('P', 3 + 3)  # Parameters (current state + reference trajectory)

        # Matrix of the states over the optimization problem
        X = ca.SX.sym('X', 3, self.N+1)
        # Predicted states
        X[:,0] = P[0:3]

        for k in range(self.N):
            st = X[:,k]
            con = U[:,k]
            f_value = self.f(st, con)
            st_next = st + (self.dt*f_value)
            X[:,k+1]=st_next

        # X = ca.vertcat(*[self.f(states, U[i, :]) for i in range(self.N)])  

        # Define the objective function
        obj = 0
        # for i in range(self.N):
        #     state_error = X[i*3:(i+1)*3] - P[3 + i*3:6 + i*3]
        #     obj += ca.mtimes([state_error.T, Q, state_error])  # State cost
        #     obj += ca.mtimes([U[i, :].T, R, U[i, :]])          # Control cost

        for k in range(self.N):
            st = X[:,k]
            con = U[:,k]
            state_error = st-P[3:6]
            obj += state_error.T@Q@state_error# State cost
            obj += con.T@R@con # Control cost

        # Define constraints (if any)
        g = []

        # Define the NMPC optimization problem
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.nlp = {'f': obj, 'x': ca.reshape(U, -1, 1), 'g': ca.vertcat(*g), 'p': P}
        self.solver = ca.nlpsol('solver', 'ipopt', self.nlp, opts)


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
        params = ca.vertcat(
            current_state_vector,
            ca.reshape(ref_trajectory, -1, 1)
        )

        # Solve the NMPC optimization problem
        sol = self.solver(lbg=0, ubg=0, p=params)
        u_opt = ca.reshape(sol['x'].full(), self.N, 2)

        print(u_opt)

        # Extract the first set of control actions
        throttle_value = float(max(u_opt[0, 0], 0))
        steer_value = float(u_opt[0, 1])
        brake_value = float(max(-u_opt[0, 0], 0))

        print(throttle_value, steer_value, brake_value)

        # Return the control commands (as a dictionary)
        return {'throttle': 0.8, 'steer': steer_value, 'brake': brake_value}