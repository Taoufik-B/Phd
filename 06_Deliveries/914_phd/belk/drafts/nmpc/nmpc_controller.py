# nmpc_controller.py
import casadi as ca
import numpy as np
import logging



class NMPCController:
    def __init__(self, trajectory, Q, R, L=2.8, dt=0.1, N=10, activate_rk4=False):
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

        self.Q = Q
        self.R = R

        # Cost weights
        self.Q = np.diag(Q)  # State cost
        self.R = np.diag(R)  # Control cost

        self.current_speed = 0

        # initial_state
        self.trajectory = trajectory
        self.x0 = trajectory.x0
        self.path_size = trajectory.size

        self.activate_rk4 = activate_rk4

        # Define the NMPC problem
        self._setup_nmpc()

    def _setup_nmpc(self):
        """
        Sets up the NMPC optimization problem.
        """
        #states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        psi = ca.SX.sym('psi')      # the yaw angle
        beta = ca.SX.sym('beta')    # the slip angle angle
        
        #controls
        v = ca.SX.sym('v')
        delta = ca.SX.sym('delta') # the steering rate
        

        # State vector and control inputs
        states = ca.vertcat(x, y, psi, beta)
        controls = ca.vertcat(v, delta)

        self.n_states = states.numel()
        self.n_controls = controls.numel()

        self.n_ref = self.n_states + self.n_controls

        # State update equations (kinematic bicycle model)
        rhs = ca.vertcat(v * ca.cos(psi+beta)
                         , v*  ca.sin(psi+beta)
                         , v/self.L * ca.sin(delta)
                         , delta
                         )

        self.f = ca.Function('f', [states, controls], [rhs])


        # Prediction horizon
        U = ca.SX.sym('U', self.n_controls, self.N)  # Control matrix (2 controls: v, delta)
        # U = ca.SX.sym('U', 1, self.N)  # Control matrix (2 controls: v, delta)
        P = ca.SX.sym('P', self.n_states+self.n_ref*self.N)  # Parameters (current state + reference trajectory)

        # Matrix of the states over the optimization problem
        X = ca.SX.sym('X', self.n_states, self.N+1)

        # Objective function and constraints

        # Define the objective function
        obj = 0
 
        # Define constraints (if any)
        g = []
        g = ca.vertcat(g,X[:,0]-P[0:self.n_states]) #initial condition constraint

        for k in range(self.N):
            # st_runge_kutta
            k1 = self.f(X[:,k], U[:,k])
            if self.activate_rk4:
                k2 = self.f(X[:,k]+self.dt/2*k1, U[:,k])
                k3 = self.f(X[:,k]+self.dt/2*k2, U[:,k])
                k4 = self.f(X[:,k]+self.dt/2*k3, U[:,k])
                st_rk4 = X[:,k]+self.dt/6*(k1+2*k2+2*k3+k4)
                st_next_aprx = st_rk4
            else:
                st_next_euler = X[:,k] + k1*self.dt
                st_next_aprx = st_next_euler
            st_next = X[:,k+1] 
            # Compute constraints
            g = ca.vertcat(g, st_next - st_next_aprx)
            state_error = X[:,k] - P[self.n_states+self.n_ref*k:self.n_states+self.n_ref*k+self.n_states]
            con = U[:,k] - P[self.n_states+self.n_ref*k+self.n_states:self.n_states+self.n_ref*k+self.n_states+self.n_controls]
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
        
        self.lb_states = [-ca.inf, -ca.inf, -ca.pi, -ca.pi]
        self.ub_states = [ca.inf, ca.inf, ca.pi, ca.pi]

        self.lb_controls = [0, -ca.pi]
        self.ub_controls = [22, ca.pi]

        self.lg_bounds = [0, 0, 0, 0]
        self.ug_bounds = [0, 0, 0, 0]


        self.u0 = ca.DM.zeros((self.n_controls, self.N))
        # TODO: need to be optimized
        self.X0 = ca.DM([x for x in self.x0]*(self.N+1))

        self.args = {
            'lbx' : self.lb_states*(self.N+1)+self.lb_controls*self.N,
            'ubx' : self.ub_states*(self.N+1)+self.ub_controls*self.N,
            'lbg' : self.lg_bounds*(self.N+1),
            'ubg' : self.ug_bounds*(self.N+1),
            'p': ca.DM.zeros((self.n_states+self.N*self.n_ref,1)),
            'x0': ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))
        }

        ## History controls and states
        self.u_opt_history=np.zeros((self.n_controls,self.N,0)) # controls
        self.x_history=np.zeros((self.n_states,self.N+1,0)) # states
        self.p_history=np.zeros((self.n_ref,self.N,0)) # parameters
        logging.info("MPC Setup Completed")



    def compute_control(self, mpciter, v_ref, delta_ref):
        """
        Compute the control action based on the current state and the reference trajectory.

        Parameters:
        - current_state: Dict, current state of the vehicle
        - ref_trajectory: Array, reference trajectory points

        Returns:
        - u_opt, X0: return optimized controls and Calculated states
        """
        # Convert current_state to the format required by the NMPC problem
        print(f"################## Compute Control at iteration {mpciter} ##################")
        print("tCurrent x0: ", self.x0)
        self.args['p'][0:self.n_states] = self.x0

        logging.info(f"MPC compute control at iteration: {mpciter}")
        print(f"MPC compute control at iteration: {mpciter}")

        print(f"#### x_ref, y_ref, psi_ref, delta_ref ####")
        for k in range(self.N):
            t_predict = (mpciter+k)*self.dt
            ref = self.trajectory.get_next_wp(t_predict)
            x_ref = ref[0]
            y_ref = ref[1]
            psi_ref = ref[2]
            beta_ref = ref[3]
            print(x_ref,y_ref, psi_ref, beta_ref)

            self.args['p'][self.n_ref*k+self.n_states:self.n_ref*k+ 2*self.n_states] = [x_ref, y_ref, psi_ref, beta_ref]
            self.args['p'][self.n_ref*k+2*self.n_states:self.n_ref*k+2*self.n_states+self.n_controls] = [v_ref, delta_ref]
    
        self.args['x0'] = ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))

        # Solve the NMPC optimization problem
        logging.info(f"MPC compute control - solving at iteration: {mpciter}")
        sol = self.solver(**self.args)
        logging.info(f"MPC compute control - solving done at iteration: {mpciter}")

        u_opt = sol['x'][self.n_states*(self.N+1):].reshape((self.n_controls,self.N))
        self.X0 = sol['x'][:self.n_states*(self.N+1)].reshape((self.n_states,self.N+1))
        logging.info(f"MPC compute control - extracting controls and states at iteration: {mpciter}")
        
        print("### Computed Solution X0", self.X0)
        print("### Constraints : ",self.args['p'])
        p = self.args['p'][self.n_states:].reshape((self.n_ref,self.N))
        self.p_history = np.dstack((self.p_history,p))


        # print(u_opt.shape, self.X0.shape, self.u_opt_history.shape)
        self.u_opt_history=np.dstack((self.u_opt_history,u_opt))
        self.x_history=np.dstack((self.x_history,self.X0))

        return u_opt
    
    def run_step(self, u, current_state=None):
        logging.info(f"MPC run step")
        if current_state is None:
            logging.info(f"MPC run step - Shifting")
                        # st_runge_kutta
            k1 = self.f(self.x0, u[:, 0])
            if self.activate_rk4:
                k2 = self.f(self.x0+self.dt/2*k1, u[:, 0])
                k3 = self.f(self.x0+self.dt/2*k2, u[:, 0])
                k4 = self.f(self.x0+self.dt/2*k3, u[:, 0])
                st_rk4 = self.x0+self.dt/6*(k1+2*k2+2*k3+k4)
                st_next_aprx = st_rk4
            else:
                st_next_euler =self.x0 + k1*self.dt
                st_next_aprx = st_next_euler
            f_value = self.f(self.x0, u[:, 0])
            # self.x0  = ca.DM.full(self.x0 + (self.dt * f_value))
            self.x0  = st_next_aprx
            print("### Calulating x0+1 using f value:", self.x0)
            logging.info(f"MPC run step - Shifting Done")
        else:
            self.x0 = current_state
        self.u0[:,:-1] = u[:,1:]
        self.X0[:,:-1]=self.X0[:,1:]