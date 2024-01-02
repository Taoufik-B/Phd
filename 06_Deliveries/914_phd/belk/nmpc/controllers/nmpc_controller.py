import casadi as ca
from utils.discretize import discretize_rk4

class NMPCController:
    def __init__(self, model, dT, N, Q, R, only_euler) -> None:
        self.model = model
        self.dT = dT
        self.N = N
        self.Q = Q
        self.R = R
        self.only_euler = only_euler
        self.bounds
        self._setup()
        pass

    def _setup(self):
        """
        Setup the NMPC optimization problem
        """

        #matrix of the states over the optimization problem
        X = ca.SX.sym('X', self.model.n_states, self.N+1)
        
        #controls
        U = ca.SX.sym('U', self.model.n_controls, self.N)
        #parameters
        P = ca.SX.sym('P', self.model.n_states+self.model.n_opt_vars*self.N) 

        #objective
        obj = 0

        #constraints
        g = []
        g = ca.vertcat(g,X[:,0]-P[0:self.model.n_states]) #initial condition constraint

        # compute g and obj
        for k in range(self.N):
            st_next_aprx = discretize_rk4(self.model.f_function, X[:,k], U[:,k], self.only_euler)
            st_next = X[:,k+1] 
            # Compute constraints
            g = ca.vertcat(g, st_next - st_next_aprx)
            state_error = X[:,k] - P[self.model.n_states+self.model.n_opt_vars*k
                                     :self.model.n_states+self.model.n_opt_vars*k+self.model.n_states]
            con = U[:,k] - \
                P[self.model.n_states+self.model.n_opt_vars*k+self.model.n_states
                  :self.model.n_states+self.model.n_opt_vars*k+self.model.n_states+self.model.n_controls]
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
        pass

    def compute_controls(self):

        pass

    def run_step(self):
        pass