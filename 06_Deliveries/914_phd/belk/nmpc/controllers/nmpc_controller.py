import casadi as ca

class NMPCController:
    def __init__(self, model, dT, N, Q, R) -> None:
        self.model = model
        self.dT = dT
        self.N = N
        self.Q = Q
        self.R = R
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


        pass

    def compute_controls(self):

        pass

    def run_step(self):
        pass