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


        #constraints
        g = []
        g = ca.vertcat(g,X[:,0]-P[0:self.model.n_states]) #initial condition constraint


        pass

    def compute_controls(self):

        pass

    def run_step(self):
        pass