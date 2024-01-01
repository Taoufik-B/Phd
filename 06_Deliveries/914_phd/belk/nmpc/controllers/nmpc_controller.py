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
        self.n_states

        #matrix of the states over the optimization problem
        X = ca.SX.sym('X', self.n_states, self.N+1)
        
        #controls
        n_controls = 2
        U = ca.SX.sym('U', self.n_controls, self.N)
        #parameters
        P = ca.SX.sym('P', self.n_states+self.n_ref*self.N) 

        #objective


        #constraints


        pass

    def compute_controls(self):

        pass

    def run_step(self):
        pass