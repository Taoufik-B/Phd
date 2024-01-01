import casadi as ca

class NMPCController:
    def __init__(self) -> None:
        
        self._setup()
        pass

    def _setup(self):
        """
        Setup the NMPC optimization problem
        """
        self.opti = ca.Opti()
        #states
        n_states = 4
        x = self.opti.variable(n_states, self.N+1)
        #controls
        n_controls = 2
        u = self.opti.variable(n_controls, self.N)
        #parameters
        p = self.opti.parameter(n_states+(n_states+n_controls)*self.N)

        #objective
        F = ca.Function('F', [x, u], )

        #constraints
        for k in self.N:
            self.opti.subject_to(x[:,k+1]==F(x[:,k],u[:,k]))

        pass

    def compute_controls(self):

        pass

    def run_step(self):
        pass