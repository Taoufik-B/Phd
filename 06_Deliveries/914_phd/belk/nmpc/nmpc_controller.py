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
        x = self.opti.variable()
        y = self.opti.variable
        psi = self.opti.variable    # the yaw angle
        beta = self.opti.variable   # the slip angle angle

        pass

    def compute_controls(self):

        pass

    def run_step(self):
        pass