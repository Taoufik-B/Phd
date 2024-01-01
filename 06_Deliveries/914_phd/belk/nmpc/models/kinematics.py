import casadi as ca

## Model definition
class VehicleKinematicModel:
    def __init__(self, L, Lr) -> None:
        self.L = L
        self.Lr = Lr
        #states
        self.x = ca.SX.sym('x')
        self.y = ca.SX.sym('y')
        self.psi = ca.SX.sym('psi')      # the yaw angle / heading angle
        self.delta = ca.SX.sym('delta')  # the steering angle

        #controls
        self.v = ca.SX.sym('v')
        self.phi = ca.SX.sym('phi') # the steering rate

        # State and Control vector
        self.states = ca.vertcat(self.x, self.y, self.psi, self.delta)
        self.controls = ca.vertcat(self.v, self.delta)
        pass
    ### desired point is at the center of the rear axle,
    def kvmodel_rac(self):
        # State update equations (kinematic bicycle model)
        rhs = ca.vertcat( self.v * ca.cos(self.psi)
                        , self.v*  ca.sin(self.psi)
                        , self.v/self.L * ca.tan(self.delta)
                        , self.phi
                        )
        
        return ca.Function('f', [self.states, self.controls], [rhs])

    ### desired point is at the center of the front axle:
    def kvmodel_fac(self):
        # State update equations (kinematic bicycle model)
        rhs = ca.vertcat( self.v * ca.cos(self.psi+self.delta)
                        , self.v*  ca.sin(self.psi+self.delta)
                        , self.v/self.L * ca.sin(self.delta)
                        , self.phi
                        )
        
        return ca.Function('f', [self.states, self.controls], [rhs])

    ### desired point is at the center of gravity:
    def kvmodel_cg(self):
        # State update equations (kinematic bicycle model)
        beta = ca.arctan2
        rhs = ca.vertcat( self.v * ca.cos(self.psi+self.beta)
                        , self.v*  ca.sin(self.psi+self.beta)
                        , self.v/self.L * ca.sin(self.delta)
                        , self.phi
                        )
        
        return ca.Function('f', [self.states, self.controls], [rhs])