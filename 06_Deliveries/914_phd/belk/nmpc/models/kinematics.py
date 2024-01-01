import casadi as ca

## Model definition
class VehicleKinematicModel:
    def __init__(self, L) -> None:
        self.L = L
        #states
        self.x = ca.SX.sym('x')
        self.y = ca.SX.sym('y')
        self.psi = ca.SX.sym('psi')      # the yaw angle / heading angle
        self.delta = ca.SX.sym('delta')  # the steering angle
        
        self.beta = ca.SX.sym('beta')    # the slip angle angle / case of desired control point is in the cog
        #controls
        self.v = ca.SX.sym('v')
        self.phi = ca.SX.sym('phi') # the steering rate

        # State and Control vector
        self.states = None
        self.controls = ca.vertcat(self.v, self.delta)
        pass
    ### desired point is at the center of the rear axle,
    def kvmodel_rac(self):
        # State vector and control inputs
        states = ca.vertcat(self.x, self.y, self.psi, self.delta)

        # State update equations (kinematic bicycle model)
        rhs = ca.vertcat( self.v * ca.cos(self.psi)
                        , self.v*  ca.sin(self.psi)
                        , self.v/self.L * ca.tan(self.delta)
                        , self.phi
                        )
        
        return ca.Function('f', [self.states, self.controls], [rhs])

    ### desired point is at the center of the front axle:
    def kvmodel_fac(self):
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

        # State update equations (kinematic bicycle model)
        rhs = ca.vertcat(v * ca.cos(psi+beta)
                            , v*  ca.sin(psi+beta)
                            , v/L * ca.sin(delta)
                            , delta
                            )
        
        return ca.Function('f', [states, controls], [rhs])

    ### desired point is at the center of gravity:
    def kvmodel_cg():
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

    # State update equations (kinematic bicycle model)
    rhs = ca.vertcat(v * ca.cos(psi+beta)
                        , v*  ca.sin(psi+beta)
                        , v/self.L * ca.sin(delta)
                        , delta
                        )
    
    return ca.Function('f', [states, controls], [rhs])