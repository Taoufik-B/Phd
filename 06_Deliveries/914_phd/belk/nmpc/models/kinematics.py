import casadi as ca

## Model definition

### desired point is at the center of the rear axle,
def kvmodel_racenter():
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

### desired point is at the center of the front axle:
def kvmodel_facenter():
    
### desired point is at the center of the front axle:
def kvmodel_facenter():
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