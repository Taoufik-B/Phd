import casadi as ca
import logging

## Model definition
class VehicleKinematicModel:
    def __init__(self, L, Lr, model_type="rac") -> None:
        self.L = L
        self.Lr = Lr
        self.model_type = model_type
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
        self.controls = ca.vertcat(self.v, self.phi)

        self.n_states = self.states.numel()
        self.n_controls = self.controls.numel()
        self.n_opt_vars = self.n_states + self.n_controls
        
        self.model = {
            'rac': self._kvmodel_rac(),
            'fac': self._kvmodel_fac(),
            'cog': self._kvmodel_cog()
        }
        self.f_function = self._getmodel()
    ### desired point is at the center of the rear axle,
    def _kvmodel_rac(self):
        # State update equations (kinematic bicycle model)
        rhs = ca.vertcat( self.v * ca.cos(self.psi)
                        , self.v * ca.sin(self.psi)
                        , self.v * ca.tan(self.delta) / self.L
                        , self.phi
                        )
        
        return ca.Function('f', [self.states, self.controls], [rhs])

    ### desired point is at the center of the front axle:
    def _kvmodel_fac(self):
        # State update equations (kinematic bicycle model)
        rhs = ca.vertcat( self.v * ca.cos(self.psi+self.delta)
                        , self.v * ca.sin(self.psi+self.delta)
                        , self.v * ca.sin(self.delta) / self.L
                        , self.phi
                        )
        
        return ca.Function('f', [self.states, self.controls], [rhs])

    ### desired point is at the center of gravity:
    def _kvmodel_cog(self):
        # State update equations (kinematic bicycle model)
        beta = ca.arctan(self.Lr/self.L*ca.tan(self.delta))
        rhs = ca.vertcat( self.v * ca.cos(self.psi+beta)
                        , self.v * ca.sin(self.psi+beta)
                        , self.v * ca.tan(self.delta) * ca.cos(beta)
                        , self.phi
                        )
        
        return ca.Function('f', [self.states, self.controls], [rhs])
    
    ### Get the kinematic model
    def _getmodel(self):
        logging.info(f"Loading the vehicle kinematic model of type: {self.model_type}")
        return self.model[self.model_type]