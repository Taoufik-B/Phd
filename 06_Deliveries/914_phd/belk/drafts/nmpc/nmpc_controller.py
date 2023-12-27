# nmpc_controller.py
import casadi as ca
import numpy as np
from collections import deque
from scipy.integrate import trapz


class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        # self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = deque(maxlen=10)

    def run_step(self, target_speed, current_speed, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

            :param target_speed: target speed in Km/h
            :param debug: boolean for debugging
            :return: throttle control
        """
        # current_speed = get_speed(self._vehicle)

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

            :param target_speed:  target speed in Km/h
            :param current_speed: current speed of the vehicle in Km/h
            :return: throttle/brake control
        """

        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class NMPCController:
    def __init__(self, ref_trajectory, L=2.8, dt=0.05, N=10):
        """
        Initialize the NMPC controller.

        Parameters:
        - L: float, wheelbase of the vehicle
        - dt: float, time step
        - N: int, prediction horizon
        """
        self.L = L
        self.dt = dt
        self.N = N

        self.current_speed = 0
        self._args_longitudinal_dict = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': self.dt}
        self._lon_controller = PIDLongitudinalController(**self._args_longitudinal_dict)

        self._target_reached = False
        self.ref_trajectory = ref_trajectory

        # Define the NMPC problem
        self._setup_nmpc()

    def _setup_nmpc(self):
        """
        Sets up the NMPC optimization problem.
        """
        #states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        psi = ca.SX.sym('psi')
        beta = ca.SX.sym('beta') # the steering angle

        # s
        
        #controls
        v = ca.SX.sym('v')
        delta = ca.SX.sym('delta') # the steering rate
        

        # State vector and control inputs
        states = ca.vertcat(x, y, psi, beta)
        controls = ca.vertcat(v, delta)

        self.n_states = states.numel()
        self.n_controls = controls.numel()

        self.n_ref = self.n_states + self.n_controls

        # State update equations (kinematic bicycle model)
        # rhs = ca.vertcat(v * ca.cos(theta+delta)
        #                  , v*  ca.sin(theta+delta)
        #                  , v/self.L * ca.sin(delta)
        #                  , phi
        #                  )
        rhs = ca.vertcat(v * ca.cos(psi+beta)-self.L/2
                         , v*  ca.sin(psi+beta)
                         , v/self.L * ca.cos(beta)*ca.tan(delta)
                         , delta
                         )
        self.f = ca.Function('f', [states, controls], [rhs])


        # Prediction horizon
        U = ca.SX.sym('U', self.n_controls, self.N)  # Control matrix (2 controls: v, delta)
        # U = ca.SX.sym('U', 1, self.N)  # Control matrix (2 controls: v, delta)
        P = ca.SX.sym('P', self.n_states+self.n_ref*self.N)  # Parameters (current state + reference trajectory)

        # Matrix of the states over the optimization problem
        X = ca.SX.sym('X', self.n_states, self.N+1)

        # Objective function and constraints
        # Cost weights
        self.Q = np.diag([20, 20, 0.1, 0.05])  # State cost
        self.R = np.diag([0.5, 0.05])       # Control cost
        # self.R = [0.1]      # Control cost

        # Define the objective function
        obj = 0
 
        # Define constraints (if any)
        g = []

        g = ca.vertcat(g,X[:,0]-P[0:self.n_states]) #initial condition constraint

        for k in range(self.N):
            # st_runge_kutta
            k1 = self.f(X[:,k], U[:,k])
            k2 = self.f(X[:,k]+self.dt/2*k1, U[:,k])
            k3 = self.f(X[:,k]+self.dt/2*k2, U[:,k])
            k4 = self.f(X[:,k]+self.dt/2*k3, U[:,k])
            st_rk4 = X[:,k]+self.dt/6*(k1+2*k2+2*k3+k4)
            # st_next_euler = X[:,k] + k1*self.dt
            st_next = X[:,k+1] 
            # Compute constraints
            # g = ca.vertcat(g, st_next - st_next_euler)
            # rk4
            g = ca.vertcat(g, st_next - st_rk4)
            # Compute the objective function
            # state_error = X[:,k] - P[3:6]
            state_error = X[:,k] - P[self.n_states+self.n_ref*k:self.n_states+self.n_ref*k+self.n_states]
            # con = U[:,k]
            con = U[:,k] - P[self.n_states+self.n_ref*k+self.n_states:self.n_states+self.n_ref*k+self.n_states+self.n_controls]
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
        
        lb_states = [-ca.inf, -ca.inf, -ca.pi, -1.2217]
        ub_states = [ca.inf, ca.inf, ca.pi, 1.2217]

        lb_controls = [0, -1.2217]
        ub_controls = [5, 1.2217]

        lg_bounds = [0, 0, 0, 0]
        ug_bounds = [0, 0, 0, 0]


        self.x0 = self.ref_trajectory.x0
        self.xs = self.ref_trajectory.xs

        self.u0 = ca.DM.zeros((self.n_controls, self.N))
        self.X0 = ca.DM([x for x in self.x0]*(self.N+1))



        self.args = {
            'lbx' : lb_states*(self.N+1)+lb_controls*self.N,
            'ubx' : ub_states*(self.N+1)+ub_controls*self.N,
            'lbg' : lg_bounds*(self.N+1),
            'ubg' : ug_bounds*(self.N+1),
            'p': ca.DM.zeros((self.n_states+self.N*self.n_ref,1)),
            'x0': ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))
        }



    def compute_control(self, current_state, target_state, iteration):
        """
        Compute the control action based on the current state and the reference trajectory.

        Parameters:
        - current_state: Dict, current state of the vehicle
        - ref_trajectory: Array, reference trajectory points

        Returns:
        - control: Dict, calculated control commands
        """
        # Convert current_state to the format required by the NMPC problem
        self.args['p'][0:self.n_states] = current_state

        print(f'###### ITERATION {iteration} ###########')

        for k in range(len(target_state)):
            x_ref = target_state[k,0]
            # x_ref = current_state[0]-t_predict*self.current_speed/3.6
            y_ref = target_state[k,1]
            # y_ref = current_state[1]
            theta_ref = target_state[k,2]
            phi_ref = target_state[k,3]
            v_ref = 5
            delta_ref = current_state[3]
            self.args['p'][self.n_ref*k+self.n_states:self.n_ref*k+ 2*self.n_states] = [x_ref, y_ref, theta_ref, phi_ref]
            self.args['p'][self.n_ref*k+2*self.n_states:self.n_ref*k+2*self.n_states+self.n_controls] = [v_ref, delta_ref]
            print('x : ', x_ref, current_state[0])
            print('y : ', y_ref, current_state[1])
            print('theta : ', theta_ref, current_state[2])
            print('Phi : ', phi_ref, current_state[3])
    
        self.args['x0'] = ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))

        # Solve the NMPC optimization problem
        sol = self.solver(**self.args)
        # print(sol)
        u_opt = sol['x'][self.n_states*(self.N+1):].reshape((self.n_controls,self.N))
        self.X0 = sol['x'][:self.n_states*(self.N+1)].reshape((self.n_states,self.N+1))
        # u_opt = ca.reshape(sol['x'].full(), self.N, 1)

        # print("Target State", target_state)
        print("Controls: ", u_opt)
        print("Calculated state : ",self.X0)
        v=u_opt[0,0]
        delta = float(u_opt[1, 0])
        print("Current Speed: ", self.current_speed)
        print("calculated v : ", v, v*3.6)

        # Extract the first set of control actions
        # throttle_value = float(max(u_opt[0, 0], 0))
        acceleration = self._lon_controller.run_step(v*3.6, self.current_speed)[0,0]
        # print(acceleration[0,0])

        if acceleration >= 0.0:
            throttle_value = min(acceleration, 1)
            brake_value = 0.0
        else:
            throttle_value = 0.0
            brake_value = min(abs(acceleration), 1)

        # beta= np.clip(steer_value, -1.0,1.0)
        etha = ca.tan(delta)*v/self.L*self.dt
        print("calculated steer_value :", delta, etha)
        # if current_state[]
        speed_value = v.full()[0][0]
        # steer_value =  etha.full()[0][0]
        # steer_value =  (delta/1.2217)/ca.pi

        # steer_value =  np.clip(delta,-1.0,1.0)
        # steer_value = np.clip(float(self.X0[3,0]), -1,1)
        # steer_value = np.clip(float(self.X0[3,1]), -1,1)
        steer_value = np.clip(float(self.X0[3,0])/1.2217, -1,1)
        # steer_value = np.clip(float(trapz(self.X0[3,:], self.dt), -1,1))

        if self._target_reached:
            speed_value = 0
            steer_value = 0



        control = {  'throttle': throttle_value
                   , 'steer': steer_value
                   , 'brake': brake_value
                   , 'speed':speed_value
                   , 'steer_rate':delta
                }
        print(control)
        # Return the control commands (as a dictionary)
        return control, u_opt
    
    def run_step(self, u):
        # f_value = self.f(self.x0, u[:, 0])
        # self.x0  = ca.DM.full(self.x0 + (self.dt * f_value))
        self.u0[:,0:-1] = u[:,1:]
        # print(self.X0)
        self.X0[:,0:-1]=self.X0[:,1:]