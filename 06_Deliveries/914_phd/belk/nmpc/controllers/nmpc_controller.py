import numpy as np
import casadi as ca
from utils.discretize import discretize_rk4
import logging


class NMPCController:
    def __init__(self, model, trajectory, dT, N, Q, R, only_euler, bounds) -> None:
        self.model = model
        self.dT = dT
        self.N = N
        self.Q = np.diag(Q)
        self.R = np.diag(R)
        self.only_euler = only_euler
        self.bounds = bounds
        self.trajectory = trajectory
        self.x0 = self.trajectory.x0
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
            st_next_aprx = discretize_rk4(self.model.f_function, X[:,k], U[:,k], self.dT, self.only_euler)
            st_next = X[:,k+1] 
            # Compute constraints
            g = ca.vertcat(g, st_next - st_next_aprx)
            state_error = X[:,k] - P[self.model.n_states+self.model.n_opt_vars*k
                                     :self.model.n_states+self.model.n_opt_vars*k+self.model.n_states]
            con = U[:,k] - P[self.model.n_states+self.model.n_opt_vars*k+self.model.n_states
                            :self.model.n_states+self.model.n_opt_vars*k+self.model.n_states+self.model.n_controls]
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
        
        lb_x = self.bounds['x'][0]
        ub_x = self.bounds['x'][1]

        lb_u = self.bounds['u'][0]
        ub_u = self.bounds['u'][1]

        lb_g = self.bounds['g'][0]
        ub_g = self.bounds['g'][1]

        self.u0 = ca.DM.zeros((self.model.n_controls, self.N))
        # TODO: need to be optimized
        self.X0 = ca.DM([x for x in self.x0]*(self.N+1))

        self.args = {
            'lbx' : lb_x*(self.N+1)+lb_u*self.N,
            'ubx' : ub_x*(self.N+1)+ub_u*self.N,
            'lbg' : lb_g*(self.N+1),
            'ubg' : ub_g*(self.N+1),
            'p': ca.DM.zeros((self.model.n_states+self.N*self.model.n_opt_vars,1)),
            'x0': ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))
        }

        ## History controls and states
        self.u_opt_history=np.zeros((self.model.n_controls,self.N,0)) # controls
        self.x_history=np.zeros((self.model.n_states,self.N+1,0)) # states
        self.p_history=np.zeros((self.model.n_opt_vars,self.N,0)) # parameters
        logging.info("MPC Setup Completed")
        pass

    def compute_controls(self, mpciter, u_ref):
        print(f"################## Compute Control at iteration {mpciter} ##################")
        print("tCurrent x0: ", self.x0)
        self.args['p'][0:self.model.n_states] = self.x0

        logging.info(f"MPC compute control at iteration: {mpciter}")
        print(f"MPC compute control at iteration: {mpciter}")

        print(f"#### x_ref, y_ref, psi_ref, delta_ref ####")
        for k in range(self.N):
            t_predict = (mpciter+k)*self.dt
            ref = self.trajectory.get_next_wp(t_predict)
            x_ref = ref[0]
            y_ref = ref[1]
            psi_ref = ref[2]
            beta_ref = ref[3]
            print(x_ref,y_ref, psi_ref, beta_ref)

            self.args['p'][self.model.n_opt_vars*k+self.model.n_states
                           :self.model.n_opt_vars*k+ 2*self.model.n_states] = [x_ref, y_ref, psi_ref, beta_ref]
            self.args['p'][self.model.n_opt_vars*k+2*self.model.n_states
                           :self.model.n_opt_vars*k+2*self.model.n_states+self.model.n_controls] = u_ref
    
        self.args['x0'] = ca.vertcat(self.X0.reshape((-1,1)),self.u0.reshape((-1,1)))

        # Solve the NMPC optimization problem
        logging.info(f"MPC compute control - solving at iteration: {mpciter}")
        sol = self.solver(**self.args)
        logging.info(f"MPC compute control - solving done at iteration: {mpciter}")

        u_opt = sol['x'][self.model.n_states*(self.N+1):].reshape((self.model.n_controls,self.N))
        self.X0 = sol['x'][:self.model.n_states*(self.N+1)].reshape((self.model.n_states,self.N+1))
        logging.info(f"MPC compute control - extracting controls and states at iteration: {mpciter}")
        
        # print("### Computed Solution X0", self.X0)
        # print("### Constraints : ",self.args['p'])
        p = self.args['p'][self.model.n_states:].reshape((self.model.n_opt_vars,self.N))
        self.p_history = np.dstack((self.p_history,p))


        # print(u_opt.shape, self.X0.shape, self.u_opt_history.shape)
        self.u_opt_history=np.dstack((self.u_opt_history,u_opt))
        self.x_history=np.dstack((self.x_history,self.X0))

        return u_opt

    def run_step(self,u, current_state=None):
        logging.info(f"MPC run step")
        if current_state is None:
            logging.info(f"MPC run step - Shifting")
            # st_runge_kutta
            st_next_aprx = discretize_rk4(self.model.f_function, self.x0, u[:,0], self.dT, self.only_euler)
            self.x0  = st_next_aprx
            print("### Calulating x0+1 using f value:", self.x0)
            logging.info(f"MPC run step - Shifting Done")
        else:
            self.x0 = current_state
        self.u0[:,:-1] = u[:,1:]
        self.X0[:,:-1]=self.X0[:,1:]