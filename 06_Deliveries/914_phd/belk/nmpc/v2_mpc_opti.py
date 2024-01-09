
import yaml
import argparse
import logging
from casadi import *
from time import time
from utils.trajectory import ReferenceTrajectory
from utils.visualization import simulate
from simu.carla_simu import Simulation



class KinematicBicycleModel:
   def __init__(self, model) -> None:
      self.L = model['L']
      self.Lr = model['Lr']
      self.type = model['model_type']
      self.x_bounds = model['bounds']['x']
      self.u_bounds = model['bounds']['u']
      pass

   def f(self, st, con):
      psi,delta   =  st[2],st[3]
      v,phi       =  con[0],con[1]
      self.dx     = None
      self.dy     = None
      self.dpsi   = None
      if self.type == 'cog':
         beta  = atan(self.Lr/self.L*tan(delta))
         self.dx    = cos(psi+beta)
         self.dy    = sin(psi+beta)
         self.dpsi  = tan(delta) * cos(beta)
      
      if self.type == 'fac':
         self.dx    = cos(psi+delta)
         self.dy    = sin(psi+delta)
         self.dpsi  = sin(delta)
      
      if self.type == 'rac':
         self.dx    = cos(psi)
         self.dy    = sin(psi)
         self.dpsi  = tan(delta)
      
      return vertcat(  v * self.dx
                     , v * self.dy
                     , v / self.L  * self.dpsi
                     , phi
                     )

class NMPC:
   def __init__(self, dae, x0, dT, N, Q, R, only_euler) -> None:
      self.opti = Opti()
      self.x0 = x0
      self.N = N
      self.dT = dT
      self.Q, self.R = diag(Q),diag(R)
      self.dae = dae
      self.euler = only_euler
      self._setup()

   def _setup(self):
      n_states=4
      n_controls=2
      # ---- decision variables ---------
      #states
      self.X = self.opti.variable(n_states,self.N+1) # state trajectory
      #controls
      self.U = self.opti.variable(n_controls,self.N)   # control trajectory (throttle)
      #parameters
      self.P_x = self.opti.parameter(n_states,self.N+1)     
      self.P_u = self.opti.parameter(n_controls,self.N)

      # Objective
      self._set_objective()

      # Constraints
      self._set_constraints()

      # Initials
      self._set_initial_values()

      ## Options
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
      ## set the solver
      self.opti.solver("ipopt",opts) # set numerical backend    
      pass

   def _set_objective(self):
      obj=0
      for k in range(self.N): # loop over control intervals
         # objective to minimize
         st = self.X[:,k]-self.P_x[:,k]
         con = self.U[:,k]-self.P_u[:,k]
         obj += st.T@self.Q@st + con.T@self.R@con

      self.opti.minimize(obj) 

   def _set_constraints(self):
      # ---- dynamics conditions -----------
      # subject to dynamics xk+1 = F(xk,uk)
      for k in range(self.N):
         st_next = self._F(self.X[:,k], self.U[:,k])
         self.opti.subject_to(self.X[:,k+1]==st_next) # close the gaps
      # ---- boundary conditions -----------
      ## Bounds
      lb_x = self.dae.x_bounds[0]
      ub_x = self.dae.x_bounds[1]
      lb_u = self.dae.u_bounds[0]
      ub_u = self.dae.u_bounds[1]
      #states
      self.opti.subject_to(self.opti.bounded(lb_x,self.X,ub_x)) # state is limited
      #controls
      self.opti.subject_to(self.opti.bounded(lb_u,self.U,ub_u)) # control is limited
      pass

   def _set_initial_values(self):
      self.opti.set_value(self.P_u,repmat([0,0],1,self.N))
      self.opti.set_value(self.P_x,repmat(self.x0,1,self.N+1))
      pass

   def _F(self,st,con):
         # Runge-Kutta 4 integration
      k1 = self.dae.f(st         ,con)
      if self.euler:
         return st+self.dT*k1
      k2 = self.dae.f(st+self.dT/2*k1 ,con)
      k3 = self.dae.f(st+self.dT/2*k2 ,con)
      k4 = self.dae.f(st+self.dT*k3   ,con)
      return st+ self.dT/6*(k1+2*k2+2*k3+k4)

   def compute_control(self, p_x_ref, p_u_ref):
      print("compute control x", p_x_ref)
      print("compute control u", p_u_ref)
      self.opti.set_value(self.P_x[:,1:],p_x_ref)
      self.opti.set_value(self.P_u,p_u_ref)
      # ---- solve NLP              ------
      sol = self.opti.solve()   # actual solve
      return sol
   
   def update_mpc(self, x_next, u_opt):
      self.opti.set_value(self.P_x[:,0],x_next)
      self.opti.set_value(self.P_u[:,:-1],u_opt[:,1:])

   def run_step(self,x0,u_opt,noise_level=0):
      ### shifting the solution
      x_next = self._F(x0,u_opt[:,0])
      # if mpciter % 10 == 0:
      #    x_next = x_next + noise_level*np.random.random_sample((4,))
      self.update_mpc(x_next, u_opt)

class History:
   def __init__(self, N) -> None:
      ## History controls and states
      self.x=np.zeros((4,N+1,0)) # states
      self.u=np.zeros((2,N,0)) # controls
      self.p=np.zeros((6,N,0)) # parameters
      pass

   def add(self, x,u,p):
      self.x = np.dstack((self.x,x))
      self.u = np.dstack((self.u,u))
      self.p = np.dstack((self.p,p))

   # TODO: find a way to load the history and save the history from a folder
   def load(self, scenario):
      self.x = np.load('data/x_'+scenario)
      self.u = np.load('data/u_'+scenario)
      self.p = np.load('data/p_'+scenario)

   def save(self,scenario):
      np.save('data/x_'+scenario, self.x)
      np.save('data/u_'+scenario, self.u)
      np.save('data/p_'+scenario, self.p)

class Config:
   def __init__(self, path) -> None:
      self.path = path
      self.data = self._load_yaml_config()

   def _load_yaml_config(self):
    with open(self.path) as f:
        return yaml.load(f, Loader=yaml.FullLoader)

def run(config):
   SCENARIO_id    = config.data['id']
   SCENARIO_OBJ   = config.data['objective']
   NMPC_internals = config.data['NMPC.internals']
   NMPC_externals = config.data['NMPC.externals']
   VEHICULE_model = config.data['NMPC.externals']['vehicle']
   PATH_data      = config.data['NMPC.environment']['trajectory']
   CARLA_simu     = config.data['NMPC.environment']['carla_simu']


   N        = NMPC_internals['N']
   dT       = NMPC_internals['dT']

   history  = History(N)
   path     = ReferenceTrajectory(**PATH_data)
   dae      = KinematicBicycleModel(VEHICULE_model)
   nmpc     = NMPC(dae,path.x0,**NMPC_internals)

   ### carla simulation
   if CARLA_simu:
      carla_simu = Simulation()
      carla_simu.setup(nmpc,path.x0)
      simu_run_step = carla_simu.run_step
   else:
      simu_run_step = nmpc.run_step


   ## run the environement
   reference = path.get_reference()
   t=[]
   mpciter=0
   t0 = 0
   try:
      while (True):
         # get ref trajectory
         p_x_ref,p_u_ref = path.get_tracking_wps(mpciter, N, dT)
         # compute mpc controls
         sol = nmpc.compute_control(p_x_ref, p_u_ref)
         # extract the solution
         u_opt = sol.value(nmpc.U)
         X0 = sol.value(nmpc.X)
         # extract parameters
         p_x=sol.value(nmpc.P_x[:,1:])
         p_u=sol.value(nmpc.P_u)
         p = vertcat(p_x,p_u)
         # keep the history
         history.add(X0,u_opt,p)
         # shift the solution and apply the first control
         simu_run_step(X0[:,0],u_opt)
         # stop condition
         distance_p = np.linalg.norm(path.xs[0:2]-history.p[0:2,0,mpciter])
         if distance_p <0.5:
            break
         mpciter += 1  
         t.append(t0)
   except Exception as e:
      print(e)   
   finally:
      if CARLA_simu:
         carla_simu.teardown()
      if history:
         history.save(SCENARIO_id)
         simulate( trajectory=path.path
                  ,params=history.p
                  ,cat_states=history.x
                  ,cat_controls=history.u
                  ,t=t
                  ,step_horizon=dT
                  ,N=N
                  ,reference=reference
                  ,scenario=SCENARIO_id
                  ,save=False
                  )

   # def plot_sim(save=False):
   # sim.to_jshtml(30,True)


def main():
   """
   ================================
   Recognizing hand-written digits
   ================================

   An example showing how the scikit-learn can be used to recognize images of
   hand-written digits.

   This example is commented in the
   :ref:`tutorial section of the user manual <introduction>`.

   """
   argparser = argparse.ArgumentParser(
      description='NMPC Simulation')
   argparser.add_argument(
      '-v', '--verbose',
      action='store_true',
      dest='debug',
      help='print debug information')
   argparser.add_argument(
      '-c','--config',
      metavar='cfg',
      default='./configs/basic.yaml',
      help='configuration file in yaml form under ./configs')


   args = argparser.parse_args()

   log_level = logging.DEBUG if args.debug else logging.INFO

   # Create a logger
   logger = logging.getLogger(args.config)
   logger.setLevel(log_level)


   # Create a file handler which logs even debug messages
   fh = logging.FileHandler(f'logs/{args.config}.log')
   fh.setLevel(logging.DEBUG)
   # Create a formatter and set the formatter for the handler
   formatter = logging.Formatter('%(asctime)s %(name)s %(levelname)-8s: %(message)s',datefmt='%Y-%m-%d %H:%M:%S')
   fh.setFormatter(formatter)

   # Add the handler to the logger
   logger.addHandler(fh)


   print(__doc__)


   ## Debug
   print(args)

   ## Mocking
   # - from arg config, specify the config folder to ŕun the simulation
   # function to list the files in the folder. config variable of the args.config
   # specify the scenario to run in the scenario config 
   # config option should enable to either save png or gif simulation data
   # config option shall enable loading and execution annimation or plotting image
   print(args.config)
   cp_config = Config(f"configs/{args.config}")
   compaign= cp_config.data
   print(compaign)
   for sc in compaign['scenarios']:
      sc_config_path = f"configs/{compaign['id']}/{sc}.yaml"
      print(sc_config_path)
      # logger.info(Config(sc_config_path).data)
      sc_config = Config(sc_config_path)
      # run the step simulation
      # - in run run simulation the log must be configured to capture all the necessary data and save the figure 
      run(sc_config)   

if __name__ == '__main__':
   main()
# plot_sim()

# ---- post-processing        ------
# from pylab import plot, step, figure, legend, show, spy, grid

# print("Priniting solutions")
# print("Sol x", x, sol.value(x))
# print("Sol y", y, sol.value(y))
# print(sol.value(X))
# X0= sol.value(X)
# xp=X0[0,:]
# yp=X0[1,:]



# plot(3,2.5, 'ok', label='Target point')
# plot(0,0, 'om', label='Initial point')

# plot(xp,yp,label="[x,y]")
# plot(sol.value(psi),label="psi")
# plot(sol.value(delta),label="delta")
# # plot(limit(sol.value(x)),'r--',label="speed limit")
# step(range(N),sol.value(v),'r--',label="v")
# step(range(N),sol.value(phi),'k--',label="phi")
# legend(loc="upper right")
# grid()


# # figure()
# # spy(sol.value(jacobian(opti.g,opti.x)))
# # figure()
# # spy(sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]))

# show()