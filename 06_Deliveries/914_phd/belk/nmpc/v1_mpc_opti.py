
from casadi import *
from time import time
from utils.trajectory import ReferenceTrajectory
from utils.config import load_yaml_config
from utils.visualization import simulate


## prepare the environement
config = load_yaml_config('./configs/basic.yaml')
print(config)
trajectory = ReferenceTrajectory(**config['NMPC.environment']['trajectory'])
## run the environement
t=[]
reference = trajectory.get_reference()
mpciter=0
t0 = 0

N = 10 # number of control intervals
dT = 0.1 # length of a control interval
Q = diag([10,10,0.5,0])
R = diag([0.5,0.05])

L=3
Lr=1.382
model = 'fac'

class Model:
   def __init__(self, model) -> None:
      self.L = model['L']
      self.Lr = model['Lr']
      self.type = model['model_type']
      pass

   def _f(self, st, con):
      psi,delta   =  st[2],st[3]
      v,phi       =  con[0],con[1]
      if model == 'cog':
         beta  = atan(self.Lr/self.L*tan(delta))
         dx    = cos(psi+beta)
         dy    = sin(psi+beta)
         dpsi  = tan(delta) * cos(beta)
      
      if model == 'fac':
         dx    = cos(psi+delta)
         dy    = sin(psi+delta)
         dpsi  = sin(delta)
      
      if model == 'rac':
         dx    = cos(psi)
         dy    = sin(psi)
         dpsi  = tan(delta)
      
      return vertcat(  v * dx
                     , v * dy
                     , v / self.L  * dpsi
                     , phi
                     )

   def F(self,st,con):
         # Runge-Kutta 4 integration
      k1 = f(st         ,con)
      k2 = f(st+dT/2*k1 ,con)
      k3 = f(st+dT/2*k2 ,con)
      k4 = f(st+dT*k3   ,con)
      return st+ dT/6*(k1+2*k2+2*k3+k4)


class NMPC:
   def __init__(self) -> None:
      self.opti = Opti()
      self.x0 = None
      self.N = None
      self._setup()
      pass

   def _setup(self):
      n_states=4
      n_controls=2
      # ---- decision variables ---------
      #states
      self.X = self.opti.variable(n_states,N+1) # state trajectory
      #controls
      self.U = self.opti.variable(n_controls,N)   # control trajectory (throttle)
      #parameters
      self.P_x = self.opti.parameter(n_states,N+1)     
      self.P_u = self.opti.parameter(n_controls,N)

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
      opti.solver("ipopt",opts) # set numerical backend    
      pass

   def _set_objective(self):
      obj=0
      for k in range(self.N): # loop over control intervals
         # objective to minimize
         st = self.X[:,k]-self.P_x[:,k]
         con = self.U[:,k]-self.P_u[:,k]
         obj += st.T@Q@st + con.T@R@con

      self.opti.minimize(obj) 

   def _set_constraints(self):
      #states
      x_x     = self.X[0,:]
      x_y     = self.X[1,:]
      x_psi   = self.X[2,:]
      x_delta = self.X[3,:]
      #controls
      u_v     = self.U[0,:]
      u_phi   = self.U[1,:]
      # ---- dynamics conditions -----------
      # subject to dynamics xk+1 = F(xk,uk)
      for k in range(self.N):
         st_next = self.F(self.X[:,k], self.U[:,k])
         self.opti.subject_to(self.X[:,k+1]==st_next) # close the gaps
      # ---- boundary conditions -----------
      #states
      self.opti.subject_to(self.opti.bounded(-inf,x_x,inf)) # state is limited
      self.opti.subject_to(self.opti.bounded(-inf,x_y,inf)) # state is limited
      self.opti.subject_to(self.opti.bounded(-pi,x_psi,pi)) # state is limited
      self.opti.subject_to(self.opti.bounded(-pi/2.5,x_delta,pi/2.5)) # state is limited
      #controls
      self.opti.subject_to(self.opti.bounded(0,u_v,25)) # control is limited
      self.opti.subject_to(self.opti.bounded(-pi/4,u_phi,pi/4)) # control is limited
      pass

   def _set_initial_values(self):
      self.opti.set_value(self.P_u,repmat([0,0],1,self.N))
      self.opti.set_value(self.P_x,repmat(self.x0,1,self.N+1))
      pass

   def compute_control(self, p_x_ref, p_u_ref):
      opti.set_value(P_x[:,k+1],[xref,yref,psiref,deltaref])
      opti.set_value(P_u[:,k],u_ref)
      # ---- solve NLP              ------
      sol = opti.solve()   # actual solve
      return sol
   # def compute_control(self):
   #    for k in range(self.N):
   #       t_predict = (mpciter+k)*dT
   #       xref,yref,psiref,deltaref = trajectory.get_next_wp(t_predict)
   #       psiref=np.clip(psiref, -pi,pi)
   #       u_ref= trajectory.get_fd_wp(t_predict)
   #       opti.set_value(P_x[:,k+1],[xref,yref,psiref,deltaref])
   #       opti.set_value(P_u[:,k],u_ref)
   #    # ---- solve NLP              ------
   #    sol = opti.solve()   # actual solve
   #    return sol

   def run_step(self,x0,u_opt,noise_level=0):
      ### shifting the solution
      x_next = self.F(x0,u_opt[:,0])
      # if mpciter % 10 == 0:
      #    x_next = x_next + noise_level*np.random.random_sample((4,))
      self.opti.set_value(self.P_x[:,0],x_next)
      self.opti.set_value(self.P_u[:,:-1],u_opt[:,1:])




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
   def load(self):
      self.x = np.load('data/x_'+str(time()))
      self.u = np.load('data/u_'+str(time()))
      self.p = np.load('data/p_'+str(time()))

   def save(self):
      np.save('data/x_'+str(time()), self.x)
      np.save('data/u_'+str(time()), self.u)
      np.save('data/p_'+str(time()), self.p)



history = History(N)
try:
   while (True):
      # compute mpc controls
      sol = compute_control()
      # extract the solution
      u_opt = sol.value(U)
      X0 = sol.value(X)
      # extract parameters
      p_x=sol.value(P_x[:,1:])
      p_u=sol.value(P_u)
      p = vertcat(p_x,p_u)
      # keep the history
      history.add(X0,u_opt,p)
      # shift the solution and apply the first control
      run_step(X0[:,0],u_opt)
      # stop condition
      distance_p = np.linalg.norm(trajectory.xs[0:2]-history.p[0:2,0,mpciter])
      if distance_p <0.5:
         break
      mpciter += 1  
      t.append(t0)
except Exception as e:
   print(e)   

def plot_sim():
   sim = simulate(trajectory.path, history.p, history.x, history.u, t, dT, N,reference, False)
   sim.to_jshtml(30,True)


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