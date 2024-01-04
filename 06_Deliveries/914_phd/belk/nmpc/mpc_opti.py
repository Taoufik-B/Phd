
from casadi import *
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
dt = 0.1 # length of a control interval
Q = diag([10,10,0.5,0])
R = diag([0.5,0.05])

L=3
Lr=1.382

opti = Opti() # Optimization problem


n_states=4
n_controls=2
# ---- decision variables ---------
X = opti.variable(n_states,N+1) # state trajectory
x   = X[0,:]
y = X[1,:]
psi = X[2,:]
delta = X[3,:]
U = opti.variable(n_controls,N)   # control trajectory (throttle)
v = U[0,:]
phi = U[1,:]
# T = opti.variable()      # final time

P_x = opti.parameter(4,N+1)     
P_u = opti.parameter(2,N)     
# opti.set_value(P,)
# ---- dynamic constraints --------
# def f(st,u):
#    psi,delta_a=st[2],st[3]
#    v,phi=u[0],u[1]
#    beta = atan(Lr/L*tan(delta_a))
#    return vertcat(  v * cos(psi+beta)
#                   , v * sin(psi+beta)
#                   , v * tan(delta_a) * cos(beta) / L
#                   , phi
#                   )
f = lambda x,u: vertcat( u[0]*cos(x[2])
                        ,u[0]*sin(x[2])
                        ,u[0]*tan(x[3]/L)
                        ,u[1]
                        # ,u[1]
                        ) # dx/dt = f(x,u)
# f = lambda x,u: vertcat( u[0]*cos(x[2])
#                         ,u[0]*sin(x[2])
#                         ,u[0]/3.0*tan(x[3])
#                         ,u[1]
#                         ) # dx/dt = f(x,u)

obj = 0
# T=20
# dt = T/N/4 # length of a control interval

# def shift(st,con,dt):
#    # euler
#    return st+ dt*f(st,         con)
def shift(st,con,dt):
   # Runge-Kutta 4 integration
   # dt=dt/4
   k1 = f(st,         con)
   k2 = f(st+dt/2*k1, con)
   k3 = f(st+dt/2*k2, con)
   k4 = f(st+dt*k3, con)
   return st+ dt/6*(k1+2*k2+2*k3+k4)
# dt = 0.5# length of a control interval
for k in range(N): # loop over control intervals
   # X[:,k+1] = shift(X[:,k], U[:,k], dt)
   st_next = shift(X[:,k], U[:,k], dt)
   opti.subject_to(X[:,k+1]==st_next) # close the gaps
   st = X[:,k]-P_x[:,k]
   con = U[:,k]-P_u[:,k]
   obj += st.T@Q@st + con.T@R@con 

# ---- objective          ---------
opti.minimize(obj) # race in minimal time

# opti.subject_to(X[:,0]==P_x[:,0])   # start at position

# ---- path constraints -----------
# limit = lambda pos: 1-sin(2*pi*pos)/2
# opti.subject_to(v<=limit(x)[:-1])   # track speed limit

# ---- boundary conditions -----------
#controls
opti.subject_to(opti.bounded(0,v,25)) # control is limited
opti.subject_to(opti.bounded(-pi/4,phi,pi/4)) # control is limited
#states
opti.subject_to(opti.bounded(-inf,x,inf)) # state is limited
opti.subject_to(opti.bounded(-inf,y,inf)) # state is limited
opti.subject_to(opti.bounded(-pi,psi,pi)) # state is limited
opti.subject_to(opti.bounded(-pi/2.5,delta,pi/2.5)) # state is limited



# ---- path constraints -----------
# opti.subject_to(v==0) # ... from stand-still 
# opti.subject_to(x[-1]==3)  # finish line at position 1
# opti.subject_to(y[-1]==2.5)  # finish line at position 1

# ---- misc. constraints  ----------
# opti.subject_to(T>=0) # Time must be positive

# ---- initial values for solver ---
# opti.set_initial(v, 1)
# opti.set_initial(X, repmat(trajectory.x0,1,N+1))
# opti.set_initial(X, repmat(trajectory.x0,1,N+1))
# opti.set_initial(T, 1)
opti.set_value(P_u,repmat([0,0],1,N))
opti.set_value(P_x,repmat(trajectory.x0,1,N+1))


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
opti.solver("ipopt",opts) # set numerical backend


## History controls and states
u_opt_history=np.zeros((2,N,0)) # controls
x_history=np.zeros((4,N+1,0)) # states
p_history=np.zeros((6,N,0)) # parameters

## noise test
mu, sigma = 0, 0.1

try:
   while (True):
      for k in range(N):
         t_predict = (mpciter+k)*dt
         xref,yref,psiref,deltaref = trajectory.get_next_wp(t_predict)
         psiref=np.clip(psiref, -pi,pi)
         u_ref= trajectory.get_fd_wp(t_predict)
         opti.set_value(P_x[:,k+1],[xref,yref,psiref,deltaref])
         opti.set_value(P_u[:,k],u_ref)
      # ---- solve NLP              ------
      sol = opti.solve()   # actual solve
      u_opt = sol.value(U)
      X0 = sol.value(X)
      p_x=sol.value(P_x[:,1:])
      p_u=sol.value(P_u)
      p = vertcat(p_x,p_u)
      p_history = np.dstack((p_history,p))
      # print(u_opt.shape, self.X0.shape, self.u_opt_history.shape)
      u_opt_history=np.dstack((u_opt_history,u_opt))
      x_history=np.dstack((x_history,X0))
      ### shifting the solution
      x_next = shift(X0[:,0],u_opt[:,0],dt)
      x_next_noise = x_next
      # if mpciter % 10 == 0:
      #    x_next_noise = np.random.random_sample((4,))*2+x_next
      print("x_next",x_next)
      print("x_next_noise", x_next_noise)
      opti.set_value(P_x[:,0],x_next_noise)
      # opti.set_value(P_x[:,0],X0[:,1])

      opti.set_value(P_u[:,:-1],u_opt[:,1:])
      # opti.set_value(P_u[:,-1],u_opt[:,-1])
      distance_p = np.linalg.norm(trajectory.xs[0:2]-p_history[0:2,0,mpciter])
      if distance_p <0.5:
         break
      mpciter += 1  
      t.append(t0)
except Exception as e:
   print(e)   


sim = simulate(trajectory.path, p_history, x_history, u_opt_history, t, dt, N,reference, False)


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