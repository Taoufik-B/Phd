#
#     MIT No Attribution
#
#     Copyright (C) 2010-2023 Joel Andersson, Joris Gillis, Moritz Diehl, KU Leuven.
#
#     Permission is hereby granted, free of charge, to any person obtaining a copy of this
#     software and associated documentation files (the "Software"), to deal in the Software
#     without restriction, including without limitation the rights to use, copy, modify,
#     merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
#     permit persons to whom the Software is furnished to do so.
#
#     THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#     INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#     PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#     HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
#     OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#     SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

# Car race along a track
# ----------------------
# An optimal control problem (OCP),
# solved with direct multiple-shooting.
#
# For more information see: http://labs.casadi.org/OCP
from casadi import *

N = 20 # number of control intervals

opti = Opti() # Optimization problem

# ---- decision variables ---------
X = opti.variable(4,N+1) # state trajectory
x   = X[0,:]
y = X[1,:]
psi = X[2,:]
delta = X[3,:]
U = opti.variable(2,N)   # control trajectory (throttle)
v = U[0,:]
phi = U[1,:]
T = opti.variable()      # final time

P_x = opti.parameter(4,N+1)     
P_u = opti.parameter(2,N)     
# opti.set_value(P,)
Q = diag([10,10,0.5,0.05])
R = diag([0.5,0.05])

# ---- dynamic constraints --------
f = lambda x,u: vertcat( u[0]*cos(x[2])
                        ,u[0]*sin(x[2])
                        ,u[0]/3.0*tan(x[3])
                        ,u[1]
                        ) # dx/dt = f(x,u)

obj = 0
dt = T/N/4 # length of a control interval
# dt = 0.5# length of a control interval
for k in range(N): # loop over control intervals
   # Runge-Kutta 4 integration
   k1 = f(X[:,k],         U[:,k])
   k2 = f(X[:,k]+dt/2*k1, U[:,k])
   k3 = f(X[:,k]+dt/2*k2, U[:,k])
   k4 = f(X[:,k]+dt*k3,   U[:,k])
   x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4)
   st = X[:,k]-P_x[:,k]
   con = U[:,k]-P_u[:,k]
   obj += st.T@Q@st + con.T@R@con 
   opti.subject_to(X[:,k+1]==x_next) # close the gaps

# ---- objective          ---------
opti.minimize(obj) # race in minimal time


# ---- path constraints -----------
# limit = lambda pos: 1-sin(2*pi*pos)/2
# opti.subject_to(v<=limit(x)[:-1])   # track speed limit
opti.subject_to(opti.bounded(0,v,5)) # control is limited
opti.subject_to(opti.bounded(-0.78,phi,0.78)) # control is limited

opti.subject_to(opti.bounded(-2,x,5)) # state is limited
opti.subject_to(opti.bounded(-2,y,5)) # state is limited
opti.subject_to(opti.bounded(-3.14,psi,3.14)) # state is limited
opti.subject_to(opti.bounded(-1.22,delta,1.22)) # state is limited



# ---- boundary conditions --------
opti.subject_to(x>=0)   # start at position 0 ...
opti.subject_to(y>=0)   # start at position 0 ...
opti.subject_to(x[0]==0)   # start at position 0 ...
opti.subject_to(y[0]==0)   # start at position 0 ...
# opti.subject_to(v==0) # ... from stand-still 
opti.subject_to(x[-1]==3)  # finish line at position 1
opti.subject_to(y[-1]==2.5)  # finish line at position 1

# ---- misc. constraints  ----------
opti.subject_to(T>=0) # Time must be positive

# ---- initial values for solver ---
opti.set_initial(v, 1)
opti.set_initial(x, 0)
opti.set_initial(y, 0)
opti.set_initial(T, 1)
opti.set_value(P_u,repmat([5,0],1,N))
opti.set_value(P_x,horzcat([0,0,0,0],repmat([0,0,0,0],1,N)))

for k in range(N):
   xref=0.5*k
   yref=2.5
   opti.set_value(P_x[0:2,k+1],[xref,yref])
# ---- solve NLP              ------
opti.solver("ipopt") # set numerical backend
sol = opti.solve()   # actual solve

# ---- post-processing        ------
from pylab import plot, step, figure, legend, show, spy, grid

print("Priniting solutions")
print("Sol x", x, sol.value(x))
print("Sol y", y, sol.value(y))
print(sol.value(X))
print(sol.value(P_x))
print(sol.value(P_u))
X0= sol.value(X)
xp=X0[0,:]
yp=X0[1,:]



plot(3,2.5, 'ok', label='Target point')
plot(0,0, 'om', label='Initial point')

plot(xp,yp,label="[x,y]")
plot(sol.value(psi),label="psi")
plot(sol.value(delta),label="delta")
# plot(limit(sol.value(x)),'r--',label="speed limit")
step(range(N),sol.value(v),'r--',label="v")
step(range(N),sol.value(phi),'k--',label="phi")
legend(loc="upper right")
grid()


# figure()
# spy(sol.value(jacobian(opti.g,opti.x)))
# figure()
# spy(sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]))

show()