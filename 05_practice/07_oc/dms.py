from casadi import *
from numpy import *
import matplotlib.pyplot as plt

N = 20      # Control discretization
T = 10.0    # End time

# Declare variables (use scalar graph)
u  = SX.sym("u")    # control
x  = SX.sym("x",2)  # states

# System dynamics
xdot = vertcat( [(1 - x[1]**2)*x[0] - x[1] + u, x[0]] )
qdot = x[0]**2 + x[1]**2 + u**2
f = SXFunction([x,u],[xdot,qdot])
f.init()

# RK4 with M steps
U = MX.sym("U")
X = MX.sym("X",2)
M = 10; DT = T/(N*M)
XF = X
QF = 0
for j in range(M):
    [k1, k1_q] = f([XF,             U])
    [k2, k2_q] = f([XF + DT/2 * k1, U])
    [k3, k3_q] = f([XF + DT/2 * k2, U])
    [k4, k4_q] = f([XF + DT   * k3, U])
    XF += DT/6*(k1   + 2*k2   + 2*k3   + k4)
    QF += DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
F = MXFunction([X,U],[XF,QF])
F.init()

# Formulate NLP (use matrix graph)
nv = 1*N + 2*(N+1)
v = MX.sym("v", nv)

# Get the state for each shooting interval
xk = [v[3*k : 3*k + 2] for k in range(N+1)]

# Get the control for each shooting interval
uk = [v[3*k + 2] for k in range(N)]

# Variable bounds and initial guess
vmin = -inf*ones(nv)
vmax =  inf*ones(nv)

# Initial solution guess
v0 = zeros(nv)

# Control bounds
vmin[2::3] = -1.0
vmax[2::3] =  1.0

# Initial condition
vmin[0] = vmax[0] = v0[0] = 0
vmin[1] = vmax[1] = v0[1] = 1

# Terminal constraint
vmin[-2] = vmax[-2] = v0[-2] = 0
vmin[-1] = vmax[-1] = v0[-1] = 0

# Constraint function with bounds
g = []; gmin = []; gmax = []

# Objective function
J=0

# Build up a graph of integrator calls
for k in range(N):
    # Call the integrator
    [xf, qf] = F([xk[k],uk[k]])

    # Add contribution to objective
    J += qf
   
    # Append continuity constraints
    g.append(xf - xk[k+1])
    gmin.append(zeros(2))
    gmax.append(zeros(2))

# Concatenate constraints
g = vertcat(g)
gmin = concatenate(gmin)
gmax = concatenate(gmax)

# Create NLP solver instance
nlp = MXFunction(nlpIn(x=v),nlpOut(f=J,g=g))
solver = NlpSolver("ipopt", nlp)
solver.init()

# Set bounds and initial guess
solver.setInput(v0,   "x0")
solver.setInput(vmin, "lbx")
solver.setInput(vmax, "ubx")
solver.setInput(gmin, "lbg")
solver.setInput(gmax, "ubg")

# Solve the problem
solver.evaluate()

# Retrieve the solution
v_opt = solver.getOutput("x")
x0_opt = v_opt[0::3]
x1_opt = v_opt[1::3]
u_opt = v_opt[2::3]

# Plot the results
plt.figure(1)
plt.clf()
plt.plot(linspace(0,T,N+1),x0_opt,'--')
plt.plot(linspace(0,T,N+1),x1_opt,'-')
plt.step(linspace(0,T,N),u_opt,'-.')
plt.title("Van der Pol optimization - multiple shooting")
plt.xlabel('time')
plt.legend(['x0 trajectory','x1 trajectory','u trajectory'])
plt.grid()
plt.show()