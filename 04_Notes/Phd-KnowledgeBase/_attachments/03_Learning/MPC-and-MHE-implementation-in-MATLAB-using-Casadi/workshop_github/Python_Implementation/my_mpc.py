from time import time
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from simulation_code import simulate


def shift(step_horizon, t0, state_init, u, f):
    f_value = f(state_init, u[:, 0])
    next_state = ca.DM.full(state_init + (step_horizon * f_value))

    t0 = t0 + step_horizon
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:, -1], -1, 1)
    )

    return t0, next_state, u0


T = 0.2     # sampling time
N = 3       # prediction horizon

v_max = 0.6
v_min = -v_max

omega_max = ca.pi/4
omega_min = -omega_max


################
# Robot state
################
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')

states = ca.vertcat(        # concat the matrice or vector vertically
    x,
    y,
    theta
)
n_states = states.numel()   # The number of elements

################
# Robot control
################
v = ca.SX.sym('v')
omega = ca.SX.sym('omega')

controls = ca.vertcat(        # concat the matrice or vector vertically
    v,
    omega
)
n_controls = controls.numel()   # The number of elements

# Define the right hand function
rhs = ca.vertcat(
    v*ca.cos(theta),
    v*ca.sin(theta),
    omega
)

# TODO : this represents the f function
## define non linear mapping function
def apply_dynamics(states, controls):
    return rhs

f = ca.Function('f', [states,controls],[rhs])

# Decision variables
U = ca.SX.sym('U', n_controls, N)   # controls
P = ca.SX.sym('P', n_states+n_states) # initial and reference state of the robot

# Matrix of the states over the optimization problem
X = ca.SX.sym('X', n_states, N+1)

# compute the solution symbolically
X[:,0] = P[0:3]

for k in range(N):
    print(k)
    st = X[:,k]
    con = U[:,k]
    f_value = f(st, con)
    st_next = st + (T*f_value)
    X[:,k+1]=st_next

# print(X)

# get optimal trajectory knowing the optimal solution
ff=ca.Function('ff', [U,P], [X])
def get_op_trajectory(U,P):
    return X

g = [] # optimization constraints : empty
# P = [] # optimization problem parameters : empty

# weighing matrices states
Q = np.diag((5, 10, 0.05))
# weighing matrices controls
R = np.diag((0.5, 0.05))

obj = 0 # Calcul of the objective
## compute the obj function
for k in range(N):
    print(k)
    st = X[:,k]
    con = U[:,k]
    obj = obj + (st-P[3:6]).T@Q@(st-P[3:6])+con.T@R@con
# print(obj)

# compute constraints
for k in range(N):
    print(k)
    g = ca.vertcat(g, X[1,k]) #state x
    g = ca.vertcat(g, X[2,k]) #state y
# print(g)

# make the decision variables one column vector
OPT_variables = U.reshape((-1,1))
nlp_prob = {'f':obj, 'x':OPT_variables, 'g':g, 'p':P}

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
solver = ca.nlpsol('solver','ipopt', nlp_prob, opts)
print(nlp_prob)

t0 = 0
t = []
x0 = ca.DM.zeros(n_states)
# print(x0.shape)
xs = [5,5,0]
u0 = ca.DM.zeros((N,2))
# print(u0)
sim_time = 20
mpciter = 0
xx1 = np.zeros((n_states,N+1, int(sim_time/T)))
xx = np.zeros((n_states,int(sim_time/T)))
xx[:,0]=x0.full().reshape((-1,))
u_cl = []
# print(np.shape(xx))
# print(sim_time/T)

args = {
    'lbx' : [v_min, omega_min]*N,
    'ubx' : [v_max, omega_max]*N,
    'lbg' : -2,
    'ubg' : 2,
    'p': ca.vertcat(x0,xs),
    'x0': u0.T.reshape((-1,1))
}


while (ca.norm_2(x0[:,0] - xs).full()[0] > 1e-2) and (mpciter <= (sim_time / T)):
    args['p'] = ca.vertcat(x0,xs)
    args['x0'] = u0.T.reshape((-1,1))
    sol = solver(**args)
    u_opt = ca.reshape(sol['x'].full(), 2, -1)
    ff_value = ff(u_opt, args['p']) # compute optimal solution trajectory
    xx1[:,range(N+1),mpciter-1] = ff_value.full()
    u_cl = ca.vertcat(u_cl, u_opt[:,0])
    t.append(t0)
    t0, x0, u0 = shift(T, t0, x0, u_opt, f)
    # print("---x0 ",x0)
    # print("---", mpciter)
    # print(xx[:,mpciter-1])
    xx[:,mpciter-1] = x0.reshape((-1,))
    mpciter = mpciter+1
    ss_error = ca.norm_2(x0[:,0] - xs).full()
    print(ss_error, mpciter)

print(sim_time/T)

reference = [0.,0.,0.,5,5,0.]
print(reference[0])

ss_error = ca.norm_2(x0[:,0] - xs)
print(ss_error)
simulate(xx1, u_cl, [sim_time]*(int(sim_time/T)), T, N,reference, True)