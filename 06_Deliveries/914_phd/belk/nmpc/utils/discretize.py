def discretize_rk4(f_function, state_vector, control_vector, dt, only_euler=False):
    # st_runge_kutta
    k1 = f_function(state_vector, control_vector)
    if only_euler:
        return state_vector + k1*dt
    k2 = f_function(state_vector+dt/2*k1, control_vector)
    k3 = f_function(state_vector+dt/2*k2, control_vector)
    k4 = f_function(state_vector+dt/2*k3, control_vector)
    
    return state_vector+dt/6*(k1+2*k2+2*k3+k4)